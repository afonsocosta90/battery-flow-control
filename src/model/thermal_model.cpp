#include "btm/model/thermal_model.hpp"
#include "btm/config/config.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace btm::model {

namespace {

inline double h_of_mdot(double mdot, double h_ref, double m_dot_ref, double n) {
    if (m_dot_ref <= 0.0 || mdot <= 0.0) return h_ref;
    return h_ref * std::pow(mdot / m_dot_ref, n);
}

} // anonymous namespace

ThermalModel::ThermalModel(const config::Config& cfg) {
    const auto& cell  = cfg.cell;
    const auto& conv  = cfg.convection;
    const auto& cool  = cfg.coolant;

    // Thermal capacitance of ONE CELL (per node = per series position, 13 parallel cells)
    // DESIGN.md §3.1.3: C_th = m_cell * cp_cell (per cell, not per position)
    // Heat exchange also uses per-cell surface area and h, but position has 13 cells.
    // We model one effective cell per node (immersion → all parallel cells identical).
    C_th_        = cell.mass_kg * cell.specific_heat_j_per_kg_k;
    A_           = cell.surface_area_m2;
    eta_ir_1c_v_ = cell.eta_ir_1c_v;
    I_1C_        = cell.capacity_ah;   // 1C current = capacity in Ah = A
    coolant_Cp_  = cool.specific_heat_j_per_kg_k;
    h_ref_       = conv.h_ref_w_per_m2_k;
    m_dot_ref_   = conv.m_dot_ref_kg_per_s;
    n_           = conv.scaling_exponent;
}

ThermalState ThermalModel::step(const ThermalState& state,
                                core::MassFlowRate mdot,
                                core::Current I_cell,
                                core::Temperature T_inlet,
                                core::Duration dt) const {
    const double dt_s = dt.value;
    if (dt_s <= 0.0) return state;

    // Ohmic heat per cell (W): Q = η_IR,1C · I² / I_1C
    const double Q = eta_ir_1c_v_ * (I_cell.value * I_cell.value / I_1C_);
    const double h = h_of_mdot(mdot.value, h_ref_, m_dot_ref_, n_);

    // -----------------------------------------------------------------------
    // Phase 1 — explicit Euler for cell temperatures (done exactly ONCE)
    //
    // C_th · dT_i/dt = Q - h·A·(T_i - Tc_i)
    // -----------------------------------------------------------------------
    std::array<double, kNumNodes> T{};
    std::array<double, kNumNodes> Tc{};

    for (std::size_t i = 0; i < kNumNodes; ++i) {
        T[i]  = state.cell_temperatures[i].value;
        Tc[i] = state.coolant_temperatures[i].value;
    }

    std::array<double, kNumNodes> T_new{};
    for (std::size_t i = 0; i < kNumNodes; ++i) {
        const double dT = (Q - h * A_ * (T[i] - Tc[i])) * (dt_s / C_th_);
        T_new[i] = T[i] + dT;
    }

    // -----------------------------------------------------------------------
    // Phase 2 — successive substitution for the algebraic coolant chain
    //
    // ṁ·Cp·(Tc[i] - Tc[i-1]) = h·A·(T_new[i] - Tc[i])
    //
    // Solved explicitly at each position:
    //   Tc[i] = (Tc[i-1] + h·A·T_new[i] / (ṁ·Cp)) / (1 + h·A / (ṁ·Cp))
    //
    // Three passes to converge the implicit coupling (residual < 0.02 °C per
    // DESIGN.md §3.1.3; enforced in test_thermal_model).
    // -----------------------------------------------------------------------
    const double mdot_Cp = mdot.value * coolant_Cp_;
    const double hA      = h * A_;

    for (int it = 0; it < kCoolantIterations; ++it) {
        Tc[0] = T_inlet.value;
        for (std::size_t i = 1; i < kNumNodes; ++i) {
            if (mdot_Cp > 0.0) {
                // Solve algebraic equation for Tc[i]
                Tc[i] = (Tc[i - 1] + hA * T_new[i] / mdot_Cp)
                        / (1.0 + hA / mdot_Cp);
            } else {
                Tc[i] = Tc[i - 1];   // no flow → no transport
            }
        }
    }

    ThermalState next;
    for (std::size_t i = 0; i < kNumNodes; ++i) {
        next.cell_temperatures[i]    = core::Temperature{T_new[i]};
        next.coolant_temperatures[i] = core::Temperature{Tc[i]};
    }
    return next;
}

std::array<core::Temperature, kNumNodes>
ThermalModel::coolant_temperatures(const ThermalState& state,
                                   core::MassFlowRate mdot,
                                   core::Temperature T_inlet) const {
    const double h     = h_of_mdot(mdot.value, h_ref_, m_dot_ref_, n_);
    const double mdot_Cp = mdot.value * coolant_Cp_;
    const double hA    = h * A_;

    std::array<double, kNumNodes> Tc{};
    Tc[0] = T_inlet.value;

    for (int it = 0; it < kCoolantIterations; ++it) {
        Tc[0] = T_inlet.value;
        for (std::size_t i = 1; i < kNumNodes; ++i) {
            if (mdot_Cp > 0.0) {
                Tc[i] = (Tc[i - 1] + hA * state.cell_temperatures[i].value / mdot_Cp)
                        / (1.0 + hA / mdot_Cp);
            } else {
                Tc[i] = Tc[i - 1];
            }
        }
    }

    std::array<core::Temperature, kNumNodes> out{};
    for (std::size_t i = 0; i < kNumNodes; ++i)
        out[i] = core::Temperature{Tc[i]};
    return out;
}

} // namespace btm::model
