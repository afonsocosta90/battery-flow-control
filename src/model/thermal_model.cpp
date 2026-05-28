#include "btm/model/thermal_model.hpp"
#include "btm/config/config.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace btm::model {

ThermalModel::ThermalModel(const config::Config& cfg) {
    const auto& cell  = cfg.cell;
    const auto& conv  = cfg.convection;
    const auto& cool  = cfg.coolant;

    // Thermal capacitance of ONE CELL (per node = per series position, 13 parallel cells)
    // DESIGN.md §3.1.3: C_th = m_cell * cp_cell (per cell, not per position)
    C_th_        = cell.mass_kg * cell.specific_heat_j_per_kg_k;
    A_           = cell.surface_area_m2;
    eta_ir_1c_v_ = cell.eta_ir_1c_v;
    I_1C_        = cell.capacity_ah;   // 1C current = capacity in Ah = A
    coolant_Cp_  = cool.specific_heat_j_per_kg_k;

    // Convection mode
    use_nusselt_ = (conv.model == "nusselt_correlation");

    // Power-law parameters (always stored — used as fallback if nusselt disabled)
    h_ref_     = conv.h_ref_w_per_m2_k;
    m_dot_ref_ = conv.m_dot_ref_kg_per_s;
    n_         = conv.scaling_exponent;

    // Nusselt parameters (only meaningful when use_nusselt_ = true)
    coolant_mu_  = cool.dynamic_viscosity_pa_s;
    coolant_k_   = cool.thermal_conductivity_w_per_m_k;
    d_hydraulic_ = conv.d_hydraulic_m;
    flow_area_   = conv.flow_area_m2;
    nusselt_c_   = conv.nusselt_c;
    nusselt_m_   = conv.nusselt_m;
    nusselt_n_   = conv.nusselt_n;

    // Two-node cell model (DESIGN.md §3.1.3)
    use_two_node_ = (cell.model == "two_node");
    if (use_two_node_) {
        R_core_can_ = cell.r_core_can_k_per_w;
        C_can_      = cell.c_can_fraction * C_th_;
        C_core_     = (1.0 - cell.c_can_fraction) * C_th_;
    }
}

double ThermalModel::h_of_mdot(double mdot) const {
    if (use_nusselt_) {
        // Nusselt correlation: Nu = c * Re^m * Pr^n,  h = Nu * k / D_h
        //   Re = mdot * D_h / (A_flow * mu)
        //   Pr = mu * cp / k
        if (flow_area_ <= 0.0 || coolant_mu_ <= 0.0 || d_hydraulic_ <= 0.0)
            return h_ref_;   // graceful fallback
        const double Re = std::max(1.0, mdot * d_hydraulic_ / (flow_area_ * coolant_mu_));
        const double Pr = coolant_mu_ * coolant_Cp_ / coolant_k_;
        const double Nu = nusselt_c_ * std::pow(Re, nusselt_m_) * std::pow(Pr, nusselt_n_);
        return Nu * coolant_k_ / d_hydraulic_;
    }
    // Power-law (default, backward-compatible)
    if (m_dot_ref_ <= 0.0 || mdot <= 0.0) return h_ref_;
    return h_ref_ * std::pow(mdot / m_dot_ref_, n_);
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
    const double h = h_of_mdot(mdot.value);

    const double mdot_Cp = mdot.value * coolant_Cp_;
    const double hA      = h * A_;

    // Coolant temperatures (previous step — used as starting guess for successive substitution)
    std::array<double, kNumNodes> Tc{};
    for (std::size_t i = 0; i < kNumNodes; ++i)
        Tc[i] = state.coolant_temperatures[i].value;

    ThermalState next;

    if (use_two_node_) {
        // -----------------------------------------------------------------
        // Two-Node Model (DESIGN.md §3.1.3)
        //
        // For each serial position i:
        //   C_core · dT_core_i/dt = Q - (T_core_i - T_can_i) / R_core_can
        //   C_can  · dT_can_i/dt  = (T_core_i - T_can_i) / R_core_can
        //                           - h·A·(T_can_i - Tc_i)
        //
        // T_can (= cell_temperatures) is the surface / sensor temperature.
        // T_core (= core_temperatures) is the safety-critical internal temp.
        // -----------------------------------------------------------------
        std::array<double, kNumNodes> T_core_new{};
        std::array<double, kNumNodes> T_can_new{};

        for (std::size_t i = 0; i < kNumNodes; ++i) {
            const double T_core_i = state.core_temperatures[i].value;
            const double T_can_i  = state.cell_temperatures[i].value;

            const double flux = (T_core_i - T_can_i) / R_core_can_;
            T_core_new[i] = T_core_i + (Q - flux) * (dt_s / C_core_);
            T_can_new[i]  = T_can_i  + (flux - hA * (T_can_i - Tc[i])) * (dt_s / C_can_);
        }

        // Coolant chain: successive substitution using T_can_new
        // (coolant contacts the outer can surface)
        for (int it = 0; it < kCoolantIterations; ++it) {
            Tc[0] = T_inlet.value;
            for (std::size_t i = 1; i < kNumNodes; ++i) {
                if (mdot_Cp > 0.0) {
                    Tc[i] = (Tc[i - 1] + hA * T_can_new[i] / mdot_Cp)
                            / (1.0 + hA / mdot_Cp);
                } else {
                    Tc[i] = Tc[i - 1];
                }
            }
        }

        for (std::size_t i = 0; i < kNumNodes; ++i) {
            next.cell_temperatures[i]    = core::Temperature{T_can_new[i]};
            next.core_temperatures[i]    = core::Temperature{T_core_new[i]};
            next.coolant_temperatures[i] = core::Temperature{Tc[i]};
        }
    } else {
        // -----------------------------------------------------------------
        // Single-Node Model (default, backward-compatible)
        //
        // Phase 1 — explicit Euler for cell temperatures:
        //   C_th · dT_i/dt = Q - h·A·(T_i - Tc_i)
        // -----------------------------------------------------------------
        std::array<double, kNumNodes> T{};
        for (std::size_t i = 0; i < kNumNodes; ++i)
            T[i] = state.cell_temperatures[i].value;

        std::array<double, kNumNodes> T_new{};
        for (std::size_t i = 0; i < kNumNodes; ++i) {
            const double dT = (Q - hA * (T[i] - Tc[i])) * (dt_s / C_th_);
            T_new[i] = T[i] + dT;
        }

        // Phase 2 — successive substitution for the algebraic coolant chain:
        //   ṁ·Cp·(Tc[i] - Tc[i-1]) = h·A·(T_new[i] - Tc[i])
        //   Tc[i] = (Tc[i-1] + h·A·T_new[i]/(ṁ·Cp)) / (1 + h·A/(ṁ·Cp))
        for (int it = 0; it < kCoolantIterations; ++it) {
            Tc[0] = T_inlet.value;
            for (std::size_t i = 1; i < kNumNodes; ++i) {
                if (mdot_Cp > 0.0) {
                    Tc[i] = (Tc[i - 1] + hA * T_new[i] / mdot_Cp)
                            / (1.0 + hA / mdot_Cp);
                } else {
                    Tc[i] = Tc[i - 1];
                }
            }
        }

        for (std::size_t i = 0; i < kNumNodes; ++i) {
            next.cell_temperatures[i]    = core::Temperature{T_new[i]};
            next.core_temperatures[i]    = core::Temperature{T_new[i]};  // core == can in single-node
            next.coolant_temperatures[i] = core::Temperature{Tc[i]};
        }
    }

    return next;
}

std::array<core::Temperature, kNumNodes>
ThermalModel::coolant_temperatures(const ThermalState& state,
                                   core::MassFlowRate mdot,
                                   core::Temperature T_inlet) const {
    const double h     = h_of_mdot(mdot.value);
    const double mdot_Cp = mdot.value * coolant_Cp_;
    const double hA    = h * A_;

    std::array<double, kNumNodes> Tc{};
    Tc[0] = T_inlet.value;

    for (int it = 0; it < kCoolantIterations; ++it) {
        Tc[0] = T_inlet.value;
        // Always uses cell_temperatures (= can/surface) — correct for both models
        // since the coolant channel contacts the can shell.
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
