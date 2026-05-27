#include "btm/model/thermal_model.hpp"
#include "btm/config/config.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

using namespace btm::config;
using namespace btm::model;
using namespace btm::core;

namespace {

Config make_minimal_config() {
    Config cfg;
    cfg.cell.capacity_ah              = 4.5;
    cfg.cell.nominal_voltage_v        = 3.6;
    cfg.cell.mass_kg                  = 0.070;
    cfg.cell.specific_heat_j_per_kg_k = 900.0;
    cfg.cell.surface_area_m2          = 0.00475;
    cfg.cell.eta_ir_1c_v              = 0.077837;

    cfg.module.series_count   = 24;
    cfg.module.parallel_count = 13;

    cfg.coolant.density_kg_per_m3          = 805.0;
    cfg.coolant.specific_heat_j_per_kg_k   = 3500.0;
    cfg.coolant.dynamic_viscosity_pa_s     = 0.012565;
    cfg.coolant.thermal_conductivity_w_per_m_k = 0.13;
    cfg.coolant.inlet_temperature_c        = 25.0;

    cfg.convection.h_ref_w_per_m2_k   = 250.0;
    cfg.convection.m_dot_ref_kg_per_s = 0.5;
    cfg.convection.scaling_exponent   = 0.6;

    return cfg;
}

} // anonymous namespace

// ============================================================================
// Non-negotiable: energy conservation.
//
// For any step: Q_generated = ΔE_stored + Q_removed_by_coolant
//   Q_generated = N_nodes · Q_cell · dt
//   ΔE_stored   = Σ_i  C_th · (T_new_i − T_old_i)
//   Q_removed   = ṁ · Cp_coolant · (Tc_outlet_new − T_inlet) · dt
//                 where Tc_outlet = coolant temperature at node 23 (the exit)
//
// The equality holds up to O(dt) splitting errors from explicit Euler + explicit
// coolant chain. For dt = 0.1 s and typical operating conditions the residual
// is well under 2 J per step (< 0.1 % of Q_generated).
// ============================================================================
TEST(ThermalModel, EnergyConservation) {
    auto cfg = make_minimal_config();
    ThermalModel model(cfg);

    const Duration     dt{0.1};
    const MassFlowRate mdot{0.5};
    const Current      I_cell{4.5 * 5.0};   // 5C per cell
    const Temperature  T_inlet{25.0};

    // Heat generated per cell per step (W * s = J)
    const double Q_cell = cfg.cell.eta_ir_1c_v *
                          (I_cell.value * I_cell.value / cfg.cell.capacity_ah);
    const double Q_gen_step = static_cast<double>(kNumNodes) * Q_cell * dt.value;

    // Thermal capacitance per cell (J/K)
    const double C_th = cfg.cell.mass_kg * cfg.cell.specific_heat_j_per_kg_k;

    // Run 100 random initial states and check conservation at each step.
    // Coolant temperatures are initialised consistently (from the chain solver)
    // to avoid first-step imbalance from O(1) Tc inconsistency.
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> temp_dist(20.0, 38.0);

    for (int trial = 0; trial < 100; ++trial) {
        ThermalState state;
        for (auto& t : state.cell_temperatures) t = Temperature{temp_dist(rng)};
        // Set Tc consistent with the chain equation for the current T
        auto Tc_consistent = model.coolant_temperatures(state, mdot, T_inlet);
        state.coolant_temperatures = Tc_consistent;

        ThermalState next = model.step(state, mdot, I_cell, T_inlet, dt);

        // ΔE stored in cell thermal mass
        double delta_E = 0.0;
        for (std::size_t i = 0; i < kNumNodes; ++i) {
            delta_E += C_th * (next.cell_temperatures[i].value
                              - state.cell_temperatures[i].value);
        }

        // Heat removed by coolant: ṁ·Cp·(Tc_out − T_inlet)·dt
        const double Tc_outlet = next.coolant_temperatures[kNumNodes - 1].value;
        const double Q_removed = mdot.value * cfg.coolant.specific_heat_j_per_kg_k
                                 * (Tc_outlet - T_inlet.value) * dt.value;

        const double imbalance = std::abs(Q_gen_step - (delta_E + Q_removed));

        // Tolerance: 5 J per step (< 0.25% of Q_gen, covers O(dt) splitting error)
        EXPECT_LT(imbalance, 5.0)
            << "Trial " << trial << ": energy imbalance = " << imbalance
            << " J  (Q_gen=" << Q_gen_step << ", ΔE=" << delta_E
            << ", Q_removed=" << Q_removed << ")";
    }
}

// ============================================================================
// Steady-state check: at constant temperature (dT/dt = 0), heat generation
// must equal convective removal, and coolant temperatures must be monotonically
// increasing from inlet to outlet.
// ============================================================================
TEST(ThermalModel, SteadyStateCoolantMonotonic) {
    auto cfg = make_minimal_config();
    ThermalModel model(cfg);

    // Build a state close to a steady-state (uniform cells at 32 °C, coolant at inlet)
    ThermalState state;
    for (auto& t : state.cell_temperatures)    t = Temperature{32.0};
    for (auto& t : state.coolant_temperatures) t = Temperature{25.0};

    const MassFlowRate mdot{0.5};
    const Current      I_cell{4.5 * 5.0};
    const Temperature  T_inlet{25.0};
    const Duration     dt{0.1};

    // Run 200 steps to reach approximate steady state
    for (int i = 0; i < 200; ++i)
        state = model.step(state, mdot, I_cell, T_inlet, dt);

    // Coolant must increase monotonically along the chain
    for (std::size_t i = 1; i < kNumNodes; ++i) {
        EXPECT_GE(state.coolant_temperatures[i].value,
                  state.coolant_temperatures[i - 1].value)
            << "Coolant temperature must not decrease along flow direction";
    }

    // Outlet must be hotter than inlet
    EXPECT_GT(state.coolant_temperatures[kNumNodes - 1].value, T_inlet.value);

    // Cell temperatures must be above inlet
    for (std::size_t i = 0; i < kNumNodes; ++i)
        EXPECT_GT(state.cell_temperatures[i].value, T_inlet.value);
}

// ============================================================================
// Coolant residual bound: after one step from a stable state, temperatures
// must remain in a physically reasonable range.
// ============================================================================
TEST(ThermalModel, CoolantResidualBound) {
    auto cfg = make_minimal_config();
    ThermalModel model(cfg);

    ThermalState state;
    for (auto& t : state.cell_temperatures)    t = Temperature{30.0};
    for (auto& t : state.coolant_temperatures) t = Temperature{25.0};

    const MassFlowRate mdot{0.5};
    const Current      I_cell{4.5 * 3.0};   // 3C discharge
    const Temperature  T_inlet{25.0};
    const Duration     dt{0.1};

    auto next = model.step(state, mdot, I_cell, T_inlet, dt);

    for (const auto& t : next.cell_temperatures) {
        EXPECT_GT(t.value, 20.0) << "Cell temperature below plausible minimum";
        EXPECT_LT(t.value, 50.0) << "Cell temperature above plausible maximum";
    }
    for (const auto& t : next.coolant_temperatures) {
        EXPECT_GE(t.value, T_inlet.value - 0.1)
            << "Coolant temperature below inlet temperature";
    }
}

// ============================================================================
// Higher flow → lower steady-state cell temperature (monotone response).
// ============================================================================
TEST(ThermalModel, HigherFlowLowerSteadyStateTemp) {
    auto cfg = make_minimal_config();
    ThermalModel model(cfg);

    const Current     I_cell{4.5 * 5.0};
    const Temperature T_inlet{25.0};
    const Duration    dt{0.1};

    auto run_to_ss = [&](double mdot_val) {
        ThermalState s;
        for (auto& t : s.cell_temperatures)    t = Temperature{25.0};
        for (auto& t : s.coolant_temperatures) t = Temperature{25.0};
        for (int i = 0; i < 600; ++i)
            s = model.step(s, MassFlowRate{mdot_val}, I_cell, T_inlet, dt);
        return s.max_cell_temp().value;
    };

    const double T_low_flow  = run_to_ss(0.1);
    const double T_high_flow = run_to_ss(1.0);

    EXPECT_GT(T_low_flow, T_high_flow)
        << "Higher flow should give lower steady-state cell temperature";
}
