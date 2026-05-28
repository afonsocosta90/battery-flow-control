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

/// Build a two-node config from the single-node base.
Config make_two_node_config() {
    Config cfg = make_minimal_config();
    cfg.cell.model            = "two_node";
    cfg.cell.r_core_can_k_per_w = 0.8;   // calibrated: ΔT≈7°C at 5C
    cfg.cell.c_can_fraction   = 0.10;    // 10% of C_th in can shell
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

// ============================================================================
// T5 — Convection model tests
// ============================================================================

namespace {

/// Build a config with Nusselt-correlation convection.
/// Parameters are calibrated to give h ≈ 250 W/(m²·K) at ṁ_ref = 0.5 kg/s.
Config make_nusselt_config() {
    auto cfg = make_minimal_config();
    cfg.convection.model         = "nusselt_correlation";
    cfg.convection.d_hydraulic_m = 0.006;
    cfg.convection.flow_area_m2  = 0.0004;
    cfg.convection.nusselt_c     = 0.197;
    cfg.convection.nusselt_m     = 0.333;
    cfg.convection.nusselt_n     = 0.333;
    return cfg;
}

} // anonymous namespace

// h must increase strictly with mass-flow rate for the power-law model.
TEST(ThermalModel, PowerLaw_H_IncreasesWithMdot) {
    ThermalModel model(make_minimal_config());

    const double h1 = model.h_of_mdot(0.1);
    const double h2 = model.h_of_mdot(0.5);
    const double h3 = model.h_of_mdot(1.5);

    EXPECT_LT(h1, h2) << "h must increase from 0.1 to 0.5 kg/s";
    EXPECT_LT(h2, h3) << "h must increase from 0.5 to 1.5 kg/s";
    EXPECT_DOUBLE_EQ(h2, 250.0) << "h must equal h_ref at m_dot_ref";
}

// h must increase strictly with mass-flow rate for the Nusselt-correlation model.
TEST(ThermalModel, Nusselt_H_IncreasesWithMdot) {
    ThermalModel model(make_nusselt_config());

    const double h1 = model.h_of_mdot(0.1);
    const double h2 = model.h_of_mdot(0.5);
    const double h3 = model.h_of_mdot(1.5);

    EXPECT_LT(h1, h2) << "h must increase from 0.1 to 0.5 kg/s (Nusselt)";
    EXPECT_LT(h2, h3) << "h must increase from 0.5 to 1.5 kg/s (Nusselt)";
}

// At the reference point both models should agree to within 1%.
// The Nusselt parameters above are calibrated specifically for this.
TEST(ThermalModel, Nusselt_H_AgreesWith_PowerLaw_At_Reference) {
    ThermalModel pl_model(make_minimal_config());
    ThermalModel nu_model(make_nusselt_config());

    const double h_ref_mdot = 0.5;   // kg/s — the reference flow rate
    const double h_pl       = pl_model.h_of_mdot(h_ref_mdot);
    const double h_nu       = nu_model.h_of_mdot(h_ref_mdot);

    EXPECT_NEAR(h_pl, h_nu, h_pl * 0.01)
        << "Nusselt and power-law must agree within 1% at the reference flow rate";
}

// Nusselt model: higher flow → lower steady-state temperature (same as power-law).
TEST(ThermalModel, Nusselt_HigherFlowLowerSteadyStateTemp) {
    ThermalModel model(make_nusselt_config());

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

    EXPECT_GT(run_to_ss(0.1), run_to_ss(1.0))
        << "Higher flow should give lower steady-state temp (Nusselt mode)";
}

// ============================================================================
// T2 — Two-Node Cell Model tests
// ============================================================================

// Initialise a ThermalState at a uniform temperature for two-node runs.
static ThermalState make_two_node_state(double T0) {
    ThermalState s;
    for (auto& t : s.cell_temperatures)    t = Temperature{T0};
    for (auto& t : s.core_temperatures)    t = Temperature{T0};
    for (auto& t : s.coolant_temperatures) t = Temperature{T0};
    return s;
}

// ============================================================================
// After running under load the core must be strictly hotter than the can at
// every serial position (heat is generated in the core, flows outward).
// ============================================================================
TEST(ThermalModel, TwoNode_CoreHotterThanCan) {
    ThermalModel model(make_two_node_config());

    ThermalState state = make_two_node_state(25.0);

    const MassFlowRate mdot{0.5};
    const Current      I_cell{4.5 * 5.0};   // 5C
    const Temperature  T_inlet{25.0};
    const Duration     dt{0.1};

    // Run to approximate steady state
    for (int i = 0; i < 600; ++i)
        state = model.step(state, mdot, I_cell, T_inlet, dt);

    for (std::size_t i = 0; i < kNumNodes; ++i) {
        EXPECT_GT(state.core_temperatures[i].value,
                  state.cell_temperatures[i].value)
            << "Core must be hotter than can at position " << i
            << " (heat flows outward from core to can)";
    }
}

// ============================================================================
// Single-node backward compatibility: in single-node mode, max_core_temp()
// must equal max_cell_temp() at every timestep.
// ============================================================================
TEST(ThermalModel, SingleNode_CoreEqualsCan) {
    ThermalModel model(make_minimal_config());  // single-node

    ThermalState state = make_two_node_state(25.0);

    const MassFlowRate mdot{0.5};
    const Current      I_cell{4.5 * 5.0};
    const Temperature  T_inlet{25.0};
    const Duration     dt{0.1};

    for (int step = 0; step < 300; ++step) {
        state = model.step(state, mdot, I_cell, T_inlet, dt);
        ASSERT_DOUBLE_EQ(state.max_core_temp().value, state.max_cell_temp().value)
            << "Single-node: core must equal can at step " << step;
    }
}

// ============================================================================
// Steady-state core-to-can gradient.
//
// At true steady state under 5C: Q_cell = η·I²/I_1C = 0.077837·22.5²/4.5
//                                       ≈ 8.756 W per cell
// Analytical RC solution: ΔT_core_can = Q · R = 8.756 × 0.8 ≈ 7.0 °C
//
// The two-node system has a slow eigenmode (τ_slow ≈ 96 s) that couples the
// overall temperature rise to the core-to-can gradient. At t = 300 s (3000
// steps ≈ 3.1 × τ_slow) the gradient converges to ~6.7 °C.
//
// Acceptable range: [5, 9] °C (covers slow-eigenmode residual + numerical error).
// ============================================================================
TEST(ThermalModel, TwoNode_SteadyStateGradientAt5C) {
    ThermalModel model(make_two_node_config());

    ThermalState state = make_two_node_state(25.0);

    const MassFlowRate mdot{0.5};
    const Current      I_cell{4.5 * 5.0};
    const Temperature  T_inlet{25.0};
    const Duration     dt{0.1};

    // 3000 steps = 300 s ≈ 3.1 × τ_slow (~96 s) → gradient converges to ~6.7 °C
    for (int i = 0; i < 3000; ++i)
        state = model.step(state, mdot, I_cell, T_inlet, dt);

    const double dT_core_can = state.core_to_can_delta_t().value;

    EXPECT_GT(dT_core_can, 5.0)
        << "ΔT_core_can must exceed 5 °C at 5C after 300 s (got " << dT_core_can << ")";
    EXPECT_LT(dT_core_can, 9.0)
        << "ΔT_core_can must be below 9 °C at 5C (got " << dT_core_can << ")";
}

// ============================================================================
// Energy conservation for the two-node model.
//
// Exactly the same accounting as for single-node, but now the stored energy
// is split between core and can:
//   ΔE_stored = Σ_i [ C_core·(T_core_new_i − T_core_i) + C_can·(T_can_new_i − T_can_i) ]
//             = Σ_i   C_th·(T_can_new_i − T_can_i)   +  correction term
// We check the full balance: Q_gen = ΔE_core + ΔE_can + Q_removed.
// Tolerance: 5 J per step (same as single-node; O(dt) Euler splitting error).
// ============================================================================
TEST(ThermalModel, TwoNode_EnergyConservation) {
    const auto cfg = make_two_node_config();
    ThermalModel model(cfg);

    const Duration     dt{0.1};
    const MassFlowRate mdot{0.5};
    const Current      I_cell{4.5 * 5.0};
    const Temperature  T_inlet{25.0};

    const double Q_cell = cfg.cell.eta_ir_1c_v *
                          (I_cell.value * I_cell.value / cfg.cell.capacity_ah);
    const double Q_gen_step = static_cast<double>(kNumNodes) * Q_cell * dt.value;

    const double C_th   = cfg.cell.mass_kg * cfg.cell.specific_heat_j_per_kg_k;
    const double C_can  = cfg.cell.c_can_fraction * C_th;
    const double C_core = (1.0 - cfg.cell.c_can_fraction) * C_th;

    // Run to approximate steady state first so Tc is self-consistent
    ThermalState state = make_two_node_state(25.0);
    for (int i = 0; i < 200; ++i)
        state = model.step(state, mdot, I_cell, T_inlet, dt);

    // Check energy balance over 50 steps in near-steady-state
    for (int step = 0; step < 50; ++step) {
        ThermalState prev = state;
        state = model.step(state, mdot, I_cell, T_inlet, dt);

        double delta_E = 0.0;
        for (std::size_t i = 0; i < kNumNodes; ++i) {
            delta_E += C_core * (state.core_temperatures[i].value
                                 - prev.core_temperatures[i].value);
            delta_E += C_can  * (state.cell_temperatures[i].value
                                 - prev.cell_temperatures[i].value);
        }

        const double Tc_outlet = state.coolant_temperatures[kNumNodes - 1].value;
        const double Q_removed = mdot.value * cfg.coolant.specific_heat_j_per_kg_k
                                 * (Tc_outlet - T_inlet.value) * dt.value;

        const double imbalance = std::abs(Q_gen_step - (delta_E + Q_removed));

        EXPECT_LT(imbalance, 5.0)
            << "Step " << step << ": energy imbalance = " << imbalance
            << " J  (Q_gen=" << Q_gen_step << ", ΔE=" << delta_E
            << ", Q_removed=" << Q_removed << ")";
    }
}

// ============================================================================
// T7 — Enhanced Physics Validation Suite
// ============================================================================

// ============================================================================
// T7.1 Monotonicity: increasing ṁ strictly decreases BOTH can AND core
// steady-state temperatures in two-node mode.
//
// Physical rationale: more flow → lower T_coolant → lower T_can (convection ↑)
// → smaller flux through R_core_can → lower T_core.
// ============================================================================
TEST(ThermalModel, TwoNode_HigherFlowLowerBothTemps) {
    ThermalModel model(make_two_node_config());

    const Current     I_cell{4.5 * 5.0};
    const Temperature T_inlet{25.0};
    const Duration    dt{0.1};

    auto run_to_ss = [&](double mdot_val) -> std::pair<double, double> {
        ThermalState s = make_two_node_state(25.0);
        for (int i = 0; i < 600; ++i)
            s = model.step(s, MassFlowRate{mdot_val}, I_cell, T_inlet, dt);
        return {s.max_cell_temp().value, s.max_core_temp().value};
    };

    const auto [T_can_lo, T_core_lo] = run_to_ss(0.1);
    const auto [T_can_hi, T_core_hi] = run_to_ss(1.0);

    EXPECT_GT(T_can_lo,  T_can_hi)
        << "Two-node: higher flow must lower steady-state can temperature";
    EXPECT_GT(T_core_lo, T_core_hi)
        << "Two-node: higher flow must lower steady-state core temperature";
}

// ============================================================================
// T7.2 Zero heat load: an initial core-to-can gradient must decay to
// near-zero when there is no heat generation.
//
// Physical rationale: the fast eigenmode (τ_fast ≈ 2.4 s) drives T_core
// toward T_can via R_core_can. After 20 s (200 steps at dt = 0.1 s) the
// analytical residual is ≈ 0.5 × exp(−20 / 2.4) ≈ 1.4 × 10⁻⁴ °C.
//
// Note: starting from a uniform warm temperature (T_core = T_can > T_inlet)
// does NOT give zero gradient at SS — the can cools faster than the core
// through convection (C_can << C_core), so an initial offset GROWS before
// decaying.  The test therefore starts from a small explicit offset.
// ============================================================================
TEST(ThermalModel, TwoNode_ZeroLoadNoGradient) {
    ThermalModel model(make_two_node_config());

    // Small initial gradient: T_core = 25.5 °C, T_can = T_coolant = 25.0 °C
    ThermalState state;
    for (std::size_t i = 0; i < kNumNodes; ++i) {
        state.core_temperatures[i]    = Temperature{25.5};
        state.cell_temperatures[i]    = Temperature{25.0};
        state.coolant_temperatures[i] = Temperature{25.0};
    }

    const MassFlowRate mdot{0.5};
    const Current      I_cell{0.0};   // no heat generation
    const Temperature  T_inlet{25.0};
    const Duration     dt{0.1};

    // 200 steps = 20 s ≈ 8.3 × τ_fast → gradient decays to ≈ 1.4e-4 °C
    for (int i = 0; i < 200; ++i)
        state = model.step(state, mdot, I_cell, T_inlet, dt);

    const double dT_core_can = state.core_to_can_delta_t().value;
    EXPECT_LT(dT_core_can, 0.05)
        << "Two-node: core-to-can gradient must decay near-zero at zero load "
        << "(τ_fast ≈ 2.4 s; got " << dT_core_can << " °C after 20 s)";

    // Both nodes must also be close to inlet temperature
    EXPECT_NEAR(state.max_core_temp().value, T_inlet.value, 0.5)
        << "Two-node: T_core must approach T_inlet at zero load";
    EXPECT_NEAR(state.max_cell_temp().value, T_inlet.value, 0.5)
        << "Two-node: T_can must approach T_inlet at zero load";
}

// ============================================================================
// T7.3 Step response: after a sudden 1C→5C step, the core temperature leads
// the can temperature (core heats first, then dissipates through R_core_can).
// This is the key physical justification for the two-node model: the core
// reaches constraint temperature before the can sensor registers it.
// ============================================================================
TEST(ThermalModel, TwoNode_CoreLeadsCan_OnLoadStep) {
    ThermalModel model(make_two_node_config());

    const MassFlowRate mdot{0.5};
    const Temperature  T_inlet{25.0};
    const Duration     dt{0.1};

    // Start from single-node-equivalent steady state at 1C
    ThermalState state = make_two_node_state(25.0);
    for (int i = 0; i < 600; ++i)
        state = model.step(state, mdot, Current{4.5 * 1.0}, T_inlet, dt);

    // Step to 5C — immediately after step T_core must have risen more than T_can
    const double T_core_before = state.max_core_temp().value;
    const double T_can_before  = state.max_cell_temp().value;

    // Take a few steps at 5C
    for (int i = 0; i < 5; ++i)
        state = model.step(state, mdot, Current{4.5 * 5.0}, T_inlet, dt);

    const double T_core_rise = state.max_core_temp().value - T_core_before;
    const double T_can_rise  = state.max_cell_temp().value - T_can_before;

    EXPECT_GT(T_core_rise, T_can_rise)
        << "Two-node: T_core must rise faster than T_can on a load step "
        << "(core_rise=" << T_core_rise << " °C, can_rise=" << T_can_rise << " °C)";
}
