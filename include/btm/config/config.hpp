#pragma once

#include "btm/core/types.hpp"

#include <string>
#include <vector>

namespace btm::config {

// -----------------------------------------------------------------------------
// Parameter groups (mirror the YAML structure)
// -----------------------------------------------------------------------------

struct CellParams {
    double capacity_ah{0.0};
    double nominal_voltage_v{0.0};
    double mass_kg{0.0};
    double specific_heat_j_per_kg_k{0.0};
    double surface_area_m2{0.0};
    double eta_ir_1c_v{0.0};           ///< heat generation coefficient

    // ── Two-node cell model (DESIGN.md §3.1.3) ─────────────────────────────
    /// "single_node" (default) — one lumped capacitance per position.
    /// "two_node"              — separate core and can capacitances connected
    ///                           by a thermal resistance R_core_can.
    std::string model{"single_node"};

    /// Thermal resistance between core and can surface (K/W).
    /// Calibrated to ~0.8 K/W for INR-21700 format:
    ///   ΔT_core_can ≈ Q × R = 8.76 W × 0.8 K/W ≈ 7 °C at 5C discharge.
    double r_core_can_k_per_w{0.8};

    /// Fraction of total cell thermal mass (m×cp) allocated to the can shell.
    /// Remainder (1 − c_can_fraction) goes to the core.
    /// Default 0.10 (10 %) — consistent with thin-wall Al cylinder geometry.
    double c_can_fraction{0.10};
};

struct ModuleGeometry {
    int series_count{0};
    int parallel_count{0};
};

struct CoolantParams {
    double density_kg_per_m3{0.0};
    double specific_heat_j_per_kg_k{0.0};
    double dynamic_viscosity_pa_s{0.0};
    double thermal_conductivity_w_per_m_k{0.0};
    double inlet_temperature_c{0.0};
};

struct ConvectionParams {
    // ── Power-law mode (default, backward-compatible) ────────────────────────
    // h(ṁ) = h_ref * (ṁ / ṁ_ref)^scaling_exponent
    double h_ref_w_per_m2_k{0.0};
    double m_dot_ref_kg_per_s{0.0};
    double scaling_exponent{0.0};

    // ── Nusselt-correlation mode (optional, YAML: convection.model: nusselt_correlation) ─
    // Nu = nusselt_c * Re^nusselt_m * Pr^nusselt_n  →  h = Nu * k_coolant / D_h
    // Re = ṁ * D_h / (flow_area * μ_coolant)
    // Pr = μ_coolant * cp_coolant / k_coolant          (all from coolant: section)
    std::string model{"power_law"};     ///< "power_law" | "nusselt_correlation"
    double d_hydraulic_m{0.0};          ///< hydraulic diameter of coolant channel (m)
    double flow_area_m2{0.0};           ///< effective total cross-sectional flow area (m²)
    double nusselt_c{0.197};            ///< Nu correlation coefficient (default: calibrated laminar)
    double nusselt_m{0.333};            ///< Re exponent (0.333 laminar, 0.8 turbulent Dittus-Boelter)
    double nusselt_n{0.333};            ///< Pr exponent (0.333 laminar, 0.4 turbulent)
};

struct ThermalConstraints {
    // ── Backward-compatible primary constraint ────────────────────────────────
    /// Safety limit on cell temperature.  In single-node mode this is the only
    /// constraint needed.  In two-node mode it is used as the fallback default
    /// for max_core_temperature_c and max_can_temperature_c when those are absent.
    double max_cell_temperature_c{0.0};
    double max_temperature_delta_c{0.0};   ///< inter-cell ΔT limit (°C)

    // ── T6: Named two-node-aware constraints (DESIGN.md §3.2.3) ─────────────
    /// Hard safety limit on the jellyroll core temperature (°C).
    /// Defaults to max_cell_temperature_c if absent from YAML.
    /// Violations are logged to SimResult.violation_core_count.
    double max_core_temperature_c{0.0};

    /// Surface (can) temperature limit (°C).
    /// Typically equal to or slightly above max_core_temperature_c since the
    /// core is always hotter than the can under load.
    /// Defaults to max_cell_temperature_c if absent from YAML.
    double max_can_temperature_c{0.0};
};

struct PumpLimits {
    double min_flow_kg_per_s{0.0};
    double max_flow_kg_per_s{0.0};
};

struct MpcWeights {
    double tracking{0.0};
    double delta_t{0.0};
    double pump_energy{0.0};
    double input_rate{0.0};
};

struct MpcSolverParams {
    int    max_iterations{0};
    double step_size{0.0};
    double convergence_tol{0.0};
    double finite_diff_epsilon{0.0};
};

struct MpcConfig {
    int horizon_steps{0};
    double setpoint_c{35.0};              ///< temperature tracking target (default = constraint)

    // ── Soft constraint penalties ─────────────────────────────────────────────
    double soft_T_max_penalty{10000.0};   ///< quadratic penalty: T_core > max_core_temperature_c
    double soft_T_can_penalty{0.0};       ///< T6: quadratic penalty: T_can > max_can_temperature_c
                                          ///<   (0.0 = disabled by default; only active in two-node)
    double soft_dT_penalty{10000.0};      ///< quadratic penalty: ΔT > max_temperature_delta_c

    MpcWeights weights;
    MpcSolverParams solver;
};

struct PidConfig {
    double kp{0.0};
    double ki{0.0};
    double setpoint_c{0.0};
    double integrator_limit{0.0};
    double deadband_c{0.0};    ///< |error| < deadband_c is treated as zero (default: disabled)
};

/// Sensor / observability model configuration.
/// Controls which cell temperatures the controller and logger treat as "observed".
struct SensorConfig {
    std::string      mode{"perfect"};   ///< "perfect" | "downstream" | "sparse"
    std::vector<int> positions{};       ///< series-position indices for "sparse" mode
};

struct ScenarioConfig {
    std::string type;                  // "constant_c_rate", "step_transient", "rising_ambient"
    double c_rate{0.0};                // used by constant_c_rate
    double initial_c_rate{0.0};
    double final_c_rate{0.0};
    double step_time_s{0.0};
    double initial_inlet_c{0.0};
    double final_inlet_c{0.0};
    double ramp_start_s{0.0};
    double ramp_duration_s{0.0};
};

struct SimConfig {
    double duration_s{0.0};
    double timestep_s{0.0};
    std::string log_path;
};

// -----------------------------------------------------------------------------
// Top-level configuration (the single source of truth at runtime)
// -----------------------------------------------------------------------------
struct Config {
    CellParams         cell;
    ModuleGeometry     module;
    CoolantParams      coolant;
    ConvectionParams   convection;
    ThermalConstraints thermal_constraints;
    PumpLimits         pump;
    MpcConfig          mpc;
    PidConfig          pid;
    SensorConfig       sensor;          ///< observability model (default: perfect)
    ScenarioConfig     scenario;
    SimConfig          simulation;

    std::string controller_type;        ///< "pid" or "mpc"
};

// -----------------------------------------------------------------------------
// Loading & validation
// -----------------------------------------------------------------------------

/// Load YAML from the given path, populate Config, and run full validation.
/// Throws std::runtime_error with a descriptive message naming the offending field
/// on any missing, out-of-range, or inconsistent value.
Config load_and_validate(const std::string& path);

} // namespace btm::config
