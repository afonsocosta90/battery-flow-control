#pragma once

#include "btm/core/types.hpp"
#include "btm/model/thermal_state.hpp"
#include "btm/model/thermal_model.hpp"
#include "btm/sim/sensor_model.hpp"

#include <functional>
#include <string>

namespace btm::sim {

// ---------------------------------------------------------------------------
// SimResult — returned by Simulator::run().
//
// All metrics are computed from the true physical state (not the sensor view).
// A JSON summary is also written alongside the CSV at <log_path_base>_summary.json.
// ---------------------------------------------------------------------------
struct SimResult {
    int    steps{0};

    // ── Temperature metrics ──────────────────────────────────────────────────
    double peak_T_max_c{0.0};      ///< Peak core temperature over the run (°C)
    double peak_dT_c{0.0};         ///< Peak inter-cell ΔT (can temps) over the run (°C)
    double time_avg_T_max_c{0.0};  ///< Time-averaged core temperature (°C)

    // ── T6: Named two-node constraint violations ─────────────────────────────
    /// Timesteps where T_core > max_core_temperature_c or ΔT > max_temperature_delta_c.
    int    violation_count{0};
    double violation_time_s{0.0};
    double violation_T_integral_c_s{0.0};  ///< ∫(T_core − T_core_limit)⁺ dt

    /// Timesteps where T_core > max_core_temperature_c (core-only violation tracking).
    int    violation_core_count{0};
    double violation_core_time_s{0.0};

    /// Timesteps where T_can > max_can_temperature_c (surface temperature guard).
    int    violation_can_count{0};
    double violation_can_time_s{0.0};

    // ── Pump energy ──────────────────────────────────────────────────────────
    double pump_integral{0.0};     ///< ∫ṁ² dt [(kg/s)²·s]  — control effort proxy
};

// ---------------------------------------------------------------------------

template <typename Controller>
class Simulator {
public:
    Simulator(model::ThermalModel& model,
              Controller& controller,
              core::Duration dt,
              core::Duration duration,
              const std::string& log_path,
              std::function<core::Current(core::Duration)> current_fn,
              std::function<core::Temperature(core::Duration)> inlet_fn,
              double T_max_constraint,         ///< core temperature limit (from thermal_constraints)
              double dT_max_constraint,        ///< inter-cell ΔT limit
              SensorModel sensor = SensorModel{},   ///< logging observability (default: perfect)
              double T_can_constraint = 0.0);  ///< T6: can temperature limit (0.0 = use T_max_constraint)

    /// Run the simulation.  Returns comprehensive metrics and also writes a JSON
    /// summary to <log_path>_summary.json for machine-readable downstream use.
    SimResult run();

private:
    model::ThermalModel& model_;
    Controller& controller_;
    core::Duration dt_;
    core::Duration duration_;
    std::string log_path_;

    std::function<core::Current(core::Duration)> current_fn_;
    std::function<core::Temperature(core::Duration)> inlet_fn_;

    double T_max_constraint_;    ///< core temperature limit
    double dT_max_constraint_;
    double T_can_constraint_;    ///< T6: can/surface temperature limit
    SensorModel sensor_;
};

} // namespace btm::sim

// Include implementation (header-only template for simplicity in this portfolio project)
#include "btm/sim/simulator.ipp"
