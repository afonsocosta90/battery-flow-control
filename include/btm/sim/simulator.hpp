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

    // Temperature metrics
    double peak_T_max_c{0.0};      ///< Peak true max cell temperature over the run (°C)
    double peak_dT_c{0.0};         ///< Peak true inter-cell ΔT over the run (°C)
    double time_avg_T_max_c{0.0};  ///< Time-averaged true max cell temperature (°C)

    // Constraint violation
    /// Number of timesteps in which T_max > T_max_constraint OR ΔT > dT_max_constraint.
    int    violation_count{0};
    /// Cumulative time spent above any constraint [s] = violation_count * dt.
    double violation_time_s{0.0};
    /// Integral of the excess over the temperature constraint [°C·s].
    double violation_T_integral_c_s{0.0};

    // Pump energy
    double pump_integral{0.0};      ///< ∫ṁ² dt [(kg/s)²·s]  — control effort proxy
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
              double T_max_constraint,    ///< from thermal_constraints.max_cell_temperature_c
              double dT_max_constraint,   ///< from thermal_constraints.max_temperature_delta_c
              SensorModel sensor = SensorModel{});  ///< logging observability model (default: perfect)

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

    double T_max_constraint_;
    double dT_max_constraint_;
    SensorModel sensor_;
};

} // namespace btm::sim

// Include implementation (header-only template for simplicity in this portfolio project)
#include "btm/sim/simulator.ipp"
