#pragma once

#include "btm/core/types.hpp"
#include "btm/model/thermal_state.hpp"
#include "btm/sim/sensor_model.hpp"

namespace btm::control {

/// PID controller on maximum cell temperature.
///
/// Inputs I_cell and T_inlet are accepted in compute_command to satisfy the
/// Controller concept but are not used in PID logic.  Output is clamped to
/// [mdot_min, mdot_max].
///
/// T4 enhancements:
///   - Back-calculation anti-windup: when the output saturates, the integrator
///     is back-calculated to the value that would have produced the saturated
///     output exactly.  This prevents windup during extended saturation and
///     eliminates the overshoot seen when the load drops.
///   - Deadband: |error| < deadband_c is treated as zero (no output / no
///     integrator accumulation).  Disabled by default (deadband_c = 0.0).
///   - Configurable sensor: the "observed" max temperature that drives the
///     controller is taken from a SensorModel (default: perfect — uses true
///     global maximum, backward-compatible with all prior behaviour).
class PidController {
public:
    /// \param kp               Proportional gain.
    /// \param ki               Integral gain.
    /// \param setpoint_c       Target cell temperature (°C).
    /// \param integrator_limit Hard saturation limit on the integrator state.
    /// \param mdot_min         Minimum allowable pump flow.
    /// \param mdot_max         Maximum allowable pump flow.
    /// \param deadband_c       Errors below this magnitude are treated as zero
    ///                         (default 0.0 = disabled; matches all prior tests).
    /// \param sensor           Observability model — selects which cell temperature
    ///                         the controller reacts to (default: Perfect = true global max).
    PidController(double kp,
                  double ki,
                  double setpoint_c,
                  double integrator_limit,
                  core::MassFlowRate mdot_min,
                  core::MassFlowRate mdot_max,
                  double deadband_c         = 0.0,
                  sim::SensorModel   sensor = sim::SensorModel{});

    core::MassFlowRate compute_command(const model::ThermalState& state,
                                       core::Current            I_cell,   // unused (satisfies concept)
                                       core::Temperature        T_inlet,  // unused (satisfies concept)
                                       core::Duration           dt);

    void reset();

private:
    double kp_;
    double ki_;
    double setpoint_c_;
    double integrator_limit_;
    core::MassFlowRate mdot_min_;
    core::MassFlowRate mdot_max_;
    double deadband_c_;
    sim::SensorModel sensor_;

    double integrator_{0.0};
    double prev_error_{0.0};
    bool   first_step_{true};
};

} // namespace btm::control
