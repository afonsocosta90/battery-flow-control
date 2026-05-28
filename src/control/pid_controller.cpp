#include "btm/control/pid_controller.hpp"

#include <algorithm>
#include <cmath>

namespace btm::control {

PidController::PidController(double kp, double ki, double setpoint_c,
                             double integrator_limit,
                             core::MassFlowRate mdot_min,
                             core::MassFlowRate mdot_max,
                             double deadband_c,
                             sim::SensorModel sensor)
    : kp_(kp),
      ki_(ki),
      setpoint_c_(setpoint_c),
      integrator_limit_(integrator_limit),
      mdot_min_(mdot_min),
      mdot_max_(mdot_max),
      deadband_c_(deadband_c),
      sensor_(std::move(sensor)) {}

core::MassFlowRate PidController::compute_command(const model::ThermalState& state,
                                                   [[maybe_unused]] core::Current    I_cell,
                                                   [[maybe_unused]] core::Temperature T_inlet,
                                                   core::Duration dt) {
    const double dt_s = dt.value;
    if (dt_s <= 0.0) return mdot_min_;

    // Observed temperature (SensorModel decides which cells to use)
    const double T_obs = sensor_.observed_max(state).value;
    const double raw_error = T_obs - setpoint_c_;   // positive → too hot → need more flow

    // Deadband: treat small errors as zero to avoid unnecessary actuation.
    const double effective_error =
        (std::abs(raw_error) < deadband_c_) ? 0.0 : raw_error;

    // PI: tentative output from current integrator state.
    const double u_raw = kp_ * effective_error + ki_ * integrator_;

    // Saturate to physical pump range.
    const double u_sat = std::clamp(u_raw, mdot_min_.value, mdot_max_.value);

    const bool saturated = (u_raw < mdot_min_.value) || (u_raw > mdot_max_.value);

    // Back-calculation anti-windup:
    //   If saturated AND ki > 0, force the integrator to the value that would
    //   have produced u_sat exactly.  This is stronger than a clamp and
    //   eliminates windup instantly when the saturating condition ends.
    //   If ki == 0 the integrator has no effect, so skip the division.
    if (saturated && ki_ > 0.0) {
        integrator_ = (u_sat - kp_ * effective_error) / ki_;
    } else {
        integrator_ += effective_error * dt_s;
    }

    // Safety hard-clamp: guard against extreme scenarios where back-calc alone
    // is insufficient (e.g. very large kp overshoot).
    integrator_ = std::clamp(integrator_, -integrator_limit_, integrator_limit_);

    return core::MassFlowRate{u_sat};
}

void PidController::reset() {
    integrator_  = 0.0;
    prev_error_  = 0.0;
    first_step_  = true;
}

} // namespace btm::control
