#include "btm/control/pid_controller.hpp"

#include <algorithm>

namespace btm::control {

PidController::PidController(double kp, double ki, double setpoint_c, double integrator_limit,
                             core::MassFlowRate mdot_min, core::MassFlowRate mdot_max)
    : kp_(kp),
      ki_(ki),
      setpoint_c_(setpoint_c),
      integrator_limit_(integrator_limit),
      mdot_min_(mdot_min),
      mdot_max_(mdot_max) {}

core::MassFlowRate PidController::compute_command(const model::ThermalState& state,
                                                   [[maybe_unused]] core::Current    I_cell,
                                                   [[maybe_unused]] core::Temperature T_inlet,
                                                   core::Duration dt) {
    const double dt_s = dt.value;
    if (dt_s <= 0.0) return mdot_min_;

    const double T_max = state.max_cell_temp().value;
    const double error = T_max - setpoint_c_;   // positive → too hot → need more flow

    // Anti-windup PI
    integrator_ += error * dt_s;
    integrator_ = std::clamp(integrator_, -integrator_limit_, integrator_limit_);

    double u = kp_ * error + ki_ * integrator_;

    // Clamp to physical pump range
    u = std::clamp(u, mdot_min_.value, mdot_max_.value);

    return core::MassFlowRate{u};
}

void PidController::reset() {
    integrator_  = 0.0;
    prev_error_  = 0.0;
    first_step_  = true;
}

} // namespace btm::control
