#pragma once

#include "btm/core/types.hpp"
#include "btm/model/thermal_state.hpp"

namespace btm::control {

/// PID controller on maximum cell temperature.
///
/// I_cell and T_inlet are accepted in compute_command to satisfy the Controller concept
/// but are not used in PID logic. Output is clamped to [mdot_min, mdot_max].
class PidController {
public:
    PidController(double kp,
                  double ki,
                  double setpoint_c,
                  double integrator_limit,
                  core::MassFlowRate mdot_min,
                  core::MassFlowRate mdot_max);

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

    double integrator_{0.0};
    double prev_error_{0.0};
    bool   first_step_{true};
};

} // namespace btm::control
