#pragma once

#include "btm/core/types.hpp"
#include "btm/model/thermal_model.hpp"
#include "btm/model/thermal_state.hpp"
#include "btm/solver/gradient_descent.hpp"

#include <vector>

namespace btm::control {

class MpcController {
public:
    MpcController(const model::ThermalModel&          model,
                  int                                  horizon,
                  double                               setpoint_c,
                  double                               T_max_constraint,    ///< core temp limit
                  double                               dT_max_constraint,   ///< inter-cell ΔT limit
                  double                               w_track,
                  double                               w_delta_t,
                  double                               w_pump,
                  double                               w_slew,
                  double                               soft_T_max_penalty,
                  double                               soft_dT_penalty,
                  const solver::GradientDescentSolver& solver,
                  core::MassFlowRate                   mdot_min,
                  core::MassFlowRate                   mdot_max,
                  // T6: optional can temperature constraint (0.0 = use T_max_constraint)
                  double                               T_can_constraint   = 0.0,
                  double                               soft_T_can_penalty = 0.0);

    /// Compute the optimal mass-flow command via single-shooting gradient descent.
    ///
    /// I_cell and T_inlet are the current (measured) exogenous inputs.
    /// They are assumed constant over the horizon — no preview of future values.
    core::MassFlowRate compute_command(const model::ThermalState& state,
                                       core::Current              I_cell,
                                       core::Temperature          T_inlet,
                                       core::Duration             dt);

    void reset();

private:
    const model::ThermalModel&          model_;
    int                                  horizon_;
    double                               setpoint_c_;
    double                               T_max_constraint_;
    double                               dT_max_constraint_;
    double                               w_track_;
    double                               w_delta_t_;
    double                               w_pump_;
    double                               w_slew_;
    double                               soft_T_max_penalty_;
    double                               soft_dT_penalty_;
    double                               T_can_constraint_;     ///< T6: can temperature limit
    double                               soft_T_can_penalty_;   ///< T6: penalty when T_can > limit
    const solver::GradientDescentSolver& solver_;
    core::MassFlowRate                   mdot_min_;
    core::MassFlowRate                   mdot_max_;

    std::vector<core::MassFlowRate> last_u_sequence_;  // warm-start buffer

    double evaluate_cost(const model::ThermalState&          current_state,
                         const std::vector<core::MassFlowRate>& u_seq,
                         core::Current                        I_cell,
                         core::Temperature                    T_inlet,
                         core::Duration                       dt) const;
};

} // namespace btm::control
