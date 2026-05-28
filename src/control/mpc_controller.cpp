#include "btm/control/mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace btm::control {

MpcController::MpcController(const model::ThermalModel& model,
                             int horizon,
                             double setpoint_c,
                             double T_max_constraint,
                             double dT_max_constraint,
                             double w_track,
                             double w_delta_t,
                             double w_pump,
                             double w_slew,
                             double soft_T_max_penalty,
                             double soft_dT_penalty,
                             const solver::GradientDescentSolver& solver,
                             core::MassFlowRate mdot_min,
                             core::MassFlowRate mdot_max,
                             double T_can_constraint,
                             double soft_T_can_penalty)
    : model_(model),
      horizon_(horizon),
      setpoint_c_(setpoint_c),
      T_max_constraint_(T_max_constraint),
      dT_max_constraint_(dT_max_constraint),
      w_track_(w_track),
      w_delta_t_(w_delta_t),
      w_pump_(w_pump),
      w_slew_(w_slew),
      soft_T_max_penalty_(soft_T_max_penalty),
      soft_dT_penalty_(soft_dT_penalty),
      // T6: can constraint defaults to core constraint (single-node backward-compat)
      T_can_constraint_(T_can_constraint > 0.0 ? T_can_constraint : T_max_constraint),
      soft_T_can_penalty_(soft_T_can_penalty),
      solver_(solver),
      mdot_min_(mdot_min),
      mdot_max_(mdot_max) {
    // Warm-start at mid-range flow on first call
    const double mid = 0.5 * (mdot_min_.value + mdot_max_.value);
    last_u_sequence_.assign(horizon_, core::MassFlowRate{mid});
}

core::MassFlowRate MpcController::compute_command(const model::ThermalState& state,
                                                   core::Current I_cell,
                                                   core::Temperature T_inlet,
                                                   core::Duration dt) {
    // Warm-start: shift previous solution left, repeat last entry
    if (static_cast<int>(last_u_sequence_.size()) == horizon_ && horizon_ > 1) {
        for (int i = 0; i + 1 < horizon_; ++i)
            last_u_sequence_[i] = last_u_sequence_[i + 1];
        // last entry stays as-is (copy of second-to-last after shift)
    }

    solver::Problem prob;

    // Cost closure: captures current exogenous inputs (constant over horizon — no preview)
    prob.cost = [&](const std::vector<core::MassFlowRate>& u) -> double {
        return evaluate_cost(state, u, I_cell, T_inlet, dt);
    };

    // Projection: clamp each u_k onto [mdot_min, mdot_max]
    prob.project = [&](std::vector<core::MassFlowRate>& u) {
        for (auto& m : u) {
            m.value = std::clamp(m.value, mdot_min_.value, mdot_max_.value);
        }
    };

    auto sol = solver_.solve(prob, last_u_sequence_);
    last_u_sequence_ = sol.u_opt;

    return last_u_sequence_.empty() ? core::MassFlowRate{mdot_min_} : last_u_sequence_.front();
}

void MpcController::reset() {
    const double mid = 0.5 * (mdot_min_.value + mdot_max_.value);
    last_u_sequence_.assign(horizon_, core::MassFlowRate{mid});
}

double MpcController::evaluate_cost(const model::ThermalState& current_state,
                                    const std::vector<core::MassFlowRate>& u_seq,
                                    core::Current I_cell,
                                    core::Temperature T_inlet,
                                    core::Duration dt) const {
    model::ThermalState s = current_state;
    double cost = 0.0;
    core::MassFlowRate prev_u = u_seq.empty() ? last_u_sequence_.front() : u_seq.front();

    for (const auto& u : u_seq) {
        s = model_.step(s, u, I_cell, T_inlet, dt);

        // Safety constraint tracking uses the CORE temperature (T_core ≥ T_can always).
        // In single-node mode max_core_temp() == max_cell_temp() — backward-compatible.
        const double Tmax       = s.max_core_temp().value;
        const double track_err  = Tmax - setpoint_c_;   // tracking target from config
        const double dT         = s.delta_t().value;
        const double slew       = u.value - prev_u.value;

        cost += w_track_   * track_err * track_err;
        cost += w_delta_t_ * dT * dT;
        cost += w_pump_    * u.value * u.value;
        cost += w_slew_    * slew * slew;

        // Soft constraint: T_core < T_max_constraint (from thermal_constraints config)
        if (Tmax > T_max_constraint_) {
            const double excess = Tmax - T_max_constraint_;
            cost += soft_T_max_penalty_ * excess * excess;
        }
        // T6: Soft constraint: T_can < T_can_constraint (surface temperature guard)
        if (soft_T_can_penalty_ > 0.0) {
            const double T_can = s.max_cell_temp().value;
            if (T_can > T_can_constraint_) {
                const double excess = T_can - T_can_constraint_;
                cost += soft_T_can_penalty_ * excess * excess;
            }
        }
        // Soft constraint: ΔT < dT_max_constraint (from thermal_constraints config)
        if (dT > dT_max_constraint_) {
            const double excess = dT - dT_max_constraint_;
            cost += soft_dT_penalty_ * excess * excess;
        }

        prev_u = u;
    }

    return cost;
}

} // namespace btm::control
