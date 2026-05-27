#include "btm/scenario/scenario.hpp"

#include <algorithm>
#include <stdexcept>

namespace btm::scenario {

ScenarioFunctions make_scenario(const config::Config& cfg) {
    const std::string& type = cfg.scenario.type;

    // Per-cell 1C current (A)
    const double I_1C = cfg.cell.capacity_ah;

    if (type == "constant_c_rate") {
        const double I_cell_a = I_1C * cfg.scenario.c_rate;
        const double T_in_c   = cfg.coolant.inlet_temperature_c;

        return ScenarioFunctions{
            .current_at = [I_cell_a](core::Duration) {
                return core::Current{I_cell_a};
            },
            .inlet_at = [T_in_c](core::Duration) {
                return core::Temperature{T_in_c};
            }
        };

    } else if (type == "step_transient") {
        const double I_low    = I_1C * cfg.scenario.initial_c_rate;
        const double I_high   = I_1C * cfg.scenario.final_c_rate;
        const double t_step   = cfg.scenario.step_time_s;
        const double T_in_c   = cfg.coolant.inlet_temperature_c;

        return ScenarioFunctions{
            .current_at = [I_low, I_high, t_step](core::Duration t) {
                return core::Current{t.value >= t_step ? I_high : I_low};
            },
            .inlet_at = [T_in_c](core::Duration) {
                return core::Temperature{T_in_c};
            }
        };

    } else if (type == "rising_ambient") {
        // Constant current at c_rate (defaults to 5C if not set; validated by loader)
        const double I_cell_a   = I_1C * (cfg.scenario.c_rate > 0.0 ? cfg.scenario.c_rate : 5.0);
        const double T_init     = cfg.scenario.initial_inlet_c;
        const double T_final    = cfg.scenario.final_inlet_c;
        const double t_ramp_s   = cfg.scenario.ramp_start_s;
        const double t_ramp_dur = cfg.scenario.ramp_duration_s;

        return ScenarioFunctions{
            .current_at = [I_cell_a](core::Duration) {
                return core::Current{I_cell_a};
            },
            .inlet_at = [T_init, T_final, t_ramp_s, t_ramp_dur](core::Duration t) -> core::Temperature {
                if (t.value <= t_ramp_s) return core::Temperature{T_init};
                if (t_ramp_dur <= 0.0 || t.value >= t_ramp_s + t_ramp_dur)
                    return core::Temperature{T_final};
                const double alpha = (t.value - t_ramp_s) / t_ramp_dur;
                return core::Temperature{T_init + alpha * (T_final - T_init)};
            }
        };

    } else {
        throw std::runtime_error("make_scenario: unknown scenario type '" + type + "'");
    }
}

} // namespace btm::scenario
