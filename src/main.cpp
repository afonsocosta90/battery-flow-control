#include "btm/config/config.hpp"
#include "btm/control/mpc_controller.hpp"
#include "btm/control/pid_controller.hpp"
#include "btm/model/thermal_model.hpp"
#include "btm/scenario/scenario.hpp"
#include "btm/sim/sensor_model.hpp"
#include "btm/sim/simulator.hpp"
#include "btm/solver/gradient_descent.hpp"

#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
    std::string config_path    = "config/default.yaml";
    std::string controller_arg;   // empty → use yaml's controller.type

    if (argc > 1) config_path    = argv[1];
    if (argc > 2) controller_arg = argv[2];

    try {
        auto cfg = btm::config::load_and_validate(config_path);

        // Command-line controller override (useful for running both PID and MPC on
        // the same scenario yaml without editing the file).
        const std::string controller_type =
            controller_arg.empty() ? cfg.controller_type : controller_arg;

        std::cout << "Config     : " << config_path    << "\n"
                  << "Scenario   : " << cfg.scenario.type << "\n"
                  << "Controller : " << controller_type << "\n"
                  << "Log path   : " << cfg.simulation.log_path << "\n\n";

        btm::model::ThermalModel thermal_model(cfg);

        // Build scenario functions (current_at, inlet_at)
        auto [current_fn, inlet_fn] = btm::scenario::make_scenario(cfg);

        // Sensor model — shared between PID and logger.
        // MPC always uses the perfect (full-state) model internally; only PID
        // and the logging column are affected by the configured sensor.
        const btm::sim::SensorModel sensor =
            btm::sim::SensorModel::from_string(cfg.sensor.mode, cfg.sensor.positions);

        if (controller_type == "pid") {
            btm::control::PidController pid(
                cfg.pid.kp,
                cfg.pid.ki,
                cfg.pid.setpoint_c,
                cfg.pid.integrator_limit,
                btm::core::MassFlowRate{cfg.pump.min_flow_kg_per_s},
                btm::core::MassFlowRate{cfg.pump.max_flow_kg_per_s},
                cfg.pid.deadband_c,
                sensor
            );

            btm::sim::Simulator simulator(
                thermal_model,
                pid,
                btm::core::Duration{cfg.simulation.timestep_s},
                btm::core::Duration{cfg.simulation.duration_s},
                cfg.simulation.log_path,
                current_fn,
                inlet_fn,
                cfg.thermal_constraints.max_cell_temperature_c,
                cfg.thermal_constraints.max_temperature_delta_c,
                sensor
            );

            simulator.run();

        } else if (controller_type == "mpc") {
            btm::solver::GradientDescentSolver solver(
                cfg.mpc.solver.max_iterations,
                cfg.mpc.solver.step_size,
                cfg.mpc.solver.convergence_tol,
                cfg.mpc.solver.finite_diff_epsilon
            );

            btm::control::MpcController mpc(
                thermal_model,
                cfg.mpc.horizon_steps,
                cfg.mpc.setpoint_c,
                cfg.thermal_constraints.max_cell_temperature_c,
                cfg.thermal_constraints.max_temperature_delta_c,
                cfg.mpc.weights.tracking,
                cfg.mpc.weights.delta_t,
                cfg.mpc.weights.pump_energy,
                cfg.mpc.weights.input_rate,
                cfg.mpc.soft_T_max_penalty,
                cfg.mpc.soft_dT_penalty,
                solver,
                btm::core::MassFlowRate{cfg.pump.min_flow_kg_per_s},
                btm::core::MassFlowRate{cfg.pump.max_flow_kg_per_s}
            );

            // MPC uses the perfect sensor internally; pass perfect to the logger too
            // so T_max_observed == T_max (no logging artefact for MPC runs).
            btm::sim::Simulator simulator(
                thermal_model,
                mpc,
                btm::core::Duration{cfg.simulation.timestep_s},
                btm::core::Duration{cfg.simulation.duration_s},
                cfg.simulation.log_path,
                current_fn,
                inlet_fn,
                cfg.thermal_constraints.max_cell_temperature_c,
                cfg.thermal_constraints.max_temperature_delta_c,
                btm::sim::SensorModel{}   // perfect sensor for MPC logging
            );

            simulator.run();

        } else {
            std::cerr << "Unknown controller type '" << controller_type
                      << "'. Use 'pid' or 'mpc'.\n";
            return 2;
        }

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
