#include "btm/config/config.hpp"
#include "btm/control/mpc_controller.hpp"
#include "btm/control/pid_controller.hpp"
#include "btm/model/thermal_model.hpp"
#include "btm/scenario/scenario.hpp"
#include "btm/sim/sensor_model.hpp"
#include "btm/sim/simulator.hpp"
#include "btm/solver/gradient_descent.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

using namespace btm;

namespace {

// Build a complete, valid Config for a short constant-5C run.
config::Config make_test_config(const std::string& log_path) {
    config::Config cfg;

    cfg.cell.capacity_ah             = 4.5;
    cfg.cell.nominal_voltage_v       = 3.6;
    cfg.cell.mass_kg                 = 0.070;
    cfg.cell.specific_heat_j_per_kg_k = 900.0;
    cfg.cell.surface_area_m2         = 0.00475;
    cfg.cell.eta_ir_1c_v             = 0.077837;

    cfg.module.series_count   = 24;
    cfg.module.parallel_count = 13;

    cfg.coolant.density_kg_per_m3          = 805.0;
    cfg.coolant.specific_heat_j_per_kg_k   = 3500.0;
    cfg.coolant.dynamic_viscosity_pa_s     = 0.012565;
    cfg.coolant.thermal_conductivity_w_per_m_k = 0.13;
    cfg.coolant.inlet_temperature_c        = 25.0;

    cfg.convection.h_ref_w_per_m2_k   = 250.0;
    cfg.convection.m_dot_ref_kg_per_s = 0.5;
    cfg.convection.scaling_exponent   = 0.6;

    cfg.thermal_constraints.max_cell_temperature_c  = 35.0;
    cfg.thermal_constraints.max_temperature_delta_c = 5.0;

    cfg.pump.min_flow_kg_per_s = 0.01;
    cfg.pump.max_flow_kg_per_s = 2.0;

    cfg.simulation.duration_s  = 30.0;   // short: fast test
    cfg.simulation.timestep_s  = 0.1;
    cfg.simulation.log_path    = log_path;

    cfg.scenario.type   = "constant_c_rate";
    cfg.scenario.c_rate = 5.0;

    cfg.controller_type = "pid";
    cfg.pid.kp                = 0.05;
    cfg.pid.ki                = 0.001;
    cfg.pid.setpoint_c        = 30.0;
    cfg.pid.integrator_limit  = 50.0;

    cfg.mpc.horizon_steps          = 10;
    cfg.mpc.setpoint_c             = 35.0;
    cfg.mpc.soft_T_max_penalty     = 10000.0;
    cfg.mpc.soft_dT_penalty        = 10000.0;
    cfg.mpc.weights.tracking       = 1.0;
    cfg.mpc.weights.delta_t        = 10.0;
    cfg.mpc.weights.pump_energy    = 0.1;
    cfg.mpc.weights.input_rate     = 1.0;
    cfg.mpc.solver.max_iterations      = 20;
    cfg.mpc.solver.step_size           = 0.005;
    cfg.mpc.solver.convergence_tol     = 1e-4;
    cfg.mpc.solver.finite_diff_epsilon = 1e-4;

    return cfg;
}

} // anonymous namespace

// ============================================================================

TEST(Integration, PidRunCompletes) {
    auto cfg = make_test_config("/tmp/test_pid_run.csv");
    model::ThermalModel model(cfg);
    auto [current_fn, inlet_fn] = scenario::make_scenario(cfg);

    control::PidController pid(
        cfg.pid.kp, cfg.pid.ki, cfg.pid.setpoint_c, cfg.pid.integrator_limit,
        core::MassFlowRate{cfg.pump.min_flow_kg_per_s},
        core::MassFlowRate{cfg.pump.max_flow_kg_per_s});

    sim::Simulator sim(
        model, pid,
        core::Duration{cfg.simulation.timestep_s},
        core::Duration{cfg.simulation.duration_s},
        cfg.simulation.log_path,
        current_fn, inlet_fn,
        cfg.thermal_constraints.max_cell_temperature_c,
        cfg.thermal_constraints.max_temperature_delta_c);

    EXPECT_NO_THROW(sim.run());
    EXPECT_TRUE(std::filesystem::exists(cfg.simulation.log_path));
}

TEST(Integration, MpcRunCompletes) {
    auto cfg = make_test_config("/tmp/test_mpc_run.csv");
    model::ThermalModel model(cfg);
    auto [current_fn, inlet_fn] = scenario::make_scenario(cfg);

    solver::GradientDescentSolver gdsolver(
        cfg.mpc.solver.max_iterations,
        cfg.mpc.solver.step_size,
        cfg.mpc.solver.convergence_tol,
        cfg.mpc.solver.finite_diff_epsilon);

    control::MpcController mpc(
        model, cfg.mpc.horizon_steps,
        cfg.mpc.setpoint_c,
        cfg.thermal_constraints.max_cell_temperature_c,
        cfg.thermal_constraints.max_temperature_delta_c,
        cfg.mpc.weights.tracking, cfg.mpc.weights.delta_t,
        cfg.mpc.weights.pump_energy, cfg.mpc.weights.input_rate,
        cfg.mpc.soft_T_max_penalty, cfg.mpc.soft_dT_penalty,
        gdsolver,
        core::MassFlowRate{cfg.pump.min_flow_kg_per_s},
        core::MassFlowRate{cfg.pump.max_flow_kg_per_s});

    sim::Simulator sim(
        model, mpc,
        core::Duration{cfg.simulation.timestep_s},
        core::Duration{cfg.simulation.duration_s},
        cfg.simulation.log_path,
        current_fn, inlet_fn,
        cfg.thermal_constraints.max_cell_temperature_c,
        cfg.thermal_constraints.max_temperature_delta_c);

    EXPECT_NO_THROW(sim.run());
    EXPECT_TRUE(std::filesystem::exists(cfg.simulation.log_path));
}

TEST(Integration, PidWithDownstreamSensorRunsCorrectly) {
    // Verify that a PID using the Downstream sensor model runs without error
    // and produces a log file.  Also checks that the Controller concept is
    // satisfied when a non-default SensorModel is provided.
    auto cfg = make_test_config("/tmp/test_pid_downstream.csv");
    model::ThermalModel model(cfg);
    auto [current_fn, inlet_fn] = scenario::make_scenario(cfg);

    const auto sensor = sim::SensorModel::from_string("downstream");

    control::PidController pid(
        cfg.pid.kp, cfg.pid.ki, cfg.pid.setpoint_c, cfg.pid.integrator_limit,
        core::MassFlowRate{cfg.pump.min_flow_kg_per_s},
        core::MassFlowRate{cfg.pump.max_flow_kg_per_s},
        0.0,    // deadband_c
        sensor);

    sim::Simulator sim_ds(
        model, pid,
        core::Duration{cfg.simulation.timestep_s},
        core::Duration{cfg.simulation.duration_s},
        cfg.simulation.log_path,
        current_fn, inlet_fn,
        cfg.thermal_constraints.max_cell_temperature_c,
        cfg.thermal_constraints.max_temperature_delta_c,
        sensor);

    EXPECT_NO_THROW(sim_ds.run());
    EXPECT_TRUE(std::filesystem::exists(cfg.simulation.log_path));
}

TEST(Integration, BothControllersProduceOutputFile) {
    for (const auto& name : {"pid", "mpc"}) {
        const std::string log = std::string("/tmp/test_both_") + name + ".csv";
        auto cfg = make_test_config(log);
        model::ThermalModel model(cfg);
        auto [current_fn, inlet_fn] = scenario::make_scenario(cfg);

        solver::GradientDescentSolver gdsolver(20, 0.005, 1e-4, 1e-4);
        control::MpcController mpc(
            model, cfg.mpc.horizon_steps,
            cfg.mpc.setpoint_c,
            cfg.thermal_constraints.max_cell_temperature_c,
            cfg.thermal_constraints.max_temperature_delta_c,
            cfg.mpc.weights.tracking, cfg.mpc.weights.delta_t,
            cfg.mpc.weights.pump_energy, cfg.mpc.weights.input_rate,
            cfg.mpc.soft_T_max_penalty, cfg.mpc.soft_dT_penalty,
            gdsolver,
            core::MassFlowRate{cfg.pump.min_flow_kg_per_s},
            core::MassFlowRate{cfg.pump.max_flow_kg_per_s});
        control::PidController pid(
            cfg.pid.kp, cfg.pid.ki, cfg.pid.setpoint_c, cfg.pid.integrator_limit,
            core::MassFlowRate{cfg.pump.min_flow_kg_per_s},
            core::MassFlowRate{cfg.pump.max_flow_kg_per_s});

        if (std::string(name) == "pid") {
            sim::Simulator sim(model, pid,
                core::Duration{cfg.simulation.timestep_s},
                core::Duration{cfg.simulation.duration_s},
                log, current_fn, inlet_fn,
                cfg.thermal_constraints.max_cell_temperature_c,
                cfg.thermal_constraints.max_temperature_delta_c);
            sim.run();
        } else {
            sim::Simulator sim(model, mpc,
                core::Duration{cfg.simulation.timestep_s},
                core::Duration{cfg.simulation.duration_s},
                log, current_fn, inlet_fn,
                cfg.thermal_constraints.max_cell_temperature_c,
                cfg.thermal_constraints.max_temperature_delta_c);
            sim.run();
        }
        EXPECT_TRUE(std::filesystem::exists(log));
    }
}
