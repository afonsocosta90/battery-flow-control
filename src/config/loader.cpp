#include "btm/config/config.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <stdexcept>
#include <string>

namespace btm::config {

namespace {

void require_positive(double v, const char* name) {
    if (v <= 0.0) {
        throw std::runtime_error(std::string(name) + " must be > 0 (got " + std::to_string(v) + ")");
    }
}

void require_non_negative(double v, const char* name) {
    if (v < 0.0) {
        throw std::runtime_error(std::string(name) + " must be >= 0 (got " + std::to_string(v) + ")");
    }
}

void require_int_positive(int v, const char* name) {
    if (v <= 0) {
        throw std::runtime_error(std::string(name) + " must be a positive integer (got " + std::to_string(v) + ")");
    }
}

void require_range(double v, double lo, double hi, const char* name) {
    if (v < lo || v > hi) {
        throw std::runtime_error(std::string(name) + " out of range [" + std::to_string(lo) + ", " +
                                 std::to_string(hi) + "] (got " + std::to_string(v) + ")");
    }
}

} // anonymous namespace

Config load_and_validate(const std::string& path) {
    if (!std::filesystem::exists(path)) {
        throw std::runtime_error("Config file does not exist: " + path);
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Failed to parse YAML: " + std::string(e.what()));
    }

    Config cfg;

    // --- cell ---
    auto cell = root["cell"];
    if (!cell) throw std::runtime_error("Missing top-level key 'cell'");

    cfg.cell.capacity_ah           = cell["capacity_ah"].as<double>();
    cfg.cell.nominal_voltage_v     = cell["nominal_voltage_v"].as<double>();
    cfg.cell.mass_kg               = cell["mass_kg"].as<double>();
    cfg.cell.specific_heat_j_per_kg_k = cell["specific_heat_j_per_kg_k"].as<double>();
    cfg.cell.surface_area_m2       = cell["surface_area_m2"].as<double>();
    cfg.cell.eta_ir_1c_v           = cell["eta_ir_1c_v"].as<double>();

    require_positive(cfg.cell.capacity_ah, "cell.capacity_ah");
    require_positive(cfg.cell.nominal_voltage_v, "cell.nominal_voltage_v");
    require_positive(cfg.cell.mass_kg, "cell.mass_kg");
    require_positive(cfg.cell.specific_heat_j_per_kg_k, "cell.specific_heat_j_per_kg_k");
    require_positive(cfg.cell.surface_area_m2, "cell.surface_area_m2");
    require_positive(cfg.cell.eta_ir_1c_v, "cell.eta_ir_1c_v");

    // --- module ---
    auto module = root["module"];
    if (!module) throw std::runtime_error("Missing top-level key 'module'");

    cfg.module.series_count   = module["series_count"].as<int>();
    cfg.module.parallel_count = module["parallel_count"].as<int>();

    require_int_positive(cfg.module.series_count, "module.series_count");
    require_int_positive(cfg.module.parallel_count, "module.parallel_count");

    // --- coolant ---
    auto coolant = root["coolant"];
    if (!coolant) throw std::runtime_error("Missing top-level key 'coolant'");

    cfg.coolant.density_kg_per_m3          = coolant["density_kg_per_m3"].as<double>();
    cfg.coolant.specific_heat_j_per_kg_k   = coolant["specific_heat_j_per_kg_k"].as<double>();
    cfg.coolant.dynamic_viscosity_pa_s     = coolant["dynamic_viscosity_pa_s"].as<double>();
    cfg.coolant.thermal_conductivity_w_per_m_k = coolant["thermal_conductivity_w_per_m_k"].as<double>();
    cfg.coolant.inlet_temperature_c        = coolant["inlet_temperature_c"].as<double>();

    require_positive(cfg.coolant.density_kg_per_m3, "coolant.density_kg_per_m3");
    require_positive(cfg.coolant.specific_heat_j_per_kg_k, "coolant.specific_heat_j_per_kg_k");
    require_positive(cfg.coolant.dynamic_viscosity_pa_s, "coolant.dynamic_viscosity_pa_s");
    require_positive(cfg.coolant.thermal_conductivity_w_per_m_k, "coolant.thermal_conductivity_w_per_m_k");

    // --- convection ---
    auto conv = root["convection"];
    if (!conv) throw std::runtime_error("Missing top-level key 'convection'");

    cfg.convection.h_ref_w_per_m2_k   = conv["h_ref_w_per_m2_k"].as<double>();
    cfg.convection.m_dot_ref_kg_per_s = conv["m_dot_ref_kg_per_s"].as<double>();
    cfg.convection.scaling_exponent   = conv["scaling_exponent"].as<double>();

    require_positive(cfg.convection.h_ref_w_per_m2_k, "convection.h_ref_w_per_m2_k");
    require_positive(cfg.convection.m_dot_ref_kg_per_s, "convection.m_dot_ref_kg_per_s");
    require_range(cfg.convection.scaling_exponent, 0.1, 2.0, "convection.scaling_exponent");

    // --- thermal_constraints ---
    auto tc = root["thermal_constraints"];
    if (!tc) throw std::runtime_error("Missing top-level key 'thermal_constraints'");

    cfg.thermal_constraints.max_cell_temperature_c = tc["max_cell_temperature_c"].as<double>();
    cfg.thermal_constraints.max_temperature_delta_c = tc["max_temperature_delta_c"].as<double>();

    require_positive(cfg.thermal_constraints.max_cell_temperature_c, "thermal_constraints.max_cell_temperature_c");
    require_positive(cfg.thermal_constraints.max_temperature_delta_c, "thermal_constraints.max_temperature_delta_c");

    // --- pump ---
    auto pump = root["pump"];
    if (!pump) throw std::runtime_error("Missing top-level key 'pump'");

    cfg.pump.min_flow_kg_per_s = pump["min_flow_kg_per_s"].as<double>();
    cfg.pump.max_flow_kg_per_s = pump["max_flow_kg_per_s"].as<double>();

    require_positive(cfg.pump.min_flow_kg_per_s, "pump.min_flow_kg_per_s");
    require_positive(cfg.pump.max_flow_kg_per_s, "pump.max_flow_kg_per_s");

    if (cfg.pump.min_flow_kg_per_s >= cfg.pump.max_flow_kg_per_s) {
        throw std::runtime_error("pump.min_flow_kg_per_s must be < pump.max_flow_kg_per_s");
    }

    // --- simulation ---
    auto sim = root["simulation"];
    if (!sim) throw std::runtime_error("Missing top-level key 'simulation'");

    cfg.simulation.duration_s  = sim["duration_s"].as<double>();
    cfg.simulation.timestep_s  = sim["timestep_s"].as<double>();
    cfg.simulation.log_path    = sim["log_path"].as<std::string>();

    require_positive(cfg.simulation.duration_s, "simulation.duration_s");
    require_positive(cfg.simulation.timestep_s, "simulation.timestep_s");

    // --- scenario ---
    auto sc = root["scenario"];
    if (!sc) throw std::runtime_error("Missing top-level key 'scenario'");

    cfg.scenario.type = sc["type"].as<std::string>();

    if (cfg.scenario.type == "constant_c_rate") {
        cfg.scenario.c_rate = sc["c_rate"].as<double>();
        require_positive(cfg.scenario.c_rate, "scenario.c_rate");
    } else if (cfg.scenario.type == "step_transient") {
        cfg.scenario.initial_c_rate = sc["initial_c_rate"].as<double>();
        cfg.scenario.final_c_rate   = sc["final_c_rate"].as<double>();
        cfg.scenario.step_time_s    = sc["step_time_s"].as<double>();
        require_non_negative(cfg.scenario.initial_c_rate, "scenario.initial_c_rate");
        require_positive(cfg.scenario.final_c_rate, "scenario.final_c_rate");
        require_positive(cfg.scenario.step_time_s, "scenario.step_time_s");
    } else if (cfg.scenario.type == "rising_ambient") {
        cfg.scenario.initial_inlet_c = sc["initial_inlet_c"].as<double>();
        cfg.scenario.final_inlet_c   = sc["final_inlet_c"].as<double>();
        cfg.scenario.ramp_start_s    = sc["ramp_start_s"].as<double>();
        cfg.scenario.ramp_duration_s = sc["ramp_duration_s"].as<double>();
        // Optional c_rate (defaults to 5 if absent; scenario.cpp handles 0.0 → 5.0)
        cfg.scenario.c_rate = sc["c_rate"] ? sc["c_rate"].as<double>() : 5.0;
    } else {
        throw std::runtime_error("Unknown scenario.type: " + cfg.scenario.type);
    }

    // --- controller ---
    auto ctrl = root["controller"];
    if (!ctrl) throw std::runtime_error("Missing top-level key 'controller'");

    cfg.controller_type = ctrl["type"].as<std::string>();
    if (cfg.controller_type != "pid" && cfg.controller_type != "mpc") {
        throw std::runtime_error("controller.type must be 'pid' or 'mpc'");
    }

    // Always load whichever controller sections are present — the binary may be
    // invoked with an override controller type (e.g., ./btm config.yaml pid on a
    // yaml that has type:mpc).  Sections that are absent are silently skipped.
    if (auto p = ctrl["pid"]) {
        cfg.pid.kp               = p["kp"].as<double>();
        cfg.pid.ki               = p["ki"].as<double>();
        cfg.pid.setpoint_c       = p["setpoint_c"].as<double>();
        cfg.pid.integrator_limit = p["integrator_limit"].as<double>();
        cfg.pid.deadband_c       = p["deadband_c"] ? p["deadband_c"].as<double>() : 0.0;
        require_positive(cfg.pid.kp, "pid.kp");
        require_non_negative(cfg.pid.ki, "pid.ki");
        require_non_negative(cfg.pid.deadband_c, "pid.deadband_c");
    } else if (cfg.controller_type == "pid") {
        throw std::runtime_error("Missing 'pid' section for controller.type = pid");
    }

    if (auto m = ctrl["mpc"]) {
        cfg.mpc.horizon_steps = m["horizon_steps"].as<int>();
        require_int_positive(cfg.mpc.horizon_steps, "mpc.horizon_steps");
        cfg.mpc.setpoint_c = m["setpoint_c"] ? m["setpoint_c"].as<double>() : 35.0;
        cfg.mpc.soft_T_max_penalty = m["soft_T_max_penalty"] ? m["soft_T_max_penalty"].as<double>() : 10000.0;
        cfg.mpc.soft_dT_penalty    = m["soft_dT_penalty"]    ? m["soft_dT_penalty"].as<double>()    : 10000.0;

        auto w = m["weights"];
        cfg.mpc.weights.tracking    = w["tracking"].as<double>();
        cfg.mpc.weights.delta_t     = w["delta_t"].as<double>();
        cfg.mpc.weights.pump_energy = w["pump_energy"].as<double>();
        cfg.mpc.weights.input_rate  = w["input_rate"].as<double>();
        require_positive(cfg.mpc.weights.tracking, "mpc.weights.tracking");

        auto sol = m["solver"];
        cfg.mpc.solver.max_iterations      = sol["max_iterations"].as<int>();
        cfg.mpc.solver.step_size           = sol["step_size"].as<double>();
        cfg.mpc.solver.convergence_tol     = sol["convergence_tol"].as<double>();
        cfg.mpc.solver.finite_diff_epsilon = sol["finite_diff_epsilon"].as<double>();
        require_positive(cfg.mpc.solver.max_iterations, "mpc.solver.max_iterations");
        require_positive(cfg.mpc.solver.step_size, "mpc.solver.step_size");
        require_positive(cfg.mpc.solver.convergence_tol, "mpc.solver.convergence_tol");
        require_positive(cfg.mpc.solver.finite_diff_epsilon, "mpc.solver.finite_diff_epsilon");
    } else if (cfg.controller_type == "mpc") {
        throw std::runtime_error("Missing 'mpc' section for controller.type = mpc");
    }

    // --- sensor (optional; defaults to perfect / no positions) ---
    if (auto s = root["sensor"]) {
        cfg.sensor.mode = s["mode"] ? s["mode"].as<std::string>() : "perfect";
        if (cfg.sensor.mode != "perfect" &&
            cfg.sensor.mode != "downstream" &&
            cfg.sensor.mode != "sparse") {
            throw std::runtime_error(
                "sensor.mode must be 'perfect', 'downstream', or 'sparse' (got '" +
                cfg.sensor.mode + "')");
        }
        if (s["positions"]) {
            for (const auto& pos : s["positions"]) {
                cfg.sensor.positions.push_back(pos.as<int>());
            }
        }
    }

    // Cross-field sanity checks
    if (cfg.thermal_constraints.max_cell_temperature_c <= cfg.coolant.inlet_temperature_c) {
        throw std::runtime_error("thermal_constraints.max_cell_temperature_c must be > coolant.inlet_temperature_c");
    }

    return cfg;
}

} // namespace btm::config
