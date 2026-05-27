#include "btm/config/config.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

using namespace btm::config;

static std::string write_temp_yaml(const std::string& content) {
    std::filesystem::path p = std::filesystem::temp_directory_path() / "btm_test_config.yaml";
    std::ofstream f(p);
    f << content;
    f.close();
    return p.string();
}

TEST(ConfigValidation, LoadsDefaultSuccessfully) {
    // This assumes the repo default.yaml is valid (it is, by construction)
    auto cfg = load_and_validate("config/default.yaml");
    EXPECT_EQ(cfg.module.series_count, 24);
    EXPECT_EQ(cfg.controller_type, "mpc");
}

TEST(ConfigValidation, RejectsNegativeMass) {
    std::string yaml = R"(
cell:
  capacity_ah: 4.5
  nominal_voltage_v: 3.6
  mass_kg: -0.1
  specific_heat_j_per_kg_k: 900
  surface_area_m2: 0.00475
  eta_ir_1c_v: 0.077837
module:
  series_count: 24
  parallel_count: 13
coolant:
  density_kg_per_m3: 805
  specific_heat_j_per_kg_k: 3500
  dynamic_viscosity_pa_s: 0.012565
  thermal_conductivity_w_per_m_k: 0.13
  inlet_temperature_c: 25
convection:
  h_ref_w_per_m2_k: 250
  m_dot_ref_kg_per_s: 0.5
  scaling_exponent: 0.6
thermal_constraints:
  max_cell_temperature_c: 35
  max_temperature_delta_c: 5
pump:
  min_flow_kg_per_s: 0.01
  max_flow_kg_per_s: 2.0
simulation:
  duration_s: 600
  timestep_s: 0.1
  log_path: "results/test.csv"
scenario:
  type: constant_c_rate
  c_rate: 5.0
controller:
  type: mpc
  mpc:
    horizon_steps: 20
    weights: {tracking: 1.0, delta_t: 10.0, pump_energy: 0.1, input_rate: 1.0}
    solver: {max_iterations: 50, step_size: 0.001, convergence_tol: 1e-4, finite_diff_epsilon: 1e-4}
)";

    std::string path = write_temp_yaml(yaml);
    EXPECT_THROW(load_and_validate(path), std::runtime_error);
}

TEST(ConfigValidation, RejectsMinFlowGreaterThanMax) {
    std::string yaml = R"(
cell: {capacity_ah: 4.5, nominal_voltage_v: 3.6, mass_kg: 0.07, specific_heat_j_per_kg_k: 900, surface_area_m2: 0.00475, eta_ir_1c_v: 0.077837}
module: {series_count: 24, parallel_count: 13}
coolant: {density_kg_per_m3: 805, specific_heat_j_per_kg_k: 3500, dynamic_viscosity_pa_s: 0.012565, thermal_conductivity_w_per_m_k: 0.13, inlet_temperature_c: 25}
convection: {h_ref_w_per_m2_k: 250, m_dot_ref_kg_per_s: 0.5, scaling_exponent: 0.6}
thermal_constraints: {max_cell_temperature_c: 35, max_temperature_delta_c: 5}
pump:
  min_flow_kg_per_s: 3.0
  max_flow_kg_per_s: 1.0
simulation: {duration_s: 600, timestep_s: 0.1, log_path: "results/test.csv"}
scenario: {type: constant_c_rate, c_rate: 5.0}
controller:
  type: mpc
  mpc:
    horizon_steps: 20
    weights: {tracking: 1.0, delta_t: 10.0, pump_energy: 0.1, input_rate: 1.0}
    solver: {max_iterations: 50, step_size: 0.001, convergence_tol: 1e-4, finite_diff_epsilon: 1e-4}
)";

    std::string path = write_temp_yaml(yaml);
    EXPECT_THROW(load_and_validate(path), std::runtime_error);
}
