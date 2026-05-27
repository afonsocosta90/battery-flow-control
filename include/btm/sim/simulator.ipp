#pragma once

#include "btm/sim/csv_logger.hpp"

#include <filesystem>
#include <iostream>

namespace btm::sim {

template <typename Controller>
Simulator<Controller>::Simulator(model::ThermalModel& model,
                                 Controller& controller,
                                 core::Duration dt,
                                 core::Duration duration,
                                 const std::string& log_path,
                                 std::function<core::Current(core::Duration)> current_fn,
                                 std::function<core::Temperature(core::Duration)> inlet_fn,
                                 double T_max_constraint,
                                 double dT_max_constraint)
    : model_(model),
      controller_(controller),
      dt_(dt),
      duration_(duration),
      log_path_(log_path),
      current_fn_(std::move(current_fn)),
      inlet_fn_(std::move(inlet_fn)),
      T_max_constraint_(T_max_constraint),
      dT_max_constraint_(dT_max_constraint) {}

template <typename Controller>
void Simulator<Controller>::run() {
    // Ensure the output directory exists
    if (const auto parent = std::filesystem::path(log_path_).parent_path(); !parent.empty())
        std::filesystem::create_directories(parent);

    // Cold-start: all cells at coolant inlet temperature
    model::ThermalState state;
    const core::Temperature T0 = inlet_fn_(core::Duration{0.0});
    for (auto& t : state.cell_temperatures)    t = T0;
    for (auto& t : state.coolant_temperatures) t = T0;

    CsvLogger logger(log_path_);

    core::Duration t{0.0};
    int step = 0;

    // Summary accumulators
    double peak_T_max = 0.0;
    double peak_dT    = 0.0;
    double pump_integral = 0.0;     // ∫ṁ² dt
    int violations = 0;

    while (t.value < duration_.value) {
        const auto I_cell  = current_fn_(t);
        const auto T_inlet = inlet_fn_(t);

        // Controller receives current exogenous inputs (no preview)
        const auto mdot = controller_.compute_command(state, I_cell, T_inlet, dt_);

        state = model_.step(state, mdot, I_cell, T_inlet, dt_);

        // Constraint check
        const double T_max = state.max_cell_temp().value;
        const double dT    = state.delta_t().value;
        if (T_max > T_max_constraint_ || dT > dT_max_constraint_) ++violations;

        peak_T_max    = std::max(peak_T_max, T_max);
        peak_dT       = std::max(peak_dT, dT);
        pump_integral += mdot.value * mdot.value * dt_.value;

        logger.log(t.value, state, mdot, I_cell, T_inlet);

        t.value += dt_.value;
        ++step;

        if (step % 1000 == 0) {
            std::cout << "t=" << t.value << "s  T_max=" << T_max
                      << "°C  mdot=" << mdot.value << "kg/s\n";
        }
    }

    logger.flush();

    std::cout << "=== Simulation complete: " << step << " steps ===\n"
              << "  Peak T_cell : " << peak_T_max << " °C\n"
              << "  Peak ΔT     : " << peak_dT    << " °C\n"
              << "  ∫ṁ² dt      : " << pump_integral << " (kg/s)²·s\n"
              << "  Violations  : " << violations << "\n";
}

} // namespace btm::sim
