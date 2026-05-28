#pragma once

#include "btm/sim/csv_logger.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

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
                                 double dT_max_constraint,
                                 SensorModel sensor)
    : model_(model),
      controller_(controller),
      dt_(dt),
      duration_(duration),
      log_path_(log_path),
      current_fn_(std::move(current_fn)),
      inlet_fn_(std::move(inlet_fn)),
      T_max_constraint_(T_max_constraint),
      dT_max_constraint_(dT_max_constraint),
      sensor_(std::move(sensor)) {}

template <typename Controller>
SimResult Simulator<Controller>::run() {
    // Ensure the output directory exists
    if (const auto parent = std::filesystem::path(log_path_).parent_path(); !parent.empty())
        std::filesystem::create_directories(parent);

    // Cold-start: all cells at coolant inlet temperature
    model::ThermalState state;
    const core::Temperature T0 = inlet_fn_(core::Duration{0.0});
    for (auto& t : state.cell_temperatures)    t = T0;
    for (auto& t : state.core_temperatures)    t = T0;
    for (auto& t : state.coolant_temperatures) t = T0;

    CsvLogger logger(log_path_);

    core::Duration t{0.0};

    SimResult result;

    // Running accumulators for time-average
    double sum_T_max = 0.0;

    while (t.value < duration_.value) {
        const auto I_cell  = current_fn_(t);
        const auto T_inlet = inlet_fn_(t);

        // Controller receives current exogenous inputs (no preview)
        const auto mdot = controller_.compute_command(state, I_cell, T_inlet, dt_);

        state = model_.step(state, mdot, I_cell, T_inlet, dt_);

        // Safety constraints enforce on CORE temperature (T_core ≥ T_can always).
        // In single-node mode max_core_temp() == max_cell_temp() — backward-compatible.
        const double T_core_max = state.max_core_temp().value;
        const double dT         = state.delta_t().value;

        const bool T_viol  = T_core_max > T_max_constraint_;
        const bool dT_viol = dT         > dT_max_constraint_;
        if (T_viol || dT_viol) {
            ++result.violation_count;
            result.violation_time_s += dt_.value;
            if (T_viol)
                result.violation_T_integral_c_s += (T_core_max - T_max_constraint_) * dt_.value;
        }

        result.peak_T_max_c = std::max(result.peak_T_max_c, T_core_max);
        result.peak_dT_c    = std::max(result.peak_dT_c, dT);
        result.pump_integral += mdot.value * mdot.value * dt_.value;
        sum_T_max += T_core_max;

        // Observed temperature for logging (sensor sees CAN/surface, not core)
        const double T_max_observed = sensor_.observed_max(state).value;
        logger.log(t.value, state, mdot, I_cell, T_inlet, T_max_observed, T_core_max);

        t.value += dt_.value;
        ++result.steps;

        if (result.steps % 1000 == 0) {
            std::cout << "t=" << t.value << "s  T_max=" << T_max
                      << "°C  mdot=" << mdot.value << "kg/s\n";
        }
    }

    logger.flush();
    result.time_avg_T_max_c = (result.steps > 0) ? sum_T_max / result.steps : 0.0;

    // Print human-readable summary
    std::cout << std::fixed << std::setprecision(3)
              << "=== Simulation complete: " << result.steps << " steps ===\n"
              << "  Peak T_core      : " << result.peak_T_max_c      << " °C\n"
              << "  Time-avg T_core  : " << result.time_avg_T_max_c  << " °C\n"
              << "  Peak ΔT          : " << result.peak_dT_c         << " °C\n"
              << "  ∫ṁ² dt           : " << result.pump_integral     << " (kg/s)²·s\n"
              << "  Violations       : " << result.violation_count   << " steps"
              << " (" << result.violation_time_s << " s)\n"
              << "  ∫(T-T_max)⁺ dt   : " << result.violation_T_integral_c_s << " °C·s\n";

    // Write machine-readable JSON summary
    const std::string json_path = [&] {
        std::string p = log_path_;
        const auto pos = p.rfind(".csv");
        if (pos != std::string::npos) p.replace(pos, 4, "_summary.json");
        else p += "_summary.json";
        return p;
    }();

    {
        std::ofstream js(json_path);
        js << std::fixed << std::setprecision(6) << "{\n"
           << "  \"steps\": "                       << result.steps                    << ",\n"
           << "  \"peak_T_max_c\": "                << result.peak_T_max_c             << ",\n"
           << "  \"time_avg_T_max_c\": "            << result.time_avg_T_max_c         << ",\n"
           << "  \"peak_dT_c\": "                   << result.peak_dT_c               << ",\n"
           << "  \"violation_count\": "             << result.violation_count          << ",\n"
           << "  \"violation_time_s\": "            << result.violation_time_s         << ",\n"
           << "  \"violation_T_integral_c_s\": "   << result.violation_T_integral_c_s << ",\n"
           << "  \"pump_integral\": "               << result.pump_integral            << "\n"
           << "}\n";
    }

    return result;
}

} // namespace btm::sim
