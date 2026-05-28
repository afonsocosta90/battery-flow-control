#pragma once

#include "btm/core/types.hpp"
#include "btm/model/thermal_state.hpp"

#include <fstream>
#include <string>

namespace btm::sim {

class CsvLogger {
public:
    explicit CsvLogger(const std::string& path);
    ~CsvLogger();

    /// Log one timestep row.
    ///
    /// \param t               Simulation time (s).
    /// \param state           Full thermal state (provides T_max, T_min, ΔT).
    /// \param mdot            Commanded mass flow rate.
    /// \param I_cell          Per-cell current.
    /// \param T_inlet         Coolant inlet temperature.
    /// \param t_max_observed  Observed max temperature as seen by the sensor model
    ///                        (may differ from the true global max when not in
    ///                        perfect-sensor mode).
    void log(double t, const model::ThermalState& state,
             core::MassFlowRate mdot,
             core::Current I_cell,
             core::Temperature T_inlet,
             double t_max_observed);

    void flush();

private:
    std::ofstream file_;
    bool header_written_{false};
};

} // namespace btm::sim
