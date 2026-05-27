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

    void log(double t, const model::ThermalState& state,
             core::MassFlowRate mdot,
             core::Current I_cell,
             core::Temperature T_inlet);

    void flush();

private:
    std::ofstream file_;
    bool header_written_{false};
};

} // namespace btm::sim
