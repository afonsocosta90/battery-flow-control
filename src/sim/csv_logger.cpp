#include "btm/sim/csv_logger.hpp"

#include <iomanip>

namespace btm::sim {

CsvLogger::CsvLogger(const std::string& path) {
    file_.open(path);
}

CsvLogger::~CsvLogger() {
    if (file_.is_open()) {
        file_.close();
    }
}

void CsvLogger::log(double t, const model::ThermalState& state,
                    core::MassFlowRate mdot,
                    core::Current I_cell,
                    core::Temperature T_inlet,
                    double t_max_observed,
                    double t_core_max) {
    if (!header_written_) {
        file_ << "t,mdot,T_max,T_min,delta_T,I_cell,T_inlet,T_max_observed,T_core_max\n";
        header_written_ = true;
    }

    const double T_max = state.max_cell_temp().value;
    const double T_min = state.min_cell_temp().value;
    const double dT    = state.delta_t().value;

    file_ << std::fixed << std::setprecision(6)
          << t << ","
          << mdot.value << ","
          << T_max << ","
          << T_min << ","
          << dT << ","
          << I_cell.value << ","
          << T_inlet.value << ","
          << t_max_observed << ","
          << t_core_max << "\n";
}

void CsvLogger::flush() {
    if (file_.is_open()) file_.flush();
}

} // namespace btm::sim
