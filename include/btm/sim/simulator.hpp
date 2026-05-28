#pragma once

#include "btm/core/types.hpp"
#include "btm/model/thermal_state.hpp"
#include "btm/model/thermal_model.hpp"
#include "btm/sim/sensor_model.hpp"

#include <functional>
#include <string>

namespace btm::sim {

template <typename Controller>
class Simulator {
public:
    Simulator(model::ThermalModel& model,
              Controller& controller,
              core::Duration dt,
              core::Duration duration,
              const std::string& log_path,
              std::function<core::Current(core::Duration)> current_fn,
              std::function<core::Temperature(core::Duration)> inlet_fn,
              double T_max_constraint,    ///< from thermal_constraints.max_cell_temperature_c
              double dT_max_constraint,   ///< from thermal_constraints.max_temperature_delta_c
              SensorModel sensor = SensorModel{});  ///< logging observability model (default: perfect)

    void run();

private:
    model::ThermalModel& model_;
    Controller& controller_;
    core::Duration dt_;
    core::Duration duration_;
    std::string log_path_;

    std::function<core::Current(core::Duration)> current_fn_;
    std::function<core::Temperature(core::Duration)> inlet_fn_;

    double T_max_constraint_;
    double dT_max_constraint_;
    SensorModel sensor_;
};

} // namespace btm::sim

// Include implementation (header-only template for simplicity in this portfolio project)
#include "btm/sim/simulator.ipp"
