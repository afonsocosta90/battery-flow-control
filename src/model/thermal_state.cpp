#include "btm/model/thermal_state.hpp"

#include <algorithm>

namespace btm::model {

// ── Can / surface temperature accessors ─────────────────────────────────────

core::Temperature ThermalState::max_cell_temp() const {
    return *std::max_element(cell_temperatures.begin(), cell_temperatures.end());
}

core::Temperature ThermalState::min_cell_temp() const {
    return *std::min_element(cell_temperatures.begin(), cell_temperatures.end());
}

core::Temperature ThermalState::delta_t() const {
    return max_cell_temp() - min_cell_temp();
}

// ── Core temperature accessors ───────────────────────────────────────────────

core::Temperature ThermalState::max_core_temp() const {
    return *std::max_element(core_temperatures.begin(), core_temperatures.end());
}

core::Temperature ThermalState::min_core_temp() const {
    return *std::min_element(core_temperatures.begin(), core_temperatures.end());
}

core::Temperature ThermalState::core_to_can_delta_t() const {
    // Maximum (T_core_i - T_can_i) across all positions.
    // Zero in single-node mode; > 0 under load in two-node mode.
    double max_delta = 0.0;
    for (std::size_t i = 0; i < kNumNodes; ++i) {
        const double delta = core_temperatures[i].value - cell_temperatures[i].value;
        if (delta > max_delta) max_delta = delta;
    }
    return core::Temperature{max_delta};
}

} // namespace btm::model
