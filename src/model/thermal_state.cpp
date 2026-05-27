#include "btm/model/thermal_state.hpp"

#include <algorithm>

namespace btm::model {

core::Temperature ThermalState::max_cell_temp() const {
    return *std::max_element(cell_temperatures.begin(), cell_temperatures.end());
}

core::Temperature ThermalState::min_cell_temp() const {
    return *std::min_element(cell_temperatures.begin(), cell_temperatures.end());
}

core::Temperature ThermalState::delta_t() const {
    return max_cell_temp() - min_cell_temp();
}

} // namespace btm::model
