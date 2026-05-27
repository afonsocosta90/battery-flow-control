#pragma once

#include "btm/core/types.hpp"

#include <array>
#include <cstddef>

namespace btm::model {

inline constexpr std::size_t kNumNodes = 24;

struct ThermalState {
    std::array<core::Temperature, kNumNodes> cell_temperatures{};
    std::array<core::Temperature, kNumNodes> coolant_temperatures{};

    core::Temperature max_cell_temp() const;
    core::Temperature min_cell_temp() const;
    core::Temperature delta_t() const;   // max_cell - min_cell
};

} // namespace btm::model
