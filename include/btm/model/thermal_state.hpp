#pragma once

#include "btm/core/types.hpp"

#include <array>
#include <cstddef>

namespace btm::model {

inline constexpr std::size_t kNumNodes = 24;

/// Lumped thermal state of the 24-node module.
///
/// cell_temperatures  — CAN / surface temperature at each serial position (T_can).
///                      Externally observable; used by sensor model and coolant chain.
///                      In single-node mode this equals core_temperatures.
///
/// core_temperatures  — CORE temperature at each serial position (T_core).
///                      In single-node mode: core_temperatures == cell_temperatures.
///                      In two-node mode: T_core > T_can at any positive heat load.
///                      Safety constraints are enforced on core_temperatures.
///
/// coolant_temperatures — Algebraic coolant chain temperatures.
struct ThermalState {
    std::array<core::Temperature, kNumNodes> cell_temperatures{};    ///< T_can (surface)
    std::array<core::Temperature, kNumNodes> core_temperatures{};    ///< T_core (internal)
    std::array<core::Temperature, kNumNodes> coolant_temperatures{};

    // ── Can / surface temperature accessors (externally observable) ──────────
    core::Temperature max_cell_temp() const;
    core::Temperature min_cell_temp() const;
    core::Temperature delta_t() const;   // max_cell - min_cell (can temps)

    // ── Core temperature accessors (safety-critical) ─────────────────────────
    core::Temperature max_core_temp() const;
    core::Temperature min_core_temp() const;

    /// Maximum core-to-can temperature difference across all positions.
    /// Zero in single-node mode; positive in two-node mode under load.
    core::Temperature core_to_can_delta_t() const;
};

} // namespace btm::model
