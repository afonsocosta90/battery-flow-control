#pragma once

#include "btm/core/types.hpp"
#include "btm/model/thermal_state.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace btm::sim {

/// Configurable sensor model for the thermal simulation.
///
/// Decouples the true plant state from what any controller or logging path can
/// actually observe.  Three modes are supported:
///
///   Perfect     — returns the true global max / delta_T.  Backward-compatible
///                 default; matches all pre-T1 behaviour exactly.
///
///   Downstream  — returns the temperature of the last series position (index 23),
///                 the physically hottest location under serial coolant flow.  The
///                 most defensible single-sensor placement for a real pack.
///
///   Sparse      — returns the max of a specified subset of positions.  Positions
///                 are validated to be in [0, kNumNodes-1] at construction.
///
/// Header-only — no .cpp needed.
struct SensorModel {
    enum class Mode { Perfect, Downstream, Sparse };

    Mode             mode{Mode::Perfect};
    std::vector<int> positions{};   ///< used by Sparse mode only

    // -------------------------------------------------------------------------
    // Convenience factory from a YAML-style string
    // -------------------------------------------------------------------------
    static SensorModel from_string(const std::string& s,
                                   const std::vector<int>& pos = {}) {
        if (s == "perfect")    return SensorModel{Mode::Perfect,    {}};
        if (s == "downstream") return SensorModel{Mode::Downstream, {}};
        if (s == "sparse") {
            for (int p : pos) {
                if (p < 0 || p >= static_cast<int>(model::kNumNodes)) {
                    throw std::runtime_error(
                        "sensor.positions: index " + std::to_string(p) +
                        " is out of range [0, " +
                        std::to_string(model::kNumNodes - 1) + "]");
                }
            }
            return SensorModel{Mode::Sparse, pos};
        }
        throw std::runtime_error("Unknown sensor.mode: '" + s + "'. "
                                 "Must be 'perfect', 'downstream', or 'sparse'.");
    }

    // -------------------------------------------------------------------------
    // Observation interface
    // -------------------------------------------------------------------------

    /// Observed maximum cell temperature.
    [[nodiscard]] core::Temperature
    observed_max(const model::ThermalState& state) const {
        switch (mode) {
        case Mode::Perfect:
            return state.max_cell_temp();
        case Mode::Downstream:
            return state.cell_temperatures[model::kNumNodes - 1];
        case Mode::Sparse: {
            if (positions.empty()) return state.max_cell_temp();
            core::Temperature best = state.cell_temperatures[positions[0]];
            for (std::size_t i = 1; i < positions.size(); ++i) {
                const auto t = state.cell_temperatures[positions[i]];
                if (t.value > best.value) best = t;
            }
            return best;
        }
        }
        return state.max_cell_temp();  // unreachable — silences compiler
    }

    /// Observed minimum cell temperature (used for delta_T computation).
    ///
    /// For Perfect / Downstream modes the true global minimum is used; for Sparse
    /// the minimum is taken over the same sensor positions as the maximum.
    [[nodiscard]] core::Temperature
    observed_min(const model::ThermalState& state) const {
        switch (mode) {
        case Mode::Perfect:
            return state.min_cell_temp();
        case Mode::Downstream:
            // Single sensor: min == max (no spread observable)
            return state.cell_temperatures[model::kNumNodes - 1];
        case Mode::Sparse: {
            if (positions.empty()) return state.min_cell_temp();
            core::Temperature best = state.cell_temperatures[positions[0]];
            for (std::size_t i = 1; i < positions.size(); ++i) {
                const auto t = state.cell_temperatures[positions[i]];
                if (t.value < best.value) best = t;
            }
            return best;
        }
        }
        return state.min_cell_temp();
    }

    /// Observed inter-cell temperature spread (max_observed - min_observed).
    [[nodiscard]] core::Temperature
    observed_delta_t(const model::ThermalState& state) const {
        return core::Temperature{observed_max(state).value -
                                 observed_min(state).value};
    }
};

} // namespace btm::sim
