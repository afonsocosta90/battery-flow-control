#pragma once

#include "btm/core/types.hpp"
#include "btm/model/thermal_state.hpp"

#include <array>
#include <cstddef>

namespace btm::config { struct Config; }

namespace btm::model {

class ThermalModel {
public:
    // Constructed once from a validated Config.
    // The model holds the parameters it needs (or a const reference).
    explicit ThermalModel(const config::Config& cfg);

    // Pure step: (state, ṁ, I_cell_per_cell, T_inlet, dt) → next_state
    // T_inlet is the current exogenous inlet coolant temperature for this step.
    [[nodiscard]] ThermalState step(const ThermalState& state,
                                    core::MassFlowRate mdot,
                                    core::Current I_cell,           // current through one cell
                                    core::Temperature T_inlet,
                                    core::Duration dt) const;

    // Returns the coolant temperatures that would result from the given state + mdot + T_inlet.
    // Useful for logging, analysis, and MPC cost evaluation.
    [[nodiscard]] std::array<core::Temperature, kNumNodes>
    coolant_temperatures(const ThermalState& state,
                         core::MassFlowRate mdot,
                         core::Temperature T_inlet) const;

private:
    // Parameters extracted / referenced at construction time
    double C_th_{0.0};                 // thermal capacity per node (J/K)
    double A_{0.0};                    // surface area per cell (m²)
    double eta_ir_1c_v_{0.0};
    double I_1C_{0.0};                 // 1C current = capacity in A
    double coolant_Cp_{0.0};           // J/(kg·K)
    double h_ref_{0.0};
    double m_dot_ref_{0.0};
    double n_{0.0};

    // Fixed 3 iterations as specified in DESIGN.md
    static constexpr int kCoolantIterations = 3;
};

} // namespace btm::model
