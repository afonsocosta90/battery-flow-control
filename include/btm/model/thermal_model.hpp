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
    explicit ThermalModel(const config::Config& cfg);

    // Pure step: (state, ṁ, I_cell_per_cell, T_inlet, dt) → next_state
    [[nodiscard]] ThermalState step(const ThermalState& state,
                                    core::MassFlowRate mdot,
                                    core::Current I_cell,
                                    core::Temperature T_inlet,
                                    core::Duration dt) const;

    // Returns the coolant temperatures for the given state + mdot + T_inlet.
    [[nodiscard]] std::array<core::Temperature, kNumNodes>
    coolant_temperatures(const ThermalState& state,
                         core::MassFlowRate mdot,
                         core::Temperature T_inlet) const;

    /// Convective coefficient h [W/(m²·K)] at the given mass-flow rate.
    ///
    /// In power_law mode:  h = h_ref · (ṁ / ṁ_ref)^n
    /// In nusselt_correlation mode:
    ///   Re  = ṁ · D_h / (A_flow · μ)
    ///   Pr  = μ · cp / k_coolant
    ///   Nu  = c · Re^m · Pr^n
    ///   h   = Nu · k_coolant / D_h
    [[nodiscard]] double h_of_mdot(double mdot) const;

private:
    // Cell / node parameters
    double C_th_{0.0};           ///< thermal capacity per node (J/K) = m_cell·cp_cell
    double A_{0.0};              ///< surface area per cell (m²)
    double eta_ir_1c_v_{0.0};
    double I_1C_{0.0};           ///< 1C current = capacity in A
    double coolant_Cp_{0.0};     ///< J/(kg·K)

    // Power-law convection parameters
    double h_ref_{0.0};
    double m_dot_ref_{0.0};
    double n_{0.0};

    // Nusselt-correlation parameters (only used when use_nusselt_ = true)
    bool   use_nusselt_{false};
    double coolant_mu_{0.0};     ///< dynamic viscosity (Pa·s)
    double coolant_k_{0.0};      ///< thermal conductivity (W/(m·K))
    double d_hydraulic_{0.0};    ///< hydraulic diameter (m)
    double flow_area_{0.0};      ///< effective total cross-sectional flow area (m²)
    double nusselt_c_{0.197};
    double nusselt_m_{0.333};
    double nusselt_n_{0.333};

    // Fixed 3 iterations as specified in DESIGN.md
    static constexpr int kCoolantIterations = 3;
};

} // namespace btm::model
