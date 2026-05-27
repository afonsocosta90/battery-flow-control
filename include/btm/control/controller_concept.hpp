#pragma once

#include "btm/core/types.hpp"
#include "btm/model/thermal_state.hpp"

namespace btm::control {

/// Controller concept: any type satisfying this can be plugged into Simulator<Controller>.
///
/// compute_command receives:
///   - the current ThermalState (24 cell temperatures, 24 coolant temperatures)
///   - the current per-cell current I_cell  — used by MPC for horizon rollouts;
///     the no-preview policy means this is the instantaneous value, not a future preview
///   - the current inlet coolant temperature T_inlet — same rationale
///   - dt, the simulation timestep
///
/// reset() must clear all integrators and warm-start buffers.
/// Zero virtual functions; controller type is resolved once in main().
template <typename C>
concept Controller =
    requires(C& c,
             const model::ThermalState& s,
             core::Current I_cell,
             core::Temperature T_inlet,
             core::Duration dt)
    {
        { c.compute_command(s, I_cell, T_inlet, dt) }
            -> std::convertible_to<core::MassFlowRate>;
        { c.reset() } -> std::same_as<void>;
    };

} // namespace btm::control
