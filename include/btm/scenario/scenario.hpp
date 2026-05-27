#pragma once

#include "btm/core/types.hpp"
#include "btm/config/config.hpp"

#include <functional>

namespace btm::scenario {

/// The two time-varying exogenous inputs fed to the simulator each step.
struct ScenarioFunctions {
    /// Per-cell current as a function of elapsed simulation time.
    std::function<core::Current(core::Duration)> current_at;

    /// Coolant inlet temperature as a function of elapsed simulation time.
    std::function<core::Temperature(core::Duration)> inlet_at;
};

/// Build the scenario from a validated Config.
/// The returned closures are self-contained and capture all parameters by value.
ScenarioFunctions make_scenario(const config::Config& cfg);

} // namespace btm::scenario
