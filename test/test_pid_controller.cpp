#include "btm/control/pid_controller.hpp"
#include "btm/model/thermal_state.hpp"

#include <gtest/gtest.h>

using namespace btm::control;
using namespace btm::core;
using namespace btm::model;

namespace {

// Build a ThermalState with all cells at the given temperature.
ThermalState uniform_state(double T_c) {
    ThermalState s;
    for (auto& t : s.cell_temperatures)    t = Temperature{T_c};
    for (auto& t : s.coolant_temperatures) t = Temperature{T_c};
    return s;
}

const MassFlowRate kMin{0.01};
const MassFlowRate kMax{2.0};
const Current      kI{22.5};   // 5C per cell (not used by PID but required by concept)
const Temperature  kTin{25.0}; // inlet temperature

} // anonymous namespace

// --- Tracking ----------------------------------------------------------------

TEST(PidController, ZeroErrorProducesMinimumFlow) {
    // At exactly the setpoint there should be no control action.
    // With zero error and zero integrator the PI output is 0, which clamps to mdot_min.
    PidController pid(0.05, 0.001, 30.0, 50.0, kMin, kMax);
    auto state = uniform_state(30.0);

    auto mdot = pid.compute_command(state, kI, kTin, Duration{0.1});
    EXPECT_DOUBLE_EQ(mdot.value, kMin.value);
}

TEST(PidController, OverTempProducesHigherFlow) {
    // When cells are above setpoint the controller must demand more cooling.
    PidController pid(0.05, 0.001, 30.0, 50.0, kMin, kMax);
    auto cold = uniform_state(30.0);
    auto hot  = uniform_state(35.0);  // 5 °C above setpoint

    auto mdot_cold = pid.compute_command(cold, kI, kTin, Duration{0.1});
    auto mdot_hot  = pid.compute_command(hot,  kI, kTin, Duration{0.1});

    EXPECT_GT(mdot_hot.value, mdot_cold.value)
        << "Hotter state must demand higher flow";
}

// --- Saturation --------------------------------------------------------------

TEST(PidController, OutputSaturatesAtMax) {
    // A huge proportional gain on a very hot state must saturate at mdot_max.
    PidController pid(100.0, 0.0, 25.0, 50.0, kMin, kMax);
    auto hot = uniform_state(40.0);

    auto mdot = pid.compute_command(hot, kI, kTin, Duration{0.1});
    EXPECT_DOUBLE_EQ(mdot.value, kMax.value);
}

TEST(PidController, OutputNeverExceedsMax) {
    PidController pid(0.5, 0.01, 28.0, 100.0, kMin, kMax);
    Duration dt{0.1};
    auto state = uniform_state(38.0);

    for (int i = 0; i < 100; ++i) {
        auto mdot = pid.compute_command(state, kI, kTin, dt);
        EXPECT_GE(mdot.value, kMin.value);
        EXPECT_LE(mdot.value, kMax.value);
    }
}

// --- Reset -------------------------------------------------------------------

TEST(PidController, ResetClearsIntegrator) {
    PidController pid(0.05, 0.1, 30.0, 50.0, kMin, kMax);
    Duration dt{0.1};
    auto hot = uniform_state(36.0);

    // Wind up the integrator over 50 steps
    for (int i = 0; i < 50; ++i)
        pid.compute_command(hot, kI, kTin, dt);

    pid.reset();

    // After reset, a step at exactly the setpoint must give minimum flow
    auto at_setpoint = uniform_state(30.0);
    auto mdot = pid.compute_command(at_setpoint, kI, kTin, dt);
    EXPECT_DOUBLE_EQ(mdot.value, kMin.value);
}

// --- Anti-windup (integrator clamping) ----------------------------------------

TEST(PidController, IntegratorDoesNotExplode) {
    // With a large ki and sustained error the integrator must be bounded.
    PidController pid(0.0, 1.0, 20.0, 10.0 /*limit*/, kMin, kMax);
    Duration dt{0.1};
    auto hot = uniform_state(40.0);   // error = +20 °C continuously

    for (int i = 0; i < 1000; ++i) {
        auto mdot = pid.compute_command(hot, kI, kTin, dt);
        EXPECT_LE(mdot.value, kMax.value);
        EXPECT_GE(mdot.value, kMin.value);
    }
}
