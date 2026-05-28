#include "btm/control/pid_controller.hpp"
#include "btm/model/thermal_state.hpp"
#include "btm/sim/sensor_model.hpp"

#include <gtest/gtest.h>

using namespace btm::control;
using namespace btm::core;
using namespace btm::model;
using namespace btm::sim;

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

// --- T4: Back-calculation anti-windup ----------------------------------------

TEST(PidController, BackCalculationPreventsWindup) {
    // Scenario (kp=0.5, ki=0.5, setpoint=30, kMax=2.0):
    //
    // During upper saturation (error=10):
    //   back-calc pins integrator at (kMax - kp*error)/ki = (2 - 5)/0.5 = -6.
    //   Without back-calc the integrator would accumulate to ~100 after 100 steps.
    //
    // When error drops to zero (at setpoint):
    //   back-calc integrator (-6):  u_raw = ki*(-6) = -3 → saturates LOW → output = kMin  ✓
    //   wound-up integrator (~100): u_raw = ki*(100) = 50 → saturates HIGH → output = kMax ✗
    //
    // The test asserts the back-calc outcome: output must be kMin immediately on
    // the first zero-error step after saturation, not kMax (no windup overshoot).
    PidController pid(0.5, 0.5, 30.0, 1000.0 /*limit; safety clamp must not interfere*/,
                      kMin, kMax);
    Duration dt{0.1};
    auto hot      = uniform_state(40.0);   // error = +10
    auto setpoint = uniform_state(30.0);   // error = 0

    for (int i = 0; i < 100; ++i)
        pid.compute_command(hot, kI, kTin, dt);

    // Immediately switch to setpoint: back-calc integrator is in [-6, -5],
    // so u_raw = kp*0 + ki*(≈-6) ≈ -3, which saturates at kMin.
    auto mdot_1 = pid.compute_command(setpoint, kI, kTin, dt);
    auto mdot_2 = pid.compute_command(setpoint, kI, kTin, dt);
    auto mdot_3 = pid.compute_command(setpoint, kI, kTin, dt);

    EXPECT_DOUBLE_EQ(mdot_1.value, kMin.value)
        << "Back-calculation must prevent windup; output must be kMin immediately "
           "at zero error after upper saturation";
    EXPECT_DOUBLE_EQ(mdot_2.value, kMin.value);
    EXPECT_DOUBLE_EQ(mdot_3.value, kMin.value);
}

// --- T4: Deadband ------------------------------------------------------------

TEST(PidController, DeadbandSuppressesSmallError) {
    // Deadband of 2 °C: errors smaller than 2 °C must produce no output change.
    // Initial integrator = 0, so output stays at kMin regardless of small error.
    const double deadband = 2.0;
    PidController pid(0.5, 0.01, 30.0, 50.0, kMin, kMax, deadband);
    Duration dt{0.1};

    // Error = 31.5 - 30 = 1.5 °C < 2.0 °C deadband → effective_error = 0
    auto barely_over = uniform_state(31.5);
    auto mdot = pid.compute_command(barely_over, kI, kTin, dt);
    EXPECT_DOUBLE_EQ(mdot.value, kMin.value)
        << "Error within deadband must produce no control action";

    // Run 100 steps with error in the deadband → integrator must stay at 0.
    // Verify by checking that output never rises above kMin.
    for (int i = 0; i < 100; ++i) {
        mdot = pid.compute_command(barely_over, kI, kTin, dt);
        EXPECT_DOUBLE_EQ(mdot.value, kMin.value)
            << "Integrator must not accumulate inside deadband (step " << i << ")";
    }
}

TEST(PidController, DeadbandDoesNotSuppressLargeError) {
    // Error = 35 - 30 = 5 °C >> deadband 2 °C → controller must respond normally.
    const double deadband = 2.0;
    PidController pid(0.5, 0.01, 30.0, 50.0, kMin, kMax, deadband);
    Duration dt{0.1};

    auto hot = uniform_state(35.0);
    auto mdot = pid.compute_command(hot, kI, kTin, dt);

    EXPECT_GT(mdot.value, kMin.value)
        << "Error outside deadband must produce a positive control action";
}

// --- T4: Downstream sensor ---------------------------------------------------

TEST(PidController, DownstreamSensorReactsToLastCell) {
    // Build a state where cell[0] is hot (would trigger perfect-sensor PID) but
    // cell[23] is at the setpoint.  A Downstream-sensor PID must output kMin.
    // A Perfect-sensor PID must output > kMin.

    ThermalState state;
    for (std::size_t i = 0; i < kNumNodes; ++i) {
        // cell[0] = 40 °C, cells 1-23 = 30 °C (setpoint)
        state.cell_temperatures[i]    = Temperature{i == 0 ? 40.0 : 30.0};
        state.coolant_temperatures[i] = Temperature{30.0};
    }

    const double sp = 30.0;
    Duration dt{0.1};

    PidController pid_perfect(0.5, 0.0, sp, 50.0, kMin, kMax, 0.0,
                               SensorModel::from_string("perfect"));
    PidController pid_downstream(0.5, 0.0, sp, 50.0, kMin, kMax, 0.0,
                                  SensorModel::from_string("downstream"));

    auto mdot_perfect    = pid_perfect.compute_command(state, kI, kTin, dt);
    auto mdot_downstream = pid_downstream.compute_command(state, kI, kTin, dt);

    // Perfect sensor sees cell[0] = 40 °C → error = +10 → output > kMin.
    EXPECT_GT(mdot_perfect.value, kMin.value);
    // Downstream sensor sees cell[23] = 30 °C → error = 0 → output = kMin.
    EXPECT_DOUBLE_EQ(mdot_downstream.value, kMin.value);
}
