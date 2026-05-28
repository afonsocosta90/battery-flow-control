#include "btm/sim/sensor_model.hpp"
#include "btm/model/thermal_state.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

using namespace btm::sim;
using namespace btm::model;
using namespace btm::core;

namespace {

// Build a ThermalState where each cell has a unique temperature.
// cell_temperatures[i] = base + i * step
ThermalState gradient_state(double base = 25.0, double step = 0.5) {
    ThermalState s;
    for (std::size_t i = 0; i < kNumNodes; ++i) {
        s.cell_temperatures[i]    = Temperature{base + static_cast<double>(i) * step};
        s.coolant_temperatures[i] = Temperature{base};
    }
    return s;
}

// Build a uniform state (all cells at the same temperature).
ThermalState uniform_state(double T_c) {
    ThermalState s;
    for (auto& t : s.cell_temperatures)    t = Temperature{T_c};
    for (auto& t : s.coolant_temperatures) t = Temperature{T_c};
    return s;
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Mode: Perfect
// ---------------------------------------------------------------------------

TEST(SensorModel, PerfectObservesGlobalMax) {
    // The true maximum is the last cell in a monotonically rising state.
    auto state = gradient_state(25.0, 0.5);   // cell[0]=25.0 … cell[23]=36.5

    const auto sensor = SensorModel::from_string("perfect");
    const double observed = sensor.observed_max(state).value;
    const double true_max = state.max_cell_temp().value;

    EXPECT_DOUBLE_EQ(observed, true_max);
}

TEST(SensorModel, PerfectObservesGlobalMin) {
    auto state = gradient_state(25.0, 0.5);

    const auto sensor = SensorModel::from_string("perfect");
    EXPECT_DOUBLE_EQ(sensor.observed_min(state).value,
                     state.min_cell_temp().value);
}

TEST(SensorModel, PerfectDeltaTMatchesTrueSpread) {
    auto state = gradient_state(25.0, 0.5);

    const auto sensor = SensorModel::from_string("perfect");
    EXPECT_DOUBLE_EQ(sensor.observed_delta_t(state).value,
                     state.delta_t().value);
}

// ---------------------------------------------------------------------------
// Mode: Downstream (position kNumNodes-1 = 23)
// ---------------------------------------------------------------------------

TEST(SensorModel, DownstreamObservesLastPosition) {
    // Reverse gradient: cell[0] is the hottest, cell[23] is the coolest.
    // Downstream mode must still return cell[23], not the global max.
    ThermalState state;
    for (std::size_t i = 0; i < kNumNodes; ++i) {
        // Decreasing: cell[0]=36.5, cell[23]=25.0
        state.cell_temperatures[i]    = Temperature{36.5 - static_cast<double>(i) * 0.5};
        state.coolant_temperatures[i] = Temperature{25.0};
    }

    const auto sensor = SensorModel::from_string("downstream");
    const double observed = sensor.observed_max(state).value;
    const double expected = state.cell_temperatures[kNumNodes - 1].value;  // 25.0

    EXPECT_DOUBLE_EQ(observed, expected);

    // Confirm it is NOT the global max (which is cell[0] = 36.5)
    EXPECT_LT(observed, state.max_cell_temp().value);
}

TEST(SensorModel, DownstreamDeltaTIsZero) {
    // Single-sensor mode: min == max, so observed delta_T == 0.
    auto state = gradient_state(25.0, 0.5);

    const auto sensor = SensorModel::from_string("downstream");
    EXPECT_DOUBLE_EQ(sensor.observed_delta_t(state).value, 0.0);
}

// ---------------------------------------------------------------------------
// Mode: Sparse
// ---------------------------------------------------------------------------

TEST(SensorModel, SparseObservesMaxOfListedPositions) {
    // Gradient state: cell[i] = 25.0 + 0.5*i.
    // Observe positions {2, 7, 15}: max should be cell[15] = 25 + 7.5 = 32.5
    auto state = gradient_state(25.0, 0.5);

    const auto sensor = SensorModel::from_string("sparse", {2, 7, 15});

    const double expected = state.cell_temperatures[15].value;  // 32.5
    EXPECT_DOUBLE_EQ(sensor.observed_max(state).value, expected);
}

TEST(SensorModel, SparseObservedNeverExceedsTrueMax) {
    // The sparse max (subset) can never be larger than the global max.
    auto state = gradient_state(20.0, 1.0);   // cell[23]=43

    for (const auto& positions : std::vector<std::vector<int>>{
            {0}, {0, 12}, {5, 10, 20}, {0, 1, 2, 22}}) {
        const auto sensor = SensorModel::from_string("sparse", positions);
        EXPECT_LE(sensor.observed_max(state).value, state.max_cell_temp().value);
    }
}

TEST(SensorModel, SparseInvalidPositionThrows) {
    // Position out of range [0, kNumNodes-1] must throw.
    EXPECT_THROW(SensorModel::from_string("sparse", {0, 24}),  std::runtime_error);
    EXPECT_THROW(SensorModel::from_string("sparse", {-1}),      std::runtime_error);
}

TEST(SensorModel, UnknownModeThrows) {
    EXPECT_THROW(SensorModel::from_string("magic"), std::runtime_error);
}

// ---------------------------------------------------------------------------
// Uniform state edge case
// ---------------------------------------------------------------------------

TEST(SensorModel, AllModesAgreeOnUniformState) {
    auto state = uniform_state(30.0);

    const auto perfect    = SensorModel::from_string("perfect");
    const auto downstream = SensorModel::from_string("downstream");
    const auto sparse     = SensorModel::from_string("sparse", {0, 12, 23});

    EXPECT_DOUBLE_EQ(perfect.observed_max(state).value, 30.0);
    EXPECT_DOUBLE_EQ(downstream.observed_max(state).value, 30.0);
    EXPECT_DOUBLE_EQ(sparse.observed_max(state).value, 30.0);
}
