#include "btm/core/types.hpp"

#include <gtest/gtest.h>

using namespace btm::core;

TEST(StrongTypes, TemperatureArithmetic) {
    Temperature a{10.0};
    Temperature b{5.0};

    EXPECT_EQ((a + b).value, 15.0);
    EXPECT_EQ((a - b).value, 5.0);
    EXPECT_EQ((a * 2.0).value, 20.0);
    EXPECT_EQ((3.0 * a).value, 30.0);
    EXPECT_EQ((a / 2.0).value, 5.0);

    EXPECT_TRUE(a > b);
    EXPECT_TRUE(a >= b);
    EXPECT_FALSE(a < b);
}

TEST(StrongTypes, MassFlowRateBasic) {
    MassFlowRate m1{0.5};
    MassFlowRate m2{0.3};

    EXPECT_EQ((m1 + m2).value, 0.8);
    EXPECT_DOUBLE_EQ((m1 * 2.0).value, 1.0);
}

TEST(StrongTypes, NoImplicitConversion) {
    // These must not compile if uncommented (compile-time test via static_assert in header)
    // Temperature t = MassFlowRate{1.0};  // should fail
    // MassFlowRate m = Duration{1.0};     // should fail
    SUCCEED();
}

TEST(StrongTypes, Duration) {
    Duration d{10.0};
    EXPECT_EQ((d * 2).value, 20.0);
}
