#include <gtest/gtest.h>
#include <cmath>
#include "logic/nav_tick.hpp"
#include "config.h"

constexpr double TOL = 1e-10;
constexpr float FTOL = 1e-6f;
constexpr double COS45 = 0.7071067811865476;
constexpr double SIN45 = 0.7071067811865476;

#define EXPECT_DOUBLE_NEAR(val1, val2) EXPECT_NEAR(val1, val2, TOL)
#define EXPECT_FLOAT_NEAR(val1, val2) EXPECT_NEAR(val1, val2, FTOL)

// ============================================================================
// Test Suite: NavTickUpdate_EncoderRecovery
// ============================================================================

TEST(NavTickUpdate_EncoderRecovery, ValidEncoderProducesCorrectDisplacement) {
    NavTickState state = {};
    SensorSnapshot snap = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_state_compact_t out = {};

    uint8_t flags = nav_tick_update(state, snap, &out);

    double expected_distance = 1.0 * ENCODER_WHEEL_RADIUS_M;
    EXPECT_FLOAT_NEAR(out.pos_x, expected_distance);
    EXPECT_FLOAT_NEAR(out.pos_y, 0.0f);
    EXPECT_FLOAT_NEAR(out.pos_z, 0.0f);
    EXPECT_EQ(flags, 0);
    EXPECT_EQ(state.encoder_fail_streak, 0);
}

TEST(NavTickUpdate_EncoderRecovery, FailedEncoderUsesLastGoodValue) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    SensorSnapshot snap1 = {
        .encoder_delta = 2.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap1, &out);

    SensorSnapshot snap2 = {
        .encoder_delta = 9999.0f,
        .encoder_valid = false,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    uint8_t flags = nav_tick_update(state, snap2, &out);

    double expected_distance = (2.0 + 2.0) * ENCODER_WHEEL_RADIUS_M;
    EXPECT_FLOAT_NEAR(out.pos_x, expected_distance);
    EXPECT_EQ(flags, NAV_FLAG_ENCODER_ESTIMATED);
    EXPECT_EQ(state.encoder_fail_streak, 1);
    EXPECT_FLOAT_NEAR(state.last_good_encoder_delta, 2.0f);
}

TEST(NavTickUpdate_EncoderRecovery, FailStreakResetsOnSuccess) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    SensorSnapshot snap1 = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap1, &out);

    SensorSnapshot snap_fail = {
        .encoder_delta = 0.0f,
        .encoder_valid = false,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_fail, &out);
    nav_tick_update(state, snap_fail, &out);
    nav_tick_update(state, snap_fail, &out);

    EXPECT_EQ(state.encoder_fail_streak, 3);

    SensorSnapshot snap2 = {
        .encoder_delta = 0.5f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    uint8_t flags = nav_tick_update(state, snap2, &out);

    EXPECT_EQ(state.encoder_fail_streak, 0);
    EXPECT_EQ(flags, 0);
}

TEST(NavTickUpdate_EncoderRecovery, FailStreakSaturatesAtUINT16MAX) {
    NavTickState state = {};
    state.encoder_fail_streak = UINT16_MAX - 1;
    nav_state_compact_t out = {};

    SensorSnapshot snap = {
        .encoder_delta = 0.0f,
        .encoder_valid = false,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    nav_tick_update(state, snap, &out);
    EXPECT_EQ(state.encoder_fail_streak, UINT16_MAX);

    nav_tick_update(state, snap, &out);
    EXPECT_EQ(state.encoder_fail_streak, UINT16_MAX);
}

TEST(NavTickUpdate_EncoderRecovery, CriticalThresholdSetsFlag) {
    NavTickState state = {};
    state.encoder_fail_streak = NAV_CRITICAL_FAIL_THRESHOLD - 1;
    nav_state_compact_t out = {};

    SensorSnapshot snap = {
        .encoder_delta = 0.0f,
        .encoder_valid = false,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    uint8_t flags = nav_tick_update(state, snap, &out);
    EXPECT_EQ(state.encoder_fail_streak, NAV_CRITICAL_FAIL_THRESHOLD);
    EXPECT_EQ(flags, NAV_FLAG_ENCODER_ESTIMATED | NAV_FLAG_NAV_CRITICAL);
}

// ============================================================================
// Test Suite: NavTickUpdate_IMURecovery
// ============================================================================

TEST(NavTickUpdate_IMURecovery, ValidIMUProducesCorrectRotation) {
    NavTickState state = {};
    SensorSnapshot snap = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {static_cast<float>(COS45), 0.0f, 0.0f, static_cast<float>(SIN45)},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_state_compact_t out = {};

    uint8_t flags = nav_tick_update(state, snap, &out);

    double expected_distance = 1.0 * ENCODER_WHEEL_RADIUS_M;
    EXPECT_FLOAT_NEAR(out.pos_x, 0.0f);
    EXPECT_FLOAT_NEAR(out.pos_y, expected_distance);
    EXPECT_FLOAT_NEAR(out.pos_z, 0.0f);
    EXPECT_EQ(flags, 0);
    EXPECT_EQ(state.imu_fail_streak, 0);
}

TEST(NavTickUpdate_IMURecovery, FailedIMUUsesLastGoodQuaternion) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    SensorSnapshot snap1 = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {static_cast<float>(COS45), 0.0f, 0.0f, static_cast<float>(SIN45)},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap1, &out);

    SensorSnapshot snap2 = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = false
    };
    uint8_t flags = nav_tick_update(state, snap2, &out);

    double expected_distance = 2.0 * ENCODER_WHEEL_RADIUS_M;
    EXPECT_FLOAT_NEAR(out.pos_x, 0.0f);
    EXPECT_FLOAT_NEAR(out.pos_y, expected_distance);
    EXPECT_EQ(flags, NAV_FLAG_IMU_ESTIMATED);
    EXPECT_EQ(state.imu_fail_streak, 1);
}

TEST(NavTickUpdate_IMURecovery, IdentityQuaternionDefaultOnFirstFailure) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    SensorSnapshot snap = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = false
    };
    uint8_t flags = nav_tick_update(state, snap, &out);

    double expected_distance = 1.0 * ENCODER_WHEEL_RADIUS_M;
    EXPECT_FLOAT_NEAR(out.pos_x, expected_distance);
    EXPECT_FLOAT_NEAR(out.pos_y, 0.0f);
    EXPECT_FLOAT_NEAR(out.pos_z, 0.0f);
    EXPECT_EQ(flags, NAV_FLAG_IMU_ESTIMATED);
}

TEST(NavTickUpdate_IMURecovery, FailStreakResetsOnSuccess) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    SensorSnapshot snap1 = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap1, &out);

    SensorSnapshot snap_fail = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = false
    };
    nav_tick_update(state, snap_fail, &out);
    nav_tick_update(state, snap_fail, &out);

    EXPECT_EQ(state.imu_fail_streak, 2);

    SensorSnapshot snap2 = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    uint8_t flags = nav_tick_update(state, snap2, &out);

    EXPECT_EQ(state.imu_fail_streak, 0);
    EXPECT_EQ(flags, 0);
}

TEST(NavTickUpdate_IMURecovery, FailStreakSaturatesAtUINT16MAX) {
    NavTickState state = {};
    state.imu_fail_streak = UINT16_MAX - 1;
    nav_state_compact_t out = {};

    SensorSnapshot snap = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = false
    };

    nav_tick_update(state, snap, &out);
    EXPECT_EQ(state.imu_fail_streak, UINT16_MAX);

    nav_tick_update(state, snap, &out);
    EXPECT_EQ(state.imu_fail_streak, UINT16_MAX);
}

TEST(NavTickUpdate_IMURecovery, CriticalThresholdSetsFlag) {
    NavTickState state = {};
    state.imu_fail_streak = NAV_CRITICAL_FAIL_THRESHOLD - 1;
    nav_state_compact_t out = {};

    SensorSnapshot snap = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = false
    };

    uint8_t flags = nav_tick_update(state, snap, &out);
    EXPECT_EQ(state.imu_fail_streak, NAV_CRITICAL_FAIL_THRESHOLD);
    EXPECT_EQ(flags, NAV_FLAG_IMU_ESTIMATED | NAV_FLAG_NAV_CRITICAL);
}

// ============================================================================
// Test Suite: NavTickUpdate_Pipeline
// ============================================================================

TEST(NavTickUpdate_Pipeline, ForwardMotionIdentityQuaternion) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    SensorSnapshot snap = {
        .encoder_delta = 2.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    nav_tick_update(state, snap, &out);

    double expected_distance = 2.0 * ENCODER_WHEEL_RADIUS_M;
    EXPECT_FLOAT_NEAR(out.pos_x, expected_distance);
    EXPECT_FLOAT_NEAR(out.pos_y, 0.0f);
    EXPECT_FLOAT_NEAR(out.pos_z, 0.0f);
    EXPECT_FLOAT_NEAR(out.angular_delta, 2.0f);
}

TEST(NavTickUpdate_Pipeline, NinetyDegreeYawRotationChangesDirection) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    SensorSnapshot snap = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {static_cast<float>(COS45), 0.0f, 0.0f, static_cast<float>(SIN45)},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    nav_tick_update(state, snap, &out);

    double expected_distance = 1.0 * ENCODER_WHEEL_RADIUS_M;
    EXPECT_NEAR(out.pos_x, 0.0f, FTOL);
    EXPECT_FLOAT_NEAR(out.pos_y, expected_distance);
    EXPECT_NEAR(out.pos_z, 0.0f, FTOL);
}

TEST(NavTickUpdate_Pipeline, ZeroEncoderDeltaNoDisplacement) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    SensorSnapshot snap = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    nav_tick_update(state, snap, &out);
    nav_tick_update(state, snap, &out);
    nav_tick_update(state, snap, &out);

    EXPECT_FLOAT_NEAR(out.pos_x, 0.0f);
    EXPECT_FLOAT_NEAR(out.pos_y, 0.0f);
    EXPECT_FLOAT_NEAR(out.pos_z, 0.0f);
    EXPECT_EQ(state.tick_count, 3U);
}

TEST(NavTickUpdate_Pipeline, DoublePrecisionMaintainedOver10000Ticks) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    SensorSnapshot snap = {
        .encoder_delta = 0.001f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    for (int i = 0; i < 10000; ++i) {
        nav_tick_update(state, snap, &out);
    }

    double expected_distance = 10000.0 * 0.001 * ENCODER_WHEEL_RADIUS_M;
    EXPECT_NEAR(state.pos_x, expected_distance, 1e-8);
    EXPECT_DOUBLE_NEAR(state.pos_y, 0.0);
    EXPECT_DOUBLE_NEAR(state.pos_z, 0.0);
    EXPECT_EQ(state.tick_count, 10000U);

    EXPECT_NEAR(out.pos_x, static_cast<float>(expected_distance), FTOL);
}

TEST(NavTickUpdate_Pipeline, CompactOutputCorrectlyDowncastsDoubles) {
    NavTickState state = {};
    state.pos_x = 123.456789012345;
    state.pos_y = -987.654321098765;
    state.pos_z = 0.123456789012345;

    nav_state_compact_t out = {};

    SensorSnapshot snap = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    nav_tick_update(state, snap, &out);

    EXPECT_FLOAT_NEAR(out.pos_x, 123.456789f);
    EXPECT_FLOAT_NEAR(out.pos_y, -987.654321f);
    EXPECT_FLOAT_NEAR(out.pos_z, 0.123456789f);
}

TEST(NavTickUpdate_Pipeline, QuaternionPassthroughToOutput) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    SensorSnapshot snap = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {0.8f, 0.1f, 0.2f, 0.3f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    nav_tick_update(state, snap, &out);

    EXPECT_FLOAT_NEAR(out.quat_w, 0.8f);
    EXPECT_FLOAT_NEAR(out.quat_x, 0.1f);
    EXPECT_FLOAT_NEAR(out.quat_y, 0.2f);
    EXPECT_FLOAT_NEAR(out.quat_z, 0.3f);
}

// ============================================================================
// Test Suite: NavTickUpdate_StatusFlags
// ============================================================================

TEST(NavTickUpdate_StatusFlags, NoFailuresZeroFlags) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    SensorSnapshot snap = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    uint8_t flags = nav_tick_update(state, snap, &out);

    EXPECT_EQ(flags, 0);
    EXPECT_EQ(out.status_flags, 0);
}

TEST(NavTickUpdate_StatusFlags, EncoderOnlyFailureEncoderEstimated) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    SensorSnapshot snap = {
        .encoder_delta = 0.0f,
        .encoder_valid = false,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    uint8_t flags = nav_tick_update(state, snap, &out);

    EXPECT_EQ(flags, NAV_FLAG_ENCODER_ESTIMATED);
    EXPECT_EQ(out.status_flags, NAV_FLAG_ENCODER_ESTIMATED);
}

TEST(NavTickUpdate_StatusFlags, IMUOnlyFailureIMUEstimated) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    SensorSnapshot snap = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = false
    };

    uint8_t flags = nav_tick_update(state, snap, &out);

    EXPECT_EQ(flags, NAV_FLAG_IMU_ESTIMATED);
    EXPECT_EQ(out.status_flags, NAV_FLAG_IMU_ESTIMATED);
}

TEST(NavTickUpdate_StatusFlags, BothSensorsFailedBothFlagsSet) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    SensorSnapshot snap = {
        .encoder_delta = 0.0f,
        .encoder_valid = false,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = false
    };

    uint8_t flags = nav_tick_update(state, snap, &out);

    EXPECT_EQ(flags, NAV_FLAG_ENCODER_ESTIMATED | NAV_FLAG_IMU_ESTIMATED);
    EXPECT_EQ(out.status_flags, NAV_FLAG_ENCODER_ESTIMATED | NAV_FLAG_IMU_ESTIMATED);
}

TEST(NavTickUpdate_StatusFlags, CriticalThresholdAddsNavCriticalFlag) {
    NavTickState state = {};
    state.encoder_fail_streak = NAV_CRITICAL_FAIL_THRESHOLD;
    nav_state_compact_t out = {};

    SensorSnapshot snap = {
        .encoder_delta = 0.0f,
        .encoder_valid = false,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    uint8_t flags = nav_tick_update(state, snap, &out);

    EXPECT_EQ(flags, NAV_FLAG_ENCODER_ESTIMATED | NAV_FLAG_NAV_CRITICAL);
    EXPECT_EQ(out.status_flags, NAV_FLAG_ENCODER_ESTIMATED | NAV_FLAG_NAV_CRITICAL);
}

TEST(NavTickUpdate_StatusFlags, FlagsClearWhenSensorsRecover) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    SensorSnapshot snap_fail = {
        .encoder_delta = 0.0f,
        .encoder_valid = false,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = false
    };
    nav_tick_update(state, snap_fail, &out);

    EXPECT_EQ(out.status_flags, NAV_FLAG_ENCODER_ESTIMATED | NAV_FLAG_IMU_ESTIMATED);

    SensorSnapshot snap_good = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    uint8_t flags = nav_tick_update(state, snap_good, &out);

    EXPECT_EQ(flags, 0);
    EXPECT_EQ(out.status_flags, 0);
}

// ============================================================================
// Test Suite: NavTickUpdate_TickCount
// ============================================================================

TEST(NavTickUpdate_TickCount, IncrementsEachCall) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    SensorSnapshot snap = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    EXPECT_EQ(state.tick_count, 0U);

    nav_tick_update(state, snap, &out);
    EXPECT_EQ(state.tick_count, 1U);

    nav_tick_update(state, snap, &out);
    EXPECT_EQ(state.tick_count, 2U);

    nav_tick_update(state, snap, &out);
    EXPECT_EQ(state.tick_count, 3U);
}

TEST(NavTickUpdate_TickCount, CorrectAfterManyIterations) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    SensorSnapshot snap = {
        .encoder_delta = 0.01f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    for (uint32_t i = 0; i < 100; ++i) {
        nav_tick_update(state, snap, &out);
    }

    EXPECT_EQ(state.tick_count, 100U);
}

TEST(NavTickUpdate_TickCount, IncrementsEvenWithFailedSensors) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    SensorSnapshot snap = {
        .encoder_delta = 0.0f,
        .encoder_valid = false,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = false
    };

    nav_tick_update(state, snap, &out);
    EXPECT_EQ(state.tick_count, 1U);

    nav_tick_update(state, snap, &out);
    EXPECT_EQ(state.tick_count, 2U);
}

// ============================================================================
// Test Suite: NavTickUpdate_EdgeCases
// ============================================================================

TEST(NavTickUpdate_EdgeCases, NegativeEncoderDeltaMovesBackward) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    SensorSnapshot snap = {
        .encoder_delta = -2.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    nav_tick_update(state, snap, &out);

    double expected_distance = -2.0 * ENCODER_WHEEL_RADIUS_M;
    EXPECT_FLOAT_NEAR(out.pos_x, expected_distance);
    EXPECT_FLOAT_NEAR(out.pos_y, 0.0f);
    EXPECT_FLOAT_NEAR(out.pos_z, 0.0f);
}

TEST(NavTickUpdate_EdgeCases, MultipleRotationsAccumulate) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    SensorSnapshot snap1 = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap1, &out);

    SensorSnapshot snap2 = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {static_cast<float>(COS45), 0.0f, 0.0f, static_cast<float>(SIN45)},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap2, &out);

    double expected_distance = ENCODER_WHEEL_RADIUS_M;
    EXPECT_FLOAT_NEAR(out.pos_x, expected_distance);
    EXPECT_FLOAT_NEAR(out.pos_y, expected_distance);
    EXPECT_NEAR(out.pos_z, 0.0f, FTOL);
}

TEST(NavTickUpdate_EdgeCases, LargePositionValues) {
    NavTickState state = {};
    state.pos_x = 1e6;
    state.pos_y = -5e5;
    state.pos_z = 3e5;

    nav_state_compact_t out = {};

    SensorSnapshot snap = {
        .encoder_delta = 0.001f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    nav_tick_update(state, snap, &out);

    double expected_x = 1e6 + 0.001 * ENCODER_WHEEL_RADIUS_M;
    EXPECT_DOUBLE_NEAR(state.pos_x, expected_x);
    EXPECT_DOUBLE_NEAR(state.pos_y, -5e5);
    EXPECT_DOUBLE_NEAR(state.pos_z, 3e5);
}
