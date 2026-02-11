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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap1, &out);

    // Encoder fails but IMU shows motion → extrapolate at last velocity
    SensorSnapshot snap2 = {
        .encoder_delta = 9999.0f,
        .encoder_valid = false,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.5f, 0.0f, 0.0f},  // IMU shows forward motion
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap1, &out);

    SensorSnapshot snap_fail = {
        .encoder_delta = 0.0f,
        .encoder_valid = false,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    uint8_t flags = nav_tick_update(state, snap, &out);
    EXPECT_EQ(state.encoder_fail_streak, NAV_CRITICAL_FAIL_THRESHOLD);
    EXPECT_EQ(flags, NAV_FLAG_ENCODER_ESTIMATED | NAV_FLAG_NAV_CRITICAL | NAV_FLAG_ENCODER_LOST);
}

// ============================================================================
// Test Suite: NavTickUpdate_EncoderTieredRecovery
// ============================================================================

// Helper: Create a valid sensor snapshot with identity quaternion
static SensorSnapshot make_good_snap(float encoder_delta) {
    return {
        .encoder_delta = encoder_delta,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
}

// Helper: Create a failed encoder snapshot (IMU valid but stationary)
// Note: With IMU-informed recovery, this will trigger immediate hard stop!
static SensorSnapshot make_encoder_fail_snap() {
    return {
        .encoder_delta = 0.0f,
        .encoder_valid = false,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
}

// Helper: Create a failed encoder snapshot with IMU showing motion
// Use this to test encoder recovery when device is still moving
static SensorSnapshot make_encoder_fail_snap_with_motion() {
    return {
        .encoder_delta = 0.0f,
        .encoder_valid = false,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.5f, 0.0f, 0.0f},  // Forward acceleration above threshold
        .imu_valid = true
    };
}

// Helper: Create a failed encoder snapshot with IMU also failed (blind recovery)
static SensorSnapshot make_encoder_fail_snap_blind() {
    return {
        .encoder_delta = 0.0f,
        .encoder_valid = false,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = false  // IMU also failed → use blind recovery
    };
}

TEST(NavTickUpdate_EncoderTieredRecovery, Tier1_ExtrapolatesAtConstantVelocity) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // One good read with encoder_delta=2.0f → velocity = (2.0*0.025)/0.01 = 5.0 m/s
    nav_tick_update(state, make_good_snap(2.0f), &out);

    // Run 9 failures with IMU showing motion → delta_dist = 5.0 * 0.01 = 0.05 each
    for (unsigned i = 0; i < 9; ++i) {
        nav_tick_update(state, make_encoder_fail_snap_with_motion(), &out);
    }

    // After 10 ticks total (1 good + 9 fail), pos_x = 10 * 0.05 = 0.5
    EXPECT_NEAR(state.pos_x, 10.0 * 0.05, 1e-6);
}

TEST(NavTickUpdate_EncoderTieredRecovery, Tier1_VelocityComputedCorrectly) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Good read with encoder_delta=3.0f → velocity = (3.0*0.025)/0.01 = 7.5 m/s
    nav_tick_update(state, make_good_snap(3.0f), &out);

    EXPECT_FLOAT_NEAR(state.last_velocity, 7.5f);
}

TEST(NavTickUpdate_EncoderTieredRecovery, Tier2_FirstDecayAtBoundary) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Good read with encoder_delta=2.0f → velocity=5.0
    nav_tick_update(state, make_good_snap(2.0f), &out);

    // Run 9 Tier 1 failures with blind recovery (IMU also failed)
    // Note: After 5 ticks, IMU throttle (0.5x) kicks in due to imu_fail_streak >= NAV_IMU_TIER1_THRESHOLD
    for (unsigned i = 0; i < 9; ++i) {
        nav_tick_update(state, make_encoder_fail_snap_blind(), &out);
    }

    // Save pos_x before the 10th failure tick
    double pos_before = state.pos_x;

    // 10th failure → Tier 2 entry: velocity becomes 5.0*0.8=4.0, delta_dist=4.0*0.01=0.04
    // But IMU throttle (0.5x) is applied: delta_dist = 0.04 * 0.5 = 0.02
    nav_tick_update(state, make_encoder_fail_snap_blind(), &out);

    double delta_for_tick_10 = state.pos_x - pos_before;
    EXPECT_NEAR(delta_for_tick_10, 0.02, 1e-6);
}

TEST(NavTickUpdate_EncoderTieredRecovery, Tier2_ProgressiveDecay) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Good read with encoder_delta=2.0f → velocity=5.0
    nav_tick_update(state, make_good_snap(2.0f), &out);

    // Run 9 Tier 1 failures with blind recovery (IMU also failed)
    for (unsigned i = 0; i < 9; ++i) {
        nav_tick_update(state, make_encoder_fail_snap_blind(), &out);
    }

    // Run 3 more failures (Tier 2): velocities should be 5.0*0.8=4.0, 4.0*0.8=3.2, 3.2*0.8=2.56
    for (unsigned i = 0; i < 3; ++i) {
        nav_tick_update(state, make_encoder_fail_snap_blind(), &out);
    }

    EXPECT_FLOAT_NEAR(state.last_velocity, 2.56f);
}

TEST(NavTickUpdate_EncoderTieredRecovery, Tier2_EncoderEstimatedFlagSet) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Good read with encoder_delta=2.0f → velocity=5.0
    nav_tick_update(state, make_good_snap(2.0f), &out);

    // Run 10 failures with blind recovery (IMU also failed) to enter Tier 2
    for (unsigned i = 0; i < 10; ++i) {
        nav_tick_update(state, make_encoder_fail_snap_blind(), &out);
    }

    uint8_t flags = nav_tick_update(state, make_encoder_fail_snap_blind(), &out);

    // ENCODER_ESTIMATED is set, ENCODER_LOST is NOT set (velocity still large)
    EXPECT_NE(flags & NAV_FLAG_ENCODER_ESTIMATED, 0);
    EXPECT_EQ(flags & NAV_FLAG_ENCODER_LOST, 0);
}

TEST(NavTickUpdate_EncoderTieredRecovery, Tier3_HardStopSetsEncoderLost) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Good read with encoder_delta=0.001f → velocity = (0.001*0.025)/0.01 = 0.0025 m/s
    nav_tick_update(state, make_good_snap(0.001f), &out);

    // Run enough failures to decay below epsilon
    // 0.0025 * 0.8^n < 1e-6 → need ~33 Tier 2 ticks, so run ~50 total to be safe
    for (int i = 0; i < 50; ++i) {
        nav_tick_update(state, make_encoder_fail_snap(), &out);
    }

    uint8_t flags = nav_tick_update(state, make_encoder_fail_snap(), &out);

    // ENCODER_LOST flag set
    EXPECT_NE(flags & NAV_FLAG_ENCODER_LOST, 0);

    // Position stopped changing (save and verify)
    double pos_frozen = state.pos_x;
    nav_tick_update(state, make_encoder_fail_snap(), &out);
    EXPECT_DOUBLE_NEAR(state.pos_x, pos_frozen);
}

TEST(NavTickUpdate_EncoderTieredRecovery, Tier3_PositionFrozen) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Get into Tier 3
    nav_tick_update(state, make_good_snap(0.001f), &out);
    for (int i = 0; i < 50; ++i) {
        nav_tick_update(state, make_encoder_fail_snap(), &out);
    }

    // Record pos_x, run 5 more failure ticks
    double pos_frozen = state.pos_x;
    for (int i = 0; i < 5; ++i) {
        nav_tick_update(state, make_encoder_fail_snap(), &out);
    }

    EXPECT_DOUBLE_NEAR(state.pos_x, pos_frozen);
}

TEST(NavTickUpdate_EncoderTieredRecovery, RecoveryFromTier2_ResetsVelocity) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Good read (velocity=5.0), 12 failures (enters Tier 2, velocity decayed)
    nav_tick_update(state, make_good_snap(2.0f), &out);
    for (int i = 0; i < 12; ++i) {
        nav_tick_update(state, make_encoder_fail_snap(), &out);
    }

    // Another good read with encoder_delta=1.0f → velocity = (1.0*0.025)/0.01 = 2.5
    uint8_t flags = nav_tick_update(state, make_good_snap(1.0f), &out);

    EXPECT_FLOAT_NEAR(state.last_velocity, 2.5f);
    EXPECT_EQ(state.encoder_fail_streak, 0);
    EXPECT_EQ(flags, 0);
}

TEST(NavTickUpdate_EncoderTieredRecovery, RecoveryFromTier3_ClearsFlagsAndResumes) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Get into Tier 3
    nav_tick_update(state, make_good_snap(0.001f), &out);
    for (int i = 0; i < 50; ++i) {
        nav_tick_update(state, make_encoder_fail_snap(), &out);
    }

    // One good read with encoder_delta=2.0f
    double pos_before = state.pos_x;
    uint8_t flags = nav_tick_update(state, make_good_snap(2.0f), &out);

    // ENCODER_LOST cleared, ENCODER_ESTIMATED cleared, position moves
    EXPECT_EQ(flags & NAV_FLAG_ENCODER_LOST, 0);
    EXPECT_EQ(flags & NAV_FLAG_ENCODER_ESTIMATED, 0);
    EXPECT_GT(state.pos_x, pos_before);
}

TEST(NavTickUpdate_EncoderTieredRecovery, TotalDistance_AccumulatesOnSuccess) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // 5 good reads with encoder_delta=1.0f → each delta_dist = 0.025
    for (int i = 0; i < 5; ++i) {
        nav_tick_update(state, make_good_snap(1.0f), &out);
    }

    EXPECT_NEAR(state.total_distance, 5.0 * 0.025, 1e-6);
}

TEST(NavTickUpdate_EncoderTieredRecovery, TotalDistance_AccumulatesDuringEstimation) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Good read with encoder_delta=2.0f (delta_dist=0.05), then 3 failures with IMU motion
    nav_tick_update(state, make_good_snap(2.0f), &out);
    for (unsigned i = 0; i < 3; ++i) {
        nav_tick_update(state, make_encoder_fail_snap_with_motion(), &out);
    }

    // total_distance = 4 * 0.05 = 0.2
    EXPECT_NEAR(state.total_distance, 4.0 * 0.05, 1e-6);
}

TEST(NavTickUpdate_EncoderTieredRecovery, TotalDistance_StopsInTier3) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Get into Tier 3
    nav_tick_update(state, make_good_snap(0.001f), &out);
    for (int i = 0; i < 50; ++i) {
        nav_tick_update(state, make_encoder_fail_snap(), &out);
    }

    // Record total_distance, run 3 more failures
    double distance_frozen = state.total_distance;
    for (int i = 0; i < 3; ++i) {
        nav_tick_update(state, make_encoder_fail_snap(), &out);
    }

    EXPECT_DOUBLE_NEAR(state.total_distance, distance_frozen);
}

TEST(NavTickUpdate_EncoderTieredRecovery, ZeroVelocity_NoSpuriousMotion) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Good read with encoder_delta=0.0f → velocity=0
    nav_tick_update(state, make_good_snap(0.0f), &out);

    // 20 failures: all should produce delta_dist=0
    for (int i = 0; i < 20; ++i) {
        nav_tick_update(state, make_encoder_fail_snap(), &out);
    }

    EXPECT_FLOAT_NEAR(state.pos_x, 0.0f);
    EXPECT_FLOAT_NEAR(state.pos_y, 0.0f);
}

TEST(NavTickUpdate_EncoderTieredRecovery, NegativeVelocity_DecaysTowardZero) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Good read with encoder_delta=-2.0f → velocity = (-2.0*0.025)/0.01 = -5.0
    nav_tick_update(state, make_good_snap(-2.0f), &out);

    // 9 Tier 1 failures with blind recovery: delta_dist = -5.0*0.01 = -0.05 each (moving backward)
    for (unsigned i = 0; i < 9; ++i) {
        nav_tick_update(state, make_encoder_fail_snap_blind(), &out);
    }

    // 3 Tier 2 failures with blind recovery: velocity = -4.0, -3.2, -2.56
    for (unsigned i = 0; i < 3; ++i) {
        nav_tick_update(state, make_encoder_fail_snap_blind(), &out);
    }

    // Velocity should be negative but magnitude decaying
    EXPECT_FLOAT_NEAR(state.last_velocity, -2.56f);

    // Verify total_distance is positive (uses fabsf)
    EXPECT_GT(state.total_distance, 0.0);
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap1, &out);

    SensorSnapshot snap2 = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap1, &out);

    SensorSnapshot snap_fail = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = false
    };

    uint8_t flags = nav_tick_update(state, snap, &out);
    EXPECT_EQ(state.imu_fail_streak, NAV_CRITICAL_FAIL_THRESHOLD);
    EXPECT_EQ(flags, NAV_FLAG_IMU_ESTIMATED | NAV_FLAG_NAV_CRITICAL | NAV_FLAG_IMU_LOST);
}

// ============================================================================
// Test Suite: NavTickUpdate_IMUTieredRecovery
// ============================================================================

// Helper: Create a failed IMU snapshot
static SensorSnapshot make_imu_fail_snap() {
    return {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = false
    };
}

// Tier 1 Tests (streak 1-4): Extrapolation via angular velocity

TEST(NavTickUpdate_IMUTieredRecovery, Tier1_ExtrapolatesWithAngularVelocity) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Setup: One good read with non-zero angular velocity (1 rad/s yaw)
    SensorSnapshot snap_good = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 1.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good, &out);

    // Record the starting quaternion (identity)
    Quat quat_before = state.last_good_quat;
    EXPECT_FLOAT_NEAR(quat_before.w, 1.0f);
    EXPECT_FLOAT_NEAR(quat_before.z, 0.0f);

    // Fail 1 tick (Tier 1): should extrapolate using omega_z = 1.0
    nav_tick_update(state, make_imu_fail_snap(), &out);

    // Verify quaternion changed (extrapolated)
    // For omega_z=1.0, dt=0.01, extrapolation does: q_out = q + 0.5*dt*(0,0,0,1)*q
    // After normalization, w should decrease slightly and z should increase
    EXPECT_LT(out.quat_w, 1.0f);
    EXPECT_GT(out.quat_z, 0.0f);
}

TEST(NavTickUpdate_IMUTieredRecovery, Tier1_ChainsExtrapolationAcrossTicks) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Setup: Good read with omega_z = 1.0
    SensorSnapshot snap_good = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 1.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good, &out);

    // Run 4 failures (all Tier 1): each should extrapolate from previous
    nav_tick_update(state, make_imu_fail_snap(), &out);
    float z_after_1 = out.quat_z;

    nav_tick_update(state, make_imu_fail_snap(), &out);
    float z_after_2 = out.quat_z;

    nav_tick_update(state, make_imu_fail_snap(), &out);
    float z_after_3 = out.quat_z;

    nav_tick_update(state, make_imu_fail_snap(), &out);
    float z_after_4 = out.quat_z;

    // Verify z component increases with each extrapolation (chaining)
    EXPECT_GT(z_after_2, z_after_1);
    EXPECT_GT(z_after_3, z_after_2);
    EXPECT_GT(z_after_4, z_after_3);
}

TEST(NavTickUpdate_IMUTieredRecovery, Tier1_ZeroAngularVelocityHoldsOrientation) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Setup: Good read with zero angular velocity
    SensorSnapshot snap_good = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good, &out);

    // Fail 1 tick with zero omega: extrapolation should not change quaternion
    nav_tick_update(state, make_imu_fail_snap(), &out);

    // Quaternion should remain identity (omega=0 means no rotation)
    EXPECT_NEAR(out.quat_w, 1.0f, 1e-6f);
    EXPECT_NEAR(out.quat_x, 0.0f, 1e-6f);
    EXPECT_NEAR(out.quat_y, 0.0f, 1e-6f);
    EXPECT_NEAR(out.quat_z, 0.0f, 1e-6f);
}

TEST(NavTickUpdate_IMUTieredRecovery, Tier1_SetsIMUEstimatedNotLost) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Setup: Good read
    SensorSnapshot snap_good = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.5f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good, &out);

    // Fail 1 tick (Tier 1)
    uint8_t flags = nav_tick_update(state, make_imu_fail_snap(), &out);

    // ESTIMATED should be set, LOST should NOT be set
    EXPECT_NE(flags & NAV_FLAG_IMU_ESTIMATED, 0);
    EXPECT_EQ(flags & NAV_FLAG_IMU_LOST, 0);
}

TEST(NavTickUpdate_IMUTieredRecovery, Tier1_DoesNotThrottleDistance) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Setup: Good read with omega
    SensorSnapshot snap_good = {
        .encoder_delta = 2.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.5f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good, &out);
    double pos_before = state.pos_x;

    // Fail 1 tick (Tier 1): distance should NOT be throttled
    SensorSnapshot snap_fail = {
        .encoder_delta = 2.0f,
        .encoder_valid = true,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = false
    };
    nav_tick_update(state, snap_fail, &out);

    // Full distance accumulation: delta_dist = 2.0 * 0.025 = 0.05
    double delta_dist = state.pos_x - pos_before;
    EXPECT_NEAR(delta_dist, 2.0 * ENCODER_WHEEL_RADIUS_M, 1e-6);
}

TEST(NavTickUpdate_IMUTieredRecovery, Tier1_DoesNotRequestReset) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Setup: Good read
    SensorSnapshot snap_good = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.5f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good, &out);

    // Fail 3 ticks (all Tier 1)
    for (int i = 0; i < 3; ++i) {
        nav_tick_update(state, make_imu_fail_snap(), &out);
    }

    // action_flags should NOT have NAV_ACTION_IMU_RESET
    EXPECT_EQ(state.action_flags & NAV_ACTION_IMU_RESET, 0);
}

// Tier 2 Tests (streak 5-49): Hold orientation, throttle distance by 50%

TEST(NavTickUpdate_IMUTieredRecovery, Tier2_HoldsOrientationAtBoundary) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Setup: Good read with omega
    SensorSnapshot snap_good = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 1.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good, &out);

    // Run 4 Tier 1 failures (extrapolation)
    for (int i = 0; i < 4; ++i) {
        nav_tick_update(state, make_imu_fail_snap(), &out);
    }

    // Record quaternion after Tier 1
    Quat quat_tier1_end = {out.quat_w, out.quat_x, out.quat_y, out.quat_z};

    // 5th failure enters Tier 2: orientation should be held (not extrapolated)
    nav_tick_update(state, make_imu_fail_snap(), &out);

    // Quaternion should match end of Tier 1 (held, not extrapolated)
    EXPECT_FLOAT_NEAR(out.quat_w, quat_tier1_end.w);
    EXPECT_FLOAT_NEAR(out.quat_x, quat_tier1_end.x);
    EXPECT_FLOAT_NEAR(out.quat_y, quat_tier1_end.y);
    EXPECT_FLOAT_NEAR(out.quat_z, quat_tier1_end.z);
}

TEST(NavTickUpdate_IMUTieredRecovery, Tier2_ThrottlesDistanceBy50Percent) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Setup: Good read
    SensorSnapshot snap_good = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.5f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good, &out);

    // Run 4 Tier 1 failures
    for (int i = 0; i < 4; ++i) {
        nav_tick_update(state, make_imu_fail_snap(), &out);
    }

    double pos_before = state.pos_x;

    // 5th failure (Tier 2): encoder_delta=2.0, throttled by 50%
    SensorSnapshot snap_tier2 = {
        .encoder_delta = 2.0f,
        .encoder_valid = true,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = false
    };
    nav_tick_update(state, snap_tier2, &out);

    // Distance should be: 2.0 * 0.025 * 0.5 = 0.025
    // Use relaxed tolerance due to float->double conversions and quaternion extrapolation accumulation
    double delta_dist = state.pos_x - pos_before;
    EXPECT_NEAR(delta_dist, 2.0 * ENCODER_WHEEL_RADIUS_M * NAV_IMU_DIST_THROTTLE, 1e-5);
}

TEST(NavTickUpdate_IMUTieredRecovery, Tier2_ThrottleStacksWithEncoderRecovery) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Setup: Good read with encoder_delta=2.0 → velocity = 5.0 m/s
    SensorSnapshot snap_good = {
        .encoder_delta = 2.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.5f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good, &out);

    // Enter IMU Tier 2 (4 IMU failures). The helper has encoder_valid=true with encoder_delta=1.0,
    // which updates encoder velocity to 2.5 m/s on each tick.
    for (int i = 0; i < 4; ++i) {
        nav_tick_update(state, make_imu_fail_snap(), &out);
    }

    // Last encoder velocity is now 2.5 m/s (from the helper's encoder_delta=1.0)
    EXPECT_FLOAT_NEAR(state.last_velocity, 2.5f);

    double pos_before = state.pos_x;

    // 5th failure with BOTH encoder and IMU failed
    SensorSnapshot snap_both_fail = {
        .encoder_delta = 0.0f,
        .encoder_valid = false,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = false
    };
    nav_tick_update(state, snap_both_fail, &out);

    // Encoder Tier 1: uses last_velocity = 2.5 m/s → delta_dist = 2.5 * 0.01 = 0.025
    // Then IMU Tier 2 throttles by 50%: 0.025 * 0.5 = 0.0125
    double delta_dist = state.pos_x - pos_before;
    EXPECT_NEAR(delta_dist, 0.0125, 1e-5);
}

TEST(NavTickUpdate_IMUTieredRecovery, Tier2_SetsIMUEstimatedNotLost) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Setup: Good read
    SensorSnapshot snap_good = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.5f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good, &out);

    // Enter Tier 2 (5 failures)
    for (int i = 0; i < 5; ++i) {
        nav_tick_update(state, make_imu_fail_snap(), &out);
    }

    uint8_t flags = nav_tick_update(state, make_imu_fail_snap(), &out);

    // ESTIMATED set, LOST NOT set
    EXPECT_NE(flags & NAV_FLAG_IMU_ESTIMATED, 0);
    EXPECT_EQ(flags & NAV_FLAG_IMU_LOST, 0);
}

TEST(NavTickUpdate_IMUTieredRecovery, Tier2_DoesNotRequestReset) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Setup: Good read
    SensorSnapshot snap_good = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.5f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good, &out);

    // Run 10 failures (well into Tier 2)
    for (int i = 0; i < 10; ++i) {
        nav_tick_update(state, make_imu_fail_snap(), &out);
    }

    // action_flags should NOT have NAV_ACTION_IMU_RESET
    EXPECT_EQ(state.action_flags & NAV_ACTION_IMU_RESET, 0);
}

// Tier 3 Tests (streak 50+): Distance zeroed, IMU_LOST + NAV_CRITICAL, reset requested

TEST(NavTickUpdate_IMUTieredRecovery, Tier3_ZerosDistance) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Setup: Good read
    SensorSnapshot snap_good = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.5f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good, &out);

    // Enter Tier 3 (50 failures)
    for (int i = 0; i < 49; ++i) {
        nav_tick_update(state, make_imu_fail_snap(), &out);
    }

    double pos_before = state.pos_x;

    // 50th failure (Tier 3 entry): encoder_delta=2.0 but distance zeroed
    SensorSnapshot snap_tier3 = {
        .encoder_delta = 2.0f,
        .encoder_valid = true,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = false
    };
    nav_tick_update(state, snap_tier3, &out);

    // Distance should be zeroed
    EXPECT_DOUBLE_NEAR(state.pos_x, pos_before);
}

TEST(NavTickUpdate_IMUTieredRecovery, Tier3_SetsIMULostAndNavCritical) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Setup: Good read
    SensorSnapshot snap_good = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.5f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good, &out);

    // Enter Tier 3 (50 failures)
    for (int i = 0; i < 49; ++i) {
        nav_tick_update(state, make_imu_fail_snap(), &out);
    }

    uint8_t flags = nav_tick_update(state, make_imu_fail_snap(), &out);

    // Both IMU_LOST and NAV_CRITICAL should be set
    EXPECT_NE(flags & NAV_FLAG_IMU_LOST, 0);
    EXPECT_NE(flags & NAV_FLAG_NAV_CRITICAL, 0);
}

TEST(NavTickUpdate_IMUTieredRecovery, Tier3_RequestsResetOnce) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Setup: Good read
    SensorSnapshot snap_good = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.5f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good, &out);

    // Enter Tier 3 (50 failures)
    for (int i = 0; i < 49; ++i) {
        nav_tick_update(state, make_imu_fail_snap(), &out);
    }

    // 50th failure: NAV_ACTION_IMU_RESET should be set
    nav_tick_update(state, make_imu_fail_snap(), &out);
    EXPECT_NE(state.action_flags & NAV_ACTION_IMU_RESET, 0);
    EXPECT_TRUE(state.imu_reset_requested);
}

TEST(NavTickUpdate_IMUTieredRecovery, Tier3_DoesNotReRequestReset) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Setup: Good read
    SensorSnapshot snap_good = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.5f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good, &out);

    // Enter Tier 3 and trigger reset request
    for (int i = 0; i < 50; ++i) {
        nav_tick_update(state, make_imu_fail_snap(), &out);
    }

    // Consumer clears action_flags (simulating core1_nav.cpp handling it)
    state.action_flags = 0;

    // Subsequent Tier 3 tick should NOT re-set action_flags
    nav_tick_update(state, make_imu_fail_snap(), &out);
    EXPECT_EQ(state.action_flags & NAV_ACTION_IMU_RESET, 0);
}

TEST(NavTickUpdate_IMUTieredRecovery, Tier3_TotalDistanceFrozen) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Setup: Good read
    SensorSnapshot snap_good = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.5f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good, &out);

    // Enter Tier 3
    for (int i = 0; i < 50; ++i) {
        nav_tick_update(state, make_imu_fail_snap(), &out);
    }

    double total_dist_frozen = state.total_distance;

    // Run 5 more Tier 3 ticks with encoder motion
    for (int i = 0; i < 5; ++i) {
        SensorSnapshot snap = {
            .encoder_delta = 2.0f,
            .encoder_valid = true,
            .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
            .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
            .imu_valid = false
        };
        nav_tick_update(state, snap, &out);
    }

    // total_distance should not have increased
    EXPECT_DOUBLE_NEAR(state.total_distance, total_dist_frozen);
}

// Recovery Tests

TEST(NavTickUpdate_IMUTieredRecovery, RecoveryFromTier1_ResetsStateFully) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Setup: Good read with omega
    SensorSnapshot snap_good1 = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.5f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good1, &out);

    // Run 3 Tier 1 failures
    for (int i = 0; i < 3; ++i) {
        nav_tick_update(state, make_imu_fail_snap(), &out);
    }

    EXPECT_EQ(state.imu_fail_streak, 3);

    // Recovery: Good read with new quaternion and angular velocity
    SensorSnapshot snap_good2 = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {0.9f, 0.1f, 0.2f, 0.3f},
        .imu_angular_velocity = {0.1f, 0.2f, 0.3f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    uint8_t flags = nav_tick_update(state, snap_good2, &out);

    // Verify reset
    EXPECT_EQ(state.imu_fail_streak, 0);
    EXPECT_FALSE(state.imu_reset_requested);
    EXPECT_EQ(state.action_flags & NAV_ACTION_IMU_RESET, 0);
    EXPECT_EQ(flags, 0);
}

TEST(NavTickUpdate_IMUTieredRecovery, RecoveryFromTier2_ClearsThrottleAndFlags) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Setup: Good read
    SensorSnapshot snap_good1 = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.5f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good1, &out);

    // Enter Tier 2 (6 failures)
    for (int i = 0; i < 6; ++i) {
        nav_tick_update(state, make_imu_fail_snap(), &out);
    }

    double pos_before = state.pos_x;

    // Recovery: Good read with encoder motion
    SensorSnapshot snap_good2 = {
        .encoder_delta = 2.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    uint8_t flags = nav_tick_update(state, snap_good2, &out);

    // Distance should be full (not throttled): 2.0 * 0.025 = 0.05
    double delta_dist = state.pos_x - pos_before;
    EXPECT_NEAR(delta_dist, 2.0 * ENCODER_WHEEL_RADIUS_M, 1e-6);

    // Flags cleared
    EXPECT_EQ(flags, 0);
    EXPECT_EQ(state.imu_fail_streak, 0);
}

TEST(NavTickUpdate_IMUTieredRecovery, RecoveryFromTier3_ClearsAllFlagsAndResetRequest) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Setup: Good read
    SensorSnapshot snap_good1 = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.5f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good1, &out);

    // Enter Tier 3 (50 failures)
    for (int i = 0; i < 50; ++i) {
        nav_tick_update(state, make_imu_fail_snap(), &out);
    }

    EXPECT_TRUE(state.imu_reset_requested);
    EXPECT_NE(state.action_flags & NAV_ACTION_IMU_RESET, 0);

    // Recovery: Good read
    SensorSnapshot snap_good2 = {
        .encoder_delta = 2.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    uint8_t flags = nav_tick_update(state, snap_good2, &out);

    // All flags cleared
    EXPECT_EQ(flags & NAV_FLAG_IMU_LOST, 0);
    EXPECT_EQ(flags & NAV_FLAG_NAV_CRITICAL, 0);
    EXPECT_EQ(flags & NAV_FLAG_IMU_ESTIMATED, 0);
    EXPECT_EQ(state.imu_fail_streak, 0);
    EXPECT_FALSE(state.imu_reset_requested);
    EXPECT_EQ(state.action_flags & NAV_ACTION_IMU_RESET, 0);
}

TEST(NavTickUpdate_IMUTieredRecovery, NewFailureAfterRecovery_ReRaisesResetRequest) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Setup: Good read
    SensorSnapshot snap_good1 = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.5f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good1, &out);

    // Enter Tier 3 (50 failures)
    for (int i = 0; i < 50; ++i) {
        nav_tick_update(state, make_imu_fail_snap(), &out);
    }

    // Recovery
    SensorSnapshot snap_good2 = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.5f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good2, &out);

    EXPECT_FALSE(state.imu_reset_requested);

    // Fail again to Tier 3 (50 more failures)
    for (int i = 0; i < 50; ++i) {
        nav_tick_update(state, make_imu_fail_snap(), &out);
    }

    // Reset should be requested again
    EXPECT_TRUE(state.imu_reset_requested);
    EXPECT_NE(state.action_flags & NAV_ACTION_IMU_RESET, 0);
}

// Edge Cases

TEST(NavTickUpdate_IMUTieredRecovery, UINT16MAX_SaturationWithTiers) {
    NavTickState state = {};
    state.imu_fail_streak = UINT16_MAX;
    nav_state_compact_t out = {};

    // Fail one more time
    uint8_t flags = nav_tick_update(state, make_imu_fail_snap(), &out);

    // Should still be UINT16_MAX (saturated)
    EXPECT_EQ(state.imu_fail_streak, UINT16_MAX);

    // Should behave as Tier 3 (IMU_LOST + NAV_CRITICAL)
    EXPECT_NE(flags & NAV_FLAG_IMU_LOST, 0);
    EXPECT_NE(flags & NAV_FLAG_NAV_CRITICAL, 0);
}

TEST(NavTickUpdate_IMUTieredRecovery, SimultaneousEncoderAndIMUTier3) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Setup: Good read with encoder_delta=0.001f to quickly reach encoder Tier 3
    SensorSnapshot snap_good = {
        .encoder_delta = 0.001f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.5f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good, &out);

    // Fail both sensors for 50 ticks to reach both Tier 3
    SensorSnapshot snap_both_fail = {
        .encoder_delta = 0.0f,
        .encoder_valid = false,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = false
    };
    for (int i = 0; i < 50; ++i) {
        nav_tick_update(state, snap_both_fail, &out);
    }

    uint8_t flags = nav_tick_update(state, snap_both_fail, &out);

    // Verify all expected flags are set
    EXPECT_NE(flags & NAV_FLAG_ENCODER_ESTIMATED, 0);
    EXPECT_NE(flags & NAV_FLAG_ENCODER_LOST, 0);
    EXPECT_NE(flags & NAV_FLAG_IMU_ESTIMATED, 0);
    EXPECT_NE(flags & NAV_FLAG_IMU_LOST, 0);
    EXPECT_NE(flags & NAV_FLAG_NAV_CRITICAL, 0);
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    uint8_t flags = nav_tick_update(state, snap, &out);

    EXPECT_EQ(flags, NAV_FLAG_ENCODER_ESTIMATED | NAV_FLAG_NAV_CRITICAL | NAV_FLAG_ENCODER_LOST);
    EXPECT_EQ(out.status_flags, NAV_FLAG_ENCODER_ESTIMATED | NAV_FLAG_NAV_CRITICAL | NAV_FLAG_ENCODER_LOST);
}

TEST(NavTickUpdate_StatusFlags, FlagsClearWhenSensorsRecover) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    SensorSnapshot snap_fail = {
        .encoder_delta = 0.0f,
        .encoder_valid = false,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = false
    };
    nav_tick_update(state, snap_fail, &out);

    EXPECT_EQ(out.status_flags, NAV_FLAG_ENCODER_ESTIMATED | NAV_FLAG_IMU_ESTIMATED);

    SensorSnapshot snap_good = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap1, &out);

    SensorSnapshot snap2 = {
        .encoder_delta = 1.0f,
        .encoder_valid = true,
        .imu_quaternion = {static_cast<float>(COS45), 0.0f, 0.0f, static_cast<float>(SIN45)},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
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
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    nav_tick_update(state, snap, &out);

    double expected_x = 1e6 + 0.001 * ENCODER_WHEEL_RADIUS_M;
    EXPECT_DOUBLE_NEAR(state.pos_x, expected_x);
    EXPECT_DOUBLE_NEAR(state.pos_y, -5e5);
    EXPECT_DOUBLE_NEAR(state.pos_z, 3e5);
}

// ============================================================================
// Test Suite: NavTickUpdate_DeltaDistOutput
// ============================================================================

TEST(NavTickUpdate_DeltaDistOutput, DeltaDistOutputIsPreThrottle) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Good encoder read, good IMU
    SensorSnapshot snap_good = {
        .encoder_delta = 2.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good, &out);

    // delta_dist = encoder_delta * ENCODER_WHEEL_RADIUS_M = 2.0 * 0.025 = 0.05
    EXPECT_FLOAT_NEAR(out.delta_dist, 2.0f * ENCODER_WHEEL_RADIUS_M);
}

TEST(NavTickUpdate_DeltaDistOutput, DeltaDistPreservedDuringIMUTier2Throttle) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Good read to set up velocity
    SensorSnapshot snap_good = {
        .encoder_delta = 2.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.5f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good, &out);

    // Enter IMU Tier 2 (5+ failures, which throttles delta_dist by 50% for position)
    for (int i = 0; i < 6; ++i) {
        nav_tick_update(state, make_imu_fail_snap(), &out);
    }

    // Encoder valid with encoder_delta=4.0, IMU Tier 2 throttles internally
    // but delta_dist output should be PRE-throttle
    SensorSnapshot snap_enc = {
        .encoder_delta = 4.0f,
        .encoder_valid = true,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = false
    };
    nav_tick_update(state, snap_enc, &out);

    // Output delta_dist should be pre-throttle: 4.0 * 0.025 = 0.1
    EXPECT_FLOAT_NEAR(out.delta_dist, 4.0f * ENCODER_WHEEL_RADIUS_M);
}

TEST(NavTickUpdate_DeltaDistOutput, DeltaDistPreservedDuringIMUTier3Zero) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Good read to set up velocity
    SensorSnapshot snap_good = {
        .encoder_delta = 2.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.5f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
    nav_tick_update(state, snap_good, &out);

    // Enter IMU Tier 3 (50+ failures, which zeros delta_dist for position)
    for (int i = 0; i < 50; ++i) {
        nav_tick_update(state, make_imu_fail_snap(), &out);
    }

    // Encoder valid with encoder_delta=4.0, IMU Tier 3 zeros delta_dist internally
    // but output delta_dist should be PRE-throttle
    SensorSnapshot snap_enc = {
        .encoder_delta = 4.0f,
        .encoder_valid = true,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = false
    };
    nav_tick_update(state, snap_enc, &out);

    // Output delta_dist should be pre-throttle: 4.0 * 0.025 = 0.1
    EXPECT_FLOAT_NEAR(out.delta_dist, 4.0f * ENCODER_WHEEL_RADIUS_M);
}

// ============================================================================
// Test Suite: NavTickUpdate_ConflictDetection_CaseA (Wheel Slip)
// ============================================================================

// Helper: Create a snapshot with wheel slip scenario (encoder moving, IMU stationary)
static SensorSnapshot make_wheel_slip_snap(float encoder_delta) {
    return {
        .encoder_delta = encoder_delta,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
}

// Helper: Create a snapshot with normal motion (encoder moving, IMU shows motion)
static SensorSnapshot make_normal_motion_snap(float encoder_delta, float accel_x) {
    return {
        .encoder_delta = encoder_delta,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {accel_x, 0.0f, 0.0f},
        .imu_valid = true
    };
}

TEST(NavTickUpdate_ConflictDetection_CaseA, NoConflictWhenBothStationary) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Both encoder and IMU stationary - no conflict
    SensorSnapshot snap = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    for (int i = 0; i < 20; ++i) {
        nav_tick_update(state, snap, &out);
    }

    EXPECT_EQ(state.conflict_streak, 0);
    EXPECT_EQ(out.status_flags & NAV_FLAG_SENSOR_CONFLICT, 0);
}

TEST(NavTickUpdate_ConflictDetection_CaseA, NoConflictOnForwardMotion) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Normal forward motion: encoder moving, IMU shows forward acceleration
    for (int i = 0; i < 20; ++i) {
        nav_tick_update(state, make_normal_motion_snap(1.0f, 0.5f), &out);
    }

    EXPECT_EQ(state.conflict_streak, 0);
    EXPECT_EQ(out.status_flags & NAV_FLAG_SENSOR_CONFLICT, 0);
}

TEST(NavTickUpdate_ConflictDetection_CaseA, NoConflictWhenRotating) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Rotating in place: encoder stationary, IMU shows rotation
    SensorSnapshot snap = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.1f},  // Above threshold
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    for (int i = 0; i < 20; ++i) {
        nav_tick_update(state, snap, &out);
    }

    EXPECT_EQ(state.conflict_streak, 0);
    EXPECT_EQ(out.status_flags & NAV_FLAG_SENSOR_CONFLICT, 0);
}

TEST(NavTickUpdate_ConflictDetection_CaseA, NoConflictWhenTurning) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Turning: encoder moving, IMU shows rotation
    SensorSnapshot snap = {
        .encoder_delta = 1.0f,  // Above threshold
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.1f},  // Above threshold
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    for (int i = 0; i < 20; ++i) {
        nav_tick_update(state, snap, &out);
    }

    EXPECT_EQ(state.conflict_streak, 0);
    EXPECT_EQ(out.status_flags & NAV_FLAG_SENSOR_CONFLICT, 0);
}

TEST(NavTickUpdate_ConflictDetection_CaseA, ConflictDetectedOnWheelSlip) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Wheel slip: encoder moving, IMU completely stationary
    // Need CONFLICT_WINDOW_TICKS (5) to complete one window, then streak increments
    // After CONFLICT_TIER1_THRESHOLD (10) windows, flag is set
    for (unsigned i = 0; i < CONFLICT_WINDOW_TICKS * (CONFLICT_TIER1_THRESHOLD + 1); ++i) {
        nav_tick_update(state, make_wheel_slip_snap(0.5f), &out);
    }

    EXPECT_GE(state.conflict_streak, CONFLICT_TIER1_THRESHOLD);
    EXPECT_NE(out.status_flags & NAV_FLAG_SENSOR_CONFLICT, 0);
}

TEST(NavTickUpdate_ConflictDetection_CaseA, TieredAttenuation_Tier2) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Build up to Tier 2 threshold
    for (unsigned i = 0; i < CONFLICT_WINDOW_TICKS * (CONFLICT_TIER2_THRESHOLD + 1); ++i) {
        nav_tick_update(state, make_wheel_slip_snap(1.0f), &out);
    }

    // Position should advance at 50% rate due to attenuation
    // Note: the exact calculation depends on how many ticks applied attenuation
    EXPECT_GE(state.conflict_streak, CONFLICT_TIER2_THRESHOLD);
}

TEST(NavTickUpdate_ConflictDetection_CaseA, TieredAttenuation_Tier3) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Build up to Tier 3 threshold
    for (unsigned i = 0; i < CONFLICT_WINDOW_TICKS * (CONFLICT_TIER3_THRESHOLD + 1); ++i) {
        nav_tick_update(state, make_wheel_slip_snap(1.0f), &out);
    }

    double pos_before = state.pos_x;

    // One more window of wheel slip - distance should be zeroed
    for (unsigned i = 0; i < CONFLICT_WINDOW_TICKS; ++i) {
        nav_tick_update(state, make_wheel_slip_snap(2.0f), &out);
    }

    // Position should not advance at Tier 3
    EXPECT_DOUBLE_NEAR(state.pos_x, pos_before);
    EXPECT_GE(state.conflict_streak, CONFLICT_TIER3_THRESHOLD);
}

TEST(NavTickUpdate_ConflictDetection_CaseA, ConflictStreakResetsOnNormalMotion) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Build up some conflict streak
    for (unsigned i = 0; i < CONFLICT_WINDOW_TICKS * 5; ++i) {
        nav_tick_update(state, make_wheel_slip_snap(0.5f), &out);
    }

    EXPECT_GT(state.conflict_streak, 0);

    // Normal motion should reset streak (IMU shows acceleration)
    for (unsigned i = 0; i < CONFLICT_WINDOW_TICKS * 2; ++i) {
        nav_tick_update(state, make_normal_motion_snap(0.5f, 0.5f), &out);
    }

    EXPECT_EQ(state.conflict_streak, 0);
    EXPECT_EQ(out.status_flags & NAV_FLAG_SENSOR_CONFLICT, 0);
}

// ============================================================================
// Test Suite: NavTickUpdate_ConflictDetection_CaseB (IMU-Informed Encoder Recovery)
// ============================================================================

// Helper: Create encoder fail snapshot with IMU showing motion
static SensorSnapshot make_encoder_fail_imu_motion_snap() {
    return {
        .encoder_delta = 0.0f,
        .encoder_valid = false,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.1f},  // Above threshold
        .imu_linear_accel = {0.5f, 0.0f, 0.0f},  // Forward motion
        .imu_valid = true
    };
}

// Helper: Create encoder fail snapshot with IMU showing stationary
static SensorSnapshot make_encoder_fail_imu_stationary_snap() {
    return {
        .encoder_delta = 0.0f,
        .encoder_valid = false,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };
}

// Helper: Create encoder fail snapshot with IMU showing deceleration
static SensorSnapshot make_encoder_fail_imu_decel_snap() {
    return {
        .encoder_delta = 0.0f,
        .encoder_valid = false,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {-0.5f, 0.0f, 0.0f},  // Deceleration
        .imu_valid = true
    };
}

TEST(NavTickUpdate_ConflictDetection_CaseB, IMUMotion_ContinuesExtrapolation) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Establish velocity with good encoder read
    nav_tick_update(state, make_good_snap(2.0f), &out);
    float expected_velocity = 2.0f * ENCODER_WHEEL_RADIUS_M / 0.01f;  // 5.0 m/s
    EXPECT_FLOAT_NEAR(state.last_velocity, expected_velocity);

    double pos_before = state.pos_x;

    // Encoder fails but IMU shows motion -> continue extrapolation
    nav_tick_update(state, make_encoder_fail_imu_motion_snap(), &out);

    // Position should advance at last velocity
    double expected_delta = expected_velocity * 0.01;  // dt = 10ms
    EXPECT_NEAR(state.pos_x - pos_before, expected_delta, 1e-6);
    EXPECT_FLOAT_NEAR(state.last_velocity, expected_velocity);
}

TEST(NavTickUpdate_ConflictDetection_CaseB, IMUStationary_ImmediateStop) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Establish velocity with good encoder read
    nav_tick_update(state, make_good_snap(2.0f), &out);
    EXPECT_FLOAT_NEAR(state.last_velocity, 5.0f);

    double pos_before = state.pos_x;

    // Encoder fails, IMU shows device stopped -> immediate hard stop
    nav_tick_update(state, make_encoder_fail_imu_stationary_snap(), &out);

    // Position should not advance, velocity zeroed
    EXPECT_DOUBLE_NEAR(state.pos_x, pos_before);
    EXPECT_FLOAT_NEAR(state.last_velocity, 0.0f);
}

TEST(NavTickUpdate_ConflictDetection_CaseB, IMUDeceleration_FasterDecay) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Establish velocity with good encoder read
    nav_tick_update(state, make_good_snap(2.0f), &out);
    float initial_velocity = state.last_velocity;
    EXPECT_FLOAT_NEAR(initial_velocity, 5.0f);

    // Encoder fails, IMU shows deceleration -> 60% decay per tick
    nav_tick_update(state, make_encoder_fail_imu_decel_snap(), &out);

    // Velocity should decay at 60% factor
    EXPECT_FLOAT_NEAR(state.last_velocity, initial_velocity * CONFLICT_ACCEL_DECAY_FACTOR);
}

TEST(NavTickUpdate_ConflictDetection_CaseB, BlindRecovery_WhenIMUAlsoFails) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Establish velocity with good encoder read
    nav_tick_update(state, make_good_snap(2.0f), &out);
    EXPECT_FLOAT_NEAR(state.last_velocity, 5.0f);

    // Both sensors fail -> should fall back to original tiered recovery
    SensorSnapshot snap_both_fail = {
        .encoder_delta = 0.0f,
        .encoder_valid = false,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = false
    };

    double pos_before = state.pos_x;
    nav_tick_update(state, snap_both_fail, &out);

    // Should use Tier 1 extrapolation (streak < 10)
    EXPECT_NEAR(state.pos_x - pos_before, 5.0 * 0.01, 1e-6);
}

// ============================================================================
// Test Suite: NavTickUpdate_ConflictDetection_CaseC (Stuck Wheel)
// ============================================================================

// Helper: Create stuck wheel scenario (encoder zero, IMU shows motion)
static SensorSnapshot make_stuck_wheel_snap() {
    return {
        .encoder_delta = 0.0f,  // Wheel not turning
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.1f},  // Device rotating
        .imu_linear_accel = {0.5f, 0.0f, 0.0f},  // Forward motion
        .imu_valid = true
    };
}

TEST(NavTickUpdate_ConflictDetection_CaseC, StuckWheelDetected) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Establish velocity first
    nav_tick_update(state, make_good_snap(2.0f), &out);
    EXPECT_FLOAT_NEAR(state.last_velocity, 5.0f);

    // Run stuck wheel scenario for CONFLICT_ZERO_ENCODER_TICKS
    for (unsigned i = 0; i < CONFLICT_ZERO_ENCODER_TICKS; ++i) {
        nav_tick_update(state, make_stuck_wheel_snap(), &out);
    }

    // Stuck wheel should be detected, using IMU-informed recovery
    EXPECT_GE(state.encoder_zero_imu_motion_streak, CONFLICT_ZERO_ENCODER_TICKS);
    EXPECT_NE(out.status_flags & NAV_FLAG_SENSOR_CONFLICT, 0);
}

TEST(NavTickUpdate_ConflictDetection_CaseC, StuckWheelUsesLastVelocity) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Establish velocity first
    nav_tick_update(state, make_good_snap(2.0f), &out);
    double pos_before = state.pos_x;

    // Run stuck wheel scenario - should use last velocity for extrapolation
    for (unsigned i = 0; i < CONFLICT_ZERO_ENCODER_TICKS; ++i) {
        nav_tick_update(state, make_stuck_wheel_snap(), &out);
    }

    // Position should have advanced using last velocity
    double delta = state.pos_x - pos_before;
    EXPECT_GT(delta, 0.0);
}

TEST(NavTickUpdate_ConflictDetection_CaseC, FalsePositivePrevention_IMUAlsoStationary) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Both encoder and IMU stationary - valid stop, not stuck wheel
    SensorSnapshot snap = {
        .encoder_delta = 0.0f,
        .encoder_valid = true,
        .imu_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = true
    };

    for (int i = 0; i < 20; ++i) {
        nav_tick_update(state, snap, &out);
    }

    // No stuck wheel detection since IMU also shows stationary
    EXPECT_EQ(state.encoder_zero_imu_motion_streak, 0);
    EXPECT_EQ(out.status_flags & NAV_FLAG_SENSOR_CONFLICT, 0);
}

TEST(NavTickUpdate_ConflictDetection_CaseC, RecoveryWhenEncoderResumes) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Build up stuck wheel detection
    nav_tick_update(state, make_good_snap(2.0f), &out);
    for (unsigned i = 0; i < CONFLICT_ZERO_ENCODER_TICKS; ++i) {
        nav_tick_update(state, make_stuck_wheel_snap(), &out);
    }

    EXPECT_GE(state.encoder_zero_imu_motion_streak, CONFLICT_ZERO_ENCODER_TICKS);

    // Encoder resumes working - streak should reset
    nav_tick_update(state, make_good_snap(1.0f), &out);

    EXPECT_EQ(state.encoder_zero_imu_motion_streak, 0);
    EXPECT_EQ(state.encoder_fail_streak, 0);
}

// ============================================================================
// Test Suite: NavTickUpdate_ConflictDetection_Integration
// ============================================================================

TEST(NavTickUpdate_ConflictDetection_Integration, ConflictAttenuationStacksWithIMUThrottle) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Build up conflict streak to Tier 2
    for (unsigned i = 0; i < CONFLICT_WINDOW_TICKS * (CONFLICT_TIER2_THRESHOLD + 1); ++i) {
        nav_tick_update(state, make_wheel_slip_snap(1.0f), &out);
    }

    // Also enter IMU Tier 2
    for (int i = 0; i < 6; ++i) {
        nav_tick_update(state, make_imu_fail_snap(), &out);
    }

    double pos_before = state.pos_x;

    // Both attenuations should stack
    SensorSnapshot snap = {
        .encoder_delta = 4.0f,
        .encoder_valid = true,
        .imu_quaternion = {0.0f, 0.0f, 0.0f, 0.0f},
        .imu_angular_velocity = {0.0f, 0.0f, 0.0f},
        .imu_linear_accel = {0.0f, 0.0f, 0.0f},
        .imu_valid = false
    };
    nav_tick_update(state, snap, &out);

    // Position delta should be reduced by both conflict (50%) and IMU (50%) attenuation
    double delta = state.pos_x - pos_before;
    double expected = 4.0f * ENCODER_WHEEL_RADIUS_M * CONFLICT_DIST_ATTENUATION * NAV_IMU_DIST_THROTTLE;
    EXPECT_NEAR(delta, expected, 1e-5);
}

TEST(NavTickUpdate_ConflictDetection_Integration, DeltaDistPhysicalPreserved) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Build up conflict streak to Tier 2 (attenuation applied)
    for (unsigned i = 0; i < CONFLICT_WINDOW_TICKS * (CONFLICT_TIER2_THRESHOLD + 1); ++i) {
        nav_tick_update(state, make_wheel_slip_snap(1.0f), &out);
    }

    // delta_dist output should still be pre-attenuation value
    nav_tick_update(state, make_wheel_slip_snap(2.0f), &out);

    // delta_dist_physical should be the encoder reading * wheel radius
    EXPECT_FLOAT_NEAR(out.delta_dist, 2.0f * ENCODER_WHEEL_RADIUS_M);
}

TEST(NavTickUpdate_ConflictDetection_Integration, WindowResetOnSensorFailure) {
    NavTickState state = {};
    nav_state_compact_t out = {};

    // Start accumulating in window
    for (unsigned i = 0; i < CONFLICT_WINDOW_TICKS - 1; ++i) {
        nav_tick_update(state, make_wheel_slip_snap(1.0f), &out);
    }

    // Window accumulators should have values
    EXPECT_GT(state.accumulated_encoder_delta, 0.0f);

    // Sensor failure should reset window (can't cross-validate)
    nav_tick_update(state, make_imu_fail_snap(), &out);

    EXPECT_FLOAT_NEAR(state.accumulated_encoder_delta, 0.0f);
    EXPECT_FLOAT_NEAR(state.accumulated_omega_magnitude, 0.0f);
    EXPECT_FLOAT_NEAR(state.accumulated_accel_magnitude, 0.0f);
    EXPECT_EQ(state.conflict_window_count, 0);
}
