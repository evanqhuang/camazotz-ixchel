/*
 * Unit tests for depth_recovery tiered fallback logic
 * Tests: NormalOperation, Tier1, Tier2, Slew, Tier3, EdgeCases
 */

#include <gtest/gtest.h>
#include <cmath>
#include "logic/depth_recovery.hpp"
#include "logic/nav_math.hpp"
#include "config.h"

constexpr double TOL = 1e-10;
constexpr double PI = 3.14159265358979323846;

// ============================================================================
// Test Helpers
// ============================================================================

static DepthSnapshot make_valid_depth(float depth_m, float pitch_rad, float delta_dist) {
    return {
        .new_reading = true,
        .depth_m = depth_m,
        .pitch_rad = pitch_rad,
        .delta_dist = delta_dist
    };
}

static DepthSnapshot make_no_reading(float pitch_rad, float delta_dist) {
    return {
        .new_reading = false,
        .depth_m = 0.0f,
        .pitch_rad = pitch_rad,
        .delta_dist = delta_dist
    };
}

// Helper: extract pitch from a quaternion (for test setup validation)
static float pitch_from_quat(float qw, float qx, float qy, float qz) {
    Quat q = {qw, qx, qy, qz};
    double R[3][3];
    quaternion_to_rotation_matrix(q, R);
    double heading, pitch;
    extract_heading_pitch(R, &heading, &pitch);
    return static_cast<float>(pitch);
}

// ============================================================================
// Test Suite: DepthRecovery_NormalOperation
// ============================================================================

TEST(DepthRecovery_NormalOperation, ContinuousValidReadings) {
    DepthRecoveryState state = {};
    double z_out = 0.0;

    // Feed 10 valid readings at different depths
    for (int i = 0; i < 10; ++i) {
        float depth = 5.0f + static_cast<float>(i) * 0.5f;
        DepthSnapshot snap = make_valid_depth(depth, 0.0f, 0.01f);
        uint8_t flags = depth_recovery_update(state, snap, &z_out);

        EXPECT_NEAR(z_out, depth, TOL);
        EXPECT_EQ(flags, 0);
        EXPECT_EQ(state.ticks_since_valid, 0U);
    }
}

TEST(DepthRecovery_NormalOperation, GracePeriodNoFalseTriggering) {
    DepthRecoveryState state = {};
    double z_out = 0.0;

    // First valid reading establishes baseline
    DepthSnapshot snap_init = make_valid_depth(10.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_init, &z_out);

    // Stop readings for 9 ticks (< GRACE=10)
    for (int i = 0; i < 9; ++i) {
        DepthSnapshot snap = make_no_reading(0.1f, 0.01f);
        uint8_t flags = depth_recovery_update(state, snap, &z_out);

        EXPECT_NEAR(z_out, 10.0, TOL);  // Still using last sensor Z
        EXPECT_EQ(flags, 0);            // No DEPTH_VIRTUAL
    }
}

TEST(DepthRecovery_NormalOperation, FirstReadingInitializesState) {
    DepthRecoveryState state = {};
    double z_out = 0.0;

    EXPECT_EQ(state.last_sensor_z, 0.0);
    EXPECT_EQ(state.virtual_z, 0.0);

    DepthSnapshot snap = make_valid_depth(15.5f, 0.0f, 0.0f);
    uint8_t flags = depth_recovery_update(state, snap, &z_out);

    EXPECT_NEAR(z_out, 15.5, TOL);
    EXPECT_NEAR(state.last_sensor_z, 15.5, TOL);
    EXPECT_NEAR(state.virtual_z, 15.5, TOL);
    EXPECT_EQ(flags, 0);
}

// ============================================================================
// Test Suite: DepthRecovery_Tier1
// ============================================================================

TEST(DepthRecovery_Tier1, EntryAtGraceBoundary) {
    DepthRecoveryState state = {};
    double z_out = 0.0;

    // Initialize with valid reading
    DepthSnapshot snap_init = make_valid_depth(5.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_init, &z_out);

    // Stop readings for exactly GRACE ticks (10)
    for (uint32_t i = 0; i < DEPTH_FAIL_GRACE_TICKS; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        uint8_t flags = depth_recovery_update(state, snap, &z_out);

        if (i < DEPTH_FAIL_GRACE_TICKS - 1) {
            EXPECT_EQ(flags, 0);  // Still in grace
        } else {
            EXPECT_EQ(flags, NAV_FLAG_DEPTH_VIRTUAL);  // Tier 1 entered
        }
    }
}

TEST(DepthRecovery_Tier1, GeometricAccuracy_45DegreePitch) {
    DepthRecoveryState state = {};
    double z_out = 0.0;

    // Initialize at z=0
    DepthSnapshot snap_init = make_valid_depth(0.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_init, &z_out);

    // Enter Tier 1 (skip grace period)
    for (uint32_t i = 0; i < DEPTH_FAIL_GRACE_TICKS; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }

    // Now in Tier 1: feed 100 ticks with pitch=45deg, delta_dist=0.01m each
    float pitch_45 = static_cast<float>(PI / 4.0);  // sin(45deg) ≈ 0.7071
    for (int i = 0; i < 100; ++i) {
        DepthSnapshot snap = make_no_reading(pitch_45, 0.01f);
        depth_recovery_update(state, snap, &z_out);
    }

    // Expected Z: 100 * 0.01 * sin(45deg) ≈ 0.7071
    double expected_z = 100.0 * 0.01 * sin(PI / 4.0);
    EXPECT_NEAR(z_out, expected_z, 1e-8);
}

TEST(DepthRecovery_Tier1, ZeroPitchNoZChange) {
    DepthRecoveryState state = {};
    double z_out = 0.0;

    // Initialize at z=5.0
    DepthSnapshot snap_init = make_valid_depth(5.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_init, &z_out);

    // Enter Tier 1
    for (uint32_t i = 0; i < DEPTH_FAIL_GRACE_TICKS; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }

    // 50 ticks with pitch=0, delta_dist=1.0m (large distance, but sin(0)=0)
    for (int i = 0; i < 50; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 1.0f);
        depth_recovery_update(state, snap, &z_out);
    }

    // Z should still be 5.0 (no change)
    EXPECT_NEAR(z_out, 5.0, TOL);
}

TEST(DepthRecovery_Tier1, RecoverySnap) {
    DepthRecoveryState state = {};
    double z_out = 0.0;

    // Initialize at z=0
    DepthSnapshot snap_init = make_valid_depth(0.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_init, &z_out);

    // Enter Tier 1 and accumulate some geometric Z
    for (uint32_t i = 0; i < DEPTH_FAIL_GRACE_TICKS + 50; ++i) {
        DepthSnapshot snap = make_no_reading(0.5f, 0.01f);  // Accumulate Z
        depth_recovery_update(state, snap, &z_out);
    }
    EXPECT_GT(z_out, 0.0);  // Geometric Z accumulated

    // Sensor returns with reading of 10.0m (Tier 1 recovery: snap)
    DepthSnapshot snap_recover = make_valid_depth(10.0f, 0.0f, 0.0f);
    uint8_t flags = depth_recovery_update(state, snap_recover, &z_out);

    EXPECT_NEAR(z_out, 10.0, TOL);  // Snapped to sensor
    EXPECT_EQ(flags, 0);            // All flags cleared
    EXPECT_EQ(state.ticks_since_valid, 0U);
}

// ============================================================================
// Test Suite: DepthRecovery_Tier2
// ============================================================================

TEST(DepthRecovery_Tier2, EntryAt200Ticks) {
    DepthRecoveryState state = {};
    double z_out = 0.0;

    // Initialize
    DepthSnapshot snap_init = make_valid_depth(0.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_init, &z_out);

    // Run to just before Tier 2 boundary (199 ticks)
    for (uint32_t i = 0; i < DEPTH_TIER2_THRESHOLD_TICKS - 1; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }
    EXPECT_LT(state.ticks_since_valid, DEPTH_TIER2_THRESHOLD_TICKS);

    // 200th tick enters Tier 2
    DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
    uint8_t flags = depth_recovery_update(state, snap, &z_out);

    EXPECT_EQ(state.ticks_since_valid, DEPTH_TIER2_THRESHOLD_TICKS);
    EXPECT_EQ(flags, NAV_FLAG_DEPTH_VIRTUAL);  // Still geometric, same flag
}

TEST(DepthRecovery_Tier2, RecoveryInitiatesSlew) {
    DepthRecoveryState state = {};
    double z_out = 0.0;

    // Initialize at z=0, accumulate geometric to ~5m
    DepthSnapshot snap_init = make_valid_depth(0.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_init, &z_out);

    // Enter Tier 2 with geometric Z accumulation
    for (uint32_t i = 0; i < DEPTH_TIER2_THRESHOLD_TICKS + 10; ++i) {
        float pitch = static_cast<float>(PI / 6.0);  // 30 deg
        DepthSnapshot snap = make_no_reading(pitch, 0.01f);
        depth_recovery_update(state, snap, &z_out);
    }
    double virtual_z_before = state.virtual_z;
    EXPECT_FALSE(state.slew_active);

    // Sensor returns with 20.0m (Tier 2 recovery: slew)
    DepthSnapshot snap_recover = make_valid_depth(20.0f, 0.0f, 0.0f);
    uint8_t flags = depth_recovery_update(state, snap_recover, &z_out);

    EXPECT_TRUE(state.slew_active);
    EXPECT_NEAR(state.slew_start_z, virtual_z_before, TOL);
    EXPECT_NEAR(state.slew_target_z, 20.0, TOL);
    EXPECT_NEAR(z_out, virtual_z_before, TOL);  // First tick of slew
    EXPECT_EQ(flags, NAV_FLAG_DEPTH_VIRTUAL);
}

TEST(DepthRecovery_Tier2, SlewMidpoint) {
    DepthRecoveryState state = {};
    double z_out = 0.0;

    // Initialize at z=0
    DepthSnapshot snap_init = make_valid_depth(0.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_init, &z_out);

    // Enter Tier 2 with virtual_z = 0 (no pitch)
    for (uint32_t i = 0; i < DEPTH_TIER2_THRESHOLD_TICKS + 10; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }
    EXPECT_NEAR(state.virtual_z, 0.0, TOL);

    // Sensor returns with 10.0m → slew from 0 to 10
    DepthSnapshot snap_recover = make_valid_depth(10.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_recover, &z_out);
    EXPECT_TRUE(state.slew_active);

    // Run to midpoint (100 ticks into 200-tick slew)
    for (uint32_t i = 0; i < DEPTH_SLEW_DURATION_TICKS / 2; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }

    // At 50% through slew, Z should be ~5.0
    EXPECT_NEAR(z_out, 5.0, 0.1);
}

TEST(DepthRecovery_Tier2, SlewCompletion) {
    DepthRecoveryState state = {};
    double z_out = 0.0;

    // Initialize and enter Tier 2 with virtual_z ≈ 0
    DepthSnapshot snap_init = make_valid_depth(0.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_init, &z_out);

    for (uint32_t i = 0; i < DEPTH_TIER2_THRESHOLD_TICKS + 10; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }

    // Sensor returns with 10.0m → slew from 0 to 10
    DepthSnapshot snap_recover = make_valid_depth(10.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_recover, &z_out);

    // Run slew to completion
    for (uint32_t i = 0; i < DEPTH_SLEW_DURATION_TICKS; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }

    // Slew complete: Z = sensor, all flags cleared
    EXPECT_NEAR(z_out, 10.0, TOL);
    EXPECT_FALSE(state.slew_active);
    EXPECT_EQ(state.ticks_since_valid, 0U);
}

// ============================================================================
// Test Suite: DepthRecovery_Slew
// ============================================================================

TEST(DepthRecovery_Slew, TargetUpdateDuringSlew) {
    DepthRecoveryState state = {};
    double z_out = 0.0;

    // Initialize and enter Tier 2
    DepthSnapshot snap_init = make_valid_depth(0.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_init, &z_out);

    for (uint32_t i = 0; i < DEPTH_TIER2_THRESHOLD_TICKS + 10; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }

    // Start slew to 10.0m
    DepthSnapshot snap_recover1 = make_valid_depth(10.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_recover1, &z_out);
    EXPECT_NEAR(state.slew_target_z, 10.0, TOL);

    // Run 50 ticks into slew
    for (int i = 0; i < 50; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }

    // New sensor reading updates target to 15.0m
    DepthSnapshot snap_recover2 = make_valid_depth(15.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_recover2, &z_out);

    EXPECT_NEAR(state.slew_target_z, 15.0, TOL);
    EXPECT_TRUE(state.slew_active);  // Still slewing
}

TEST(DepthRecovery_Slew, VirtualZTrackedDuringSlew) {
    DepthRecoveryState state = {};
    double z_out = 0.0;

    // Initialize and enter Tier 2
    DepthSnapshot snap_init = make_valid_depth(0.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_init, &z_out);

    for (uint32_t i = 0; i < DEPTH_TIER2_THRESHOLD_TICKS + 10; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }
    double virtual_z_pre_slew = state.virtual_z;

    // Start slew
    DepthSnapshot snap_recover = make_valid_depth(10.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_recover, &z_out);

    // Run slew ticks with pitch motion (geometric Z should still accumulate)
    float pitch = static_cast<float>(PI / 4.0);
    for (int i = 0; i < 50; ++i) {
        DepthSnapshot snap = make_no_reading(pitch, 0.01f);
        depth_recovery_update(state, snap, &z_out);
    }

    // virtual_z should have accumulated
    double expected_accumulation = 50.0 * 0.01 * sin(PI / 4.0);
    EXPECT_NEAR(state.virtual_z, virtual_z_pre_slew + expected_accumulation, 1e-8);
}

TEST(DepthRecovery_Slew, SlewFromTier3Recovery) {
    DepthRecoveryState state = {};
    double z_out = 0.0;

    // Initialize and enter Tier 3
    DepthSnapshot snap_init = make_valid_depth(5.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_init, &z_out);

    for (uint32_t i = 0; i < DEPTH_TIER3_THRESHOLD_TICKS + 10; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }
    EXPECT_TRUE(state.ceiling_locked);
    double locked_z = state.locked_z;

    // Sensor returns → slew from locked_z
    DepthSnapshot snap_recover = make_valid_depth(20.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_recover, &z_out);

    EXPECT_TRUE(state.slew_active);
    EXPECT_NEAR(state.slew_start_z, locked_z, TOL);
    EXPECT_NEAR(state.slew_target_z, 20.0, TOL);
}

// ============================================================================
// Test Suite: DepthRecovery_Tier3
// ============================================================================

TEST(DepthRecovery_Tier3, EntryAt1000Ticks) {
    DepthRecoveryState state = {};
    double z_out = 0.0;

    // Initialize
    DepthSnapshot snap_init = make_valid_depth(5.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_init, &z_out);

    // Run to Tier 3 boundary (999 ticks)
    for (uint32_t i = 0; i < DEPTH_TIER3_THRESHOLD_TICKS - 1; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }
    EXPECT_FALSE(state.ceiling_locked);

    // 1000th tick enters Tier 3
    DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
    uint8_t flags = depth_recovery_update(state, snap, &z_out);

    EXPECT_TRUE(state.ceiling_locked);
    EXPECT_EQ(flags, NAV_FLAG_DEPTH_VIRTUAL | NAV_FLAG_DEPTH_UNVERIFIED | NAV_FLAG_NAV_CRITICAL);
}

TEST(DepthRecovery_Tier3, PositionFrozen) {
    DepthRecoveryState state = {};
    double z_out = 0.0;

    // Initialize and accumulate some geometric Z
    DepthSnapshot snap_init = make_valid_depth(0.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_init, &z_out);

    float pitch = static_cast<float>(PI / 4.0);
    for (uint32_t i = 0; i < DEPTH_TIER3_THRESHOLD_TICKS; ++i) {
        DepthSnapshot snap = make_no_reading(pitch, 0.01f);
        depth_recovery_update(state, snap, &z_out);
    }
    double locked_z = z_out;
    EXPECT_TRUE(state.ceiling_locked);

    // Continue with motion — Z should NOT change
    for (int i = 0; i < 100; ++i) {
        DepthSnapshot snap = make_no_reading(pitch, 0.1f);  // Large motion
        depth_recovery_update(state, snap, &z_out);
    }

    EXPECT_NEAR(z_out, locked_z, TOL);
}

TEST(DepthRecovery_Tier3, RecoveryInitiatesSlew) {
    DepthRecoveryState state = {};
    double z_out = 0.0;

    // Initialize and enter Tier 3
    DepthSnapshot snap_init = make_valid_depth(5.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_init, &z_out);

    for (uint32_t i = 0; i < DEPTH_TIER3_THRESHOLD_TICKS + 100; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }
    EXPECT_TRUE(state.ceiling_locked);

    // Sensor returns
    DepthSnapshot snap_recover = make_valid_depth(30.0f, 0.0f, 0.0f);
    uint8_t flags = depth_recovery_update(state, snap_recover, &z_out);

    EXPECT_TRUE(state.slew_active);
    EXPECT_EQ(flags, NAV_FLAG_DEPTH_VIRTUAL);  // During slew
}

// ============================================================================
// Test Suite: DepthRecovery_EdgeCases
// ============================================================================

TEST(DepthRecovery_EdgeCases, SteepPitch90Degrees) {
    DepthRecoveryState state = {};
    double z_out = 0.0;

    // Initialize at z=0
    DepthSnapshot snap_init = make_valid_depth(0.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_init, &z_out);

    // Enter Tier 1
    for (uint32_t i = 0; i < DEPTH_FAIL_GRACE_TICKS; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }

    // Pitch = 90deg (sin=1.0), delta_dist = 0.1m
    float pitch_90 = static_cast<float>(PI / 2.0);
    DepthSnapshot snap = make_no_reading(pitch_90, 0.1f);
    depth_recovery_update(state, snap, &z_out);

    // All distance goes to Z: delta_z = 0.1 * sin(90deg) = 0.1
    EXPECT_NEAR(z_out, 0.1, 1e-6);
}

TEST(DepthRecovery_EdgeCases, NegativePitchDiving) {
    DepthRecoveryState state = {};
    double z_out = 0.0;

    // Initialize at z=10.0
    DepthSnapshot snap_init = make_valid_depth(10.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_init, &z_out);

    // Enter Tier 1
    for (uint32_t i = 0; i < DEPTH_FAIL_GRACE_TICKS; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }

    // Negative pitch (diving): pitch = -30deg, delta_dist = 1.0m
    float pitch_neg30 = static_cast<float>(-PI / 6.0);
    for (int i = 0; i < 10; ++i) {
        DepthSnapshot snap = make_no_reading(pitch_neg30, 1.0f);
        depth_recovery_update(state, snap, &z_out);
    }

    // Expected: 10.0 + 10 * 1.0 * sin(-30deg) = 10.0 - 5.0 = 5.0
    double expected = 10.0 + 10.0 * 1.0 * sin(-PI / 6.0);
    EXPECT_NEAR(z_out, expected, 1e-6);
    EXPECT_LT(z_out, 10.0);  // Z decreased (diving)
}

TEST(DepthRecovery_EdgeCases, MultipleFailRecoverCycles) {
    DepthRecoveryState state = {};
    double z_out = 0.0;

    // Cycle 1: Normal → Tier 1 → snap recovery
    DepthSnapshot snap_init1 = make_valid_depth(5.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_init1, &z_out);

    for (uint32_t i = 0; i < DEPTH_FAIL_GRACE_TICKS + 50; ++i) {
        DepthSnapshot snap = make_no_reading(0.1f, 0.01f);
        depth_recovery_update(state, snap, &z_out);
    }
    EXPECT_NE(z_out, 5.0);  // Geometric accumulated

    DepthSnapshot snap_recover1 = make_valid_depth(7.0f, 0.0f, 0.0f);
    uint8_t flags1 = depth_recovery_update(state, snap_recover1, &z_out);
    EXPECT_NEAR(z_out, 7.0, TOL);  // Tier 1 snap
    EXPECT_EQ(flags1, 0);

    // Cycle 2: Normal → Tier 2 → slew recovery
    for (uint32_t i = 0; i < DEPTH_TIER2_THRESHOLD_TICKS + 50; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }

    DepthSnapshot snap_recover2 = make_valid_depth(15.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_recover2, &z_out);
    EXPECT_TRUE(state.slew_active);

    // Complete slew
    for (uint32_t i = 0; i < DEPTH_SLEW_DURATION_TICKS; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }
    EXPECT_NEAR(z_out, 15.0, TOL);
    EXPECT_FALSE(state.slew_active);

    // Cycle 3: Normal → Tier 3 → slew recovery
    for (uint32_t i = 0; i < DEPTH_TIER3_THRESHOLD_TICKS + 50; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }
    EXPECT_TRUE(state.ceiling_locked);

    DepthSnapshot snap_recover3 = make_valid_depth(25.0f, 0.0f, 0.0f);
    depth_recovery_update(state, snap_recover3, &z_out);
    EXPECT_TRUE(state.slew_active);

    // Complete slew
    for (uint32_t i = 0; i < DEPTH_SLEW_DURATION_TICKS; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }
    EXPECT_NEAR(z_out, 25.0, TOL);
    EXPECT_FALSE(state.slew_active);
    EXPECT_FALSE(state.ceiling_locked);
}

TEST(DepthRecovery_EdgeCases, FlagsCorrectlyPropagated) {
    DepthRecoveryState state = {};
    double z_out = 0.0;

    // Normal: flags = 0
    DepthSnapshot snap_init = make_valid_depth(5.0f, 0.0f, 0.0f);
    uint8_t flags_normal = depth_recovery_update(state, snap_init, &z_out);
    EXPECT_EQ(flags_normal, 0);

    // Tier 1: DEPTH_VIRTUAL only
    for (uint32_t i = 0; i < DEPTH_FAIL_GRACE_TICKS; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }
    uint8_t flags_tier1 = depth_recovery_update(state, make_no_reading(0.0f, 0.0f), &z_out);
    EXPECT_EQ(flags_tier1, NAV_FLAG_DEPTH_VIRTUAL);
    EXPECT_EQ(flags_tier1 & NAV_FLAG_DEPTH_UNVERIFIED, 0);
    EXPECT_EQ(flags_tier1 & NAV_FLAG_NAV_CRITICAL, 0);

    // Tier 2: DEPTH_VIRTUAL only
    for (uint32_t i = state.ticks_since_valid; i < DEPTH_TIER2_THRESHOLD_TICKS + 10; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }
    uint8_t flags_tier2 = depth_recovery_update(state, make_no_reading(0.0f, 0.0f), &z_out);
    EXPECT_EQ(flags_tier2, NAV_FLAG_DEPTH_VIRTUAL);
    EXPECT_EQ(flags_tier2 & NAV_FLAG_DEPTH_UNVERIFIED, 0);
    EXPECT_EQ(flags_tier2 & NAV_FLAG_NAV_CRITICAL, 0);

    // Tier 3: DEPTH_VIRTUAL | DEPTH_UNVERIFIED | NAV_CRITICAL
    for (uint32_t i = state.ticks_since_valid; i < DEPTH_TIER3_THRESHOLD_TICKS; ++i) {
        DepthSnapshot snap = make_no_reading(0.0f, 0.0f);
        depth_recovery_update(state, snap, &z_out);
    }
    uint8_t flags_tier3 = depth_recovery_update(state, make_no_reading(0.0f, 0.0f), &z_out);
    EXPECT_EQ(flags_tier3, NAV_FLAG_DEPTH_VIRTUAL | NAV_FLAG_DEPTH_UNVERIFIED | NAV_FLAG_NAV_CRITICAL);
}

// ============================================================================
// Test: Nav Math Integration (validate pitch extraction helper)
// ============================================================================

TEST(DepthRecovery_NavMathIntegration, PitchExtractionFromQuaternion) {
    // 45-degree pitch forward: quaternion for rotation about Y axis
    // q = cos(theta/2) + j*sin(theta/2) for pitch about Y
    float theta = static_cast<float>(PI / 4.0);  // 45 deg
    float qw = cosf(theta / 2.0f);
    float qy = sinf(theta / 2.0f);

    float pitch = pitch_from_quat(qw, 0.0f, qy, 0.0f);

    // Should be ~45 degrees = pi/4
    EXPECT_NEAR(pitch, PI / 4.0, 0.01);
}
