#include <gtest/gtest.h>
#include <cmath>
#include "logic/nav_math.hpp"

constexpr double TOL = 1e-10;
constexpr double COS45 = 0.7071067811865476; // cos(π/4)
constexpr double SIN45 = 0.7071067811865476; // sin(π/4)

// Helper to check if two doubles are approximately equal
#define EXPECT_DOUBLE_NEAR(val1, val2) EXPECT_NEAR(val1, val2, TOL)

// ============================================================================
// Test Suite: QuaternionToRotationMatrix
// ============================================================================

TEST(QuaternionToRotationMatrix, IdentityQuaternion) {
    Quat q = {.w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f};
    double R[3][3];
    quaternion_to_rotation_matrix(q, R);

    // Verify identity matrix
    EXPECT_DOUBLE_NEAR(R[0][0], 1.0);
    EXPECT_DOUBLE_NEAR(R[0][1], 0.0);
    EXPECT_DOUBLE_NEAR(R[0][2], 0.0);
    EXPECT_DOUBLE_NEAR(R[1][0], 0.0);
    EXPECT_DOUBLE_NEAR(R[1][1], 1.0);
    EXPECT_DOUBLE_NEAR(R[1][2], 0.0);
    EXPECT_DOUBLE_NEAR(R[2][0], 0.0);
    EXPECT_DOUBLE_NEAR(R[2][1], 0.0);
    EXPECT_DOUBLE_NEAR(R[2][2], 1.0);
}

TEST(QuaternionToRotationMatrix, Rotation90Z) {
    Quat q = {.w = static_cast<float>(COS45), .x = 0.0f, .y = 0.0f, .z = static_cast<float>(SIN45)};
    double R[3][3];
    quaternion_to_rotation_matrix(q, R);

    // 90° rotation about Z-axis: [[0,-1,0],[1,0,0],[0,0,1]]
    EXPECT_DOUBLE_NEAR(R[0][0], 0.0);
    EXPECT_DOUBLE_NEAR(R[0][1], -1.0);
    EXPECT_DOUBLE_NEAR(R[0][2], 0.0);
    EXPECT_DOUBLE_NEAR(R[1][0], 1.0);
    EXPECT_DOUBLE_NEAR(R[1][1], 0.0);
    EXPECT_DOUBLE_NEAR(R[1][2], 0.0);
    EXPECT_DOUBLE_NEAR(R[2][0], 0.0);
    EXPECT_DOUBLE_NEAR(R[2][1], 0.0);
    EXPECT_DOUBLE_NEAR(R[2][2], 1.0);
}

TEST(QuaternionToRotationMatrix, Rotation90X) {
    Quat q = {.w = static_cast<float>(COS45), .x = static_cast<float>(SIN45), .y = 0.0f, .z = 0.0f};
    double R[3][3];
    quaternion_to_rotation_matrix(q, R);

    // 90° rotation about X-axis: [[1,0,0],[0,0,-1],[0,1,0]]
    EXPECT_DOUBLE_NEAR(R[0][0], 1.0);
    EXPECT_DOUBLE_NEAR(R[0][1], 0.0);
    EXPECT_DOUBLE_NEAR(R[0][2], 0.0);
    EXPECT_DOUBLE_NEAR(R[1][0], 0.0);
    EXPECT_DOUBLE_NEAR(R[1][1], 0.0);
    EXPECT_DOUBLE_NEAR(R[1][2], -1.0);
    EXPECT_DOUBLE_NEAR(R[2][0], 0.0);
    EXPECT_DOUBLE_NEAR(R[2][1], 1.0);
    EXPECT_DOUBLE_NEAR(R[2][2], 0.0);
}

TEST(QuaternionToRotationMatrix, Rotation90Y) {
    Quat q = {.w = static_cast<float>(COS45), .x = 0.0f, .y = static_cast<float>(SIN45), .z = 0.0f};
    double R[3][3];
    quaternion_to_rotation_matrix(q, R);

    // 90° rotation about Y-axis: [[0,0,1],[0,1,0],[-1,0,0]]
    EXPECT_DOUBLE_NEAR(R[0][0], 0.0);
    EXPECT_DOUBLE_NEAR(R[0][1], 0.0);
    EXPECT_DOUBLE_NEAR(R[0][2], 1.0);
    EXPECT_DOUBLE_NEAR(R[1][0], 0.0);
    EXPECT_DOUBLE_NEAR(R[1][1], 1.0);
    EXPECT_DOUBLE_NEAR(R[1][2], 0.0);
    EXPECT_DOUBLE_NEAR(R[2][0], -1.0);
    EXPECT_DOUBLE_NEAR(R[2][1], 0.0);
    EXPECT_DOUBLE_NEAR(R[2][2], 0.0);
}

TEST(QuaternionToRotationMatrix, Rotation180Z) {
    Quat q = {.w = 0.0f, .x = 0.0f, .y = 0.0f, .z = 1.0f};
    double R[3][3];
    quaternion_to_rotation_matrix(q, R);

    // 180° rotation about Z-axis: [[-1,0,0],[0,-1,0],[0,0,1]]
    EXPECT_DOUBLE_NEAR(R[0][0], -1.0);
    EXPECT_DOUBLE_NEAR(R[0][1], 0.0);
    EXPECT_DOUBLE_NEAR(R[0][2], 0.0);
    EXPECT_DOUBLE_NEAR(R[1][0], 0.0);
    EXPECT_DOUBLE_NEAR(R[1][1], -1.0);
    EXPECT_DOUBLE_NEAR(R[1][2], 0.0);
    EXPECT_DOUBLE_NEAR(R[2][0], 0.0);
    EXPECT_DOUBLE_NEAR(R[2][1], 0.0);
    EXPECT_DOUBLE_NEAR(R[2][2], 1.0);
}

TEST(QuaternionToRotationMatrix, DegenerateZeroQuaternion) {
    Quat q = {.w = 0.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f};
    double R[3][3];
    quaternion_to_rotation_matrix(q, R);

    // Should return identity matrix due to threshold guard
    EXPECT_DOUBLE_NEAR(R[0][0], 1.0);
    EXPECT_DOUBLE_NEAR(R[0][1], 0.0);
    EXPECT_DOUBLE_NEAR(R[0][2], 0.0);
    EXPECT_DOUBLE_NEAR(R[1][0], 0.0);
    EXPECT_DOUBLE_NEAR(R[1][1], 1.0);
    EXPECT_DOUBLE_NEAR(R[1][2], 0.0);
    EXPECT_DOUBLE_NEAR(R[2][0], 0.0);
    EXPECT_DOUBLE_NEAR(R[2][1], 0.0);
    EXPECT_DOUBLE_NEAR(R[2][2], 1.0);
}

TEST(QuaternionToRotationMatrix, NonUnitQuaternionNormalized) {
    Quat q = {.w = 2.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f};
    double R[3][3];
    quaternion_to_rotation_matrix(q, R);

    // Should normalize to identity quaternion
    EXPECT_DOUBLE_NEAR(R[0][0], 1.0);
    EXPECT_DOUBLE_NEAR(R[0][1], 0.0);
    EXPECT_DOUBLE_NEAR(R[0][2], 0.0);
    EXPECT_DOUBLE_NEAR(R[1][0], 0.0);
    EXPECT_DOUBLE_NEAR(R[1][1], 1.0);
    EXPECT_DOUBLE_NEAR(R[1][2], 0.0);
    EXPECT_DOUBLE_NEAR(R[2][0], 0.0);
    EXPECT_DOUBLE_NEAR(R[2][1], 0.0);
    EXPECT_DOUBLE_NEAR(R[2][2], 1.0);
}

TEST(QuaternionToRotationMatrix, Orthogonality) {
    Quat q = {.w = 0.9f, .x = 0.1f, .y = 0.2f, .z = 0.3f};
    double R[3][3];
    quaternion_to_rotation_matrix(q, R);

    // Compute R^T * R
    double RTR[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            RTR[i][j] = 0.0;
            for (int k = 0; k < 3; ++k) {
                RTR[i][j] += R[k][i] * R[k][j];
            }
        }
    }

    // Verify R^T * R ≈ I
    EXPECT_DOUBLE_NEAR(RTR[0][0], 1.0);
    EXPECT_DOUBLE_NEAR(RTR[0][1], 0.0);
    EXPECT_DOUBLE_NEAR(RTR[0][2], 0.0);
    EXPECT_DOUBLE_NEAR(RTR[1][0], 0.0);
    EXPECT_DOUBLE_NEAR(RTR[1][1], 1.0);
    EXPECT_DOUBLE_NEAR(RTR[1][2], 0.0);
    EXPECT_DOUBLE_NEAR(RTR[2][0], 0.0);
    EXPECT_DOUBLE_NEAR(RTR[2][1], 0.0);
    EXPECT_DOUBLE_NEAR(RTR[2][2], 1.0);

    // Compute det(R)
    double det = R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
               - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
               + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]);

    // Verify det(R) ≈ 1
    EXPECT_DOUBLE_NEAR(det, 1.0);
}

TEST(QuaternionToRotationMatrix, NearZeroQuaternion) {
    Quat q = {.w = 1e-5f, .x = 0.0f, .y = 0.0f, .z = 0.0f};
    double R[3][3];
    quaternion_to_rotation_matrix(q, R);

    // norm_sq = 1e-10 > 1e-12, so normalization path runs
    // Should normalize to identity (w dominates)
    EXPECT_DOUBLE_NEAR(R[0][0], 1.0);
    EXPECT_DOUBLE_NEAR(R[0][1], 0.0);
    EXPECT_DOUBLE_NEAR(R[0][2], 0.0);
    EXPECT_DOUBLE_NEAR(R[1][0], 0.0);
    EXPECT_DOUBLE_NEAR(R[1][1], 1.0);
    EXPECT_DOUBLE_NEAR(R[1][2], 0.0);
    EXPECT_DOUBLE_NEAR(R[2][0], 0.0);
    EXPECT_DOUBLE_NEAR(R[2][1], 0.0);
    EXPECT_DOUBLE_NEAR(R[2][2], 1.0);
}

// ============================================================================
// Test Suite: ComputeDisplacement
// ============================================================================

TEST(ComputeDisplacement, IdentityForward) {
    double R[3][3] = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };
    double dx, dy, dz;
    compute_displacement(R, 1.0f, &dx, &dy, &dz);

    EXPECT_DOUBLE_NEAR(dx, 1.0);
    EXPECT_DOUBLE_NEAR(dy, 0.0);
    EXPECT_DOUBLE_NEAR(dz, 0.0);
}

TEST(ComputeDisplacement, Rotated90Z) {
    double R[3][3] = {
        {0.0, -1.0, 0.0},
        {1.0,  0.0, 0.0},
        {0.0,  0.0, 1.0}
    };
    double dx, dy, dz;
    compute_displacement(R, 1.0f, &dx, &dy, &dz);

    EXPECT_DOUBLE_NEAR(dx, 0.0);
    EXPECT_DOUBLE_NEAR(dy, 1.0);
    EXPECT_DOUBLE_NEAR(dz, 0.0);
}

TEST(ComputeDisplacement, ZeroDistance) {
    double R[3][3] = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };
    double dx, dy, dz;
    compute_displacement(R, 0.0f, &dx, &dy, &dz);

    EXPECT_DOUBLE_NEAR(dx, 0.0);
    EXPECT_DOUBLE_NEAR(dy, 0.0);
    EXPECT_DOUBLE_NEAR(dz, 0.0);
}

TEST(ComputeDisplacement, NegativeDistance) {
    double R[3][3] = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };
    double dx, dy, dz;
    compute_displacement(R, -2.5f, &dx, &dy, &dz);

    EXPECT_DOUBLE_NEAR(dx, -2.5);
    EXPECT_DOUBLE_NEAR(dy, 0.0);
    EXPECT_DOUBLE_NEAR(dz, 0.0);
}

TEST(ComputeDisplacement, ArbitraryRotation) {
    // 90° Y rotation
    double R[3][3] = {
        { 0.0, 0.0, 1.0},
        { 0.0, 1.0, 0.0},
        {-1.0, 0.0, 0.0}
    };
    double dx, dy, dz;
    compute_displacement(R, 3.0f, &dx, &dy, &dz);

    EXPECT_DOUBLE_NEAR(dx, 0.0);
    EXPECT_DOUBLE_NEAR(dy, 0.0);
    EXPECT_DOUBLE_NEAR(dz, -3.0);
}

// ============================================================================
// Test Suite: IntegratePosition
// ============================================================================

TEST(IntegratePosition, SingleStep) {
    double pos_x = 0.0, pos_y = 0.0, pos_z = 0.0;
    integrate_position(&pos_x, &pos_y, &pos_z, 1.5, -0.3, 0.7);

    EXPECT_DOUBLE_NEAR(pos_x, 1.5);
    EXPECT_DOUBLE_NEAR(pos_y, -0.3);
    EXPECT_DOUBLE_NEAR(pos_z, 0.7);
}

TEST(IntegratePosition, MultiStep) {
    double pos_x = 0.0, pos_y = 0.0, pos_z = 0.0;

    integrate_position(&pos_x, &pos_y, &pos_z, 1.0, 2.0, 3.0);
    integrate_position(&pos_x, &pos_y, &pos_z, 0.5, -1.0, 0.5);
    integrate_position(&pos_x, &pos_y, &pos_z, -0.5, 0.5, -1.0);

    EXPECT_DOUBLE_NEAR(pos_x, 1.0);
    EXPECT_DOUBLE_NEAR(pos_y, 1.5);
    EXPECT_DOUBLE_NEAR(pos_z, 2.5);
}

TEST(IntegratePosition, NegativeDisplacement) {
    double pos_x = 10.0, pos_y = 5.0, pos_z = 3.0;

    integrate_position(&pos_x, &pos_y, &pos_z, -2.0, -1.5, -0.5);

    EXPECT_DOUBLE_NEAR(pos_x, 8.0);
    EXPECT_DOUBLE_NEAR(pos_y, 3.5);
    EXPECT_DOUBLE_NEAR(pos_z, 2.5);
}

TEST(IntegratePosition, DoublePrecisionPreservation) {
    double pos_x = 1e6, pos_y = 0.0, pos_z = 0.0;

    // Add 1e-6 one thousand times
    for (int i = 0; i < 1000; ++i) {
        integrate_position(&pos_x, &pos_y, &pos_z, 1e-6, 0.0, 0.0);
    }

    // 1e6 + 1000 * 1e-6 = 1e6 + 1e-3 = 1000000.001
    // Use 1e-8 tolerance to account for accumulated floating-point error
    EXPECT_NEAR(pos_x, 1000000.001, 1e-8);
    EXPECT_DOUBLE_NEAR(pos_y, 0.0);
    EXPECT_DOUBLE_NEAR(pos_z, 0.0);
}

// ============================================================================
// Test Suite: ExtractHeadingPitch
// ============================================================================

TEST(ExtractHeadingPitch, IdentityMatrix) {
    double R[3][3] = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };
    double heading, pitch;
    extract_heading_pitch(R, &heading, &pitch);

    EXPECT_DOUBLE_NEAR(heading, 0.0);
    EXPECT_DOUBLE_NEAR(pitch, 0.0);
}

TEST(ExtractHeadingPitch, Heading90Degrees) {
    double R[3][3] = {
        {0.0, -1.0, 0.0},
        {1.0,  0.0, 0.0},
        {0.0,  0.0, 1.0}
    };
    double heading, pitch;
    extract_heading_pitch(R, &heading, &pitch);

    EXPECT_DOUBLE_NEAR(heading, M_PI / 2.0);
    EXPECT_DOUBLE_NEAR(pitch, 0.0);
}

TEST(ExtractHeadingPitch, Heading180Degrees) {
    double R[3][3] = {
        {-1.0, 0.0, 0.0},
        { 0.0, -1.0, 0.0},
        { 0.0, 0.0, 1.0}
    };
    double heading, pitch;
    extract_heading_pitch(R, &heading, &pitch);

    EXPECT_NEAR(std::abs(heading), M_PI, TOL);
    EXPECT_DOUBLE_NEAR(pitch, 0.0);
}

TEST(ExtractHeadingPitch, PurePitch45Down) {
    // 45° pitch down: rotate Y by +45°
    double R[3][3] = {
        {COS45,   0.0, -SIN45},
        {  0.0,   1.0,    0.0},
        {SIN45,   0.0,  COS45}
    };
    double heading, pitch;
    extract_heading_pitch(R, &heading, &pitch);

    EXPECT_DOUBLE_NEAR(heading, 0.0);
    EXPECT_DOUBLE_NEAR(pitch, -M_PI / 4.0);
}

TEST(ExtractHeadingPitch, NearGimbalLock) {
    // Exactly 90° Y rotation (gimbal lock condition)
    double R[3][3] = {
        { 0.0, 0.0, 1.0},
        { 0.0, 1.0, 0.0},
        {-1.0, 0.0, 0.0}
    };
    double heading, pitch;
    extract_heading_pitch(R, &heading, &pitch);

    // Verify both are finite (not NaN/Inf)
    EXPECT_TRUE(std::isfinite(heading));
    EXPECT_TRUE(std::isfinite(pitch));

    // heading = atan2(0, 0) = 0, pitch = atan2(1, 0) = π/2
    EXPECT_DOUBLE_NEAR(heading, 0.0);
    EXPECT_DOUBLE_NEAR(pitch, M_PI / 2.0);
}

TEST(ExtractHeadingPitch, FullPipelineRoundtrip) {
    // Create quaternion for 22.5° Z rotation (π/8 half-angle)
    Quat q = {
        .w = static_cast<float>(std::cos(M_PI / 8.0)),
        .x = 0.0f,
        .y = 0.0f,
        .z = static_cast<float>(std::sin(M_PI / 8.0))
    };

    double R[3][3];
    quaternion_to_rotation_matrix(q, R);

    double heading, pitch;
    extract_heading_pitch(R, &heading, &pitch);

    // Should represent 45° heading (π/4) with zero pitch
    // Use 1e-7 tolerance due to float->double conversion in quaternion pipeline
    EXPECT_NEAR(heading, M_PI / 4.0, 1e-7);
    EXPECT_DOUBLE_NEAR(pitch, 0.0);
}

// ============================================================================
// Test Suite: QuaternionExtrapolate
// ============================================================================

TEST(QuaternionExtrapolate, ZeroOmegaReturnsInput) {
    Quat q_curr = {1.0f, 0.0f, 0.0f, 0.0f};
    Vec3 omega = {0.0f, 0.0f, 0.0f};
    Quat q_out;

    quaternion_extrapolate(&q_curr, &omega, 0.01f, &q_out);

    // With zero omega, quaternion should remain unchanged (identity)
    EXPECT_NEAR(q_out.w, 1.0f, 1e-6f);
    EXPECT_NEAR(q_out.x, 0.0f, 1e-6f);
    EXPECT_NEAR(q_out.y, 0.0f, 1e-6f);
    EXPECT_NEAR(q_out.z, 0.0f, 1e-6f);
}

TEST(QuaternionExtrapolate, PureYawRotation) {
    Quat q_curr = {1.0f, 0.0f, 0.0f, 0.0f};
    Vec3 omega = {0.0f, 0.0f, 1.0f};  // 1 rad/s yaw
    Quat q_out;

    quaternion_extrapolate(&q_curr, &omega, 0.01f, &q_out);

    // For omega_z=1.0, dt=0.01, we expect:
    // Pure yaw quaternion: (0, 0, 0, 1) represents the angular velocity
    // Hamilton product: q + 0.5*dt*(0,0,0,1)*q
    // q_dot has components: pw = 0.5*dt*0*1 = 0, pz = 0.5*dt*1*1 = 0.005
    // After normalization: w slightly less than 1.0, z slightly greater than 0
    EXPECT_LT(q_out.w, 1.0f);
    EXPECT_GT(q_out.z, 0.0f);

    // Verify rotation is in the expected direction (positive yaw)
    EXPECT_NEAR(q_out.x, 0.0f, 1e-6f);
    EXPECT_NEAR(q_out.y, 0.0f, 1e-6f);
}

TEST(QuaternionExtrapolate, OutputIsUnitLength) {
    // Arbitrary non-identity quaternion
    Quat q_curr = {0.9f, 0.1f, 0.2f, 0.3f};
    // Arbitrary angular velocity
    Vec3 omega = {0.5f, -0.3f, 0.7f};
    Quat q_out;

    quaternion_extrapolate(&q_curr, &omega, 0.01f, &q_out);

    // Verify output is unit length
    float norm = std::sqrt(q_out.w * q_out.w + q_out.x * q_out.x +
                           q_out.y * q_out.y + q_out.z * q_out.z);
    EXPECT_NEAR(norm, 1.0f, 1e-6f);
}

TEST(QuaternionExtrapolate, DegenerateInputFallsBackToIdentity) {
    // Zero quaternion (degenerate)
    Quat q_curr = {0.0f, 0.0f, 0.0f, 0.0f};
    Vec3 omega = {1.0f, 1.0f, 1.0f};
    Quat q_out;

    quaternion_extrapolate(&q_curr, &omega, 0.01f, &q_out);

    // Degenerate guard should trigger, returning identity
    EXPECT_NEAR(q_out.w, 1.0f, 1e-6f);
    EXPECT_NEAR(q_out.x, 0.0f, 1e-6f);
    EXPECT_NEAR(q_out.y, 0.0f, 1e-6f);
    EXPECT_NEAR(q_out.z, 0.0f, 1e-6f);
}

TEST(QuaternionExtrapolate, LargeOmegaNormalizes) {
    Quat q_curr = {1.0f, 0.0f, 0.0f, 0.0f};
    // Very large angular velocity (100 rad/s)
    Vec3 omega = {100.0f, 100.0f, 100.0f};
    Quat q_out;

    quaternion_extrapolate(&q_curr, &omega, 0.01f, &q_out);

    // Even with large omega, output should be normalized
    float norm = std::sqrt(q_out.w * q_out.w + q_out.x * q_out.x +
                           q_out.y * q_out.y + q_out.z * q_out.z);
    EXPECT_NEAR(norm, 1.0f, 1e-6f);

    // Verify no component is NaN or Inf
    EXPECT_TRUE(std::isfinite(q_out.w));
    EXPECT_TRUE(std::isfinite(q_out.x));
    EXPECT_TRUE(std::isfinite(q_out.y));
    EXPECT_TRUE(std::isfinite(q_out.z));
}
