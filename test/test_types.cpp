/*
 * Unit tests for CalibrationResult logic in types.h
 */

#include <gtest/gtest.h>
#include "types.h"

// ============================================================================
// Test Suite: CalibrationResultAllPassed
// ============================================================================

TEST(CalibrationResultAllPassed, AllSuccess) {
    CalibrationResult r;
    r.encoder.status = CalibrationStatus::Success;
    r.imu.status     = CalibrationStatus::Success;
    r.depth.status   = CalibrationStatus::Success;

    EXPECT_TRUE(r.all_passed());
}

TEST(CalibrationResultAllPassed, OneWarningRestSuccess) {
    CalibrationResult r;
    r.encoder.status = CalibrationStatus::Success;
    r.imu.status     = CalibrationStatus::Warning;
    r.depth.status   = CalibrationStatus::Success;

    EXPECT_FALSE(r.all_passed());
}

TEST(CalibrationResultAllPassed, OneFailed) {
    CalibrationResult r;
    r.encoder.status = CalibrationStatus::Failed;
    r.imu.status     = CalibrationStatus::Success;
    r.depth.status   = CalibrationStatus::Success;

    EXPECT_FALSE(r.all_passed());
}

TEST(CalibrationResultAllPassed, AllNotRun) {
    CalibrationResult r;
    // Default is NotRun

    EXPECT_FALSE(r.all_passed());
}

// ============================================================================
// Test Suite: CalibrationResultAnyFatal
// ============================================================================

TEST(CalibrationResultAnyFatal, AllSuccess) {
    CalibrationResult r;
    r.encoder.status = CalibrationStatus::Success;
    r.imu.status     = CalibrationStatus::Success;
    r.depth.status   = CalibrationStatus::Success;

    EXPECT_FALSE(r.any_fatal());
}

TEST(CalibrationResultAnyFatal, OneFailedRestSuccess) {
    CalibrationResult r;
    r.encoder.status = CalibrationStatus::Success;
    r.imu.status     = CalibrationStatus::Failed;
    r.depth.status   = CalibrationStatus::Success;

    EXPECT_TRUE(r.any_fatal());
}

TEST(CalibrationResultAnyFatal, AllFailed) {
    CalibrationResult r;
    r.encoder.status = CalibrationStatus::Failed;
    r.imu.status     = CalibrationStatus::Failed;
    r.depth.status   = CalibrationStatus::Failed;

    EXPECT_TRUE(r.any_fatal());
}

TEST(CalibrationResultAnyFatal, OneWarningOnly) {
    CalibrationResult r;
    r.encoder.status = CalibrationStatus::Success;
    r.imu.status     = CalibrationStatus::Warning;
    r.depth.status   = CalibrationStatus::Success;

    EXPECT_FALSE(r.any_fatal());
}
