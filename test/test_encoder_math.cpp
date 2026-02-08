/*
 * Unit tests for AS5600 encoder math
 * 12-bit offset, wraparound delta, tick-to-radian conversion
 */

#include <gtest/gtest.h>
#include <cmath>
#include "logic/encoder_math.hpp"

constexpr float PI = 3.14159265358979f;

// ============================================================================
// Test Suite: EncoderApplyOffset
// ============================================================================

TEST(EncoderApplyOffset, ZeroOffset) {
    EXPECT_EQ(encoder_apply_offset(1000, 0), 1000);
}

TEST(EncoderApplyOffset, NormalOffset) {
    EXPECT_EQ(encoder_apply_offset(1000, 500), 500);
}

TEST(EncoderApplyOffset, WraparoundOffset) {
    // raw=100, offset=200 → (100 - 200) & 0x0FFF = 0xFFFFFF9C & 0x0FFF = 3996
    EXPECT_EQ(encoder_apply_offset(100, 200), 3996);
}

TEST(EncoderApplyOffset, BothZero) {
    EXPECT_EQ(encoder_apply_offset(0, 0), 0);
}

TEST(EncoderApplyOffset, MaxValues) {
    // raw=4095, offset=4095 → 0
    EXPECT_EQ(encoder_apply_offset(4095, 4095), 0);
}

// ============================================================================
// Test Suite: EncoderShortestDelta
// ============================================================================

TEST(EncoderShortestDelta, NormalForward) {
    EXPECT_EQ(encoder_shortest_delta(200, 100), 100);
}

TEST(EncoderShortestDelta, NormalBackward) {
    EXPECT_EQ(encoder_shortest_delta(100, 200), -100);
}

TEST(EncoderShortestDelta, WraparoundForward) {
    // 4090→5: raw delta = 5 - 4090 = -4085 as int16_t
    // Since -4085 < -2048: delta += 4096 → 11
    EXPECT_EQ(encoder_shortest_delta(5, 4090), 11);
}

TEST(EncoderShortestDelta, WraparoundBackward) {
    // 5→4090: raw delta = 4090 - 5 = 4085 as int16_t
    // Since 4085 > 2048: delta -= 4096 → -11
    EXPECT_EQ(encoder_shortest_delta(4090, 5), -11);
}

TEST(EncoderShortestDelta, SameValue) {
    EXPECT_EQ(encoder_shortest_delta(2000, 2000), 0);
}

TEST(EncoderShortestDelta, ExactHalfRevForward) {
    // Delta of exactly 2048 should stay as 2048 (> check, not >=)
    EXPECT_EQ(encoder_shortest_delta(2048, 0), 2048);
}

TEST(EncoderShortestDelta, BeyondHalfRevForward) {
    // Delta of 2049 → 2049 > 2048, wraps to -2047
    EXPECT_EQ(encoder_shortest_delta(2049, 0), -2047);
}

// ============================================================================
// Test Suite: EncoderTicksToRad
// ============================================================================

TEST(EncoderTicksToRad, FullRevolution) {
    float rad = encoder_ticks_to_rad(4096);
    EXPECT_NEAR(rad, 2.0f * PI, 1e-4f);
}

TEST(EncoderTicksToRad, SingleTick) {
    float rad = encoder_ticks_to_rad(1);
    EXPECT_NEAR(rad, 2.0f * PI / 4096.0f, 1e-8f);
}

TEST(EncoderTicksToRad, ZeroTicks) {
    EXPECT_FLOAT_EQ(encoder_ticks_to_rad(0), 0.0f);
}

TEST(EncoderTicksToRad, NegativeTicks) {
    float rad = encoder_ticks_to_rad(-100);
    EXPECT_NEAR(rad, -100.0f * 2.0f * PI / 4096.0f, 1e-6f);
}
