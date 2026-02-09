/*
 * Unit tests for battery voltage monitoring math
 * ADC conversion, EMA smoothing, piecewise LiPo percentage
 */

#include <gtest/gtest.h>
#include <cmath>
#include "logic/battery_math.hpp"

// ============================================================================
// Test Suite: BatteryMath_RawToVoltage
// ============================================================================

TEST(BatteryMath_RawToVoltage, Midscale) {
    // 2048 (half of 4096) with 3.3V ref and 3x divider
    // Expected: 2048 * 3.3 / 4096 * 3 = 4.95V
    float v = battery_raw_to_voltage(2048, 3.3f, 4096, 3.0f);
    EXPECT_NEAR(v, 4.95f, 0.01f);
}

TEST(BatteryMath_RawToVoltage, Zero) {
    float v = battery_raw_to_voltage(0, 3.3f, 4096, 3.0f);
    EXPECT_FLOAT_EQ(v, 0.0f);
}

TEST(BatteryMath_RawToVoltage, Max) {
    // 4095 (max 12-bit) → 3.3 * 4095/4096 * 3 ≈ 9.897V
    float v = battery_raw_to_voltage(4095, 3.3f, 4096, 3.0f);
    EXPECT_NEAR(v, 9.897f, 0.01f);
}

TEST(BatteryMath_RawToVoltage, Typical4V) {
    // 4.0V battery: raw = 4.0 / 3 / 3.3 * 4096 ≈ 1654
    float v = battery_raw_to_voltage(1654, 3.3f, 4096, 3.0f);
    EXPECT_NEAR(v, 4.0f, 0.05f);
}

TEST(BatteryMath_RawToVoltage, ZeroResolutionGuard) {
    // Should return 0 and not crash on division by zero
    float v = battery_raw_to_voltage(2048, 3.3f, 0, 3.0f);
    EXPECT_FLOAT_EQ(v, 0.0f);
}

TEST(BatteryMath_RawToVoltage, NoDivider) {
    // Divider=1.0 means no voltage divider
    float v = battery_raw_to_voltage(2048, 3.3f, 4096, 1.0f);
    EXPECT_NEAR(v, 1.65f, 0.01f);
}

TEST(BatteryMath_RawToVoltage, HighDivider) {
    // Divider=10.0 for larger voltage ranges
    float v = battery_raw_to_voltage(2048, 3.3f, 4096, 10.0f);
    EXPECT_NEAR(v, 16.5f, 0.01f);
}

// ============================================================================
// Test Suite: BatteryMath_EMA
// ============================================================================

TEST(BatteryMath_EMA, FirstSampleSeeds) {
    BatteryState state = {};
    EXPECT_FALSE(state.initialized);

    battery_ema_update(state, 4.0f, 0.1f, 4.5f);

    EXPECT_TRUE(state.initialized);
    EXPECT_FLOAT_EQ(state.filtered_voltage, 4.0f);
}

TEST(BatteryMath_EMA, SubsequentSmoothing) {
    BatteryState state = {};
    battery_ema_update(state, 4.0f, 0.1f, 4.5f);  // Seed

    // New sample at 4.2V with alpha=0.1
    // Expected: 0.1 * 4.2 + 0.9 * 4.0 = 0.42 + 3.6 = 4.02
    battery_ema_update(state, 4.2f, 0.1f, 4.5f);
    EXPECT_NEAR(state.filtered_voltage, 4.02f, 0.001f);
}

TEST(BatteryMath_EMA, Convergence) {
    BatteryState state = {};
    battery_ema_update(state, 3.5f, 0.05f, 4.5f);  // Seed at 3.5V

    // Apply 100 samples at 4.0V — should converge close to 4.0V
    for (int i = 0; i < 100; i++) {
        battery_ema_update(state, 4.0f, 0.05f, 4.5f);
    }
    EXPECT_NEAR(state.filtered_voltage, 4.0f, 0.01f);
}

TEST(BatteryMath_EMA, USBDetection) {
    BatteryState state = {};

    // Below threshold → on battery
    battery_ema_update(state, 4.0f, 0.1f, 4.5f);
    EXPECT_TRUE(state.on_battery);

    // Above threshold → USB power
    battery_ema_update(state, 5.0f, 0.1f, 4.5f);
    EXPECT_FALSE(state.on_battery);
}

TEST(BatteryMath_EMA, ExactlyAtThreshold) {
    BatteryState state = {};
    battery_ema_update(state, 4.5f, 0.1f, 4.5f);
    // Exactly at threshold → considered on battery (<=)
    EXPECT_TRUE(state.on_battery);
}

TEST(BatteryMath_EMA, AlphaZero) {
    BatteryState state = {};
    battery_ema_update(state, 4.0f, 0.0f, 4.5f);  // Seed
    battery_ema_update(state, 5.0f, 0.0f, 4.5f);
    // Alpha=0 means new sample has no weight → stays at 4.0
    EXPECT_FLOAT_EQ(state.filtered_voltage, 4.0f);
}

TEST(BatteryMath_EMA, AlphaOne) {
    BatteryState state = {};
    battery_ema_update(state, 4.0f, 1.0f, 4.5f);  // Seed
    battery_ema_update(state, 5.0f, 1.0f, 4.5f);
    // Alpha=1 means immediate response → jumps to 5.0
    EXPECT_FLOAT_EQ(state.filtered_voltage, 5.0f);
}

TEST(BatteryMath_EMA, MultipleUpdates) {
    BatteryState state = {};
    battery_ema_update(state, 4.0f, 0.2f, 4.5f);  // Seed at 4.0V

    // Apply series of voltage readings simulating noise
    battery_ema_update(state, 4.1f, 0.2f, 4.5f);  // 4.0 + 0.2*(4.1-4.0) = 4.02
    EXPECT_NEAR(state.filtered_voltage, 4.02f, 0.001f);

    battery_ema_update(state, 3.9f, 0.2f, 4.5f);  // 4.02 + 0.2*(3.9-4.02) = 3.996
    EXPECT_NEAR(state.filtered_voltage, 3.996f, 0.001f);

    battery_ema_update(state, 4.05f, 0.2f, 4.5f);  // Converging back
    EXPECT_NEAR(state.filtered_voltage, 4.0068f, 0.001f);
}

TEST(BatteryMath_EMA, NegativeVoltageHandling) {
    BatteryState state = {};
    // Negative voltage (shouldn't happen in practice, but test robustness)
    battery_ema_update(state, -0.5f, 0.1f, 4.5f);
    EXPECT_TRUE(state.on_battery);  // Negative is definitely < threshold
    EXPECT_FLOAT_EQ(state.filtered_voltage, -0.5f);
}

// ============================================================================
// Test Suite: BatteryMath_VoltageToPercent
// ============================================================================

TEST(BatteryMath_VoltageToPercent, BelowMin) {
    EXPECT_EQ(battery_voltage_to_percent(2.5f), 0);
    EXPECT_EQ(battery_voltage_to_percent(3.0f), 0);  // Exactly at min
}

TEST(BatteryMath_VoltageToPercent, AboveMax) {
    EXPECT_EQ(battery_voltage_to_percent(4.5f), 100);
    EXPECT_EQ(battery_voltage_to_percent(4.2f), 100);  // Exactly at max
}

TEST(BatteryMath_VoltageToPercent, ExactTablePoints) {
    EXPECT_EQ(battery_voltage_to_percent(3.30f), 10);
    EXPECT_EQ(battery_voltage_to_percent(3.60f), 40);
    EXPECT_EQ(battery_voltage_to_percent(3.80f), 70);
    EXPECT_EQ(battery_voltage_to_percent(4.00f), 90);
}

TEST(BatteryMath_VoltageToPercent, InterpolationMidpoints) {
    // Between 3.0V (0%) and 3.3V (10%) at midpoint 3.15V → ~5%
    uint8_t pct = battery_voltage_to_percent(3.15f);
    EXPECT_GE(pct, 4);
    EXPECT_LE(pct, 6);

    // Between 3.6V (40%) and 3.8V (70%) at midpoint 3.7V → ~55%
    pct = battery_voltage_to_percent(3.70f);
    EXPECT_GE(pct, 53);
    EXPECT_LE(pct, 57);
}

TEST(BatteryMath_VoltageToPercent, Monotonicity) {
    // Percentage should never decrease as voltage increases
    uint8_t prev = 0;
    for (float v = 3.0f; v <= 4.2f; v += 0.01f) {
        uint8_t curr = battery_voltage_to_percent(v);
        EXPECT_GE(curr, prev) << "Non-monotonic at voltage " << v;
        prev = curr;
    }
}

TEST(BatteryMath_VoltageToPercent, CommonVoltages) {
    // 3.7V is typical nominal LiPo voltage (roughly half charge)
    uint8_t pct = battery_voltage_to_percent(3.7f);
    EXPECT_GE(pct, 50);
    EXPECT_LE(pct, 60);

    // 3.85V is ~75% charged
    pct = battery_voltage_to_percent(3.85f);
    EXPECT_GE(pct, 73);
    EXPECT_LE(pct, 78);
}

TEST(BatteryMath_VoltageToPercent, JustAboveMin) {
    // 3.01V is so close to min that it rounds to 0% (0.33% → 0)
    uint8_t pct = battery_voltage_to_percent(3.01f);
    EXPECT_EQ(pct, 0);

    // 3.05V should be slightly above 0%
    pct = battery_voltage_to_percent(3.05f);
    EXPECT_GT(pct, 0);
    EXPECT_LT(pct, 3);
}

TEST(BatteryMath_VoltageToPercent, JustBelowMax) {
    // 4.19V is so close to max (4.2V) that it rounds to 100% (95% → 100%)
    uint8_t pct = battery_voltage_to_percent(4.19f);
    EXPECT_EQ(pct, 100);

    // 4.15V should be clearly below 100%
    pct = battery_voltage_to_percent(4.15f);
    EXPECT_GE(pct, 97);
    EXPECT_LT(pct, 100);
}

TEST(BatteryMath_VoltageToPercent, InterpolationAccuracy) {
    // Verify interpolation formula correctness
    // Between 3.8V (70%) and 4.0V (90%):
    // At 3.9V (midpoint): (3.9-3.8)/(4.0-3.8) = 0.5, so 70 + 0.5*(90-70) = 80%
    uint8_t pct = battery_voltage_to_percent(3.9f);
    EXPECT_EQ(pct, 80);

    // Between 3.3V (10%) and 3.6V (40%):
    // At 3.45V (midpoint): (3.45-3.3)/(3.6-3.3) = 0.5, so 10 + 0.5*(40-10) = 25%
    pct = battery_voltage_to_percent(3.45f);
    EXPECT_EQ(pct, 25);
}

TEST(BatteryMath_VoltageToPercent, CriticalLowBattery) {
    // 3.1V is critically low for LiPo (should be very low %)
    uint8_t pct = battery_voltage_to_percent(3.1f);
    EXPECT_GE(pct, 2);
    EXPECT_LE(pct, 5);
}

TEST(BatteryMath_VoltageToPercent, FullyChargedRegion) {
    // 4.1V is in the 90-100% segment
    // At 4.1V: (4.1-4.0)/(4.2-4.0) = 0.5, so 90 + 0.5*10 = 95%
    uint8_t pct = battery_voltage_to_percent(4.1f);
    EXPECT_EQ(pct, 95);
}

TEST(BatteryMath_VoltageToPercent, NegativeVoltage) {
    // Shouldn't happen in practice, but verify safe handling
    EXPECT_EQ(battery_voltage_to_percent(-1.0f), 0);
}

TEST(BatteryMath_VoltageToPercent, VeryHighVoltage) {
    // Far above max (e.g., measurement error or different battery chemistry)
    EXPECT_EQ(battery_voltage_to_percent(10.0f), 100);
}

// ============================================================================
// Test Suite: BatteryMath_Integration
// ============================================================================

TEST(BatteryMath_Integration, CompleteWorkflow) {
    // Simulate complete ADC → voltage → filter → percentage workflow
    BatteryState state = {};

    // First reading: ADC value representing 3.7V
    // raw = 3.7 / 3 / 3.3 * 4096 ≈ 1530
    float v1 = battery_raw_to_voltage(1530, 3.3f, 4096, 3.0f);
    battery_ema_update(state, v1, 0.1f, 4.5f);
    uint8_t pct1 = battery_voltage_to_percent(state.filtered_voltage);

    EXPECT_TRUE(state.on_battery);
    EXPECT_NEAR(state.filtered_voltage, 3.7f, 0.05f);
    EXPECT_GE(pct1, 50);
    EXPECT_LE(pct1, 60);

    // Second reading: voltage dropped to 3.6V
    float v2 = battery_raw_to_voltage(1489, 3.3f, 4096, 3.0f);
    battery_ema_update(state, v2, 0.1f, 4.5f);
    uint8_t pct2 = battery_voltage_to_percent(state.filtered_voltage);

    // EMA: 0.1*3.6 + 0.9*3.7 = 3.69V
    EXPECT_NEAR(state.filtered_voltage, 3.69f, 0.01f);
    // 3.69V is between 3.6V (40%) and 3.8V (70%) → ~53%
    EXPECT_GE(pct2, 52);
    EXPECT_LE(pct2, 54);
}

TEST(BatteryMath_Integration, USBPluggedIn) {
    // Simulate USB connection (voltage jumps above threshold)
    BatteryState state = {};

    // On battery initially
    float v1 = battery_raw_to_voltage(1530, 3.3f, 4096, 3.0f);  // 3.7V
    battery_ema_update(state, v1, 0.1f, 4.5f);
    EXPECT_TRUE(state.on_battery);

    // USB plugged in (voltage spikes to 5V)
    float v2 = battery_raw_to_voltage(2068, 3.3f, 4096, 3.0f);  // ~5V
    battery_ema_update(state, v2, 0.1f, 4.5f);
    EXPECT_FALSE(state.on_battery);

    // EMA: 0.1*5.0 + 0.9*3.7 = 3.83V (smoothing prevents instant jump)
    EXPECT_NEAR(state.filtered_voltage, 3.83f, 0.01f);
    // 3.83V is between 3.8V (70%) and 4.0V (90%) → ~73%
    uint8_t pct = battery_voltage_to_percent(state.filtered_voltage);
    EXPECT_GE(pct, 72);
    EXPECT_LE(pct, 74);
}

TEST(BatteryMath_Integration, NoisyReadings) {
    // Verify EMA smooths out noisy ADC readings
    BatteryState state = {};

    // Start with stable 4.0V
    float v1 = battery_raw_to_voltage(1654, 3.3f, 4096, 3.0f);
    battery_ema_update(state, v1, 0.05f, 4.5f);

    // Add noisy readings around 4.0V (±0.1V)
    float readings[5] = {4.1f, 3.9f, 4.05f, 3.95f, 4.0f};
    for (float reading : readings) {
        battery_ema_update(state, reading, 0.05f, 4.5f);
    }

    // Filtered voltage should remain close to 4.0V despite noise
    EXPECT_NEAR(state.filtered_voltage, 4.0f, 0.05f);
}
