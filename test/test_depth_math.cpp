/*
 * Unit tests for MS5837-30BA depth sensor math
 * CRC4, compensation algorithms, unit conversions, depth formula
 */

#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include "logic/depth_math.hpp"

// ============================================================================
// Test Suite: MS5837_CRC4
// ============================================================================

TEST(MS5837_CRC4, KnownPROM) {
    // PROM calibration values; compute CRC4, then verify it matches when embedded
    uint16_t prom[7] = {0x041E, 46372, 43981, 29059, 27842, 31553, 28165};
    uint8_t crc = ms5837_crc4(prom);

    // Embed computed CRC into prom[0] upper nibble (as the sensor would)
    prom[0] = (static_cast<uint16_t>(crc) << 12) | (prom[0] & 0x0FFF);
    uint8_t crc_read = prom[0] >> 12;

    // CRC extracted from prom[0] should match computed CRC
    EXPECT_EQ(ms5837_crc4(prom), crc_read);
    // Verify it's a valid nibble
    EXPECT_LE(crc, 0x0F);
}

TEST(MS5837_CRC4, AllZeroPROM) {
    uint16_t prom[7] = {0, 0, 0, 0, 0, 0, 0};
    // Deterministic output — just verify it doesn't crash and returns valid nibble
    uint8_t crc = ms5837_crc4(prom);
    EXPECT_LE(crc, 0x0F);
}

TEST(MS5837_CRC4, MaxCRCNibble) {
    // Set CRC nibble to 0xF in prom[0] upper bits — masking must ignore it
    uint16_t prom[7] = {0xF000, 0, 0, 0, 0, 0, 0};
    uint8_t crc = ms5837_crc4(prom);
    EXPECT_LE(crc, 0x0F);

    // Same PROM with different CRC nibble should produce same result
    // (CRC is computed on the lower 12 bits of prom[0])
    uint16_t prom2[7] = {0x0000, 0, 0, 0, 0, 0, 0};
    EXPECT_EQ(ms5837_crc4(prom), ms5837_crc4(prom2));
}

TEST(MS5837_CRC4, InputArrayNotModified) {
    uint16_t prom[7] = {0xB41E, 46372, 43981, 29059, 27842, 31553, 28165};
    uint16_t prom_copy[7];
    std::memcpy(prom_copy, prom, sizeof(prom));

    ms5837_crc4(prom);

    EXPECT_EQ(std::memcmp(prom, prom_copy, sizeof(prom)), 0);
}

// ============================================================================
// Test Suite: MS5837_Compensate
// ============================================================================

TEST(MS5837_Compensate, StandardConditions) {
    // Realistic PROM values from a real MS5837-30BA sensor
    uint16_t prom[7] = {0xB41E, 46372, 43981, 29059, 27842, 31553, 28165};

    // D1/D2 values that produce ~20°C, ~1013 mbar (sea level atmospheric)
    // D2 chosen to produce TEMP near 2000 (20.00°C)
    // dT = D2 - prom[5]*256 = D2 - 31553*256 = D2 - 8077568
    // TEMP = 2000 + dT * prom[6] / 8388608
    // For TEMP=2000 (20°C exactly): dT = 0
    // So D2 = prom[5]*256 = 8077568
    uint32_t d2_for_20c = static_cast<uint32_t>(31553) * 256;

    // SENS = prom[1]*32768 + (prom[3]*dT)/256  (dT=0 → SENS = prom[1]*32768)
    // OFF = prom[2]*65536 + (prom[4]*dT)/128   (dT=0 → OFF = prom[2]*65536)
    // P = (D1*SENS/2097152 - OFF) / 8192
    // For P=10130 (1013.0 mbar): D1*SENS/2097152 - OFF = 10130*8192
    // SENS = 46372*32768 = 1519517696
    // OFF = 43981*65536 = 2882732032 (NOTE: fits int64_t)
    // D1 = (10130*8192 + OFF)*2097152/SENS
    //    = (82984960 + 2882732032)*2097152/1519517696
    //    = 2965716992*2097152/1519517696
    // This requires careful int64_t math; use a simpler approach: just verify
    // the output is in a reasonable range
    uint32_t d1_approx = 4098000;  // Approximate D1 for ~1 atm

    int32_t temperature = 0;
    int32_t pressure = 0;
    ms5837_compensate(prom, d1_approx, d2_for_20c, &temperature, &pressure);

    // At dT=0 and TEMP >= 2000, we're in the high-temperature branch
    // TEMP should be exactly 2000 (20.00°C)
    EXPECT_EQ(temperature, 2000);

    // Pressure should be in a reasonable atmospheric range
    EXPECT_GT(pressure, 9000);   // > 900.0 mbar
    EXPECT_LT(pressure, 11000);  // < 1100.0 mbar
}

TEST(MS5837_Compensate, LowTemperatureBranch) {
    // PROM from known sensor
    uint16_t prom[7] = {0xB41E, 46372, 43981, 29059, 27842, 31553, 28165};

    // Force TEMP < 2000 by choosing D2 so dT is negative enough
    // TEMP = 2000 + dT * 28165 / 8388608
    // For TEMP = 1000 (10°C): dT = (1000-2000)*8388608/28165 = -297796
    // D2 = dT + prom[5]*256 = -297796 + 8077568 = 7779772
    uint32_t d2_cold = 7779772;
    uint32_t d1_approx = 4098000;

    int32_t temperature = 0;
    int32_t pressure = 0;
    ms5837_compensate(prom, d1_approx, d2_cold, &temperature, &pressure);

    // Temperature should be approximately 10°C (1000 in raw units)
    // Second-order correction will adjust slightly
    EXPECT_GT(temperature, 500);
    EXPECT_LT(temperature, 1500);
}

TEST(MS5837_Compensate, VeryLowTemperatureBranch) {
    // Force TEMP < -1500 (-15°C)
    uint16_t prom[7] = {0xB41E, 46372, 43981, 29059, 27842, 31553, 28165};

    // For TEMP = -2000 (-20°C): dT = (-2000-2000)*8388608/28165 = -1191184
    // D2 = -1191184 + 8077568 = 6886384
    uint32_t d2_very_cold = 6886384;
    uint32_t d1_approx = 4098000;

    int32_t temperature = 0;
    int32_t pressure = 0;
    ms5837_compensate(prom, d1_approx, d2_very_cold, &temperature, &pressure);

    // Temperature should be below -15°C (-1500 raw)
    EXPECT_LT(temperature, -1400);
}

TEST(MS5837_Compensate, HighTemperatureBranch) {
    // Force TEMP >= 2000 (the else branch)
    uint16_t prom[7] = {0xB41E, 46372, 43981, 29059, 27842, 31553, 28165};

    // For TEMP = 3000 (30°C): dT = (3000-2000)*8388608/28165 = 297796
    // D2 = 297796 + 8077568 = 8375364
    uint32_t d2_warm = 8375364;
    uint32_t d1_approx = 4098000;

    int32_t temperature = 0;
    int32_t pressure = 0;
    ms5837_compensate(prom, d1_approx, d2_warm, &temperature, &pressure);

    // Temperature should be approximately 30°C
    EXPECT_GT(temperature, 2500);
    EXPECT_LT(temperature, 3500);
}

// Cross-validation: verify that running compensation twice with same inputs
// produces identical results (deterministic, no state)
TEST(MS5837_Compensate, Deterministic) {
    uint16_t prom[7] = {0xB41E, 46372, 43981, 29059, 27842, 31553, 28165};
    uint32_t d1 = 4098000;
    uint32_t d2 = 8077568;

    int32_t t1 = 0, p1 = 0, t2 = 0, p2 = 0;
    ms5837_compensate(prom, d1, d2, &t1, &p1);
    ms5837_compensate(prom, d1, d2, &t2, &p2);

    EXPECT_EQ(t1, t2);
    EXPECT_EQ(p1, p2);
}

// ============================================================================
// Test Suite: MS5837_PressureMbar
// ============================================================================

TEST(MS5837_PressureMbar, StandardConversion) {
    EXPECT_FLOAT_EQ(ms5837_pressure_mbar(10130), 1013.0f);
}

TEST(MS5837_PressureMbar, ZeroPressure) {
    EXPECT_FLOAT_EQ(ms5837_pressure_mbar(0), 0.0f);
}

// ============================================================================
// Test Suite: MS5837_TemperatureC
// ============================================================================

TEST(MS5837_TemperatureC, StandardConversion) {
    EXPECT_FLOAT_EQ(ms5837_temperature_c(2000), 20.0f);
}

TEST(MS5837_TemperatureC, NegativeTemperature) {
    EXPECT_FLOAT_EQ(ms5837_temperature_c(-1500), -15.0f);
}

// ============================================================================
// Test Suite: MS5837_DepthM
// ============================================================================

TEST(MS5837_DepthM, AtmosphericPressureZeroDepth) {
    // At atmospheric pressure (1013.25 mbar), depth should be approximately 0
    float depth = ms5837_depth_m(1013.25f, 1029.0f);
    EXPECT_NEAR(depth, 0.0f, 0.01f);
}

TEST(MS5837_DepthM, TenMetersSeawater) {
    // ~2 atm ≈ 2013.25 mbar → ~10m depth in seawater (1029 kg/m³)
    // depth = (2013.25*100 - 101300) / (1029 * 9.80665) = 100025/10091.04 ≈ 9.91m
    float depth = ms5837_depth_m(2013.25f, 1029.0f);
    EXPECT_NEAR(depth, 9.91f, 0.1f);
}

TEST(MS5837_DepthM, FreshwaterGreaterDepth) {
    // Same pressure but freshwater (998 kg/m³) → deeper than seawater
    float depth_sea = ms5837_depth_m(2013.25f, 1029.0f);
    float depth_fresh = ms5837_depth_m(2013.25f, 998.0f);
    EXPECT_GT(depth_fresh, depth_sea);
}
