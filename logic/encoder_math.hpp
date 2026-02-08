/*
 * AS5600 Encoder Math - Pure Algorithms (no SDK dependencies)
 * 12-bit wraparound delta detection, zero-offset calibration, unit conversion
 */

#ifndef ENCODER_MATH_HPP
#define ENCODER_MATH_HPP

#include <cstdint>

/* 12-bit encoder constants */
static constexpr uint16_t ENCODER_TICKS_PER_REV = 4096;
static constexpr int16_t  ENCODER_HALF_REV      = 2048;
static constexpr float    ENCODER_TICKS_TO_RAD   = (2.0f * 3.14159265358979f) / 4096.0f;

/*
 * Apply zero-offset calibration to a 12-bit raw angle.
 * Returns (raw - offset) masked to 12 bits [0, 4095].
 */
uint16_t encoder_apply_offset(uint16_t raw, uint16_t offset);

/*
 * Compute shortest-path signed delta between two 12-bit angles.
 * Handles the 4095→0 and 0→4095 wraparound boundaries.
 * Returns a value in [-2048, +2048].
 */
int16_t encoder_shortest_delta(uint16_t current, uint16_t previous);

/*
 * Convert an encoder tick delta to radians.
 * 4096 ticks = 2*pi radians.
 */
float encoder_ticks_to_rad(int16_t delta_ticks);

#endif // ENCODER_MATH_HPP
