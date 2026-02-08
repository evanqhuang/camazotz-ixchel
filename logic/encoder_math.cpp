/*
 * AS5600 Encoder Math Implementation
 * 12-bit wraparound handling and unit conversion
 */

#include "logic/encoder_math.hpp"

uint16_t encoder_apply_offset(uint16_t raw, uint16_t offset) {
    return (raw - offset) & 0x0FFFU;
}

int16_t encoder_shortest_delta(uint16_t current, uint16_t previous) {
    int16_t delta = static_cast<int16_t>(current - previous);
    if (delta > ENCODER_HALF_REV) {
        delta -= static_cast<int16_t>(ENCODER_TICKS_PER_REV);
    }
    if (delta < -ENCODER_HALF_REV) {
        delta += static_cast<int16_t>(ENCODER_TICKS_PER_REV);
    }
    return delta;
}

float encoder_ticks_to_rad(int16_t delta_ticks) {
    return static_cast<float>(delta_ticks) * ENCODER_TICKS_TO_RAD;
}
