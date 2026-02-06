/*
 * Encoder_Wrapper - AS5600 Magnetic Encoder Driver
 * Safety-critical wrapper with error checking for RP2350 firmware
 */

#ifndef ENCODER_WRAPPER_HPP
#define ENCODER_WRAPPER_HPP

#include "config.h"
#include "types.h"

#include "hardware/i2c.h"
#include "pico/stdlib.h"

extern "C" {
#include "dwm_pico_AS5600.h"
}

#include <cstdint>

class Encoder_Wrapper {
public:
    bool init();
    float get_angle_delta();

    uint16_t consecutive_failures = 0;

private:
    static constexpr uint8_t AS5600_ADDR = 0x36;
    static constexpr uint8_t RAW_ANGLE_REG = 0x0C;
    static constexpr uint16_t TICKS_PER_REV = 4096;
    static constexpr float TICKS_TO_RAD = (2.0f * 3.14159265358979f) / 4096.0f;
    static constexpr int16_t HALF_REV = 2048;

    as5600_t encoder_ = {};
    uint16_t last_raw_angle_ = 0;
    bool first_read_ = true;

    bool read_raw_angle(uint16_t *out);
};

#endif // ENCODER_WRAPPER_HPP
