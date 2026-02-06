/*
 * Encoder_Wrapper - AS5600 Magnetic Encoder Driver Implementation
 * Safety-critical wrapper with error checking for RP2350 firmware
 */

#include "drivers/encoder_wrapper.hpp"

#include "hardware/irq.h"

bool Encoder_Wrapper::init() {
    // 1. Initialize I2C0 hardware at 400kHz
    //    MUST be called before as5600_init() because as5600_init() derives
    //    the i2c_inst from the SDA pin number.
    i2c_init(i2c0, ENCODER_I2C_FREQ_HZ);

    // 2. Initialize AS5600 (sets GPIO functions and pull-ups)
    //    Returns 0 on success, -1 on failure
    int8_t ret = as5600_init(ENCODER_SDA_PIN, ENCODER_SCL_PIN, &encoder_);
    if (ret != 0) {
        return false;
    }

    // 3. Set I2C0 interrupt priority to 0 (highest) in NVIC
    //    Ensures encoder bus transactions are never preempted by other
    //    peripherals. Critical for maintaining 100Hz navigation loop timing.
    irq_set_priority(I2C0_IRQ, 0);

    consecutive_failures = 0;
    last_raw_angle_ = 0;
    first_read_ = true;

    return true;
}

float Encoder_Wrapper::get_angle_delta() {
    uint16_t current_raw = 0;

    if (!read_raw_angle(&current_raw)) {
        consecutive_failures++;
        return 0.0f;
    }

    consecutive_failures = 0;

    if (first_read_) {
        first_read_ = false;
        last_raw_angle_ = current_raw;
        return 0.0f;
    }

    // Compute shortest-path delta with 12-bit wraparound handling
    // Example: 4095 -> 0 should be +1 tick, not -4095 ticks
    int16_t delta = static_cast<int16_t>(current_raw - last_raw_angle_);
    if (delta > HALF_REV) {
        delta -= static_cast<int16_t>(TICKS_PER_REV);
    }
    if (delta < -HALF_REV) {
        delta += static_cast<int16_t>(TICKS_PER_REV);
    }

    last_raw_angle_ = current_raw;

    return static_cast<float>(delta) * TICKS_TO_RAD;
}

bool Encoder_Wrapper::read_raw_angle(uint16_t *out) {
    uint8_t reg = RAW_ANGLE_REG;
    uint8_t buf[2] = {0, 0};

    // Write register address with repeated start (nostop=true)
    // Returns number of bytes written or PICO_ERROR_GENERIC (-1) on failure
    int ret = i2c_write_blocking(encoder_.i2c_inst, AS5600_ADDR, &reg, 1, true);
    if (ret < 0) {
        return false;
    }

    // Read 2 bytes of angle data
    // Returns number of bytes read or PICO_ERROR_GENERIC (-1) on failure
    ret = i2c_read_blocking(encoder_.i2c_inst, AS5600_ADDR, buf, 2, false);
    if (ret < 0) {
        return false;
    }

    // AS5600 transmits angle in big-endian format as 12-bit value
    // buf[0] = high byte (bits 11-4), buf[1] = low byte (bits 3-0)
    *out = (static_cast<uint16_t>(buf[0]) << 8U) | buf[1];
    *out &= 0x0FFFU; // Mask to 12 bits (0-4095)

    return true;
}
