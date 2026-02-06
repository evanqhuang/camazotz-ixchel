/*
 * IMU Wrapper for BNO08x on RP2350
 * Safety-critical embedded system - handles hardware reset and sensor fusion
 */

#ifndef IMU_WRAPPER_HPP
#define IMU_WRAPPER_HPP

#include "config.h"
#include "types.h"

#include "pico/stdlib.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "bno08x.h"

#include <cstdint>

class IMU_Wrapper {
public:
    bool init();
    Quat get_quaternion();
    Vec3 get_angular_velocity();
    void hardware_reset();

    uint16_t consecutive_failures = 0;

private:
    static constexpr uint8_t IMU_ADDR = 0x4A;
    static constexpr uint16_t REPORT_INTERVAL_MS = 10; // 100Hz
    static constexpr int MAX_DRAIN_ITERATIONS = 10;

    BNO08x imu_;
    Quat cached_quat_ = {0.0f, 0.0f, 0.0f, 1.0f};
    Vec3 cached_angular_vel_ = {0.0f, 0.0f, 0.0f};

    bool drain_events();
    bool enable_reports();
};

#endif // IMU_WRAPPER_HPP
