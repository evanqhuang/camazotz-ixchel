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
    bool poll();
    void flush();
    Quat get_quaternion() const;
    Vec3 get_angular_velocity() const;
    void hardware_reset();

    uint8_t get_calibration_accuracy();
    uint8_t get_gyro_accuracy() const { return last_gyro_accuracy_; }
    uint8_t get_accel_accuracy() const { return last_accel_accuracy_; }
    uint8_t get_mag_accuracy() const { return last_mag_accuracy_; }
    bool tare_now(bool z_axis_only = false);
    bool save_tare();
    bool clear_tare();

    uint16_t consecutive_failures = 0;

private:
    static constexpr uint8_t IMU_ADDR = 0x4B;
    static constexpr uint16_t REPORT_INTERVAL_MS = 10; // 100Hz
    static constexpr int MAX_DRAIN_ITERATIONS = 10;
    static constexpr int MAX_FLUSH_ITERATIONS = 200;
    static constexpr uint32_t I2C_PROBE_TIMEOUT_US = 100000;

    static constexpr int MAX_PROBE_CYCLES = 20;
    static constexpr int MAX_READ_RETRIES = 10;
    static constexpr uint32_t HINTN_WAIT_MS = 1000;
    static constexpr int BUS_RECOVERY_INTERVAL = 3;
    static constexpr uint32_t BUS_RECOVERY_SETTLE_MS = 100;
    static constexpr uint16_t SHTP_DRAIN_BUF_SIZE = 256;

    BNO08x imu_;
    Quat cached_quat_ = {1.0f, 0.0f, 0.0f, 0.0f};
    Vec3 cached_angular_vel_ = {0.0f, 0.0f, 0.0f};

    // Per-sensor accuracy tracking (library has bugs, we capture directly)
    uint8_t last_rv_accuracy_ = 0;    // Rotation vector
    uint8_t last_gyro_accuracy_ = 0;  // Calibrated gyroscope
    uint8_t last_accel_accuracy_ = 0; // Accelerometer
    uint8_t last_mag_accuracy_ = 0;   // Magnetometer

    void bus_recovery();
    bool drain_events();
    bool enable_reports();
};

#endif // IMU_WRAPPER_HPP
