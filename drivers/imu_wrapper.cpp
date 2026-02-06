/*
 * IMU Wrapper Implementation for BNO08x on RP2350
 * Safety-critical embedded system - handles hardware reset and sensor fusion
 */

#include "drivers/imu_wrapper.hpp"

#include "sh2.h"

bool IMU_Wrapper::init() {
    // 1. Initialize RST pin - drive HIGH (deassert active-low reset)
    gpio_init(IMU_RST_PIN);
    gpio_set_dir(IMU_RST_PIN, GPIO_OUT);
    gpio_put(IMU_RST_PIN, 1);

    // 2. Initialize I2C1 hardware at 400kHz
    i2c_init(i2c1, IMU_I2C_FREQ_HZ);

    // 3. Configure GPIO pins for I2C1
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);

    // 4. Perform a proper hardware reset before begin()
    gpio_put(IMU_RST_PIN, 0);
    sleep_ms(10);
    gpio_put(IMU_RST_PIN, 1);
    sleep_ms(100); // BNO08x needs time to boot after reset

    // 5. Initialize BNO08x
    if (!imu_.begin(IMU_ADDR, i2c1)) {
        return false;
    }

    // 6. Enable sensor reports at 100Hz
    if (!enable_reports()) {
        return false;
    }

    consecutive_failures = 0;
    return true;
}

Quat IMU_Wrapper::get_quaternion() {
    if (drain_events()) {
        consecutive_failures = 0;
    } else {
        consecutive_failures++;
    }
    return cached_quat_;
}

Vec3 IMU_Wrapper::get_angular_velocity() {
    if (drain_events()) {
        consecutive_failures = 0;
    } else {
        consecutive_failures++;
    }
    return cached_angular_vel_;
}

void IMU_Wrapper::hardware_reset() {
    // Toggle RST pin: LOW → wait → HIGH → wait
    gpio_put(IMU_RST_PIN, 0);
    sleep_ms(10);
    gpio_put(IMU_RST_PIN, 1);
    sleep_ms(100);

    // Reinitialize the sensor after hardware reset
    imu_.begin(IMU_ADDR, i2c1);
    enable_reports();

    consecutive_failures = 0;
}

bool IMU_Wrapper::drain_events() {
    bool got_event = false;

    for (int i = 0; i < MAX_DRAIN_ITERATIONS; i++) {
        if (!imu_.getSensorEvent()) {
            break;
        }

        uint8_t event_id = imu_.getSensorEventID();

        if (event_id == SH2_ROTATION_VECTOR) {
            cached_quat_.x = imu_.getQuatI();
            cached_quat_.y = imu_.getQuatJ();
            cached_quat_.z = imu_.getQuatK();
            cached_quat_.w = imu_.getQuatReal();
            got_event = true;
        } else if (event_id == SH2_GYRO_INTEGRATED_RV) {
            cached_angular_vel_.x = imu_.getGyroIntegratedRVangVelX();
            cached_angular_vel_.y = imu_.getGyroIntegratedRVangVelY();
            cached_angular_vel_.z = imu_.getGyroIntegratedRVangVelZ();
            got_event = true;
        }
    }

    return got_event;
}

bool IMU_Wrapper::enable_reports() {
    if (!imu_.enableRotationVector(REPORT_INTERVAL_MS)) {
        return false;
    }
    if (!imu_.enableGyroIntegratedRotationVector(REPORT_INTERVAL_MS)) {
        return false;
    }
    return true;
}
