/*
 * IMU Wrapper Implementation for BNO08x on RP2350
 * Safety-critical embedded system - handles hardware reset and sensor fusion
 */

#include "drivers/imu_wrapper.hpp"

#include "sh2.h"

#include <cstdio>

bool IMU_Wrapper::init() {
    // 1. Configure INT pin as input (HINTN: sensor asserts LOW when ready)
    gpio_init(IMU_INT_PIN);
    gpio_set_dir(IMU_INT_PIN, GPIO_IN);
    gpio_pull_up(IMU_INT_PIN);

    // 2. RST pin — drive LOW then HIGH for a clean hardware reset
    gpio_init(IMU_RST_PIN);
    gpio_set_dir(IMU_RST_PIN, GPIO_OUT);
    gpio_put(IMU_RST_PIN, 0);
    sleep_ms(10);
    gpio_put(IMU_RST_PIN, 1);
    sleep_ms(100);

    // 3. Initialize I2C bus
    i2c_init(i2c1, IMU_I2C_FREQ_HZ);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);

    // 4. Probe loop — write+read at candidate addresses, bus recovery periodically,
    //    chunked 100ms timeouts to keep USB CDC responsive
    bool read_ok = false;
    static constexpr uint8_t addrs[] = {0x4B, 0x4A, 0x28};

    for (int cycle = 0; cycle < MAX_PROBE_CYCLES; cycle++) {
        // Wait for HINTN LOW (sensor ready) - but don't skip if it times out
        // Some boards don't have INT wired, so we fall back to polling
        absolute_time_t deadline = make_timeout_time_ms(HINTN_WAIT_MS);
        bool hint = false;
        while (!time_reached(deadline)) {
            if (!gpio_get(IMU_INT_PIN)) { hint = true; break; }
            sleep_us(100);
        }
        if (!hint && cycle < MAX_PROBE_CYCLES - 5) {
            // First 15 cycles: require HINTN. Last 5 cycles: try anyway (polling mode)
            continue;
        }
        if (!hint) {
            printf("[IMU] HINTN not asserted, trying I2C anyway (cycle %d)\n", cycle);
        }

        for (uint8_t addr : addrs) {
            uint8_t dummy = 0;
            int wr = i2c_write_timeout_us(i2c1, addr, &dummy, 1, false,
                                          I2C_PROBE_TIMEOUT_US);
            if (wr < 0) continue;

            uint8_t hdr[4] = {0};
            int rd = -2;
            for (int retry = 0; retry < MAX_READ_RETRIES && rd == -2; retry++) {
                rd = i2c_read_timeout_us(i2c1, addr, hdr, 4, false,
                                         I2C_PROBE_TIMEOUT_US);
                if (rd == -2) sleep_ms(1);
            }

            if (rd == 4) {
                read_ok = true;
                uint16_t pkt_len = (hdr[0] | (hdr[1] << 8)) & 0x7FFF;
                if (pkt_len > 4) {
                    static uint8_t drain[SHTP_DRAIN_BUF_SIZE];
                    uint16_t remain = (pkt_len - 4 > SHTP_DRAIN_BUF_SIZE)
                                      ? SHTP_DRAIN_BUF_SIZE
                                      : static_cast<uint16_t>(pkt_len - 4);
                    i2c_read_timeout_us(i2c1, addr, drain, remain, false,
                                        I2C_PROBE_TIMEOUT_US);
                }
                break;
            }
        }
        if (read_ok) break;

        // Bus recovery every BUS_RECOVERY_INTERVAL cycles
        if (cycle % BUS_RECOVERY_INTERVAL == (BUS_RECOVERY_INTERVAL - 1)) {
            bus_recovery();
        }
    }

    if (!read_ok) {
        printf("[IMU] No SHTP response after %d probe cycles\n", MAX_PROBE_CYCLES);
        return false;
    }

    // 5. Sensor is responding — let the library take over
    if (!imu_.begin(IMU_ADDR, i2c1)) {
        printf("[IMU] begin() failed\n");
        return false;
    }

    // 5b. Print product IDs and reset reason for diagnostics
    printf("[IMU] Product IDs (%u entries):\n", imu_.prodIds.numEntries);
    for (int i = 0; i < imu_.prodIds.numEntries && i < 5; i++) {
        printf("  [%d] ver=%u.%u.%u part=%lu build=%lu rst=%u\n",
               i,
               imu_.prodIds.entry[i].swVersionMajor,
               imu_.prodIds.entry[i].swVersionMinor,
               imu_.prodIds.entry[i].swVersionPatch,
               static_cast<unsigned long>(imu_.prodIds.entry[i].swPartNumber),
               static_cast<unsigned long>(imu_.prodIds.entry[i].swBuildNumber),
               imu_.prodIds.entry[i].resetCause);
    }
    stdio_flush();

    // 6. Enable dynamic calibration for gyro, accel, and magnetometer
    // Per SH-2 Reference Manual and Calibration Procedure 1000-4044
    // SH2_CAL_ACCEL=0x01, SH2_CAL_GYRO=0x02, SH2_CAL_MAG=0x04
    if (!imu_.setCalibrationConfig(0x07)) {
        printf("[IMU] Warning: failed to enable calibration config\n");
        // Not fatal - continue anyway
    } else {
        printf("[IMU] setCalibrationConfig(0x07) succeeded\n");
    }

    // 6b. Read back calibration config to verify it was set
    uint8_t cal_config_readback = 0;
    if (sh2_getCalConfig(&cal_config_readback) == SH2_OK) {
        printf("[IMU] Calibration config readback: 0x%02X (accel=%d gyro=%d mag=%d)\n",
               cal_config_readback,
               (cal_config_readback & 0x01) ? 1 : 0,
               (cal_config_readback & 0x02) ? 1 : 0,
               (cal_config_readback & 0x04) ? 1 : 0);
    } else {
        printf("[IMU] Warning: failed to read back calibration config\n");
    }
    stdio_flush();

    // 7. Enable sensor reports at 100Hz
    if (!enable_reports()) {
        printf("[IMU] Failed to enable reports\n");
        return false;
    }

    printf("[IMU] Initialized OK\n");
    consecutive_failures = 0;
    return true;
}

void IMU_Wrapper::bus_recovery() {
    gpio_init(IMU_SDA_PIN);
    gpio_init(IMU_SCL_PIN);
    gpio_set_dir(IMU_SDA_PIN, GPIO_IN);
    gpio_set_dir(IMU_SCL_PIN, GPIO_OUT);

    // 9 clock pulses to release stuck slave
    for (int j = 0; j < 9; j++) {
        gpio_put(IMU_SCL_PIN, 0);
        sleep_us(10);
        gpio_put(IMU_SCL_PIN, 1);
        sleep_us(10);
        if (gpio_get(IMU_SDA_PIN)) break;
    }

    // Generate STOP condition: SDA LOW→HIGH while SCL HIGH
    gpio_set_dir(IMU_SDA_PIN, GPIO_OUT);
    gpio_put(IMU_SCL_PIN, 0);
    sleep_us(10);
    gpio_put(IMU_SDA_PIN, 0);
    sleep_us(10);
    gpio_put(IMU_SCL_PIN, 1);
    sleep_us(10);
    gpio_put(IMU_SDA_PIN, 1);
    sleep_us(10);

    // Restore I2C function
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);
    sleep_ms(BUS_RECOVERY_SETTLE_MS);
}

bool IMU_Wrapper::poll() {
    if (drain_events()) {
        consecutive_failures = 0;
        return true;
    }
    if (consecutive_failures < UINT16_MAX) { consecutive_failures++; }
    return false;
}

void IMU_Wrapper::flush() {
    for (int i = 0; i < MAX_FLUSH_ITERATIONS; i++) {
        if (!imu_.getSensorEvent()) {
            break;
        }
    }
}

Quat IMU_Wrapper::get_quaternion() const {
    return cached_quat_;
}

Vec3 IMU_Wrapper::get_angular_velocity() const {
    return cached_angular_vel_;
}

void IMU_Wrapper::hardware_reset() {
    // Toggle RST pin: LOW → wait → HIGH → wait
    gpio_put(IMU_RST_PIN, 0);
    sleep_ms(10);
    gpio_put(IMU_RST_PIN, 1);
    sleep_ms(100);

    // Reinitialize the sensor after hardware reset
    if (imu_.begin(IMU_ADDR, i2c1) && enable_reports()) {
        consecutive_failures = 0;
    }
}

bool IMU_Wrapper::drain_events() {
    bool got_event = false;

    for (int i = 0; i < MAX_DRAIN_ITERATIONS; i++) {
        if (!imu_.getSensorEvent()) {
            break;
        }

        uint8_t event_id = imu_.getSensorEventID();
        // Capture status/accuracy immediately after getSensorEvent()
        // The library's _sensor_value->status contains accuracy for THIS event
        uint8_t event_status = imu_.getQuatAccuracy(); // Works for all event types

        if (event_id == SH2_ROTATION_VECTOR) {
            cached_quat_.x = imu_.getQuatI();
            cached_quat_.y = imu_.getQuatJ();
            cached_quat_.z = imu_.getQuatK();
            cached_quat_.w = imu_.getQuatReal();
            last_rv_accuracy_ = event_status;
            got_event = true;
        } else if (event_id == SH2_GYRO_INTEGRATED_RV) {
            cached_angular_vel_.x = imu_.getGyroIntegratedRVangVelX();
            cached_angular_vel_.y = imu_.getGyroIntegratedRVangVelY();
            cached_angular_vel_.z = imu_.getGyroIntegratedRVangVelZ();
            got_event = true;
        } else if (event_id == SH2_GYROSCOPE_CALIBRATED) {
            last_gyro_accuracy_ = event_status;
        } else if (event_id == SH2_ACCELEROMETER) {
            last_accel_accuracy_ = event_status;
        } else if (event_id == SH2_MAGNETIC_FIELD_CALIBRATED) {
            last_mag_accuracy_ = event_status;
        }
    }

    return got_event;
}

bool IMU_Wrapper::enable_reports() {
    if (!imu_.enableRotationVector(REPORT_INTERVAL_MS)) {
        printf("[IMU] Failed to enable rotation vector\n");
        return false;
    }
    printf("[IMU] Rotation vector enabled at %u ms\n", REPORT_INTERVAL_MS);

    if (!imu_.enableGyroIntegratedRotationVector(REPORT_INTERVAL_MS)) {
        printf("[IMU] Failed to enable gyro integrated RV\n");
        return false;
    }
    printf("[IMU] Gyro integrated RV enabled at %u ms\n", REPORT_INTERVAL_MS);

    // Enable additional reports to check individual sensor calibration
    if (!imu_.enableGyro(100)) { // 10Hz calibrated gyro
        printf("[IMU] Warning: failed to enable gyro report\n");
    } else {
        printf("[IMU] Calibrated gyro enabled at 100 ms\n");
    }

    if (!imu_.enableAccelerometer(100)) { // 10Hz calibrated accel
        printf("[IMU] Warning: failed to enable accel report\n");
    } else {
        printf("[IMU] Calibrated accel enabled at 100 ms\n");
    }

    if (!imu_.enableMagnetometer(100)) { // 10Hz calibrated mag
        printf("[IMU] Warning: failed to enable mag report\n");
    } else {
        printf("[IMU] Calibrated mag enabled at 100 ms\n");
    }

    stdio_flush();
    return true;
}

uint8_t IMU_Wrapper::get_calibration_accuracy() {
    // Drain events to update all accuracy values (fixes library bugs where
    // getGyroAccuracy() returns an uninitialized member variable)
    drain_events();

    // Debug: print sensor calibration status every 2 seconds
    static uint32_t debug_counter = 0;
    if (++debug_counter % 20 == 0) {
        // Use our tracked values (captured in drain_events) instead of buggy accessors
        printf("[IMU] rv=%u gyro=%u accel=%u mag=%u\n",
               last_rv_accuracy_, last_gyro_accuracy_,
               last_accel_accuracy_, last_mag_accuracy_);
        stdio_flush();
    }

    return last_rv_accuracy_;
}

bool IMU_Wrapper::tare_now(bool z_axis_only) {
    return imu_.tareNow(z_axis_only, SH2_TARE_BASIS_ROTATION_VECTOR);
}

bool IMU_Wrapper::save_tare() {
    return imu_.saveTare();
}

bool IMU_Wrapper::clear_tare() {
    return imu_.clearTare();
}
