/*
 * Calibration_Manager Implementation - Boot-time sensor calibration
 * Runs encoder, IMU, and depth calibration sequentially
 * Reports failures via display, never halts
 */

#include "logic/calibration_manager.hpp"

#include "pico/stdlib.h"

#include <cstdio>
#include <cstring>

Calibration_Manager::Calibration_Manager(Encoder_Wrapper &enc, IMU_Wrapper &imu,
                                         Depth_Wrapper &depth, PIO_I2C &depth_bus,
                                         Display_Interface &display)
    : encoder_(enc), imu_(imu), depth_(depth), depth_bus_(depth_bus),
      display_(display) {}

CalibrationResult Calibration_Manager::run_boot_calibration(bool enc_ok, bool imu_ok,
                                                              bool depth_ok) {
    CalibrationResult result = {};

    display_.show_status("Boot", "Starting sensor calibration...");

    if (enc_ok) {
        result.encoder = calibrate_encoder();
    } else {
        set_error(result.encoder, CalibrationStatus::Failed, "Init failed");
        display_.show_error("Encoder", "Skipped - init failed", DisplaySeverity::Error);
    }

    if (imu_ok) {
        result.imu = calibrate_imu();
    } else {
        set_error(result.imu, CalibrationStatus::Failed, "Init failed");
        display_.show_error("IMU", "Skipped - init failed", DisplaySeverity::Error);
    }

    if (depth_ok) {
        result.depth = calibrate_depth();
    } else {
        set_error(result.depth, CalibrationStatus::Failed, "Init failed");
        display_.show_error("Depth", "Skipped - init failed", DisplaySeverity::Error);
    }

    // Display summary
    const char *status_labels[] = {"NOT_RUN", "OK", "WARN", "FAIL"};
    char buf[80];

    snprintf(buf, sizeof(buf), "Encoder: %s",
             status_labels[static_cast<uint8_t>(result.encoder.status)]);
    display_.show_status("Result", buf);

    snprintf(buf, sizeof(buf), "IMU: %s",
             status_labels[static_cast<uint8_t>(result.imu.status)]);
    display_.show_status("Result", buf);

    snprintf(buf, sizeof(buf), "Depth: %s",
             status_labels[static_cast<uint8_t>(result.depth.status)]);
    display_.show_status("Result", buf);

    if (result.any_fatal()) {
        display_.show_error("Boot", "Some sensors failed - running in degraded mode",
                            DisplaySeverity::Warning);
    } else {
        display_.show_status("Boot", "Calibration complete - all sensors OK");
    }

    return result;
}

bool Calibration_Manager::manual_tare() {
    display_.show_status("Tare", "Performing manual tare...");
    if (!imu_.tare_now()) {
        display_.show_error("Tare", "Tare failed", DisplaySeverity::Error);
        return false;
    }
    display_.show_status("Tare", "Tare applied");
    return true;
}

SensorCalibResult Calibration_Manager::calibrate_encoder() {
    SensorCalibResult result = {};

    display_.show_status("Encoder", "Checking magnet...");

    if (!encoder_.check_magnet_present()) {
        set_error(result, CalibrationStatus::Failed, "No magnet detected");
        display_.show_error("Encoder", result.error_msg, DisplaySeverity::Fatal);
        return result;
    }

    display_.show_status("Encoder", "Setting zero offset...");

    if (!encoder_.set_zero_offset()) {
        set_error(result, CalibrationStatus::Failed, "Failed to read angle");
        display_.show_error("Encoder", result.error_msg, DisplaySeverity::Fatal);
        return result;
    }

    char buf[80];
    snprintf(buf, sizeof(buf), "Zero offset: %u",
             static_cast<unsigned>(encoder_.get_zero_offset()));
    display_.show_status("Encoder", buf);

    result.status = CalibrationStatus::Success;
    return result;
}

SensorCalibResult Calibration_Manager::calibrate_imu() {
    SensorCalibResult result = {};

    display_.show_status("IMU", "Checking sensor...");

    // Just verify IMU is responding - walkthrough will handle accuracy wait and tare
    imu_.flush();
    uint8_t accuracy = imu_.get_calibration_accuracy();

    char buf[80];
    snprintf(buf, sizeof(buf), "Initial accuracy: %u/3", static_cast<unsigned>(accuracy));
    display_.show_status("IMU", buf);

    // Don't wait for accuracy, don't tare - walkthrough will do this
    result.status = CalibrationStatus::Success;
    return result;
}

SensorCalibResult Calibration_Manager::calibrate_depth() {
    SensorCalibResult result = {};

    display_.show_status("Depth", "Initializing MS5837...");

    bool init_ok = false;
    for (uint8_t attempt = 0; attempt < CALIB_DEPTH_CRC_MAX_RETRIES; attempt++) {
        if (depth_.begin(depth_bus_)) {
            init_ok = true;
            break;
        }

        char buf[80];
        snprintf(buf, sizeof(buf), "CRC/init failed, retry %u/%u",
                 static_cast<unsigned>(attempt + 1),
                 static_cast<unsigned>(CALIB_DEPTH_CRC_MAX_RETRIES));
        display_.show_error("Depth", buf, DisplaySeverity::Warning);

        sleep_ms(100);
    }

    if (!init_ok) {
        set_error(result, CalibrationStatus::Failed,
                  "PROM/CRC validation failed after retries");
        display_.show_error("Depth", result.error_msg, DisplaySeverity::Fatal);
        return result;
    }

    display_.show_status("Depth", "Waiting for initial reading...");

    // Run initial measurement cycle (non-blocking, poll for up to 100ms)
    uint32_t start = to_ms_since_boot(get_absolute_time());
    bool got_reading = false;

    while (to_ms_since_boot(get_absolute_time()) - start < 100) {
        if (depth_.update()) {
            got_reading = true;
            break;
        }
        sleep_ms(1);
    }

    if (got_reading) {
        char buf[80];
        snprintf(buf, sizeof(buf), "P=%.1f mbar, T=%.1f C",
                 static_cast<double>(depth_.pressure_mbar()),
                 static_cast<double>(depth_.temperature_c()));
        display_.show_status("Depth", buf);
        result.status = CalibrationStatus::Success;
    } else {
        snprintf(result.error_msg, sizeof(result.error_msg),
                 "No initial reading within 100ms");
        display_.show_error("Depth", result.error_msg, DisplaySeverity::Warning);
        result.status = CalibrationStatus::Warning;
    }

    return result;
}

void Calibration_Manager::set_error(SensorCalibResult &r, CalibrationStatus status,
                                     const char *msg) {
    r.status = status;
    strncpy(r.error_msg, msg, sizeof(r.error_msg) - 1);
    r.error_msg[sizeof(r.error_msg) - 1] = '\0';
}
