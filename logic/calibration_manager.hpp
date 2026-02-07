/*
 * Calibration_Manager - Boot-time sensor calibration orchestrator
 * Sequentially calibrates encoder, IMU, and depth sensors
 * Reports failures via display, never halts
 */

#ifndef CALIBRATION_MANAGER_HPP
#define CALIBRATION_MANAGER_HPP

#include "config.h"
#include "types.h"

#include "drivers/encoder_wrapper.hpp"
#include "drivers/imu_wrapper.hpp"
#include "drivers/depth_wrapper.hpp"
#include "drivers/pio_i2c.hpp"
#include "utils/display_interface.hpp"

class Calibration_Manager {
public:
    Calibration_Manager(Encoder_Wrapper &enc, IMU_Wrapper &imu,
                        Depth_Wrapper &depth, PIO_I2C &depth_bus,
                        Display_Interface &display);

    CalibrationResult run_boot_calibration(bool enc_ok, bool imu_ok, bool depth_ok);
    bool manual_tare();

private:
    Encoder_Wrapper &encoder_;
    IMU_Wrapper &imu_;
    Depth_Wrapper &depth_;
    PIO_I2C &depth_bus_;
    Display_Interface &display_;

    SensorCalibResult calibrate_encoder();
    SensorCalibResult calibrate_imu();
    SensorCalibResult calibrate_depth();

    static void set_error(SensorCalibResult &result, CalibrationStatus status,
                          const char *msg);
};

#endif // CALIBRATION_MANAGER_HPP
