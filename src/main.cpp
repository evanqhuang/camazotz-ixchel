/*
 * RP2350 Mapper Firmware - Main Entry Point
 * Initializes hardware buses, runs boot calibration, enters main loop
 */

#include "config.h"
#include "types.h"

#include "drivers/encoder_wrapper.hpp"
#include "drivers/imu_wrapper.hpp"
#include "drivers/pio_i2c.hpp"
#include "drivers/depth_wrapper.hpp"
#include "logic/calibration_manager.hpp"
#include "utils/stdio_display.hpp"

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"

#include <cstdio>

int main() {
    stdio_init_all();

    // Wait for USB host serial connection (up to 5s), then proceed regardless.
    // Without this, early printf output is buffered and lost.
    for (int i = 0; i < 50 && !stdio_usb_connected(); i++) {
        sleep_ms(100);
    }

    printf("\n========================================\n");
    printf("RP2350 Mapper Firmware\n");
    printf("Build: %s %s\n", __DATE__, __TIME__);
    printf("Board: %s\n", PICO_BOARD);
    printf("SDK:   %s\n", PICO_SDK_VERSION_STRING);
    printf("========================================\n\n");
    stdio_flush();

    Encoder_Wrapper encoder;
    IMU_Wrapper imu;
    PIO_I2C depth_bus;
    Depth_Wrapper depth;
    Stdio_Display display;

    // Initialize each sensor — failures are non-fatal
    display.show_status("Init", "Encoder (I2C0)...");
    bool enc_ok = encoder.init();
    if (enc_ok) {
        display.show_status("Init", "Encoder OK");
    } else {
        display.show_error("Init", "Encoder FAILED", DisplaySeverity::Error);
    }
    stdio_flush();

    display.show_status("Init", "IMU (I2C1)...");
    bool imu_ok = imu.init();
    if (imu_ok) {
        display.show_status("Init", "IMU OK");
    } else {
        display.show_error("Init", "IMU FAILED", DisplaySeverity::Error);
    }
    stdio_flush();

    display.show_status("Init", "Depth bus (PIO1)...");
    bool bus_ok = depth_bus.init(DEPTH_PIO_INSTANCE, DEPTH_SDA_PIN, DEPTH_SCL_PIN,
                                 DEPTH_I2C_FREQ_HZ);
    if (bus_ok) {
        display.show_status("Init", "Depth bus OK");
    } else {
        display.show_error("Init", "Depth bus FAILED", DisplaySeverity::Error);
    }
    stdio_flush();

    printf("\nInit summary: encoder=%s imu=%s depth_bus=%s\n\n",
           enc_ok ? "OK" : "FAIL", imu_ok ? "OK" : "FAIL", bus_ok ? "OK" : "FAIL");
    stdio_flush();

    // Boot calibration — skips sensors that failed init
    Calibration_Manager calibrator(encoder, imu, depth, depth_bus, display);
    CalibrationResult cal = calibrator.run_boot_calibration(enc_ok, imu_ok, bus_ok);

    printf("\nCalibration summary: encoder=%u imu=%u depth=%u\n",
           static_cast<unsigned>(cal.encoder.status),
           static_cast<unsigned>(cal.imu.status),
           static_cast<unsigned>(cal.depth.status));
    printf("Entering main loop\n\n");

    // Main loop
    static constexpr uint32_t HEARTBEAT_INTERVAL = 500;  // iterations (~5s at 10ms sleep)
    static constexpr uint32_t LOOP_SLEEP_MS = 10;
    uint32_t loop_count = 0;
    while (true) {
        if (bus_ok) {
            depth.update();
        }

        if (++loop_count >= HEARTBEAT_INTERVAL) {
            printf("[heartbeat] uptime=%lu ms\n",
                   static_cast<unsigned long>(to_ms_since_boot(get_absolute_time())));
            stdio_flush();
            loop_count = 0;
        }

        sleep_ms(LOOP_SLEEP_MS);
    }
}
