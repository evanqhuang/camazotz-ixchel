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
#include "logic/core1_nav.hpp"
#include "utils/stdio_display.hpp"

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/multicore.h"
#include "hardware/watchdog.h"

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

    // Enable watchdog — Core 0 and Core 1 both feed it
    watchdog_enable(NAV_WATCHDOG_TIMEOUT_MS, true);
    printf("Watchdog enabled: %u ms timeout\n", NAV_WATCHDOG_TIMEOUT_MS);

    // Launch Core 1 navigation loop
    static Core1Context nav_ctx = {&encoder, &imu};
    multicore_launch_core1(core1_entry);
    multicore_fifo_push_blocking(reinterpret_cast<uint32_t>(&nav_ctx));
    printf("Core 1 navigation loop launched (100Hz)\n");

    printf("Entering main loop\n\n");
    stdio_flush();

    // Main loop — Core 0 handles depth sensor, FIFO consumption, and heartbeat logging
    static constexpr uint32_t HEARTBEAT_INTERVAL = 500;  // iterations (~5s at 10ms sleep)
    static constexpr uint32_t LOOP_SLEEP_MS = 10;
    uint32_t loop_count = 0;
    while (true) {
        watchdog_update();

        if (bus_ok) {
            depth.update();
        }

        // Drain FIFO notifications from Core 1 (non-blocking)
        uint32_t seq;
        while (multicore_fifo_pop_timeout_us(0, &seq)) {
            // Sequence number consumed — latest nav state available
        }

        if (++loop_count >= HEARTBEAT_INTERVAL) {
            nav_state_compact_t nav = core1_get_nav_state();
            JitterStats jitter = core1_get_jitter_stats();
            uint32_t drops = core1_get_dropped_frames();

            printf("[heartbeat] uptime=%lu ms  pos=(%.4f, %.4f, %.4f)  "
                   "flags=0x%02X  jitter=%lu/%lu/%lu us  drops=%lu\n",
                   static_cast<unsigned long>(to_ms_since_boot(get_absolute_time())),
                   static_cast<double>(nav.pos_x),
                   static_cast<double>(nav.pos_y),
                   static_cast<double>(nav.pos_z),
                   nav.status_flags,
                   static_cast<unsigned long>(jitter.min_us),
                   static_cast<unsigned long>(jitter.last_us),
                   static_cast<unsigned long>(jitter.max_us),
                   static_cast<unsigned long>(drops));
            stdio_flush();
            loop_count = 0;
        }

        sleep_ms(LOOP_SLEEP_MS);
    }
}
