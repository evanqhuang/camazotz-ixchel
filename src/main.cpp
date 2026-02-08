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
#include "drivers/sdio_logger.hpp"
#include "logic/calibration_manager.hpp"
#include "logic/core1_nav.hpp"
#include "logic/depth_recovery.hpp"
#include "logic/nav_math.hpp"
#include "utils/stdio_display.hpp"
#include "utils/amoled_display.hpp"
#include "utils/dual_display.hpp"

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/multicore.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"

#include <cstdio>

int main() {
    /* Set system clock to 150MHz for QSPI display bandwidth */
    set_sys_clock_khz(SYS_CLOCK_KHZ, true);
    clock_configure(
        clk_peri,
        0,
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
        SYS_CLOCK_KHZ * 1000U,
        SYS_CLOCK_KHZ * 1000U
    );

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
    printf("Clock: %lu kHz\n", static_cast<unsigned long>(SYS_CLOCK_KHZ));
    printf("========================================\n\n");
    stdio_flush();

    /* Initialize AMOLED display */
    static AMOLED_Display amoled;
    Stdio_Display stdio_display;
    bool display_ok = amoled.init();

    if (display_ok) {
        printf("AMOLED display initialized OK\n");
    } else {
        printf("AMOLED display init FAILED — falling back to stdio only\n");
    }
    stdio_flush();

    /* Both AMOLED and stdio receive all display calls simultaneously.
     * Falls back to stdio-only if AMOLED init fails. */
    static Dual_Display dual(amoled, stdio_display);
    Display_Interface &display = display_ok
        ? static_cast<Display_Interface &>(dual)
        : static_cast<Display_Interface &>(stdio_display);

    Encoder_Wrapper encoder;
    IMU_Wrapper imu;
    PIO_I2C depth_bus;
    Depth_Wrapper depth;

    // I2C bus scan for diagnostics
    printf("\n[DIAG] I2C Bus Scan\n");

    // Scan I2C0 (encoder bus)
    i2c_init(i2c0, 100000);  // 100kHz for scan
    gpio_set_function(ENCODER_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(ENCODER_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(ENCODER_SDA_PIN);
    gpio_pull_up(ENCODER_SCL_PIN);
    printf("[DIAG] I2C0 (GP%d/GP%d): ", ENCODER_SDA_PIN, ENCODER_SCL_PIN);
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        uint8_t dummy;
        int ret = i2c_read_timeout_us(i2c0, addr, &dummy, 1, false, 5000);
        if (ret >= 0) {
            printf("0x%02X ", addr);
        }
    }
    printf("\n");
    i2c_deinit(i2c0);

    // Scan I2C1 (IMU bus)
    i2c_init(i2c1, 100000);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);
    printf("[DIAG] I2C1 (GP%d/GP%d): ", IMU_SDA_PIN, IMU_SCL_PIN);
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        uint8_t dummy;
        int ret = i2c_read_timeout_us(i2c1, addr, &dummy, 1, false, 5000);
        if (ret >= 0) {
            printf("0x%02X ", addr);
        }
    }
    printf("\n");
    i2c_deinit(i2c1);
    stdio_flush();

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

    {
        char summary[80];
        snprintf(summary, sizeof(summary), "encoder=%s imu=%s depth=%s",
                 enc_ok ? "OK" : "FAIL", imu_ok ? "OK" : "FAIL", bus_ok ? "OK" : "FAIL");
        display.show_status("Init", summary);
    }
    stdio_flush();

    // Boot calibration — skips sensors that failed init
    Calibration_Manager calibrator(encoder, imu, depth, depth_bus, display);
    CalibrationResult cal = calibrator.run_boot_calibration(enc_ok, imu_ok, bus_ok);

    {
        char cal_summary[80];
        snprintf(cal_summary, sizeof(cal_summary), "encoder=%u imu=%u depth=%u",
                 static_cast<unsigned>(cal.encoder.status),
                 static_cast<unsigned>(cal.imu.status),
                 static_cast<unsigned>(cal.depth.status));
        display.show_status("Cal", cal_summary);
    }

    // Initialize SD card logger (before Core 1 launch so boot count is stable)
    static SDIO_Logger sd_logger;
    bool sd_ok = sd_logger.init();
    if (sd_ok) {
        display.show_status("SD", "Logger initialized");
    } else {
        display.show_error("SD", "Logger init FAILED", DisplaySeverity::Warning);
    }
    stdio_flush();

    // Enable watchdog — Core 0 and Core 1 both feed it
    watchdog_enable(NAV_WATCHDOG_TIMEOUT_MS, true);
    display.show_status("Sys", "Watchdog enabled");

    // Launch Core 1 navigation loop
    static Core1Context nav_ctx = {&encoder, &imu};
    multicore_launch_core1(core1_entry);
    multicore_fifo_push_blocking(reinterpret_cast<uint32_t>(&nav_ctx));
    display.show_status("Sys", "Nav loop launched (100Hz)");

    display.show_status("Sys", "Running");
    stdio_flush();

    // Main loop — Core 0 handles depth sensor, FIFO consumption, display, logging, and heartbeat
    static constexpr uint32_t HEARTBEAT_INTERVAL = 500;  // iterations (~5s at 10ms sleep)
    static constexpr uint32_t FLUSH_INTERVAL = 10;       // iterations (~100ms at 10ms = 10Hz flush)
    static constexpr uint32_t LOOP_SLEEP_MS = 10;
    uint32_t loop_count = 0;
    uint32_t flush_count = 0;
    static DepthRecoveryState depth_recovery_state = {};
    double z_recovered = 0.0;
    uint8_t depth_flags = 0;
    uint8_t prev_status_flags = 0;

    while (true) {
        watchdog_update();

        /* Read nav state from Core 1 every iteration for depth recovery */
        nav_state_compact_t nav = core1_get_nav_state();

        /* Extract pitch from quaternion for depth recovery geometric estimation */
        Quat q = {nav.quat_w, nav.quat_x, nav.quat_y, nav.quat_z};
        double R[3][3];
        quaternion_to_rotation_matrix(q, R);
        double heading_unused, pitch_rad;
        extract_heading_pitch(R, &heading_unused, &pitch_rad);

        /* Update depth sensor and run tiered recovery */
        bool depth_updated = false;
        if (bus_ok) {
            depth_updated = depth.update();
        }

        DepthSnapshot dsnap = {
            .new_reading = depth_updated,
            .depth_m = depth.depth_m(),
            .pitch_rad = static_cast<float>(pitch_rad),
            .delta_dist = nav.delta_dist,
        };
        depth_flags = depth_recovery_update(depth_recovery_state, dsnap, &z_recovered);

        /* Log navigation state to SD card */
        if (sd_ok) {
            sd_logger.log_state(nav);
        }

        /* Flush SD buffer at 10Hz */
        if (++flush_count >= FLUSH_INTERVAL) {
            if (sd_ok) {
                sd_logger.flush();
            }
            flush_count = 0;
        }

        /* Log critical status flag transitions */
        uint8_t combined_flags = nav.status_flags | depth_flags;
        uint8_t new_flags = combined_flags & ~prev_status_flags;
        if (sd_ok && new_flags != 0) {
            if (new_flags & NAV_FLAG_ENCODER_LOST) {
                sd_logger.log_event("ENCODER_LOST", combined_flags);
            }
            if (new_flags & NAV_FLAG_IMU_LOST) {
                sd_logger.log_event("IMU_LOST", combined_flags);
            }
            if (new_flags & NAV_FLAG_NAV_CRITICAL) {
                sd_logger.log_event("NAV_CRITICAL", combined_flags);
            }
            if (new_flags & NAV_FLAG_ENCODER_ESTIMATED) {
                sd_logger.log_event("ENCODER_ESTIMATED", combined_flags);
            }
            if (new_flags & NAV_FLAG_IMU_ESTIMATED) {
                sd_logger.log_event("IMU_ESTIMATED", combined_flags);
            }
        }
        prev_status_flags = combined_flags;

        /* Process LVGL rendering events */
        if (display_ok) {
            amoled.task_handler();
        }

        // Drain FIFO notifications from Core 1 (non-blocking)
        uint32_t seq;
        while (multicore_fifo_pop_timeout_us(0, &seq)) {
            // Sequence number consumed — latest nav state available
        }

        /* Check for depth sensor NAV_CRITICAL */
        if (depth_flags & NAV_FLAG_NAV_CRITICAL) {
            display.show_error("NAV", "DEPTH LOST - NAV CRITICAL",
                               DisplaySeverity::Fatal);
        }

        if (++loop_count >= HEARTBEAT_INTERVAL) {
            JitterStats jitter = core1_get_jitter_stats();
            uint32_t drops = core1_get_dropped_frames();

            if (nav.status_flags & NAV_FLAG_IMU_LOST) {
                display.show_error("NAV", "IMU LOST - NAV CRITICAL",
                                   DisplaySeverity::Fatal);
            }

            /* SD card sync and recovery during heartbeat (every 5s) */
            if (sd_ok) {
                if (sd_logger.is_operational()) {
                    sd_logger.sync();
                } else {
                    /* Attempt recovery if in error state */
                    if (sd_logger.try_recovery()) {
                        sd_logger.log_event("SD_RECOVERED", combined_flags);
                    }
                }
            }

            /* Get SD logger stats for heartbeat output */
            LoggerStats sd_stats = sd_logger.get_stats();

            printf("[heartbeat] uptime=%lu ms  pos=(%.4f, %.4f, %.4f)  depth_z=%.4f  "
                   "flags=0x%02X  jitter=%lu/%lu/%lu us  drops=%lu  "
                   "sd=%s rec=%lu/%lu\n",
                   static_cast<unsigned long>(to_ms_since_boot(get_absolute_time())),
                   static_cast<double>(nav.pos_x),
                   static_cast<double>(nav.pos_y),
                   static_cast<double>(nav.pos_z),
                   z_recovered,
                   combined_flags,
                   static_cast<unsigned long>(jitter.min_us),
                   static_cast<unsigned long>(jitter.last_us),
                   static_cast<unsigned long>(jitter.max_us),
                   static_cast<unsigned long>(drops),
                   sd_stats.mounted ? (sd_stats.critical_error ? "ERR" : "OK") : "OFF",
                   static_cast<unsigned long>(sd_stats.records_written),
                   static_cast<unsigned long>(sd_stats.records_dropped));
            stdio_flush();
            loop_count = 0;
        }

        sleep_ms(LOOP_SLEEP_MS);
    }
}
