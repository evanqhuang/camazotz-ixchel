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
#include "drivers/battery_monitor.hpp"
#include "logic/calibration_manager.hpp"
#include "logic/core1_nav.hpp"
#include "logic/depth_recovery.hpp"
#include "logic/nav_math.hpp"
#include "utils/stdio_display.hpp"
#include "utils/amoled_display.hpp"
#include "utils/dual_display.hpp"
#include "utils/nav_screen.hpp"
#include "utils/calib_screen.hpp"
#include "utils/splash_screen.hpp"
#include "utils/button_debounce.hpp"

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/multicore.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "utils/power_button.hpp"
#include "hardware/xosc.h"
#include "hardware/pll.h"
#include "hardware/structs/watchdog.h"

#include <cstdio>
#include <cmath>

[[noreturn]]
static void perform_shutdown(bool display_ok, bool sd_ok,
                              AMOLED_Display &amoled,
                              SDIO_Logger &sd_logger,
                              Nav_Screen &nav_screen) {
    printf("[SHUTDOWN] Initiating power-off sequence...\n");
    stdio_flush();

    /* Show shutdown message */
    if (display_ok) {
        nav_screen.show_critical_alert("Powering Off...");
        amoled.task_handler();
    }

    /* Log shutdown event */
    if (sd_ok && sd_logger.is_logging()) {
        sd_logger.log_event("SHUTDOWN", 0);
    }

    /* Flush and close SD card */
    sd_logger.deinit();
    printf("[SHUTDOWN] SD card unmounted\n");

    /* Stop Core 1 navigation loop */
    multicore_reset_core1();
    sleep_ms(10);
    printf("[SHUTDOWN] Core1 stopped\n");

    /* Disable watchdog */
    watchdog_disable();

    /* Turn off display */
    if (display_ok) {
        amoled.set_brightness(0);
    }

    /* Brief pause for visual feedback */
    sleep_ms(200);

    /* Set magic and reboot into dormant entry */
    printf("[SHUTDOWN] Entering dormant mode\n");
    stdio_flush();
    watchdog_hw->scratch[0] = POWER_OFF_MAGIC;
    watchdog_reboot(0, 0, 10);

    while (true) {
        tight_loop_contents();
    }
}

int main() {
    /* ── Dormant wake check ──────────────────────────────────────────
     * If the power-off magic is set in scratch[0], the user triggered
     * shutdown. Enter dormant immediately before any peripheral init.
     * On GPIO wake, reboot into normal boot path.
     */
    if (watchdog_hw->scratch[0] == POWER_OFF_MAGIC) {
        watchdog_hw->scratch[0] = 0U;

        /* Configure wake button */
        gpio_init(TARE_BUTTON_PIN);
        gpio_set_dir(TARE_BUTTON_PIN, GPIO_IN);
        gpio_pull_up(TARE_BUTTON_PIN);

        /* Wait for button release (user may still hold from shutdown) */
        while (!gpio_get(TARE_BUTTON_PIN)) {
            tight_loop_contents();
        }
        /* Brief debounce after release */
        busy_wait_us_32(50000U);

        /* Hold peripherals in reset during dormant */
        gpio_init(DISPLAY_RST_PIN);
        gpio_set_dir(DISPLAY_RST_PIN, GPIO_OUT);
        gpio_put(DISPLAY_RST_PIN, false);

        gpio_init(IMU_RST_PIN);
        gpio_set_dir(IMU_RST_PIN, GPIO_OUT);
        gpio_put(IMU_RST_PIN, false);

        /* Switch clk_sys to clk_ref (XOSC 12MHz) and disable PLLs */
        clock_configure_undivided(clk_sys,
            CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF,
            0,
            XOSC_HZ);
        pll_deinit(pll_sys);
        pll_deinit(pll_usb);

        /* Enter dormant until button press (falling edge) */
        gpio_set_dormant_irq_enabled(TARE_BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true);
        xosc_dormant();

        /* Woke up — clean reboot */
        gpio_acknowledge_irq(TARE_BUTTON_PIN, GPIO_IRQ_EDGE_FALL);
        watchdog_enable(1, true);
        while (true) {
            tight_loop_contents();
        }
    }

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
        show_splash_screen(3000);
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

    /* Log IMU failure but continue - nav screen will show degraded state */
    if (cal.imu.status == CalibrationStatus::Failed) {
        display.show_error("Cal", "IMU failed - running degraded",
                           DisplaySeverity::Warning);
        printf("[WARN] IMU calibration failed - continuing with degraded navigation\n");
        stdio_flush();
    }

    /* Initialize tare button (GP6, active-low with pull-up) - BEFORE walkthrough!
     * Without pull-up configured, the pin floats LOW and causes spurious skip. */
    gpio_init(TARE_BUTTON_PIN);
    gpio_set_dir(TARE_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(TARE_BUTTON_PIN);

    double north_offset_rad = 0.0;
    uint8_t mag_accuracy_at_tare = 0;

    /* Guided IMU calibration walkthrough - runs until 3/3 or user skips */
    if (imu_ok && display_ok) {
        display.clear();  // Clear boot log messages before showing walkthrough

        static Calib_Screen calib_screen;
        calib_screen.create();
        calib_screen.activate();

        CalibWalkthroughResult result = calib_screen.run(imu);

        if (result.success) {
            display.show_status("IMU", "Calibration complete - 3/3");
        } else {
            /* User skipped */
            char buf[48];
            snprintf(buf, sizeof(buf), "Skipped at accuracy %u/3", result.final_accuracy);
            display.show_status("IMU", buf);
        }

        display.clear();
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

    // Initialize battery monitor (ADC on GP26)
    static Battery_Monitor battery;
    battery.init();
    display.show_status("Sys", "Battery monitor initialized");
    stdio_flush();

    // Enable watchdog — Core 0 and Core 1 both feed it
    watchdog_enable(NAV_WATCHDOG_TIMEOUT_MS, true);
    display.show_status("Sys", "Watchdog enabled");

    // Flush any stale SHTP events before Core 1 takes over
    if (imu_ok) {
        imu.flush();
    }

    // Launch Core 1 navigation loop
    static Core1Context nav_ctx = {&encoder, &imu};
    multicore_launch_core1(core1_entry);
    multicore_fifo_push_blocking(reinterpret_cast<uint32_t>(&nav_ctx));
    display.show_status("Sys", "Nav loop launched (100Hz)");

    /* Create and activate navigation screen */
    static Nav_Screen nav_screen;
    if (display_ok) {
        nav_screen.create();
        nav_screen.activate();
        nav_screen.show_ready_alert();
    }

    display.show_status("Sys", "Entering main loop");
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
    static DebounceState tare_debounce = {};
    static PowerButtonState power_btn_state = {};
    double total_distance_core0 = 0.0;
    bool prev_low_battery = false;
    bool navigation_active = false;

    while (true) {
        watchdog_update();

        /* Update battery reading (throttled to 1Hz internally) */
        uint32_t now = to_ms_since_boot(get_absolute_time());
        battery.update(now);

        /* Read nav state from Core 1 every iteration for depth recovery */
        nav_state_compact_t nav = core1_get_nav_state();

        /* Accumulate total distance (display-only approximation: delta_dist may
         * include recovery-estimated segments from Core 1, so total is approximate) */
        total_distance_core0 += static_cast<double>(fabsf(nav.delta_dist));

        /* Extract heading and pitch from quaternion */
        Quat q = {nav.quat_w, nav.quat_x, nav.quat_y, nav.quat_z};
        double R[3][3];
        quaternion_to_rotation_matrix(q, R);
        double heading_rad, pitch_rad;
        extract_heading_pitch(R, &heading_rad, &pitch_rad);

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
        if (sd_ok && navigation_active) {
            sd_logger.log_state(nav);
        }

        /* Flush SD buffer at 10Hz */
        if (++flush_count >= FLUSH_INTERVAL) {
            if (sd_ok && navigation_active) {
                sd_logger.flush();
            }
            flush_count = 0;
        }

        /* Merge runtime flags with init-time sensor failures.
         * Core 1 flags reflect runtime degradation; init failures are
         * only known to Core 0 and must be injected here so the nav
         * screen never shows "OK" for a sensor that failed init. */
        uint8_t combined_flags = nav.status_flags | depth_flags;
        if (!enc_ok) {
            combined_flags |= NAV_FLAG_ENCODER_LOST;
        }
        if (!imu_ok) {
            combined_flags |= NAV_FLAG_IMU_LOST;
        }
        uint8_t new_flags = combined_flags & ~prev_status_flags;
        if (sd_ok && navigation_active && new_flags != 0) {
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
        /* NAV_CRITICAL edge detection for AMOLED alert */
        bool prev_critical = (prev_status_flags & NAV_FLAG_NAV_CRITICAL) != 0;
        bool curr_critical = (combined_flags & NAV_FLAG_NAV_CRITICAL) != 0;
        if (curr_critical && !prev_critical) {
            if (display_ok) {
                nav_screen.show_critical_alert("NAV CRITICAL");
            }
            printf("[ALERT] NAV CRITICAL\n");
        } else if (!curr_critical && prev_critical) {
            if (display_ok) {
                if (!navigation_active) {
                    nav_screen.show_ready_alert();
                } else {
                    nav_screen.hide_critical_alert();
                }
            }
            printf("[ALERT] NAV CRITICAL cleared\n");
        }
        prev_status_flags = combined_flags;

        /* Low battery edge-triggered alert */
        bool curr_low_battery = battery.low_battery();
        if (curr_low_battery && !prev_low_battery) {
            if (display_ok) {
                nav_screen.show_critical_alert("LOW BATTERY");
            }
            if (sd_ok && navigation_active) {
                sd_logger.log_event("LOW_BATTERY", combined_flags);
            }
            printf("[ALERT] LOW BATTERY (%u%%)\n", battery.percent());
        } else if (!curr_low_battery && prev_low_battery) {
            if (display_ok) {
                if (!navigation_active) {
                    nav_screen.show_ready_alert();
                } else {
                    nav_screen.hide_critical_alert();
                }
            }
        }
        prev_low_battery = curr_low_battery;

        /* Poll tare/power button (non-blocking) */
        bool raw_pressed = !gpio_get(TARE_BUTTON_PIN);
        debounce_check(now, raw_pressed, TARE_DEBOUNCE_MS, tare_debounce);

        ButtonAction btn_action = power_button_update(now,
            tare_debounce.stable_state,
            POWER_FEEDBACK_MS, POWER_LONG_PRESS_MS,
            power_btn_state);

        if (btn_action != ButtonAction::None) {
            printf("[BTN] action=%u raw=%d stable=%d was_pressed=%d nav_active=%d\n",
                   static_cast<unsigned>(btn_action), raw_pressed,
                   tare_debounce.stable_state, power_btn_state.was_pressed,
                   navigation_active);
            stdio_flush();
        }

        switch (btn_action) {
            case ButtonAction::Tare: {
                if (!navigation_active) {
                    /* === Start dive === */

                    /* 1. Capture magnetic heading before tare */
                    if (imu_ok) {
                        Quat pre_tare_q = imu.get_quaternion();
                        double R_tare[3][3];
                        quaternion_to_rotation_matrix(pre_tare_q, R_tare);
                        double pitch_unused;
                        extract_heading_pitch(R_tare, &north_offset_rad, &pitch_unused);
                        mag_accuracy_at_tare = imu.get_mag_accuracy();

                        printf("[START] Pre-tare heading: %.4f rad, mag accuracy: %u/3\n",
                               north_offset_rad, mag_accuracy_at_tare);

                        /* 2. Request tare — executed by Core 1 which owns I2C1 */
                        core1_request_tare();
                    }

                    /* 3. Reset Core 1 position to zero */
                    core1_request_position_reset();
                    total_distance_core0 = 0.0;

                    /* 4. Start SD logging with metadata */
                    if (sd_ok) {
                        NavLogMetadata meta = {};
                        meta.north_offset_rad = north_offset_rad;
                        meta.mag_accuracy = mag_accuracy_at_tare;
                        if (!sd_logger.start_logging(meta)) {
                            printf("[START] SD logging start failed\n");
                        }
                    }

                    /* 5. Clear trail map + hide ready overlay */
                    if (display_ok) {
                        nav_screen.reset_trail();
                        nav_screen.hide_critical_alert();
                        nav_screen.show_tare_status(true, now);
                    }

                    navigation_active = true;
                    printf("[START] Navigation active\n");
                    stdio_flush();
                } else {
                    /* Mid-dive tare disabled: breaks coordinate frame continuity */
                    if (display_ok) {
                        nav_screen.show_tare_locked(now);
                    }
                    printf("[TARE] Disabled during active navigation\n");
                    stdio_flush();
                }
                break;
            }
            case ButtonAction::ShowFeedback:
                if (display_ok) {
                    nav_screen.show_critical_alert("Hold to power off...");
                }
                printf("[POWER] Hold to power off...\n");
                stdio_flush();
                break;
            case ButtonAction::HideFeedback:
                if (display_ok) {
                    if (!navigation_active) {
                        nav_screen.show_ready_alert();
                    } else {
                        nav_screen.hide_critical_alert();
                    }
                }
                break;
            case ButtonAction::Shutdown:
                perform_shutdown(display_ok, sd_ok, amoled, sd_logger, nav_screen);
                break;
            case ButtonAction::None:
                break;
        }

        /* Update navigation screen (throttled to 5Hz internally) */
        if (display_ok) {
            float heading_deg = static_cast<float>(heading_rad * 180.0 / M_PI);
            if (heading_deg < 0.0f) {
                heading_deg += 360.0f;
            }
            nav_screen.update(heading_deg, static_cast<float>(z_recovered),
                              nav.pos_x, nav.pos_y,
                              static_cast<float>(total_distance_core0),
                              combined_flags,
                              sd_ok && sd_logger.is_operational(),
                              battery.percent(), battery.on_battery(),
                              now);
        }

        /* Process LVGL rendering events */
        if (display_ok) {
            amoled.task_handler();
        }

        // Drain FIFO notifications from Core 1 (non-blocking)
        uint32_t seq;
        while (multicore_fifo_pop_timeout_us(0, &seq)) {
            // Sequence number consumed — latest nav state available
        }

        if (++loop_count >= HEARTBEAT_INTERVAL) {
            JitterStats jitter = core1_get_jitter_stats();
            uint32_t drops = core1_get_dropped_frames();

            /* SD card sync and recovery during heartbeat (every 5s) */
            if (sd_ok) {
                if (sd_logger.is_operational()) {
                    sd_logger.sync();
                } else {
                    /* Attempt recovery if in error state */
                    sd_logger.try_recovery();
                }
            }

            /* Get SD logger stats for heartbeat output */
            LoggerStats sd_stats = sd_logger.get_stats();

            printf("[heartbeat] uptime=%lu ms  pos=(%.4f, %.4f, %.4f)  depth_z=%.4f  "
                   "dist=%.1f  flags=0x%02X  jitter=%lu/%lu/%lu us  drops=%lu  "
                   "sd=%s rec=%lu/%lu  bat=%.2fV(%u%%)  btn_raw=%d\n",
                   static_cast<unsigned long>(to_ms_since_boot(get_absolute_time())),
                   static_cast<double>(nav.pos_x),
                   static_cast<double>(nav.pos_y),
                   static_cast<double>(nav.pos_z),
                   z_recovered,
                   total_distance_core0,
                   combined_flags,
                   static_cast<unsigned long>(jitter.min_us),
                   static_cast<unsigned long>(jitter.last_us),
                   static_cast<unsigned long>(jitter.max_us),
                   static_cast<unsigned long>(drops),
                   sd_stats.mounted ? (sd_stats.critical_error ? "ERR" : "OK") : "OFF",
                   static_cast<unsigned long>(sd_stats.records_written),
                   static_cast<unsigned long>(sd_stats.records_dropped),
                   static_cast<double>(battery.voltage()),
                   battery.percent(),
                   !gpio_get(TARE_BUTTON_PIN));
            stdio_flush();
            loop_count = 0;
        }

        sleep_ms(LOOP_SLEEP_MS);
    }
}
