/*
 * Hardware Configuration for RP2350 Mapper Firmware
 * Safety-critical embedded system - MISRA-aligned conventions
 */

#ifndef MAPPER_CONFIG_H
#define MAPPER_CONFIG_H

/*============================================================================
 * PIO I2C - Depth Sensor (Software I2C via PIO)
 *============================================================================*/
#define DEPTH_SDA_PIN           0U
#define DEPTH_SCL_PIN           1U       /* Must be SDA + 1 for PIO I2C */
#define DEPTH_I2C_FREQ_HZ       100000U  /* 100kHz Standard Mode */
#define DEPTH_PIO_INSTANCE      pio1     /* PIO0 reserved for AMOLED QSPI */

/*============================================================================
 * I2C1 - BNO08x IMU
 *============================================================================*/
#define IMU_I2C_NUM             1U       /* I2C instance 1: use i2c_get_inst() */
#define IMU_SDA_PIN             2U
#define IMU_SCL_PIN             3U
#define IMU_RST_PIN             5U       /* Active low, hardware reset recovery */
#define IMU_INT_PIN             4U       /* HINTN: sensor asserts LOW when ready */
#define IMU_I2C_FREQ_HZ         100000U  /* 100kHz — clock stretching unreliable at 400kHz */

/*============================================================================
 * I2C0 - AS5600 Magnetic Encoder
 *============================================================================*/
#define ENCODER_I2C_NUM         0U       /* I2C instance 0: use i2c_get_inst() */
#define ENCODER_SDA_PIN         16U      /* Even pin = valid I2C0 SDA */
#define ENCODER_SCL_PIN         17U      /* Odd pin = valid I2C0 SCL */
#define ENCODER_I2C_FREQ_HZ     400000U  /* 400kHz Fast Mode */

/*============================================================================
 * SDIO - SD Card (4-bit mode)
 *============================================================================*/
#define SD_CLK_PIN              18U
#define SD_CMD_PIN              19U
#define SD_D0_PIN               20U
#define SD_D1_PIN               21U
#define SD_D2_PIN               22U
#define SD_D3_PIN               23U
/* SDIO constraint: CLK = D0 - 2 (18 = 20 - 2) verified */

/*============================================================================
 * System Timing
 *============================================================================*/
#define NAV_LOOP_RATE_HZ        100U     /* Core1 navigation tick rate */

/*============================================================================
 * Calibration Constants
 *============================================================================*/
#define CALIB_IMU_STABILITY_TIMEOUT_MS  60000U  /* Max wait for IMU accuracy */
#define CALIB_IMU_MIN_ACCURACY          3U      /* Minimum acceptable accuracy (0-3) */
#define CALIB_DEPTH_CRC_MAX_RETRIES     3U      /* PROM CRC retry attempts */
#define CALIB_WALKTHROUGH_POLL_MS       100U    /* 10Hz polling for walkthrough */
#define CALIB_PHASE_MIN_DWELL_MS        5000U   /* 5s per phase minimum */

/*============================================================================
 * Encoder Wheel Geometry
 *============================================================================*/
#define ENCODER_WHEEL_RADIUS_M  0.025f   /* Measuring wheel radius (meters) */

/*============================================================================
 * Core 1 Navigation Loop
 *============================================================================*/
#define NAV_TICK_PERIOD_MS              10U      /* 100Hz tick rate */
#define NAV_FIFO_TIMEOUT_US             0U       /* Non-blocking FIFO push */
#define NAV_WATCHDOG_TIMEOUT_MS         250U     /* 25 missed ticks = reboot (fits IMU reset ~110ms) */

/*============================================================================
 * AMOLED Display - QSPI via PIO0
 *============================================================================*/
#define DISPLAY_CS_PIN              9U
#define DISPLAY_SCLK_PIN            10U
#define DISPLAY_DIO0_PIN            11U
#define DISPLAY_DIO1_PIN            12U
#define DISPLAY_DIO2_PIN            13U
#define DISPLAY_DIO3_PIN            14U
#define DISPLAY_RST_PIN             15U
#define DISPLAY_PIO_INSTANCE        pio0
#define DISPLAY_WIDTH               280U
#define DISPLAY_HEIGHT              456U
#define DISPLAY_X_OFFSET            20U
#define DISPLAY_DEFAULT_BRIGHTNESS  80U

/*============================================================================
 * Tare Button
 *============================================================================*/
#define TARE_BUTTON_PIN             6U       /* GP6: active-low with pull-up */
#define TARE_DEBOUNCE_MS            50U      /* Debounce interval (ms) */

/*============================================================================
 * Power Button (Long-Press Power Off)
 *============================================================================*/
#define POWER_FEEDBACK_MS       1500U    /* Show "Hold to power off..." */
#define POWER_LONG_PRESS_MS     3000U    /* Trigger shutdown sequence */
#define POWER_OFF_MAGIC         0xCAFED00DU  /* Scratch register sentinel */

/*============================================================================
 * System Clock
 *============================================================================*/
#define SYS_CLOCK_KHZ               150000U

/*============================================================================
 * Battery Monitoring (ADC)
 *============================================================================*/
#define BAT_ADC_PIN             26U      /* GP26 — BAT_ADC via 200K/100K divider */
#define BAT_ADC_CHANNEL         0U       /* ADC input 0 (GP26 = ADC0) */
#define BAT_VOLTAGE_DIVIDER     3.0f     /* R30(200K) + R35(100K) = VCC/3 */
#define BAT_ADC_VREF            3.3f     /* ADC reference voltage */
#define BAT_ADC_RESOLUTION      4096U    /* 12-bit ADC */
#define BAT_EMA_ALPHA           0.05f    /* EMA smoothing (~20 sample lag) */
#define BAT_SAMPLE_INTERVAL_MS  1000U    /* Sample every 1s */
#define BAT_LOW_THRESHOLD_PCT   10U      /* Low battery alert (%) */
#define BAT_VBUS_THRESHOLD_V    4.5f     /* Above = USB power (no battery) */
#define BAT_MIN_VOLTAGE         3.0f     /* 0% */
#define BAT_MAX_VOLTAGE         4.2f     /* 100% */

/*============================================================================
 * Error Recovery Thresholds
 *============================================================================*/
#define NAV_ENCODER_FAIL_THRESHOLD      10U      /* Set ESTIMATED after N consecutive failures */
#define NAV_IMU_TIER1_THRESHOLD         5U       /* IMU Tier 1→2 boundary (extrapolate→hold) */
#define NAV_IMU_TIER3_THRESHOLD         50U      /* IMU Tier 2→3 boundary (hold→lost) */
#define NAV_CRITICAL_FAIL_THRESHOLD     50U      /* Set NAV_CRITICAL after N consecutive failures */
#define NAV_ENCODER_DECAY_FACTOR        0.8f    /* 20% velocity reduction per Tier 2 tick */
#define NAV_ENCODER_VELOCITY_EPSILON    1e-6f   /* m/s threshold for Tier 2 -> 3 transition */
#define NAV_IMU_DIST_THROTTLE           0.5f    /* IMU Tier 2 distance scaling factor */

/*============================================================================
 * Sensor Conflict Detection (IMU-Encoder Cross-Validation)
 *============================================================================*/
#define CONFLICT_ENCODER_MOTION_THRESHOLD   0.01f   /* rad - min encoder delta to be "moving" */
#define CONFLICT_ENCODER_ZERO_THRESHOLD     0.001f  /* rad - below this = encoder "zero" */
#define CONFLICT_IMU_OMEGA_THRESHOLD        0.05f   /* rad/s - min angular velocity to be "rotating" */
#define CONFLICT_IMU_ACCEL_THRESHOLD        0.3f    /* m/s² - min linear accel to be "moving" */
#define CONFLICT_DECEL_THRESHOLD            0.2f    /* m/s² - min deceleration to trigger faster decay */
#define CONFLICT_WINDOW_TICKS               5U      /* Ticks to accumulate before Case A evaluation */
#define CONFLICT_ZERO_ENCODER_TICKS         10U     /* Ticks of zero encoder before Case C triggers */
#define CONFLICT_TIER1_THRESHOLD            10U     /* Flag only, no attenuation */
#define CONFLICT_TIER2_THRESHOLD            30U     /* 50% distance attenuation */
#define CONFLICT_TIER3_THRESHOLD            100U    /* Zero distance (severe) */
#define CONFLICT_DIST_ATTENUATION           0.5f    /* Tier 2 distance factor */
#define CONFLICT_ACCEL_DECAY_FACTOR         0.6f    /* Faster decay when deceleration detected */

/*============================================================================
 * Error Recovery - Depth Sensor
 *============================================================================*/
#define DEPTH_FAIL_GRACE_TICKS          10U     /* 100ms grace before Tier 1 */
#define DEPTH_TIER2_THRESHOLD_TICKS     200U    /* 2s: Tier 1 → Tier 2 boundary */
#define DEPTH_TIER3_THRESHOLD_TICKS     1000U   /* 10s: Tier 2 → Tier 3 boundary */
#define DEPTH_SLEW_DURATION_TICKS       200U    /* 2s linear slew on recovery */

/*============================================================================
 * SD Card Logging
 *============================================================================*/
#define LOG_BUFFER_SIZE_BYTES   32768U   /* 32KB ring buffer (682 records) */
#define LOG_FLUSH_RATE_HZ       10U      /* Ring buffer flush rate */
#define LOG_SYNC_INTERVAL_S     5U       /* f_sync interval (FAT commit) */
#define LOG_EVENT_COOLDOWN_MS   1000U    /* Event deduplication cooldown */
#define SD_BAUD_RATE            37500000U /* 37.5MHz for 150MHz sysclk */

#endif /* MAPPER_CONFIG_H */
