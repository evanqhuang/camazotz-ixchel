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
#define IMU_RST_PIN             4U       /* Active low, hardware reset recovery */
#define IMU_INT_PIN             5U       /* HINTN: sensor asserts LOW when ready */
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
#define LOG_FLUSH_RATE_HZ       10U      /* SD card flush rate */

/*============================================================================
 * Calibration Constants
 *============================================================================*/
#define CALIB_IMU_STABILITY_TIMEOUT_MS  10000U  /* Max wait for IMU accuracy */
#define CALIB_IMU_MIN_ACCURACY          2U      /* Minimum acceptable accuracy (0-3) */
#define CALIB_DEPTH_CRC_MAX_RETRIES     3U      /* PROM CRC retry attempts */

#endif /* MAPPER_CONFIG_H */
