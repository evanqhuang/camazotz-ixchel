/*
 * FatFS Hardware Configuration for RP2350 Mapper Firmware
 * Provides SDIO interface callbacks required by no-OS-FatFS library
 */

#include "hw_config.h"
#include "config.h"
#include "hardware/gpio.h"

/*
 * SDIO Interface Configuration
 *
 * Pin relationships (per rp2040_sdio.pio):
 *   CLK_gpio = (D0_gpio + SDIO_CLK_PIN_D0_OFFSET) % 32
 *   where SDIO_CLK_PIN_D0_OFFSET = 30 (i.e., -2 in mod32)
 *
 * Our pin assignment (from config.h):
 *   SD_D0_PIN  = 20
 *   SD_CLK_PIN = 18 = (20 + 30) % 32 = 50 % 32 = 18 ✓
 *   SD_CMD_PIN = 19
 *   D1 = 21, D2 = 22, D3 = 23 (consecutive from D0)
 */
static sd_sdio_if_t sdio_if = {
    /*
     * Only CMD and D0 are explicitly set - library derives CLK (D0-2) and D1-D3.
     * Matching Waveshare example pattern exactly.
     */
    .CMD_gpio = SD_CMD_PIN,   /* 19 */
    .D0_gpio = SD_D0_PIN,     /* 20 → CLK=18, D1=21, D2=22, D3=23 */
    .SDIO_PIO = pio1,         /* PIO0 used by AMOLED QSPI */
    .DMA_IRQ_num = DMA_IRQ_1, /* DMA_IRQ_0 used by AMOLED */
    .baud_rate = 125 * 1000 * 1000 / 6,  /* ~21MHz - conservative speed */
    .set_drive_strength = true,
    .CLK_gpio_drive_strength = GPIO_DRIVE_STRENGTH_12MA,
    .CMD_gpio_drive_strength = GPIO_DRIVE_STRENGTH_4MA,
    .D0_gpio_drive_strength = GPIO_DRIVE_STRENGTH_4MA,
    .D1_gpio_drive_strength = GPIO_DRIVE_STRENGTH_4MA,
    .D2_gpio_drive_strength = GPIO_DRIVE_STRENGTH_4MA,
    .D3_gpio_drive_strength = GPIO_DRIVE_STRENGTH_4MA,
};

/* SD Card object with SDIO interface */
static sd_card_t sd_card = {
    .type = SD_IF_SDIO,
    .sdio_if_p = &sdio_if,
    .use_card_detect = false,  /* No CD pin connected */
};

/**
 * @brief Get the number of SD cards in the configuration.
 * @return Always 1 (single SD card slot)
 */
size_t sd_get_num(void) {
    return 1U;
}

/**
 * @brief Get SD card object by index.
 * @param num SD card index (must be 0)
 * @return Pointer to SD card object, or NULL if index invalid
 */
sd_card_t* sd_get_by_num(size_t num) {
    if (num == 0U) {
        return &sd_card;
    }
    return NULL;
}
