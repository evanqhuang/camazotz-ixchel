/*
 * AMOLED_Driver Implementation - QSPI + PIO + DMA for 1.64" AMOLED
 * Ported from Waveshare RP2350-Touch-AMOLED-1.64 reference SDK
 */

#include "drivers/amoled_driver.hpp"
#include "qspi_display.pio.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"

bool AMOLED_Driver::init() {
    /* CS pin — active low */
    gpio_init(DISPLAY_CS_PIN);
    gpio_pull_down(DISPLAY_CS_PIN);
    gpio_set_dir(DISPLAY_CS_PIN, GPIO_OUT);
    gpio_put(DISPLAY_CS_PIN, 1);

    /* RST pin — active low */
    gpio_init(DISPLAY_RST_PIN);
    gpio_set_dir(DISPLAY_RST_PIN, GPIO_OUT);

    /* Hardware reset sequence */
    hardware_reset();

    /* Load PIO program and init SM */
    uint offset = pio_add_program(pio_, &qspi_4wire_data_program);
    qspi_4wire_data_program_init(pio_, sm_, offset,
                                  DISPLAY_SCLK_PIN, DISPLAY_DIO0_PIN, 4);
    pio_sm_set_enabled(pio_, sm_, false);

    /* Claim and configure DMA channel */
    dma_chan_ = dma_claim_unused_channel(true);
    dma_cfg_ = dma_channel_get_default_config(static_cast<uint>(dma_chan_));
    channel_config_set_transfer_data_size(&dma_cfg_, DMA_SIZE_8);
    channel_config_set_read_increment(&dma_cfg_, true);
    channel_config_set_write_increment(&dma_cfg_, false);
    channel_config_set_dreq(&dma_cfg_, pio_get_dreq(pio_, sm_, true));

    /* MIPI DCS init sequence */
    init_registers();

    return true;
}

void AMOLED_Driver::hardware_reset() {
    gpio_put(DISPLAY_RST_PIN, 1);
    sleep_ms(50);
    gpio_put(DISPLAY_RST_PIN, 0);
    sleep_ms(50);
    gpio_put(DISPLAY_RST_PIN, 1);
    sleep_ms(300);
}

void AMOLED_Driver::cs_select() {
    gpio_put(DISPLAY_CS_PIN, 0);
}

void AMOLED_Driver::cs_deselect() {
    gpio_put(DISPLAY_CS_PIN, 1);
}

void AMOLED_Driver::pio_write(uint8_t val) {
    pio_sm_set_enabled(pio_, sm_, true);
    pio_sm_put_blocking(pio_, sm_, static_cast<uint32_t>(val) << 24U);
    /* Wait for TX FIFO to drain and PIO to shift out all bits */
    uint32_t stall_mask = 1U << (PIO_FDEBUG_TXSTALL_LSB + sm_);
    pio_->fdebug = stall_mask;  /* Clear stall flag */
    while (!(pio_->fdebug & stall_mask)) {
        tight_loop_contents();
    }
    pio_sm_set_enabled(pio_, sm_, false);
}

/*
 * QSPI 1-wire-over-4-wire encoding:
 * Each byte is split into 4 nibble writes on DIO0 only (DIO1-3 = 0).
 * Bit pairs are mapped: bit N → DIO0, bit N+1 → DIO0 (shifted).
 * This matches the Waveshare QSPI_DATA_Write / QSPI_CMD_Write encoding.
 */
void AMOLED_Driver::qspi_encode_write(uint8_t val) {
    uint8_t nibbles[4];
    for (int i = 0; i < 4; ++i) {
        uint8_t bit0 = (val & (1U << (2U * static_cast<unsigned>(i)))) ? 1U : 0U;
        uint8_t bit1 = (val & (1U << (2U * static_cast<unsigned>(i) + 1U))) ? 1U : 0U;
        nibbles[3 - i] = static_cast<uint8_t>(bit0 | (bit1 << 4U));
    }
    for (int i = 0; i < 4; ++i) {
        pio_write(nibbles[i]);
    }
}

void AMOLED_Driver::register_write(uint8_t addr) {
    /* Command prefix 0x02 for register write */
    qspi_encode_write(0x02);
    /* 24-bit address: 0x00, addr, 0x00 */
    qspi_encode_write(0x00);
    qspi_encode_write(addr);
    qspi_encode_write(0x00);
}

void AMOLED_Driver::pixel_write_prefix(uint8_t addr) {
    /* Command prefix 0x32 for pixel data */
    qspi_encode_write(0x32);
    /* 24-bit address: 0x00, addr, 0x00 */
    qspi_encode_write(0x00);
    qspi_encode_write(addr);
    qspi_encode_write(0x00);
}

void AMOLED_Driver::init_registers() {
    /* Sleep Out */
    cs_select();
    register_write(0x11);
    sleep_ms(120);
    cs_deselect();

    /* Tear Scanline */
    cs_select();
    register_write(0x44);
    qspi_encode_write(0x01);
    qspi_encode_write(0xC5);
    cs_deselect();

    /* Tearing Effect ON */
    cs_select();
    register_write(0x35);
    qspi_encode_write(0x00);
    cs_deselect();

    /* Interface Pixel Format: RGB565 */
    cs_select();
    register_write(0x3A);
    qspi_encode_write(0x55);
    cs_deselect();

    /* Set DSPI mode */
    cs_select();
    register_write(0xC4);
    qspi_encode_write(0x80);
    cs_deselect();

    /* Write Control Display */
    cs_select();
    register_write(0x53);
    qspi_encode_write(0x20);
    cs_deselect();

    /* High Brightness Mode */
    cs_select();
    register_write(0x63);
    qspi_encode_write(0xFF);
    cs_deselect();

    /* Brightness — start at 0, set_brightness() called later */
    cs_select();
    register_write(0x51);
    qspi_encode_write(0x00);
    cs_deselect();

    /* Display ON */
    cs_select();
    register_write(0x29);
    cs_deselect();

    sleep_ms(10);
}

void AMOLED_Driver::set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint16_t xa = x0 + DISPLAY_X_OFFSET;
    uint16_t xb = x1 + DISPLAY_X_OFFSET;

    /* Column Address Set (0x2A) */
    cs_select();
    register_write(0x2A);
    qspi_encode_write(static_cast<uint8_t>(xa >> 8U));
    qspi_encode_write(static_cast<uint8_t>(xa & 0xFFU));
    qspi_encode_write(static_cast<uint8_t>((xb - 1U) >> 8U));
    qspi_encode_write(static_cast<uint8_t>((xb - 1U) & 0xFFU));
    cs_deselect();

    /* Row Address Set (0x2B) */
    cs_select();
    register_write(0x2B);
    qspi_encode_write(static_cast<uint8_t>(y0 >> 8U));
    qspi_encode_write(static_cast<uint8_t>(y0 & 0xFFU));
    qspi_encode_write(static_cast<uint8_t>((y1 - 1U) >> 8U));
    qspi_encode_write(static_cast<uint8_t>((y1 - 1U) & 0xFFU));
    cs_deselect();

    /* Memory Write (0x2C) */
    cs_select();
    register_write(0x2C);
    cs_deselect();
}

void AMOLED_Driver::transfer_pixels(const uint8_t *data, uint32_t len) {
    cs_select();
    pixel_write_prefix(0x2C);

    /* Enable PIO SM for DMA-driven transfer */
    pio_sm_set_enabled(pio_, sm_, true);

    /* Reconfigure DREQ for TX direction and start DMA */
    channel_config_set_dreq(&dma_cfg_, pio_get_dreq(pio_, sm_, true));
    dma_channel_configure(
        static_cast<uint>(dma_chan_),
        &dma_cfg_,
        &pio_->txf[sm_],   /* Destination: PIO TX FIFO */
        data,               /* Source: pixel buffer */
        len,                /* Transfer count (bytes) */
        true                /* Start immediately */
    );

    /* Busy-wait for DMA completion */
    dma_channel_wait_for_finish_blocking(static_cast<uint>(dma_chan_));

    /* Wait for PIO to finish shifting out final bytes */
    uint32_t stall_mask = 1U << (PIO_FDEBUG_TXSTALL_LSB + sm_);
    pio_->fdebug = stall_mask;
    while (!(pio_->fdebug & stall_mask)) {
        tight_loop_contents();
    }

    pio_sm_set_enabled(pio_, sm_, false);
    cs_deselect();
}

void AMOLED_Driver::set_brightness(uint8_t percent) {
    if (percent > 100U) {
        percent = 100U;
    }
    uint8_t val = static_cast<uint8_t>(static_cast<uint16_t>(percent) * 255U / 100U);

    cs_select();
    register_write(0x51);
    qspi_encode_write(val);
    cs_deselect();
}
