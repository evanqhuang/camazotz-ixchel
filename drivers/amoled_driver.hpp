/*
 * AMOLED_Driver - Low-level QSPI + PIO + DMA driver for 1.64" AMOLED
 * Hardware: Waveshare RP2350-Touch-AMOLED-1.64 (280x456, RGB565)
 * Interface: QSPI via PIO0 SM0 with DMA pixel transfer
 */

#ifndef AMOLED_DRIVER_HPP
#define AMOLED_DRIVER_HPP

#include "config.h"

#include "hardware/pio.h"
#include "hardware/dma.h"

#include <cstdint>

class AMOLED_Driver {
public:
    bool init();
    void set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
    void transfer_pixels(const uint8_t *data, uint32_t len);
    void set_brightness(uint8_t percent);

private:
    PIO pio_ = DISPLAY_PIO_INSTANCE;
    uint sm_ = 0;
    int dma_chan_ = -1;
    dma_channel_config dma_cfg_ = {};

    void hardware_reset();
    void init_registers();
    void cs_select();
    void cs_deselect();
    void pio_write(uint8_t val);
    void qspi_encode_write(uint8_t val);
    void register_write(uint8_t addr);
    void pixel_write_prefix(uint8_t addr);
};

#endif // AMOLED_DRIVER_HPP
