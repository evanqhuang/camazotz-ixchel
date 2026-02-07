/*
 * PIO_I2C - PIO-based I2C master driver for RP2350
 * Used for MS5837 depth sensor on PIO1 (100kHz, open-drain)
 * Based on Raspberry Pi pico-examples pio/i2c (BSD-3-Clause)
 */

#ifndef PIO_I2C_HPP
#define PIO_I2C_HPP

#include "hardware/pio.h"

#include <cstdint>

class PIO_I2C {
public:
    bool init(PIO pio, uint sda_pin, uint scl_pin, uint freq_hz);
    bool write_blocking(uint8_t addr, const uint8_t *data, uint32_t len);
    bool read_blocking(uint8_t addr, uint8_t *data, uint32_t len);
    bool write_read_blocking(uint8_t addr, const uint8_t *wr, uint32_t wr_len,
                             uint8_t *rd, uint32_t rd_len);
    bool send_command(uint8_t addr, uint8_t cmd);

private:
    static constexpr uint8_t ICOUNT_LSB = 10;
    static constexpr uint8_t FINAL_LSB = 9;
    static constexpr uint8_t DATA_LSB = 1;
    static constexpr uint8_t NAK_LSB = 0;

    static constexpr uint32_t TIMEOUT_US = 10000;

    PIO pio_ = nullptr;
    uint sm_ = 0;
    uint offset_ = 0;
    bool error_ = false;

    void put16(uint16_t data);
    uint8_t get8();
    bool check_error();
    void clear_error();
    void start();
    void stop();
    void repstart();
    void rx_enable(bool en);
};

#endif // PIO_I2C_HPP
