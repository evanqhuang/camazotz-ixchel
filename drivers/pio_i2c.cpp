/*
 * PIO_I2C Implementation - PIO-based I2C master driver
 * Based on Raspberry Pi pico-examples pio/i2c (BSD-3-Clause)
 */

#include "drivers/pio_i2c.hpp"

#include "pio_i2c.pio.h"

#include "hardware/clocks.h"
#include "pico/time.h"

bool PIO_I2C::init(PIO pio, uint sda_pin, uint scl_pin, uint freq_hz) {
    pio_ = pio;

    if (!pio_can_add_program(pio_, &i2c_program)) {
        return false;
    }
    offset_ = pio_add_program(pio_, &i2c_program);

    int claimed = pio_claim_unused_sm(pio_, false);
    if (claimed < 0) {
        return false;
    }
    sm_ = static_cast<uint>(claimed);

    // i2c_program_init sets clock divider for 100kHz by default
    // Override if different frequency requested
    i2c_program_init(pio_, sm_, offset_, sda_pin, scl_pin);

    if (freq_hz != 100000) {
        float div = static_cast<float>(clock_get_hz(clk_sys)) /
                    (32.0f * static_cast<float>(freq_hz));
        pio_sm_set_clkdiv(pio_, sm_, div);
    }

    return true;
}

void PIO_I2C::put16(uint16_t data) {
    if (error_) { return; }

    absolute_time_t deadline = make_timeout_time_us(TIMEOUT_US);
    while (pio_sm_is_tx_fifo_full(pio_, sm_)) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            error_ = true;
            return;
        }
        tight_loop_contents();
    }
    // Use volatile pointer through uintptr_t to avoid strict-aliasing warning.
    // The hardware register supports 16-bit writes at this address.
    volatile uint16_t *txf_halfword =
        reinterpret_cast<volatile uint16_t *>(
            reinterpret_cast<uintptr_t>(&pio_->txf[sm_]));
    *txf_halfword = data;
}

uint8_t PIO_I2C::get8() {
    if (error_) { return 0; }

    absolute_time_t deadline = make_timeout_time_us(TIMEOUT_US);
    while (pio_sm_is_rx_fifo_empty(pio_, sm_)) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            error_ = true;
            return 0;
        }
        tight_loop_contents();
    }
    return static_cast<uint8_t>(pio_->rxf[sm_]);
}

bool PIO_I2C::check_error() {
    return pio_interrupt_get(pio_, sm_);
}

void PIO_I2C::clear_error() {
    pio_interrupt_clear(pio_, sm_);
    // Drain FIFOs and jump SM back to entry point
    pio_sm_drain_tx_fifo(pio_, sm_);
    pio_sm_exec(pio_, sm_, pio_encode_jmp(offset_ + i2c_offset_entry_point));
}

void PIO_I2C::rx_enable(bool en) {
    if (en) {
        hw_set_bits(&pio_->sm[sm_].shiftctrl, PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
    } else {
        hw_clear_bits(&pio_->sm[sm_].shiftctrl, PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
    }
}

void PIO_I2C::start() {
    put16(1u << ICOUNT_LSB);  // Instruction count = 1 (execute 2 instructions)
    // set_scl_sda program instructions:
    // [0] set pindirs 0, side 0  => SCL=0, SDA=0
    // [1] set pindirs 1, side 0  => SCL=0, SDA=1
    // [2] set pindirs 0, side 1  => SCL=1, SDA=0
    // [3] set pindirs 1, side 1  => SCL=1, SDA=1
    put16(set_scl_sda_program_instructions[3]);  // SCL=1, SDA=1
    put16(set_scl_sda_program_instructions[2]);  // SCL=1, SDA=0 (START: SDA falls while SCL high)
}

void PIO_I2C::stop() {
    put16(2u << ICOUNT_LSB);  // Execute 3 instructions
    put16(set_scl_sda_program_instructions[0]);  // SCL=0, SDA=0
    put16(set_scl_sda_program_instructions[2]);  // SCL=1, SDA=0
    put16(set_scl_sda_program_instructions[3]);  // SCL=1, SDA=1 (STOP: SDA rises while SCL high)
}

void PIO_I2C::repstart() {
    put16(3u << ICOUNT_LSB);  // Execute 4 instructions
    put16(set_scl_sda_program_instructions[1]);  // SCL=0, SDA=1
    put16(set_scl_sda_program_instructions[3]);  // SCL=1, SDA=1
    put16(set_scl_sda_program_instructions[2]);  // SCL=1, SDA=0 (START)
    put16(set_scl_sda_program_instructions[0]);  // SCL=0, SDA=0
}

bool PIO_I2C::write_blocking(uint8_t addr, const uint8_t *data, uint32_t len) {
    error_ = false;
    start();
    rx_enable(false);

    // Address byte: addr << 1 | 0 (write)
    put16((static_cast<uint16_t>(addr) << 2) | 1u);  // Data shifted to bits 8:1, bit 0 = NAK=0

    // Data bytes - NAK bit always set (ignore NAKs), FINAL on last byte
    for (uint32_t i = 0; i < len; i++) {
        if (error_ || check_error()) {
            clear_error();
            error_ = false;
            return false;
        }
        bool last = (i == len - 1);
        put16((static_cast<uint16_t>(data[i]) << DATA_LSB) |
              (last ? (1u << FINAL_LSB) : 0) |
              (1u << NAK_LSB));
    }

    stop();

    if (error_) {
        clear_error();
        error_ = false;
        return false;
    }

    // Drain any stale RX data
    while (!pio_sm_is_rx_fifo_empty(pio_, sm_)) {
        (void)pio_->rxf[sm_];
    }

    return !check_error();
}

bool PIO_I2C::read_blocking(uint8_t addr, uint8_t *data, uint32_t len) {
    error_ = false;
    start();
    rx_enable(true);

    // Address byte: addr << 1 | 1 (read)
    put16((static_cast<uint16_t>(addr) << 2) | 3u);  // bit 0 of addr byte = 1 (read), NAK=1

    for (uint32_t i = 0; i < len; i++) {
        if (error_ || check_error()) {
            clear_error();
            error_ = false;
            return false;
        }
        bool last = (i == len - 1);
        // For reads, send 0xFF (release SDA) and set Final+NAK on last byte
        put16((0xFFu << DATA_LSB) |
              (last ? (1u << FINAL_LSB) : 0) |
              (last ? (1u << NAK_LSB) : 0));
    }

    // Read back all received bytes
    for (uint32_t i = 0; i < len; i++) {
        data[i] = get8();
        if (error_) {
            clear_error();
            error_ = false;
            return false;
        }
    }

    stop();
    rx_enable(false);

    if (error_) {
        clear_error();
        error_ = false;
        return false;
    }

    return !check_error();
}

bool PIO_I2C::write_read_blocking(uint8_t addr, const uint8_t *wr, uint32_t wr_len,
                                   uint8_t *rd, uint32_t rd_len) {
    error_ = false;
    start();
    rx_enable(false);

    // Write phase: address + write data
    put16((static_cast<uint16_t>(addr) << 2) | 1u);

    for (uint32_t i = 0; i < wr_len; i++) {
        if (error_ || check_error()) {
            clear_error();
            error_ = false;
            return false;
        }
        put16((static_cast<uint16_t>(wr[i]) << DATA_LSB) | (1u << NAK_LSB));
    }

    // Repeated start for read phase
    repstart();
    rx_enable(true);

    // Read phase: address + read data
    put16((static_cast<uint16_t>(addr) << 2) | 3u);

    for (uint32_t i = 0; i < rd_len; i++) {
        if (error_ || check_error()) {
            clear_error();
            error_ = false;
            return false;
        }
        bool last = (i == rd_len - 1);
        put16((0xFFu << DATA_LSB) |
              (last ? (1u << FINAL_LSB) : 0) |
              (last ? (1u << NAK_LSB) : 0));
    }

    for (uint32_t i = 0; i < rd_len; i++) {
        rd[i] = get8();
        if (error_) {
            clear_error();
            error_ = false;
            return false;
        }
    }

    stop();
    rx_enable(false);

    if (error_) {
        clear_error();
        error_ = false;
        return false;
    }

    return !check_error();
}

bool PIO_I2C::send_command(uint8_t addr, uint8_t cmd) {
    return write_blocking(addr, &cmd, 1);
}
