/*
 * Depth_Wrapper Implementation - MS5837-30BA over PIO I2C
 * Non-blocking state machine with second-order temperature compensation
 * Compensation formulas ported from BlueRobotics MS5837 library (MIT)
 */

#include "drivers/depth_wrapper.hpp"


bool Depth_Wrapper::begin(PIO_I2C &bus) {
    bus_ = &bus;

    // Reset sensor
    if (!bus_->send_command(MS5837_ADDR, CMD_RESET)) {
        return false;
    }
    sleep_ms(10);

    // Read PROM calibration coefficients
    if (!read_prom()) {
        return false;
    }

    // Validate CRC4
    uint8_t crc_read = prom_[0] >> 12;
    uint8_t crc_calc = ms5837_crc4(prom_);
    if (crc_read != crc_calc) {
        return false;
    }

    state_ = State::Idle;
    consecutive_failures = 0;
    return true;
}

bool Depth_Wrapper::update() {
    if (bus_ == nullptr) {
        return false;
    }

    switch (state_) {
        case State::Idle:
            if (!start_conversion(CMD_CONVERT_D1)) {
                if (consecutive_failures < UINT16_MAX) { consecutive_failures++; }
                return false;
            }
            conversion_start_ = get_absolute_time();
            state_ = State::WaitingD1;
            return false;

        case State::WaitingD1:
            if (absolute_time_diff_us(conversion_start_, get_absolute_time()) <
                static_cast<int64_t>(CONVERSION_TIME_MS) * 1000) {
                return false;
            }
            if (!read_adc(&d1_raw_)) {
                if (consecutive_failures < UINT16_MAX) { consecutive_failures++; }
                state_ = State::Idle;
                return false;
            }
            if (!start_conversion(CMD_CONVERT_D2)) {
                if (consecutive_failures < UINT16_MAX) { consecutive_failures++; }
                state_ = State::Idle;
                return false;
            }
            conversion_start_ = get_absolute_time();
            state_ = State::WaitingD2;
            return false;

        case State::WaitingD2:
            if (absolute_time_diff_us(conversion_start_, get_absolute_time()) <
                static_cast<int64_t>(CONVERSION_TIME_MS) * 1000) {
                return false;
            }
            if (!read_adc(&d2_raw_)) {
                if (consecutive_failures < UINT16_MAX) { consecutive_failures++; }
                state_ = State::Idle;
                return false;
            }
            calculate();
            consecutive_failures = 0;
            state_ = State::Idle;
            return true;
    }

    return false;
}

float Depth_Wrapper::pressure_mbar() const {
    return ms5837_pressure_mbar(pressure_);
}

float Depth_Wrapper::temperature_c() const {
    return ms5837_temperature_c(temperature_);
}

float Depth_Wrapper::depth_m() const {
    return ms5837_depth_m(pressure_mbar(), fluid_density_);
}

void Depth_Wrapper::set_fluid_density(float kg_m3) {
    fluid_density_ = kg_m3;
}

bool Depth_Wrapper::read_prom() {
    for (uint8_t i = 0; i < 7; i++) {
        uint8_t cmd = CMD_PROM_BASE + i * 2;
        uint8_t buf[2] = {};
        if (!bus_->write_read_blocking(MS5837_ADDR, &cmd, 1, buf, 2)) {
            return false;
        }
        prom_[i] = (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
    }
    return true;
}

bool Depth_Wrapper::read_adc(uint32_t *out) {
    uint8_t cmd = CMD_ADC_READ;
    uint8_t buf[3] = {};
    if (!bus_->write_read_blocking(MS5837_ADDR, &cmd, 1, buf, 3)) {
        return false;
    }
    *out = (static_cast<uint32_t>(buf[0]) << 16) |
           (static_cast<uint32_t>(buf[1]) << 8) |
            static_cast<uint32_t>(buf[2]);
    return true;
}

bool Depth_Wrapper::start_conversion(uint8_t cmd) {
    return bus_->send_command(MS5837_ADDR, cmd);
}

void Depth_Wrapper::calculate() {
    ms5837_compensate(prom_, d1_raw_, d2_raw_, &temperature_, &pressure_);
}
