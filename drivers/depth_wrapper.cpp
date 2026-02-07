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
    uint8_t crc_calc = crc4(prom_);
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
    // MS5837-30BA: P is in 1/10 mbar
    return static_cast<float>(pressure_) / 10.0f;
}

float Depth_Wrapper::temperature_c() const {
    return static_cast<float>(temperature_) / 100.0f;
}

float Depth_Wrapper::depth_m() const {
    // Convert pressure from mbar to Pa, subtract atmospheric, divide by ρg
    float pressure_pa = pressure_mbar() * 100.0f;
    return (pressure_pa - 101300.0f) / (fluid_density_ * 9.80665f);
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

uint8_t Depth_Wrapper::crc4(uint16_t n_prom[]) {
    // CRC4 per MS5837 datasheet AN520 / BlueRobotics reference
    // Uses local copy of PROM[0] to avoid modifying the input array
    uint16_t n_rem = 0;
    uint16_t prom0_masked = n_prom[0] & 0x0FFF;  // Strip CRC nibble

    for (uint8_t i = 0; i < 16; i++) {
        uint16_t word;
        if (i == 0 || i == 1) {
            word = prom0_masked;  // Use masked version for PROM[0]
        } else if (i < 14) {
            word = n_prom[i >> 1];
        } else {
            word = 0;  // Virtual PROM[7] = 0 per MS5837 AN520
        }

        if (i % 2 == 1) {
            n_rem ^= word & 0x00FF;
        } else {
            n_rem ^= word >> 8;
        }
        for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000) {
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem = (n_rem << 1);
            }
        }
    }

    n_rem = (n_rem >> 12) & 0x000F;
    return static_cast<uint8_t>(n_rem);
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
    // MS5837-30BA first-order compensation
    int32_t dT = static_cast<int32_t>(d2_raw_) -
                 static_cast<int32_t>(prom_[5]) * 256L;

    int64_t SENS = static_cast<int64_t>(prom_[1]) * 32768LL +
                   (static_cast<int64_t>(prom_[3]) * dT) / 256LL;
    int64_t OFF = static_cast<int64_t>(prom_[2]) * 65536LL +
                  (static_cast<int64_t>(prom_[4]) * dT) / 128LL;

    int32_t TEMP = 2000L + static_cast<int32_t>(
        static_cast<int64_t>(dT) * prom_[6] / 8388608LL);

    // Second-order temperature compensation (MS5837-30BA)
    int32_t Ti = 0;
    int64_t OFFi = 0;
    int64_t SENSi = 0;

    if (TEMP < 2000) {
        // Low temperature
        Ti = static_cast<int32_t>(
            3LL * static_cast<int64_t>(dT) * static_cast<int64_t>(dT) /
            8589934592LL);
        OFFi = 3LL * static_cast<int64_t>(TEMP - 2000) *
               static_cast<int64_t>(TEMP - 2000) / 2LL;
        SENSi = 5LL * static_cast<int64_t>(TEMP - 2000) *
                static_cast<int64_t>(TEMP - 2000) / 8LL;

        if (TEMP < -1500) {
            // Very low temperature
            OFFi += 7LL * static_cast<int64_t>(TEMP + 1500) *
                    static_cast<int64_t>(TEMP + 1500);
            SENSi += 4LL * static_cast<int64_t>(TEMP + 1500) *
                     static_cast<int64_t>(TEMP + 1500);
        }
    } else {
        // High temperature
        Ti = static_cast<int32_t>(
            2LL * static_cast<int64_t>(dT) * static_cast<int64_t>(dT) /
            137438953472LL);
        OFFi = static_cast<int64_t>(TEMP - 2000) *
               static_cast<int64_t>(TEMP - 2000) / 16LL;
        SENSi = 0;
    }

    int64_t OFF2 = OFF - OFFi;
    int64_t SENS2 = SENS - SENSi;

    temperature_ = TEMP - Ti;
    pressure_ = static_cast<int32_t>(
        (static_cast<int64_t>(d1_raw_) * SENS2 / 2097152LL - OFF2) / 8192LL);
}
