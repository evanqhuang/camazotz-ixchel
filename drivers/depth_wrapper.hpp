/*
 * Depth_Wrapper - MS5837-30BA Depth/Pressure Sensor Driver
 * Non-blocking state machine over PIO I2C for RP2350 firmware
 */

#ifndef DEPTH_WRAPPER_HPP
#define DEPTH_WRAPPER_HPP

#include "drivers/pio_i2c.hpp"
#include "logic/depth_math.hpp"

#include "pico/stdlib.h"

#include <cstdint>

class Depth_Wrapper {
public:
    bool begin(PIO_I2C &bus);
    bool update();
    float pressure_mbar() const;
    float temperature_c() const;
    float depth_m() const;
    void set_fluid_density(float kg_m3);

    uint16_t consecutive_failures = 0;

private:
    static constexpr uint8_t MS5837_ADDR = 0x76;
    static constexpr uint8_t CMD_RESET = 0x1E;
    static constexpr uint8_t CMD_ADC_READ = 0x00;
    static constexpr uint8_t CMD_PROM_BASE = 0xA0;
    static constexpr uint8_t CMD_CONVERT_D1 = 0x4A;  // OSR=8192, pressure
    static constexpr uint8_t CMD_CONVERT_D2 = 0x5A;  // OSR=8192, temperature
    static constexpr uint32_t CONVERSION_TIME_MS = 20;

    enum class State : uint8_t { Idle, WaitingD1, WaitingD2 };

    PIO_I2C *bus_ = nullptr;
    State state_ = State::Idle;
    absolute_time_t conversion_start_ = {0};

    uint16_t prom_[7] = {};
    uint32_t d1_raw_ = 0;
    uint32_t d2_raw_ = 0;

    int32_t temperature_ = 0;   // 1/100 °C
    int32_t pressure_ = 0;      // 1/10 mbar (30BA) or 1/100 mbar (02BA)
    float fluid_density_ = 1029.0f;  // kg/m³ seawater default

    bool read_prom();
    bool read_adc(uint32_t *out);
    bool start_conversion(uint8_t cmd);
    void calculate();
};

#endif // DEPTH_WRAPPER_HPP
