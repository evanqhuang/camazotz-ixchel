/*
 * Battery_Monitor Implementation
 */

#include "drivers/battery_monitor.hpp"
#include "hardware/adc.h"
#include "hardware/gpio.h"

bool Battery_Monitor::init() {
    if (initialized_) {
        return true;
    }

    /* Initialize ADC hardware (safe to call multiple times) */
    adc_init();

    /* Configure GP26 as ADC input */
    adc_gpio_init(BAT_ADC_PIN);

    initialized_ = true;
    return true;
}

bool Battery_Monitor::update(uint32_t now_ms) {
    if (!initialized_) {
        return false;
    }

    /* Throttle to configured sample rate */
    if ((now_ms - last_sample_ms_) < BAT_SAMPLE_INTERVAL_MS) {
        return false;
    }
    last_sample_ms_ = now_ms;

    /* Select ADC channel and read */
    adc_select_input(BAT_ADC_CHANNEL);
    uint16_t raw = adc_read();

    /* Convert to voltage */
    float voltage = battery_raw_to_voltage(raw, BAT_ADC_VREF, BAT_ADC_RESOLUTION,
                                           BAT_VOLTAGE_DIVIDER);

    /* Update EMA filter */
    battery_ema_update(state_, voltage, BAT_EMA_ALPHA, BAT_VBUS_THRESHOLD_V);

    return true;
}

uint8_t Battery_Monitor::percent() const {
    if (!state_.on_battery) {
        return 100;  /* USB power, not meaningful */
    }
    return battery_voltage_to_percent(state_.filtered_voltage);
}

bool Battery_Monitor::low_battery() const {
    return state_.on_battery && (percent() <= BAT_LOW_THRESHOLD_PCT);
}
