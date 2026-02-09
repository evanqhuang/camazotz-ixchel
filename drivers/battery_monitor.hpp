/*
 * Battery_Monitor - ADC wrapper for LiPo voltage monitoring
 * GP26 (ADC0) via 200K/100K voltage divider
 */

#ifndef BATTERY_MONITOR_HPP
#define BATTERY_MONITOR_HPP

#include "config.h"
#include "logic/battery_math.hpp"
#include <cstdint>

class Battery_Monitor {
public:
    /*
     * Initialize ADC for battery monitoring.
     * Configures GP26/ADC0 as analog input.
     * Returns true on success.
     */
    bool init();

    /*
     * Update battery reading (throttled to BAT_SAMPLE_INTERVAL_MS).
     * Call from main loop with current time in ms.
     * Returns true if a new sample was taken.
     */
    bool update(uint32_t now_ms);

    /* Get smoothed battery voltage (V) */
    float voltage() const { return state_.filtered_voltage; }

    /* Get battery percentage (0-100) */
    uint8_t percent() const;

    /* True if running on battery power (false = USB) */
    bool on_battery() const { return state_.on_battery; }

    /* True if battery is low (below threshold) */
    bool low_battery() const;

private:
    BatteryState state_ = {};
    uint32_t last_sample_ms_ = 0;
    bool initialized_ = false;
};

#endif // BATTERY_MONITOR_HPP
