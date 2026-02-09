/*
 * Battery Voltage Math - Pure Algorithms (no SDK dependencies)
 * ADC conversion, EMA smoothing, piecewise LiPo percentage lookup
 */

#ifndef BATTERY_MATH_HPP
#define BATTERY_MATH_HPP

#include <cstdint>

/* Battery state for EMA filtering */
struct BatteryState {
    float filtered_voltage;  /* EMA-smoothed voltage */
    bool initialized;        /* True after first sample */
    bool on_battery;         /* True if running on battery (not USB) */
};

/*
 * Convert 12-bit ADC raw value to battery voltage.
 * Accounts for voltage divider (VCC / 3 via 200K/100K).
 * Formula: voltage = raw * vref / resolution * divider
 */
float battery_raw_to_voltage(uint16_t raw, float vref, uint32_t resolution, float divider);

/*
 * Update EMA filter with new voltage reading.
 * Sets on_battery = false if voltage > vbus_threshold (USB power detected).
 * First sample seeds the filter directly (no smoothing).
 */
void battery_ema_update(BatteryState &state, float voltage, float alpha, float vbus_threshold);

/*
 * Convert battery voltage to percentage using piecewise LiPo curve.
 * Returns 0-100, clamped at boundaries.
 *
 * Lookup table (typical Li-ion discharge curve):
 *   3.00V → 0%
 *   3.30V → 10%
 *   3.60V → 40%
 *   3.80V → 70%
 *   4.00V → 90%
 *   4.20V → 100%
 */
uint8_t battery_voltage_to_percent(float voltage);

#endif // BATTERY_MATH_HPP
