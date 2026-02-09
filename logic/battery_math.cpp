/*
 * Battery Voltage Math Implementation
 */

#include "logic/battery_math.hpp"

float battery_raw_to_voltage(uint16_t raw, float vref, uint32_t resolution, float divider) {
    if (resolution == 0) {
        return 0.0f;
    }
    return (static_cast<float>(raw) * vref / static_cast<float>(resolution)) * divider;
}

void battery_ema_update(BatteryState &state, float voltage, float alpha, float vbus_threshold) {
    /* Detect USB power (no battery reading meaningful) */
    state.on_battery = (voltage <= vbus_threshold);

    if (!state.initialized) {
        /* Seed filter with first sample */
        state.filtered_voltage = voltage;
        state.initialized = true;
    } else {
        /* Exponential moving average */
        state.filtered_voltage = alpha * voltage + (1.0f - alpha) * state.filtered_voltage;
    }
}

uint8_t battery_voltage_to_percent(float voltage) {
    /* Piecewise linear interpolation of typical LiPo discharge curve */
    static constexpr struct {
        float voltage;
        float percent;
    } lut[] = {
        {3.00f,   0.0f},
        {3.30f,  10.0f},
        {3.60f,  40.0f},
        {3.80f,  70.0f},
        {4.00f,  90.0f},
        {4.20f, 100.0f},
    };
    static constexpr uint8_t LUT_SIZE = sizeof(lut) / sizeof(lut[0]);

    /* Clamp below minimum */
    if (voltage <= lut[0].voltage) {
        return 0;
    }
    /* Clamp above maximum */
    if (voltage >= lut[LUT_SIZE - 1].voltage) {
        return 100;
    }

    /* Find segment and interpolate */
    for (uint8_t i = 1; i < LUT_SIZE; i++) {
        if (voltage <= lut[i].voltage) {
            float v0 = lut[i - 1].voltage;
            float v1 = lut[i].voltage;
            float p0 = lut[i - 1].percent;
            float p1 = lut[i].percent;
            float t = (voltage - v0) / (v1 - v0);
            float pct = p0 + t * (p1 - p0);
            return static_cast<uint8_t>(pct + 0.5f);  /* Round */
        }
    }

    return 100;  /* Fallback */
}
