/*
 * MS5837-30BA Depth Sensor Math Implementation
 * Compensation formulas ported from BlueRobotics MS5837 library (MIT)
 */

#include "logic/depth_math.hpp"

uint8_t ms5837_crc4(const uint16_t prom[7]) {
    // CRC4 per MS5837 datasheet AN520 / BlueRobotics reference
    // Uses local copy of PROM[0] to avoid modifying the input array
    uint16_t n_rem = 0;
    uint16_t prom0_masked = prom[0] & 0x0FFF;  // Strip CRC nibble

    for (uint8_t i = 0; i < 16; i++) {
        uint16_t word;
        if (i == 0 || i == 1) {
            word = prom0_masked;  // Use masked version for PROM[0]
        } else if (i < 14) {
            word = prom[i >> 1];
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

void ms5837_compensate(const uint16_t prom[7], uint32_t d1_raw, uint32_t d2_raw,
                       int32_t *temperature, int32_t *pressure) {
    // MS5837-30BA first-order compensation
    int32_t dT = static_cast<int32_t>(d2_raw) -
                 static_cast<int32_t>(prom[5]) * 256L;

    int64_t SENS = static_cast<int64_t>(prom[1]) * 32768LL +
                   (static_cast<int64_t>(prom[3]) * dT) / 256LL;
    int64_t OFF = static_cast<int64_t>(prom[2]) * 65536LL +
                  (static_cast<int64_t>(prom[4]) * dT) / 128LL;

    int32_t TEMP = 2000L + static_cast<int32_t>(
        static_cast<int64_t>(dT) * prom[6] / 8388608LL);

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

    *temperature = TEMP - Ti;
    *pressure = static_cast<int32_t>(
        (static_cast<int64_t>(d1_raw) * SENS2 / 2097152LL - OFF2) / 8192LL);
}

float ms5837_pressure_mbar(int32_t pressure_raw) {
    return static_cast<float>(pressure_raw) / 10.0f;
}

float ms5837_temperature_c(int32_t temperature_raw) {
    return static_cast<float>(temperature_raw) / 100.0f;
}

float ms5837_depth_m(float pressure_mbar, float fluid_density) {
    float pressure_pa = pressure_mbar * 100.0f;
    return (pressure_pa - 101300.0f) / (fluid_density * 9.80665f);
}
