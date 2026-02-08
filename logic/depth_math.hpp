/*
 * MS5837-30BA Depth Sensor Math - Pure Algorithms (no SDK dependencies)
 * CRC4, temperature/pressure compensation, unit conversions
 */

#ifndef DEPTH_MATH_HPP
#define DEPTH_MATH_HPP

#include <cstdint>

/*
 * Compute CRC4 over MS5837 PROM contents per AN520.
 * prom[0..6] — prom[0] upper nibble contains the stored CRC (masked internally).
 * Virtual prom[7] = 0 is handled internally.
 * Does NOT modify the input array.
 */
uint8_t ms5837_crc4(const uint16_t prom[7]);

/*
 * First- and second-order temperature compensation for MS5837-30BA.
 * Inputs: prom[0..6] calibration coefficients, D1 (pressure ADC), D2 (temperature ADC).
 * Outputs: temperature in 1/100 °C, pressure in 1/10 mbar.
 */
void ms5837_compensate(const uint16_t prom[7], uint32_t d1_raw, uint32_t d2_raw,
                       int32_t *temperature, int32_t *pressure);

/* Convert raw pressure (1/10 mbar) to mbar. */
float ms5837_pressure_mbar(int32_t pressure_raw);

/* Convert raw temperature (1/100 °C) to °C. */
float ms5837_temperature_c(int32_t temperature_raw);

/*
 * Compute depth in meters from pressure and fluid density.
 * Subtracts standard atmospheric pressure (1013.25 mbar) and divides by rho*g.
 */
float ms5837_depth_m(float pressure_mbar, float fluid_density);

#endif // DEPTH_MATH_HPP
