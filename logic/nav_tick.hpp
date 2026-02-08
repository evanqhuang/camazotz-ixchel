/*
 * Navigation Tick Logic - Per-tick computation for Core 1 100Hz loop
 * Sensor fusion, fallback state management, and status flag generation
 */

#ifndef NAV_TICK_HPP
#define NAV_TICK_HPP

#include "types.h"
#include <cstdint>

/*============================================================================
 * Sensor Snapshot
 *============================================================================
 * Per-tick sensor readings captured by Core 1 before nav_tick_update().
 * Validity flags indicate successful I2C reads.
 */
struct SensorSnapshot {
    float encoder_delta;       /* From Encoder_Wrapper::get_angle_delta() (radians) */
    bool encoder_valid;        /* false if I2C read failed */
    Quat imu_quaternion;       /* From IMU_Wrapper::get_quaternion() */
    Vec3 imu_angular_velocity; /* From IMU_Wrapper::get_angular_velocity() */
    bool imu_valid;            /* false if drain_events() failed */
};

/*============================================================================
 * Navigation Tick State
 *============================================================================
 * Persistent state for Core 1 navigation loop (100Hz update rate).
 * Position uses double precision to prevent catastrophic cancellation.
 * Failure streaks track consecutive sensor failures for degradation logic.
 */
struct NavTickState {
    double pos_x = 0.0, pos_y = 0.0, pos_z = 0.0;
    Quat last_good_quat = {1.0f, 0.0f, 0.0f, 0.0f};
    float last_good_encoder_delta = 0.0f;
    uint16_t encoder_fail_streak = 0;
    uint16_t imu_fail_streak = 0;
    uint32_t tick_count = 0;
    float last_velocity = 0.0f;    // m/s from last successful encoder read
    double total_distance = 0.0;   // cumulative path length (meters)
};

/*============================================================================
 * Navigation Tick Update
 *============================================================================
 * Core navigation computation for single 100Hz tick:
 * - Sensor fallback: Use last good values for failed sensors
 * - Status flag generation: ESTIMATED/NAV_CRITICAL based on failure streaks
 * - Dead reckoning: quaternion->rotation->displacement->position integration
 *
 * Returns: status_flags bitfield (NAV_FLAG_*)
 */
uint8_t nav_tick_update(NavTickState &state, const SensorSnapshot &snap,
                        nav_state_compact_t *out);

#endif // NAV_TICK_HPP
