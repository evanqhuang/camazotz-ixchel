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
    Vec3 imu_linear_accel;     /* Gravity-compensated acceleration [m/s²] */
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
    Vec3 last_angular_velocity = {0.0f, 0.0f, 0.0f};
    float last_good_encoder_delta = 0.0f;
    uint16_t encoder_fail_streak = 0;
    uint16_t imu_fail_streak = 0;
    uint32_t tick_count = 0;
    float last_velocity = 0.0f;    // m/s from last successful encoder read
    double total_distance = 0.0;   // cumulative path length (meters)
    uint8_t action_flags = 0;      // consumed by core1_nav.cpp (NAV_ACTION_*)
    bool imu_reset_requested = false; // prevents re-requesting reset each tick

    // Conflict detection state (Case A: wheel slip)
    uint16_t conflict_streak = 0;
    float accumulated_encoder_delta = 0.0f;
    float accumulated_omega_magnitude = 0.0f;
    float accumulated_accel_magnitude = 0.0f;
    uint8_t conflict_window_count = 0;

    // Stuck wheel detection state (Case C)
    uint16_t encoder_zero_imu_motion_streak = 0;
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
