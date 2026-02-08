/*
 * Navigation Tick Logic - SRAM-resident implementation
 * Uses __not_in_flash_func() for deterministic Core 1 timing
 */

#include "logic/nav_tick.hpp"
#include "logic/nav_math.hpp"
#include "config.h"
#include <pico.h>
#include <cstdint>

__not_in_flash_func(uint8_t) nav_tick_update(NavTickState &state,
                                               const SensorSnapshot &snap,
                                               nav_state_compact_t *out) {
    uint8_t status_flags = 0;
    float encoder_delta;
    Quat quat;

    /* Encoder fallback logic */
    if (snap.encoder_valid) {
        encoder_delta = snap.encoder_delta;
        state.last_good_encoder_delta = encoder_delta;
        state.encoder_fail_streak = 0;
    } else {
        encoder_delta = state.last_good_encoder_delta;
        if (state.encoder_fail_streak < UINT16_MAX) {
            state.encoder_fail_streak++;
        }
    }

    /* IMU fallback logic */
    if (snap.imu_valid) {
        quat = snap.imu_quaternion;
        state.last_good_quat = quat;
        state.imu_fail_streak = 0;
    } else {
        quat = state.last_good_quat;
        if (state.imu_fail_streak < UINT16_MAX) {
            state.imu_fail_streak++;
        }
    }

    /* Build status flags */
    if (state.encoder_fail_streak > 0) {
        status_flags |= NAV_FLAG_ENCODER_ESTIMATED;
    }
    if (state.imu_fail_streak > 0) {
        status_flags |= NAV_FLAG_IMU_ESTIMATED;
    }
    if (state.encoder_fail_streak >= NAV_CRITICAL_FAIL_THRESHOLD ||
        state.imu_fail_streak >= NAV_CRITICAL_FAIL_THRESHOLD) {
        status_flags |= NAV_FLAG_NAV_CRITICAL;
    }

    /* Convert encoder delta to distance */
    float delta_dist = encoder_delta * ENCODER_WHEEL_RADIUS_M;

    /* Navigation pipeline: quaternion -> rotation -> displacement -> position */
    double R[3][3];
    quaternion_to_rotation_matrix(quat, R);

    double dx, dy, dz;
    compute_displacement(R, delta_dist, &dx, &dy, &dz);

    integrate_position(&state.pos_x, &state.pos_y, &state.pos_z, dx, dy, dz);

    /* Pack compact output state */
    out->angular_delta = encoder_delta;
    out->quat_w = quat.w;
    out->quat_x = quat.x;
    out->quat_y = quat.y;
    out->quat_z = quat.z;
    out->pos_x = static_cast<float>(state.pos_x);
    out->pos_y = static_cast<float>(state.pos_y);
    out->pos_z = static_cast<float>(state.pos_z);
    out->status_flags = status_flags;

    /* Increment tick counter */
    state.tick_count++;

    return status_flags;
}
