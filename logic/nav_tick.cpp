/*
 * Navigation Tick Logic - SRAM-resident implementation
 * Uses __not_in_flash_func() for deterministic Core 1 timing
 */

#include "logic/nav_tick.hpp"
#include "logic/nav_math.hpp"
#include "config.h"
#include <pico.h>
#include <cstdint>
#include <cmath>

__not_in_flash_func(uint8_t) nav_tick_update(NavTickState &state,
                                               const SensorSnapshot &snap,
                                               nav_state_compact_t *out) {
    uint8_t status_flags = 0;
    float encoder_delta;
    float delta_dist;
    Quat quat;

    constexpr float dt_s = NAV_TICK_PERIOD_MS * 0.001f;

    /* Encoder tiered recovery */
    if (snap.encoder_valid) {
        encoder_delta = snap.encoder_delta;
        state.last_good_encoder_delta = encoder_delta;
        delta_dist = encoder_delta * ENCODER_WHEEL_RADIUS_M;
        state.last_velocity = delta_dist / dt_s;
        state.encoder_fail_streak = 0;
    } else {
        if (state.encoder_fail_streak < UINT16_MAX) {
            state.encoder_fail_streak++;
        }

        if (state.encoder_fail_streak < NAV_ENCODER_FAIL_THRESHOLD) {
            /* Tier 1: Short-term extrapolation at constant velocity */
            delta_dist = state.last_velocity * dt_s;
        } else if (fabsf(state.last_velocity) >= NAV_ENCODER_VELOCITY_EPSILON) {
            /* Tier 2: Decay velocity by 20% per tick */
            state.last_velocity *= NAV_ENCODER_DECAY_FACTOR;
            delta_dist = state.last_velocity * dt_s;
        } else {
            /* Tier 3: Hard stop — velocity exhausted */
            state.last_velocity = 0.0f;
            delta_dist = 0.0f;
        }

        encoder_delta = delta_dist / ENCODER_WHEEL_RADIUS_M;
    }

    /* IMU tiered recovery */
    if (snap.imu_valid) {
        quat = snap.imu_quaternion;
        state.last_good_quat = quat;
        state.last_angular_velocity = snap.imu_angular_velocity;
        state.imu_fail_streak = 0;
        state.imu_reset_requested = false;
        state.action_flags &= ~NAV_ACTION_IMU_RESET;
    } else {
        if (state.imu_fail_streak < UINT16_MAX) {
            state.imu_fail_streak++;
        }

        if (state.imu_fail_streak < NAV_IMU_TIER1_THRESHOLD) {
            /* Tier 1: Extrapolate orientation via angular velocity */
            quaternion_extrapolate(&state.last_good_quat,
                                   &state.last_angular_velocity,
                                   dt_s, &quat);
            state.last_good_quat = quat;
        } else if (state.imu_fail_streak < NAV_IMU_TIER3_THRESHOLD) {
            /* Tier 2: Hold last orientation */
            quat = state.last_good_quat;
        } else {
            /* Tier 3: Hold last orientation, request hardware reset once */
            quat = state.last_good_quat;
            if (!state.imu_reset_requested) {
                state.action_flags |= NAV_ACTION_IMU_RESET;
                state.imu_reset_requested = true;
            }
        }
    }

    /* IMU distance throttle (applied after encoder recovery) */
    if (state.imu_fail_streak >= NAV_IMU_TIER3_THRESHOLD) {
        delta_dist = 0.0f;
    } else if (state.imu_fail_streak >= NAV_IMU_TIER1_THRESHOLD) {
        delta_dist *= NAV_IMU_DIST_THROTTLE;
    }

    /* Build status flags */
    if (state.encoder_fail_streak > 0) {
        status_flags |= NAV_FLAG_ENCODER_ESTIMATED;

        if (state.encoder_fail_streak >= NAV_ENCODER_FAIL_THRESHOLD &&
            fabsf(state.last_velocity) < NAV_ENCODER_VELOCITY_EPSILON) {
            status_flags |= NAV_FLAG_ENCODER_LOST;
        }
    }
    if (state.imu_fail_streak > 0) {
        status_flags |= NAV_FLAG_IMU_ESTIMATED;
    }
    if (state.imu_fail_streak >= NAV_IMU_TIER3_THRESHOLD) {
        status_flags |= NAV_FLAG_IMU_LOST;
    }
    if (state.encoder_fail_streak >= NAV_CRITICAL_FAIL_THRESHOLD ||
        state.imu_fail_streak >= NAV_IMU_TIER3_THRESHOLD) {
        status_flags |= NAV_FLAG_NAV_CRITICAL;
    }

    /* Accumulate total path distance */
    state.total_distance += static_cast<double>(fabsf(delta_dist));

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
