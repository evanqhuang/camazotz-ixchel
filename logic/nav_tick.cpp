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

/* Helper: Compute magnitude of a Vec3 */
static inline float vec3_magnitude(const Vec3 &v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

/* Helper: Get forward (body-frame X) component of acceleration.
 * BNO08x linear acceleration is in sensor frame, so we just return x component.
 * Negative value indicates deceleration (opposing forward motion). */
static inline float get_forward_accel(const Vec3 &accel) {
    return accel.x;
}

__not_in_flash_func(uint8_t) nav_tick_update(NavTickState &state,
                                               const SensorSnapshot &snap,
                                               nav_state_compact_t *out) {
    uint8_t status_flags = 0;
    float encoder_delta;
    float delta_dist;
    Quat quat;

    constexpr float dt_s = NAV_TICK_PERIOD_MS * 0.001f;

    /* === Case B & C Detection: Determine if encoder is effectively unavailable === */
    bool encoder_unavailable = !snap.encoder_valid;

    /* Case C: Encoder valid but reads zero while IMU shows LINEAR motion (stuck wheel)
     * Note: Pure rotation (spinning in place) is NOT a stuck wheel - only check accel.
     * Angular velocity is NOT checked here because encoder=0 + omega>0 is valid (rotation). */
    if (snap.encoder_valid && fabsf(snap.encoder_delta) < CONFLICT_ENCODER_ZERO_THRESHOLD) {
        if (snap.imu_valid) {
            float accel_mag = vec3_magnitude(snap.imu_linear_accel);
            if (accel_mag > CONFLICT_IMU_ACCEL_THRESHOLD) {
                if (state.encoder_zero_imu_motion_streak < UINT16_MAX) {
                    state.encoder_zero_imu_motion_streak++;
                }
                if (state.encoder_zero_imu_motion_streak >= CONFLICT_ZERO_ENCODER_TICKS) {
                    encoder_unavailable = true;  /* Treat as encoder failure (stuck wheel) */
                }
            } else {
                state.encoder_zero_imu_motion_streak = 0;
            }
        } else {
            state.encoder_zero_imu_motion_streak = 0;
        }
    } else if (snap.encoder_valid) {
        /* Encoder is valid and non-zero: reset stuck wheel detection */
        state.encoder_zero_imu_motion_streak = 0;
    }

    /* === Encoder tiered recovery (modified for IMU-informed recovery) === */
    if (snap.encoder_valid && !encoder_unavailable) {
        encoder_delta = snap.encoder_delta;
        state.last_good_encoder_delta = encoder_delta;
        delta_dist = encoder_delta * ENCODER_WHEEL_RADIUS_M;
        /* Only update velocity when encoder shows meaningful motion.
         * This preserves last_velocity during Case C detection buildup
         * (when encoder reads zero due to stuck wheel, not actual stop). */
        if (fabsf(snap.encoder_delta) >= CONFLICT_ENCODER_ZERO_THRESHOLD) {
            state.last_velocity = delta_dist / dt_s;
        }
        state.encoder_fail_streak = 0;
    } else {
        /* Encoder is unavailable (I2C fail or stuck wheel detected) */
        if (state.encoder_fail_streak < UINT16_MAX) {
            state.encoder_fail_streak++;
        }

        /* === Case B: IMU-informed encoder recovery === */
        if (snap.imu_valid) {
            float omega_mag = vec3_magnitude(snap.imu_angular_velocity);
            float accel_mag = vec3_magnitude(snap.imu_linear_accel);
            bool imu_shows_motion = (omega_mag > CONFLICT_IMU_OMEGA_THRESHOLD) ||
                                    (accel_mag > CONFLICT_IMU_ACCEL_THRESHOLD);
            float forward_accel = get_forward_accel(snap.imu_linear_accel);

            if (!imu_shows_motion) {
                /* IMU confirms device stopped -> immediate hard stop */
                state.last_velocity = 0.0f;
                delta_dist = 0.0f;
            } else if (forward_accel < -CONFLICT_DECEL_THRESHOLD) {
                /* IMU shows deceleration -> faster decay (60%/tick instead of 80%/tick) */
                state.last_velocity *= CONFLICT_ACCEL_DECAY_FACTOR;
                delta_dist = state.last_velocity * dt_s;
            } else {
                /* IMU shows motion continues -> extrapolate at last velocity (Tier 1) */
                delta_dist = state.last_velocity * dt_s;
            }
        } else {
            /* Fallback: IMU also failed, use existing blind recovery */
            if (state.encoder_fail_streak < NAV_ENCODER_FAIL_THRESHOLD) {
                /* Tier 1: Short-term extrapolation at constant velocity */
                delta_dist = state.last_velocity * dt_s;
            } else if (fabsf(state.last_velocity) >= NAV_ENCODER_VELOCITY_EPSILON) {
                /* Tier 2: Decay velocity by 20% per tick */
                state.last_velocity *= NAV_ENCODER_DECAY_FACTOR;
                delta_dist = state.last_velocity * dt_s;
            } else {
                /* Tier 3: Hard stop - velocity exhausted */
                state.last_velocity = 0.0f;
                delta_dist = 0.0f;
            }
        }

        encoder_delta = delta_dist / ENCODER_WHEEL_RADIUS_M;
    }

    /* Save pre-IMU-throttle delta_dist for depth recovery on Core 0 */
    float delta_dist_physical = delta_dist;

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

    /* === Case A: Encoder moving, IMU stationary (Wheel Slip Detection) === */
    if (snap.encoder_valid && snap.imu_valid) {
        /* Accumulate over window */
        state.accumulated_encoder_delta += fabsf(encoder_delta);
        state.accumulated_omega_magnitude += vec3_magnitude(snap.imu_angular_velocity);
        state.accumulated_accel_magnitude += vec3_magnitude(snap.imu_linear_accel);
        state.conflict_window_count++;

        if (state.conflict_window_count >= CONFLICT_WINDOW_TICKS) {
            /* Evaluate conflict at end of window */
            bool encoder_moving = state.accumulated_encoder_delta >
                                  CONFLICT_ENCODER_MOTION_THRESHOLD * CONFLICT_WINDOW_TICKS;
            bool imu_moving = (state.accumulated_omega_magnitude >
                               CONFLICT_IMU_OMEGA_THRESHOLD * CONFLICT_WINDOW_TICKS) ||
                              (state.accumulated_accel_magnitude >
                               CONFLICT_IMU_ACCEL_THRESHOLD * CONFLICT_WINDOW_TICKS);

            if (encoder_moving && !imu_moving) {
                /* Encoder shows motion, IMU stationary -> possible wheel slip */
                if (state.conflict_streak < UINT16_MAX) {
                    state.conflict_streak++;
                }
            } else {
                /* No conflict detected -> reset streak */
                state.conflict_streak = 0;
            }

            /* Reset window accumulators */
            state.accumulated_encoder_delta = 0.0f;
            state.accumulated_omega_magnitude = 0.0f;
            state.accumulated_accel_magnitude = 0.0f;
            state.conflict_window_count = 0;
        }
    } else {
        /* Can't cross-validate with missing sensor data -> reset window */
        state.accumulated_encoder_delta = 0.0f;
        state.accumulated_omega_magnitude = 0.0f;
        state.accumulated_accel_magnitude = 0.0f;
        state.conflict_window_count = 0;
        /* Note: We do NOT reset conflict_streak here to maintain detection state */
    }

    /* Apply tiered attenuation for wheel slip (Case A) */
    if (state.conflict_streak >= CONFLICT_TIER3_THRESHOLD) {
        delta_dist = 0.0f;
    } else if (state.conflict_streak >= CONFLICT_TIER2_THRESHOLD) {
        delta_dist *= CONFLICT_DIST_ATTENUATION;
    }

    /* IMU distance throttle (applied after encoder recovery and conflict attenuation) */
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

    /* Sensor conflict flag (set at Tier 1+ for wheel slip, or during stuck wheel detection) */
    if (state.conflict_streak >= CONFLICT_TIER1_THRESHOLD ||
        state.encoder_zero_imu_motion_streak >= CONFLICT_ZERO_ENCODER_TICKS) {
        status_flags |= NAV_FLAG_SENSOR_CONFLICT;
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
    out->delta_dist = delta_dist_physical;
    out->status_flags = status_flags;

    /* Increment tick counter */
    state.tick_count++;

    return status_flags;
}
