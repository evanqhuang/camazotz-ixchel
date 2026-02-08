/*
 * Depth Sensor Recovery Logic - Tiered fallback implementation
 * Runs on Core 0 (~10ms rate), no SRAM placement needed
 */

#include "logic/depth_recovery.hpp"
#include "config.h"
#include <cmath>

uint8_t depth_recovery_update(DepthRecoveryState &state,
                               const DepthSnapshot &snap,
                               double *z_out) {
    uint8_t flags = 0;

    /* ================================================================
     * SLEW ACTIVE: recovery convergence in progress
     * ================================================================ */
    if (state.slew_active) {
        if (snap.new_reading) {
            /* Update slew target with latest sensor reading */
            state.slew_target_z = static_cast<double>(snap.depth_m);
        }

        /* Continue accumulating geometric Z during slew (for fallback if sensor fails again) */
        state.virtual_z += static_cast<double>(snap.delta_dist) *
                           sin(static_cast<double>(snap.pitch_rad));

        state.slew_elapsed_ticks++;

        if (state.slew_elapsed_ticks >= DEPTH_SLEW_DURATION_TICKS) {
            /* Slew complete: snap to sensor, transition to NORMAL */
            state.slew_active = false;
            state.ceiling_locked = false;
            state.ticks_since_valid = 0;
            state.last_sensor_z = state.slew_target_z;
            state.virtual_z = state.slew_target_z;
            *z_out = state.slew_target_z;
            return 0; /* All flags cleared */
        }

        /* Linear interpolation: alpha in [0, 1] */
        double alpha = static_cast<double>(state.slew_elapsed_ticks) /
                       static_cast<double>(DEPTH_SLEW_DURATION_TICKS);
        *z_out = state.slew_start_z +
                 alpha * (state.slew_target_z - state.slew_start_z);
        flags |= NAV_FLAG_DEPTH_VIRTUAL;
        return flags;
    }

    /* ================================================================
     * SENSOR READING AVAILABLE: handle recovery
     * ================================================================ */
    if (snap.new_reading) {
        double sensor_z = static_cast<double>(snap.depth_m);

        if (state.ticks_since_valid < DEPTH_TIER2_THRESHOLD_TICKS) {
            /* Recovery from NORMAL or TIER 1: snap directly */
            state.ticks_since_valid = 0;
            state.last_sensor_z = sensor_z;
            state.virtual_z = sensor_z;
            state.ceiling_locked = false;
            *z_out = sensor_z;
            return 0;
        }

        /* Recovery from TIER 2 or TIER 3: initiate slew */
        state.slew_active = true;
        state.slew_start_z = state.ceiling_locked ? state.locked_z : state.virtual_z;
        state.slew_target_z = sensor_z;
        state.slew_elapsed_ticks = 0;

        /* First slew tick: output is still the start value */
        *z_out = state.slew_start_z;
        flags |= NAV_FLAG_DEPTH_VIRTUAL;
        return flags;
    }

    /* ================================================================
     * NO SENSOR READING: advance failure counter and apply tier logic
     * ================================================================ */
    if (state.ticks_since_valid < UINT32_MAX) {
        state.ticks_since_valid++;
    }

    /* NORMAL: within grace period */
    if (state.ticks_since_valid < DEPTH_FAIL_GRACE_TICKS) {
        *z_out = state.last_sensor_z;
        return 0;
    }

    /* TIER 1: geometric bridge (GRACE <= ticks < TIER2) */
    if (state.ticks_since_valid < DEPTH_TIER2_THRESHOLD_TICKS) {
        state.virtual_z += static_cast<double>(snap.delta_dist) *
                           sin(static_cast<double>(snap.pitch_rad));
        *z_out = state.virtual_z;
        flags |= NAV_FLAG_DEPTH_VIRTUAL;
        return flags;
    }

    /* TIER 2: continue geometric (TIER2 <= ticks < TIER3) */
    if (state.ticks_since_valid < DEPTH_TIER3_THRESHOLD_TICKS) {
        state.virtual_z += static_cast<double>(snap.delta_dist) *
                           sin(static_cast<double>(snap.pitch_rad));
        *z_out = state.virtual_z;
        flags |= NAV_FLAG_DEPTH_VIRTUAL;
        return flags;
    }

    /* TIER 3: ceiling lock */
    if (!state.ceiling_locked) {
        state.ceiling_locked = true;
        state.locked_z = state.virtual_z;
    }
    *z_out = state.locked_z;
    flags |= NAV_FLAG_DEPTH_VIRTUAL | NAV_FLAG_DEPTH_UNVERIFIED | NAV_FLAG_NAV_CRITICAL;
    return flags;
}
