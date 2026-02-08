/*
 * Depth Sensor Recovery Logic - Tiered fallback for MS5837-30BA outages
 * Pure logic module, no hardware dependencies, testable on host
 */

#ifndef DEPTH_RECOVERY_HPP
#define DEPTH_RECOVERY_HPP

#include "types.h"
#include <cstdint>

/*============================================================================
 * Depth Snapshot
 *============================================================================
 * Per-tick depth sensor state and Core 1 nav data for recovery computation.
 * Assembled by Core 0 main loop each iteration.
 */
struct DepthSnapshot {
    bool new_reading;       /* depth.update() returned true this tick */
    float depth_m;          /* depth.depth_m() (valid only when new_reading==true) */
    float pitch_rad;        /* Current pitch extracted from Core 1 quaternion */
    float delta_dist;       /* Pre-throttle physical distance from Core 1 */
};

/*============================================================================
 * Depth Recovery State
 *============================================================================
 * Persistent state for depth sensor tiered recovery.
 * Managed on Core 0 only -- no inter-core concerns.
 *
 * Tier Thresholds (at 100Hz):
 *   NORMAL:  ticks_since_valid < GRACE (10 ticks / 100ms)
 *   TIER 1:  GRACE <= ticks < TIER2 (10-199 ticks / 0.1-2s) - Geometric bridge
 *   TIER 2:  TIER2 <= ticks < TIER3 (200-999 ticks / 2-10s) - Geometric + slew on recovery
 *   TIER 3:  ticks >= TIER3 (1000+ ticks / 10s+) - Ceiling lock, NAV_CRITICAL
 */
struct DepthRecoveryState {
    uint32_t ticks_since_valid = 0;     /* Ticks since last good depth reading */
    double virtual_z = 0.0;            /* Geometric Z estimate (accumulated) */
    double last_sensor_z = 0.0;        /* Last absolute depth from MS5837 */

    /* Slew state (recovery convergence) */
    bool slew_active = false;
    double slew_start_z = 0.0;         /* Virtual Z when slew began */
    double slew_target_z = 0.0;        /* Absolute Z target from sensor */
    uint32_t slew_elapsed_ticks = 0;   /* Ticks into current slew */

    /* Tier 3 lock */
    bool ceiling_locked = false;
    double locked_z = 0.0;
};

/*============================================================================
 * Depth Recovery Update
 *============================================================================
 * Core depth recovery computation for a single Core 0 tick (~10ms).
 *
 * State machine:
 *   NORMAL    (ticks < GRACE):            Use last sensor Z
 *   TIER 1    (GRACE <= ticks < TIER2):   Geometric bridge (delta_dist * sin(pitch))
 *   TIER 2    (TIER2 <= ticks < TIER3):   Continue geometric, slew on recovery
 *   TIER 3    (ticks >= TIER3):           Lock Z, set NAV_CRITICAL
 *   SLEW      (recovery convergence):     Linear interpolation to sensor Z
 *
 * Returns: depth status flags (NAV_FLAG_DEPTH_VIRTUAL, NAV_FLAG_DEPTH_UNVERIFIED,
 *          NAV_FLAG_NAV_CRITICAL) to be OR'd into nav state.
 * Writes final Z value to *z_out.
 */
uint8_t depth_recovery_update(DepthRecoveryState &state,
                               const DepthSnapshot &snap,
                               double *z_out);

#endif // DEPTH_RECOVERY_HPP
