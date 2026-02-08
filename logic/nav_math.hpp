/*
 * Navigation Math Pipeline - SRAM-resident functions for Core 1
 * Quaternion-based dead reckoning with double-precision position accumulation
 */

#ifndef NAV_MATH_HPP
#define NAV_MATH_HPP

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Quaternion to Rotation Matrix
 *============================================================================
 * Converts unit quaternion to 3x3 rotation matrix using Shoemake 1985
 * algebraic form. Normalizes quaternion internally with threshold guard.
 * Degenerate input (zero-length) yields identity matrix.
 *
 * SRAM placement ensures deterministic timing for Core 1 100Hz navigation loop.
 */
void quaternion_to_rotation_matrix(Quat q, double R[3][3]);

/*============================================================================
 * Compute Displacement
 *============================================================================
 * Transforms encoder angular delta into world-frame displacement vector.
 * Body-frame convention: forward = X axis (column 0 of rotation matrix).
 *
 * Precondition: delta_dist = angular_delta * ENCODER_WHEEL_RADIUS_M
 * Caller converts encoder ticks to meters before passing to this function.
 */
void compute_displacement(const double R[3][3], float delta_dist,
                          double *dx, double *dy, double *dz);

/*============================================================================
 * Integrate Position
 *============================================================================
 * Accumulates displacement into double-precision position vector.
 * Single-precision catastrophic cancellation occurs at ~16m displacement
 * with millimeter deltas. Double maintains sub-mm accuracy over kilometers.
 */
void integrate_position(double *pos_x, double *pos_y, double *pos_z,
                        double dx, double dy, double dz);

/*============================================================================
 * Extract Heading and Pitch
 *============================================================================
 * Extracts yaw (heading) and pitch from rotation matrix using gimbal-lock-safe
 * atan2 formulation. Roll is discarded (not needed for ground navigation).
 *
 * Output range: heading ∈ [-π, π], pitch ∈ [-π/2, π/2]
 */
void extract_heading_pitch(const double R[3][3], double *heading_rad,
                           double *pitch_rad);

#ifdef __cplusplus
}
#endif

#endif // NAV_MATH_HPP
