/*
 * Navigation Math Pipeline - SRAM-resident implementation
 * All functions use __not_in_flash_func() for deterministic Core 1 timing
 */

#include "logic/nav_math.hpp"
#include <pico.h>
#include <cmath>

/*============================================================================
 * Quaternion to Rotation Matrix
 *============================================================================*/
__not_in_flash_func(void) quaternion_to_rotation_matrix(Quat q, double R[3][3]) {
    double w = static_cast<double>(q.w);
    double x = static_cast<double>(q.x);
    double y = static_cast<double>(q.y);
    double z = static_cast<double>(q.z);

    double norm_sq = w * w + x * x + y * y + z * z;

    /* Normalize with threshold guard to avoid division by near-zero */
    if (norm_sq < 1e-12) {
        /* Degenerate quaternion - set identity matrix */
        R[0][0] = 1.0; R[0][1] = 0.0; R[0][2] = 0.0;
        R[1][0] = 0.0; R[1][1] = 1.0; R[1][2] = 0.0;
        R[2][0] = 0.0; R[2][1] = 0.0; R[2][2] = 1.0;
        return;
    }

    double inv_n = 1.0 / sqrt(norm_sq);
    w *= inv_n;
    x *= inv_n;
    y *= inv_n;
    z *= inv_n;

    /* Pre-compute products for Shoemake 1985 algebraic form */
    double xx = x * x;
    double yy = y * y;
    double zz = z * z;
    double xy = x * y;
    double xz = x * z;
    double yz = y * z;
    double wx = w * x;
    double wy = w * y;
    double wz = w * z;

    /* Standard rotation matrix (Shoemake 1985) */
    R[0][0] = 1.0 - 2.0 * (yy + zz);
    R[0][1] = 2.0 * (xy - wz);
    R[0][2] = 2.0 * (xz + wy);

    R[1][0] = 2.0 * (xy + wz);
    R[1][1] = 1.0 - 2.0 * (xx + zz);
    R[1][2] = 2.0 * (yz - wx);

    R[2][0] = 2.0 * (xz - wy);
    R[2][1] = 2.0 * (yz + wx);
    R[2][2] = 1.0 - 2.0 * (xx + yy);
}

/*============================================================================
 * Compute Displacement
 *============================================================================*/
__not_in_flash_func(void) compute_displacement(const double R[3][3],
                                                 float delta_dist,
                                                 double *dx, double *dy,
                                                 double *dz) {
    double dist = static_cast<double>(delta_dist);

    /* Body-frame forward = X axis -> displacement = column 0 of R scaled by distance */
    *dx = R[0][0] * dist;
    *dy = R[1][0] * dist;
    *dz = R[2][0] * dist;
}

/*============================================================================
 * Integrate Position
 *============================================================================*/
__not_in_flash_func(void) integrate_position(double *pos_x, double *pos_y,
                                               double *pos_z, double dx,
                                               double dy, double dz) {
    *pos_x += dx;
    *pos_y += dy;
    *pos_z += dz;
}

/*============================================================================
 * Extract Heading and Pitch
 *============================================================================*/
__not_in_flash_func(void) extract_heading_pitch(const double R[3][3],
                                                  double *heading_rad,
                                                  double *pitch_rad) {
    /* Gimbal-lock-safe extraction using atan2 */
    *heading_rad = atan2(R[1][0], R[0][0]);
    *pitch_rad = atan2(-R[2][0], sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0]));
}
