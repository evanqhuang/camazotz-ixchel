/*
 * Core Type Definitions for RP2350 Mapper Firmware
 * Safety-critical embedded system - MISRA-aligned conventions
 */

#ifndef MAPPER_TYPES_H
#define MAPPER_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 * Navigation State Status Flags
 *============================================================================*/

/* Sensor quality indicators - bitfield for uint8_t status_flags */
#define NAV_FLAG_ENCODER_ESTIMATED  (1U << 0U)  /* Encoder reading interpolated from previous */
#define NAV_FLAG_IMU_ESTIMATED      (1U << 1U)  /* IMU data predicted/stale (no fresh sample) */
#define NAV_FLAG_DEPTH_VIRTUAL      (1U << 2U)  /* Depth from terrain model, not sensor */
#define NAV_FLAG_DEPTH_UNVERIFIED   (1U << 3U)  /* Depth sensor uncalibrated or unvalidated */
#define NAV_FLAG_NAV_CRITICAL       (1U << 4U)  /* Navigation solution degraded/unreliable */

/*============================================================================
 * Navigation State Structure
 *============================================================================
 *
 * Design Rationale:
 * -----------------
 * - float for per-tick measurements (angular_delta, quaternion):
 *   Optimized for Cortex-M33 single-precision FPU throughput at 100Hz
 *   navigation loop rate. BNO08x quaternions are transmitted as 16-bit
 *   fixed-point; float provides sufficient precision for sensor fusion.
 *
 * - double for accumulated position (pos_x, pos_y, pos_z):
 *   Critical for drift prevention over extended missions. Single-precision
 *   catastrophic cancellation occurs at ~16m displacement with millimeter
 *   deltas. Double precision maintains sub-millimeter accuracy over kilometers.
 *
 * Memory Layout (56 bytes total):
 * --------------------------------
 * Offset | Size | Field            | Alignment
 * -------|------|------------------|----------
 *   0    |  4   | angular_delta    | 4-byte (float)
 *   4    |  4   | quat_w           | 4-byte (float)
 *   8    |  4   | quat_x           | 4-byte (float)
 *  12    |  4   | quat_y           | 4-byte (float)
 *  16    |  4   | quat_z           | 4-byte (float)
 *  20    |  4   | [padding]        | (compiler-inserted for double alignment)
 *  24    |  8   | pos_x            | 8-byte (double)
 *  32    |  8   | pos_y            | 8-byte (double)
 *  40    |  8   | pos_z            | 8-byte (double)
 *  48    |  1   | status_flags     | 1-byte (uint8_t)
 *  49    |  7   | [padding]        | (struct alignment = 8 bytes due to double)
 *  56    | TOTAL
 *
 * Inter-Core Transfer:
 * --------------------
 * Structure size (56 bytes) exceeds RP2350 multicore FIFO capacity (32 bytes).
 * Recommended approach: Use shared memory buffer with spinlock synchronization
 * instead of multicore_fifo_push_blocking(). Core0 (logging) reads via spinlock
 * while Core1 (navigation) updates at 100Hz.
 *
 * Alignment: Natural alignment preferred (NO #pragma pack) for optimal Cortex-M33
 * FPU load/store performance. Compiler will insert padding automatically.
 */

typedef struct {
    /* Per-tick incremental rotation from encoder (radians) */
    float angular_delta;

    /* Orientation quaternion from BNO08x (world frame) */
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;

    /* Accumulated world-frame position (meters) - double precision for drift prevention */
    double pos_x;
    double pos_y;
    double pos_z;

    /* Sensor quality status bitfield (NAV_FLAG_*) */
    uint8_t status_flags;

} navigation_state_t;

/* Compile-time verification of struct layout */
#ifdef __cplusplus
static_assert(sizeof(navigation_state_t) == 56U,
              "navigation_state_t must be exactly 56 bytes for defined memory layout");
#else
_Static_assert(sizeof(navigation_state_t) == 56U,
               "navigation_state_t must be exactly 56 bytes for defined memory layout");
#endif

/*============================================================================
 * Sensor Data Types
 *============================================================================*/

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quat;

typedef struct {
    float x;
    float y;
    float z;
} Vec3;

/*============================================================================
 * Calibration Types (C++ only)
 *============================================================================*/
#ifdef __cplusplus

static constexpr uint8_t CALIB_MAX_ERROR_LEN = 64;

enum class CalibrationStatus : uint8_t { NotRun, Success, Warning, Failed };

struct SensorCalibResult {
    CalibrationStatus status = CalibrationStatus::NotRun;
    char error_msg[CALIB_MAX_ERROR_LEN] = {};
};

struct CalibrationResult {
    SensorCalibResult encoder;
    SensorCalibResult imu;
    SensorCalibResult depth;

    bool all_passed() const {
        return encoder.status == CalibrationStatus::Success &&
               imu.status == CalibrationStatus::Success &&
               depth.status == CalibrationStatus::Success;
    }

    bool any_fatal() const {
        return encoder.status == CalibrationStatus::Failed ||
               imu.status == CalibrationStatus::Failed ||
               depth.status == CalibrationStatus::Failed;
    }
};

#endif /* __cplusplus */

#endif /* MAPPER_TYPES_H */
