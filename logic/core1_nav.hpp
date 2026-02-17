/*
 * Core 1 Navigation Loop - Hardware Integration Layer
 * 100Hz sensor reading, dead reckoning, and inter-core synchronization
 */

#ifndef CORE1_NAV_HPP
#define CORE1_NAV_HPP

#include "types.h"
#include <cstdint>

class Encoder_Wrapper;
class IMU_Wrapper;

struct Core1Context {
    Encoder_Wrapper *encoder;
    IMU_Wrapper *imu;
};

struct JitterStats {
    uint32_t min_us = UINT32_MAX;
    uint32_t max_us = 0;
    uint32_t last_us = 0;
};

void core1_entry();

nav_state_compact_t core1_get_nav_state();
JitterStats core1_get_jitter_stats();
uint32_t core1_get_dropped_frames();
void core1_request_position_reset();
void core1_request_tare();

#endif // CORE1_NAV_HPP
