/*
 * Core 1 Navigation Loop - Hardware Integration Layer
 * 100Hz sensor reading, dead reckoning, and inter-core synchronization
 */

#include "logic/core1_nav.hpp"
#include "logic/nav_tick.hpp"
#include "drivers/encoder_wrapper.hpp"
#include "drivers/imu_wrapper.hpp"
#include "config.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"

static spin_lock_t *nav_lock = nullptr;
static nav_state_compact_t shared_state = {};
static JitterStats jitter = {};
static uint32_t dropped_frames = 0;
static uint32_t sequence_num = 0;

__not_in_flash_func(void) core1_entry() {
    uint32_t data = multicore_fifo_pop_blocking();
    auto *ctx = reinterpret_cast<Core1Context *>(data);

    auto *encoder = ctx->encoder;
    auto *imu = ctx->imu;

    nav_lock = spin_lock_init(spin_lock_claim_unused(true));

    NavTickState state = {};
    uint16_t prev_enc_failures = 0;
    uint16_t prev_imu_failures = 0;

    while (true) {
        absolute_time_t start = get_absolute_time();
        absolute_time_t deadline = make_timeout_time_ms(NAV_TICK_PERIOD_MS);

        float enc_delta = encoder->get_angle_delta();
        bool enc_ok = (encoder->consecutive_failures == prev_enc_failures);
        prev_enc_failures = encoder->consecutive_failures;

        Quat q = imu->get_quaternion();
        Vec3 w = imu->get_angular_velocity();
        bool imu_ok = (imu->consecutive_failures == prev_imu_failures);
        prev_imu_failures = imu->consecutive_failures;

        SensorSnapshot snap = {enc_delta, enc_ok, q, w, imu_ok};
        nav_state_compact_t compact = {};
        nav_tick_update(state, snap, &compact);

        uint32_t irq_state = spin_lock_blocking(nav_lock);
        shared_state = compact;
        spin_unlock(nav_lock, irq_state);

        if (!multicore_fifo_push_timeout_us(sequence_num, NAV_FIFO_TIMEOUT_US)) {
            if (dropped_frames < UINT32_MAX) {
                dropped_frames++;
            }
        }
        sequence_num++;

        watchdog_update();

        uint32_t elapsed = absolute_time_diff_us(start, get_absolute_time());
        if (elapsed < jitter.min_us) {
            jitter.min_us = elapsed;
        }
        if (elapsed > jitter.max_us) {
            jitter.max_us = elapsed;
        }
        jitter.last_us = elapsed;

        sleep_until(deadline);
    }
}

nav_state_compact_t core1_get_nav_state() {
    uint32_t irq_state = spin_lock_blocking(nav_lock);
    nav_state_compact_t copy = shared_state;
    spin_unlock(nav_lock, irq_state);
    return copy;
}

JitterStats core1_get_jitter_stats() {
    return jitter;
}

uint32_t core1_get_dropped_frames() {
    return dropped_frames;
}
