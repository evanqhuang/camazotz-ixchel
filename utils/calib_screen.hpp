/*
 * Calib_Screen - Guided IMU calibration walkthrough display
 * Shows step-by-step instructions to achieve 3/3 IMU accuracy
 */

#ifndef CALIB_SCREEN_HPP
#define CALIB_SCREEN_HPP

#include "drivers/imu_wrapper.hpp"
#include "lvgl.h"
#include <cstdint>

enum class CalibPhase : uint8_t {
    Initial,        /* Starting up */
    Stationary,     /* Phase 1: Hold still (gyro/accel zero) */
    Orientation,    /* Phase 2: Slow rotations (accel multi-angle) */
    Magnetometer,   /* Phase 3: Figure-8 (mag calibration) */
    Complete,       /* 3/3 achieved */
    Skipped         /* User pressed skip */
};

struct CalibWalkthroughResult {
    CalibPhase final_phase;
    uint8_t final_accuracy;  /* 0-3 */
    uint32_t elapsed_ms;
    bool success;            /* true if accuracy >= 3 */
};

class Calib_Screen {
public:
    void create();
    void activate();
    CalibWalkthroughResult run(IMU_Wrapper &imu);

private:
    static constexpr uint32_t POLL_INTERVAL_MS = 100;       /* 10Hz polling */
    static constexpr uint32_t PHASE_MIN_DWELL_MS = 5000;    /* 5s per phase minimum */

    /* Layout constants for 280x456 AMOLED */
    static constexpr int16_t SCREEN_W = 280;
    static constexpr int16_t SCREEN_H = 456;
    static constexpr int16_t MARGIN = 10;

    /* Screen root */
    lv_obj_t *screen_ = nullptr;

    /* Title */
    lv_obj_t *title_label_ = nullptr;

    /* Phase dots */
    lv_obj_t *phase_container_ = nullptr;
    lv_obj_t *phase_dots_[3] = {};
    lv_obj_t *phase_labels_[3] = {};

    /* Instructions box */
    lv_obj_t *instr_box_ = nullptr;
    lv_obj_t *instr_title_ = nullptr;
    lv_obj_t *instr_detail_ = nullptr;

    /* Accuracy display */
    lv_obj_t *accuracy_label_ = nullptr;
    lv_obj_t *accuracy_bar_ = nullptr;

    /* Elapsed time */
    lv_obj_t *elapsed_label_ = nullptr;

    /* Skip hint */
    lv_obj_t *skip_hint_ = nullptr;

    /* State */
    CalibPhase current_phase_ = CalibPhase::Initial;
    uint32_t phase_start_ms_ = 0;

    /* Phase dot color state tracking (avoid redundant LVGL updates) */
    uint8_t prev_active_idx_ = 255;
    int8_t prev_completed_up_to_ = -2;

    void create_title();
    void create_phase_dots();
    void create_instruction_box();
    void create_accuracy_display();
    void create_elapsed_display();
    void create_skip_hint();

    void update_phase_dots(CalibPhase phase);
    void update_instructions(CalibPhase phase);
    void update_accuracy(uint8_t accuracy);
    void update_elapsed(uint32_t elapsed_ms);

    CalibPhase compute_next_phase(uint8_t accuracy, uint32_t phase_dwell_ms) const;
};

#endif /* CALIB_SCREEN_HPP */
