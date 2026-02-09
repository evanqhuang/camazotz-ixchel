/*
 * Calib_Screen Implementation - Guided IMU calibration walkthrough
 */

#include "utils/calib_screen.hpp"
#include "utils/button_debounce.hpp"
#include "config.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"

#include <cstdio>

/* Color palette */
static const lv_color_t COLOR_CYAN   = LV_COLOR_MAKE(0x00, 0xFF, 0xFF);
static const lv_color_t COLOR_GREEN  = LV_COLOR_MAKE(0x00, 0xFF, 0x00);
static const lv_color_t COLOR_YELLOW = LV_COLOR_MAKE(0xFF, 0xFF, 0x00);
static const lv_color_t COLOR_RED    = LV_COLOR_MAKE(0xFF, 0x00, 0x00);
static const lv_color_t COLOR_GRAY   = LV_COLOR_MAKE(0x80, 0x80, 0x80);
static const lv_color_t COLOR_WHITE  = LV_COLOR_MAKE(0xFF, 0xFF, 0xFF);
static const lv_color_t COLOR_DIM    = LV_COLOR_MAKE(0x30, 0x30, 0x30);

void Calib_Screen::create() {
    if (screen_ != nullptr) return;

    screen_ = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(screen_, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen_, LV_OPA_COVER, LV_PART_MAIN);

    create_title();
    create_phase_dots();
    create_instruction_box();
    create_accuracy_display();
    create_elapsed_display();
    create_skip_hint();
}

void Calib_Screen::activate() {
    if (screen_ != nullptr) {
        lv_scr_load(screen_);
    }
}

void Calib_Screen::create_title() {
    title_label_ = lv_label_create(screen_);
    lv_label_set_text(title_label_, "IMU CALIBRATION");
    lv_obj_set_style_text_color(title_label_, COLOR_CYAN, LV_PART_MAIN);
    lv_obj_set_style_text_font(title_label_, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_align(title_label_, LV_ALIGN_TOP_MID, 0, MARGIN);
}

void Calib_Screen::create_phase_dots() {
    phase_container_ = lv_obj_create(screen_);
    lv_obj_remove_style_all(phase_container_);
    lv_obj_set_pos(phase_container_, 0, 50);
    lv_obj_set_size(phase_container_, SCREEN_W, 60);

    static const char *phase_names[3] = {"GYRO", "ACCEL", "MAG"};
    static constexpr int16_t DOT_SPACING = 90;
    static constexpr int16_t DOT_START_X = (SCREEN_W - DOT_SPACING * 2) / 2;
    static constexpr int16_t DOT_RADIUS = 12;

    for (int i = 0; i < 3; i++) {
        /* Create dot (circle) */
        phase_dots_[i] = lv_obj_create(phase_container_);
        lv_obj_remove_style_all(phase_dots_[i]);
        lv_obj_set_size(phase_dots_[i], DOT_RADIUS * 2, DOT_RADIUS * 2);
        lv_obj_set_pos(phase_dots_[i], DOT_START_X + i * DOT_SPACING - DOT_RADIUS, 5);
        lv_obj_set_style_radius(phase_dots_[i], LV_RADIUS_CIRCLE, LV_PART_MAIN);
        lv_obj_set_style_border_width(phase_dots_[i], 2, LV_PART_MAIN);
        lv_obj_set_style_border_color(phase_dots_[i], COLOR_GRAY, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(phase_dots_[i], LV_OPA_TRANSP, LV_PART_MAIN);

        /* Create label under dot */
        phase_labels_[i] = lv_label_create(phase_container_);
        lv_label_set_text(phase_labels_[i], phase_names[i]);
        lv_obj_set_style_text_color(phase_labels_[i], COLOR_GRAY, LV_PART_MAIN);
        lv_obj_set_style_text_font(phase_labels_[i], &lv_font_montserrat_14, LV_PART_MAIN);
        lv_obj_align_to(phase_labels_[i], phase_dots_[i], LV_ALIGN_OUT_BOTTOM_MID, 0, 4);
    }
}

void Calib_Screen::create_instruction_box() {
    instr_box_ = lv_obj_create(screen_);
    lv_obj_remove_style_all(instr_box_);
    lv_obj_set_pos(instr_box_, MARGIN, 120);
    lv_obj_set_size(instr_box_, SCREEN_W - MARGIN * 2, 140);
    lv_obj_set_style_bg_color(instr_box_, COLOR_DIM, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(instr_box_, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(instr_box_, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_all(instr_box_, 12, LV_PART_MAIN);

    instr_title_ = lv_label_create(instr_box_);
    lv_label_set_text(instr_title_, "INITIALIZING...");
    lv_obj_set_style_text_color(instr_title_, COLOR_WHITE, LV_PART_MAIN);
    lv_obj_set_style_text_font(instr_title_, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_width(instr_title_, SCREEN_W - MARGIN * 2 - 24);
    lv_label_set_long_mode(instr_title_, LV_LABEL_LONG_WRAP);
    lv_obj_align(instr_title_, LV_ALIGN_TOP_MID, 0, 0);

    instr_detail_ = lv_label_create(instr_box_);
    lv_label_set_text(instr_detail_, "Please wait...");
    lv_obj_set_style_text_color(instr_detail_, COLOR_GRAY, LV_PART_MAIN);
    lv_obj_set_style_text_font(instr_detail_, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_width(instr_detail_, SCREEN_W - MARGIN * 2 - 24);
    lv_label_set_long_mode(instr_detail_, LV_LABEL_LONG_WRAP);
    lv_obj_align(instr_detail_, LV_ALIGN_TOP_MID, 0, 40);
}

void Calib_Screen::create_accuracy_display() {
    accuracy_label_ = lv_label_create(screen_);
    lv_label_set_text(accuracy_label_, "ACCURACY: 0/3");
    lv_obj_set_style_text_color(accuracy_label_, COLOR_WHITE, LV_PART_MAIN);
    lv_obj_set_style_text_font(accuracy_label_, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(accuracy_label_, LV_ALIGN_TOP_MID, 0, 280);

    accuracy_bar_ = lv_bar_create(screen_);
    lv_obj_set_size(accuracy_bar_, SCREEN_W - MARGIN * 4, 20);
    lv_bar_set_range(accuracy_bar_, 0, 3);
    lv_bar_set_value(accuracy_bar_, 0, LV_ANIM_OFF);
    lv_obj_align(accuracy_bar_, LV_ALIGN_TOP_MID, 0, 310);

    lv_obj_set_style_bg_color(accuracy_bar_, COLOR_DIM, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(accuracy_bar_, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(accuracy_bar_, 4, LV_PART_MAIN);

    lv_obj_set_style_bg_color(accuracy_bar_, COLOR_RED, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(accuracy_bar_, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(accuracy_bar_, 4, LV_PART_INDICATOR);
}

void Calib_Screen::create_elapsed_display() {
    elapsed_label_ = lv_label_create(screen_);
    lv_label_set_text(elapsed_label_, "Elapsed: 0:00");
    lv_obj_set_style_text_color(elapsed_label_, COLOR_GRAY, LV_PART_MAIN);
    lv_obj_set_style_text_font(elapsed_label_, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(elapsed_label_, LV_ALIGN_TOP_MID, 0, 350);
}

void Calib_Screen::create_skip_hint() {
    skip_hint_ = lv_label_create(screen_);
    lv_label_set_text(skip_hint_, "Press TARE to skip");
    lv_obj_set_style_text_color(skip_hint_, COLOR_DIM, LV_PART_MAIN);
    lv_obj_set_style_text_font(skip_hint_, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(skip_hint_, LV_ALIGN_BOTTOM_MID, 0, -MARGIN);
}

void Calib_Screen::update_phase_dots(CalibPhase phase) {
    int active_idx = -1;
    int completed_up_to = -1;

    switch (phase) {
        case CalibPhase::Initial:
            break;
        case CalibPhase::Stationary:
            active_idx = 0;
            break;
        case CalibPhase::Orientation:
            active_idx = 1;
            completed_up_to = 0;
            break;
        case CalibPhase::Magnetometer:
            active_idx = 2;
            completed_up_to = 1;
            break;
        case CalibPhase::Complete:
        case CalibPhase::Skipped:
            completed_up_to = 2;
            break;
    }

    /* Skip redundant LVGL style updates to avoid dirty region thrashing */
    uint8_t active_u8 = (active_idx < 0) ? 255 : static_cast<uint8_t>(active_idx);
    int8_t completed_i8 = static_cast<int8_t>(completed_up_to);
    if (active_u8 == prev_active_idx_ && completed_i8 == prev_completed_up_to_) {
        return;
    }
    prev_active_idx_ = active_u8;
    prev_completed_up_to_ = completed_i8;

    for (int i = 0; i < 3; i++) {
        if (i <= completed_up_to) {
            /* Completed - filled green */
            lv_obj_set_style_bg_color(phase_dots_[i], COLOR_GREEN, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(phase_dots_[i], LV_OPA_COVER, LV_PART_MAIN);
            lv_obj_set_style_border_color(phase_dots_[i], COLOR_GREEN, LV_PART_MAIN);
            lv_obj_set_style_text_color(phase_labels_[i], COLOR_GREEN, LV_PART_MAIN);
        } else if (i == active_idx) {
            /* Active - filled cyan */
            lv_obj_set_style_bg_color(phase_dots_[i], COLOR_CYAN, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(phase_dots_[i], LV_OPA_COVER, LV_PART_MAIN);
            lv_obj_set_style_border_color(phase_dots_[i], COLOR_CYAN, LV_PART_MAIN);
            lv_obj_set_style_text_color(phase_labels_[i], COLOR_CYAN, LV_PART_MAIN);
        } else {
            /* Pending - outline only */
            lv_obj_set_style_bg_opa(phase_dots_[i], LV_OPA_TRANSP, LV_PART_MAIN);
            lv_obj_set_style_border_color(phase_dots_[i], COLOR_GRAY, LV_PART_MAIN);
            lv_obj_set_style_text_color(phase_labels_[i], COLOR_GRAY, LV_PART_MAIN);
        }
    }
}

void Calib_Screen::update_instructions(CalibPhase phase) {
    const char *title = "";
    const char *detail = "";

    switch (phase) {
        case CalibPhase::Initial:
            title = "INITIALIZING...";
            detail = "Please wait...";
            break;
        case CalibPhase::Stationary:
            title = "HOLD DEVICE STILL";
            detail = "Place on flat surface.\nKeep completely motionless\nfor gyroscope calibration.";
            break;
        case CalibPhase::Orientation:
            title = "SLOW ROTATIONS";
            detail = "Slowly tilt device in all\ndirections (pitch, roll, yaw).\nMove about 5 degrees/second.";
            break;
        case CalibPhase::Magnetometer:
            title = "FIGURE-8 PATTERN";
            detail = "Trace figure-8 shapes in\nthe air. Rotate through all\norientations smoothly.";
            break;
        case CalibPhase::Complete:
            title = "CALIBRATION COMPLETE";
            detail = "IMU fully calibrated!\nApplying tare...";
            lv_obj_set_style_text_color(instr_title_, COLOR_GREEN, LV_PART_MAIN);
            break;
        case CalibPhase::Skipped:
            title = "CALIBRATION SKIPPED";
            detail = "Continuing with current\naccuracy level.";
            lv_obj_set_style_text_color(instr_title_, COLOR_YELLOW, LV_PART_MAIN);
            break;
    }

    lv_label_set_text(instr_title_, title);
    lv_label_set_text(instr_detail_, detail);
}

void Calib_Screen::update_accuracy(uint8_t accuracy) {
    char buf[24];
    snprintf(buf, sizeof(buf), "ACCURACY: %u/3", accuracy);
    lv_label_set_text(accuracy_label_, buf);

    lv_bar_set_value(accuracy_bar_, accuracy, LV_ANIM_ON);

    /* Color-code the bar based on accuracy */
    lv_color_t bar_color;
    if (accuracy == 0) {
        bar_color = COLOR_RED;
    } else if (accuracy == 1) {
        bar_color = COLOR_YELLOW;
    } else if (accuracy == 2) {
        bar_color = LV_COLOR_MAKE(0xFF, 0xA0, 0x00); /* Orange */
    } else {
        bar_color = COLOR_GREEN;
    }
    lv_obj_set_style_bg_color(accuracy_bar_, bar_color, LV_PART_INDICATOR);
}

void Calib_Screen::update_elapsed(uint32_t elapsed_ms) {
    uint32_t total_secs = elapsed_ms / 1000;
    uint32_t mins = total_secs / 60;
    uint32_t secs = total_secs % 60;

    char buf[24];
    snprintf(buf, sizeof(buf), "Elapsed: %lu:%02lu",
             static_cast<unsigned long>(mins), static_cast<unsigned long>(secs));
    lv_label_set_text(elapsed_label_, buf);
}

CalibPhase Calib_Screen::compute_next_phase(uint8_t accuracy, uint32_t phase_dwell_ms) const {
    /* 3/3 achieved - complete */
    if (accuracy >= 3) {
        return CalibPhase::Complete;
    }

    /* Must dwell in each phase for minimum time before advancing */
    bool dwell_ok = (phase_dwell_ms >= PHASE_MIN_DWELL_MS);

    switch (current_phase_) {
        case CalibPhase::Initial:
            /* Auto-advance to Stationary after 1 second */
            if (phase_dwell_ms >= 1000) {
                return CalibPhase::Stationary;
            }
            break;
        case CalibPhase::Stationary:
            /* Advance when accuracy >= 1 and dwell time met */
            if (accuracy >= 1 && dwell_ok) {
                return CalibPhase::Orientation;
            }
            break;
        case CalibPhase::Orientation:
            /* Advance when accuracy >= 2 and dwell time met */
            if (accuracy >= 2 && dwell_ok) {
                return CalibPhase::Magnetometer;
            }
            break;
        case CalibPhase::Magnetometer:
            /* Stay here until 3/3 (handled above) */
            break;
        default:
            break;
    }

    return current_phase_;
}

CalibWalkthroughResult Calib_Screen::run(IMU_Wrapper &imu) {
    CalibWalkthroughResult result = {
        .final_phase = CalibPhase::Initial,
        .final_accuracy = 0,
        .elapsed_ms = 0,
        .success = false
    };

    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    current_phase_ = CalibPhase::Initial;
    phase_start_ms_ = start_ms;

    DebounceState skip_debounce = {};

    printf("[CALIB] Starting guided calibration walkthrough\n");
    stdio_flush();

    /* Initialize display to current state */
    uint8_t accuracy = imu.get_calibration_accuracy();
    update_phase_dots(current_phase_);
    update_instructions(current_phase_);
    update_accuracy(accuracy);
    update_elapsed(0);
    lv_task_handler();

    while (true) {
        watchdog_update();

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        uint32_t elapsed_ms = now_ms - start_ms;
        uint32_t phase_dwell_ms = now_ms - phase_start_ms_;

        /* Check TARE button (skip) */
        bool raw_pressed = !gpio_get(TARE_BUTTON_PIN);
        if (debounce_check(now_ms, raw_pressed, TARE_DEBOUNCE_MS, skip_debounce)) {
            printf("[CALIB] User pressed TARE - skipping calibration\n");
            stdio_flush();

            current_phase_ = CalibPhase::Skipped;
            update_phase_dots(current_phase_);
            update_instructions(current_phase_);
            lv_task_handler();
            sleep_ms(500);  /* Brief pause to show skip message */

            result.final_phase = CalibPhase::Skipped;
            result.final_accuracy = accuracy;
            result.elapsed_ms = elapsed_ms;
            result.success = false;
            return result;
        }

        /* Poll IMU accuracy */
        accuracy = imu.get_calibration_accuracy();

        /* Check for completion */
        if (accuracy >= 3) {
            printf("[CALIB] Achieved 3/3 accuracy after %lu ms\n",
                   static_cast<unsigned long>(elapsed_ms));
            stdio_flush();

            current_phase_ = CalibPhase::Complete;
            update_phase_dots(current_phase_);
            update_instructions(current_phase_);
            update_accuracy(accuracy);
            update_elapsed(elapsed_ms);
            lv_task_handler();
            sleep_ms(1000);  /* Brief pause to show complete message */

            result.final_phase = CalibPhase::Complete;
            result.final_accuracy = accuracy;
            result.elapsed_ms = elapsed_ms;
            result.success = true;
            return result;
        }

        /* Compute phase transitions */
        CalibPhase next_phase = compute_next_phase(accuracy, phase_dwell_ms);
        if (next_phase != current_phase_) {
            printf("[CALIB] Phase transition: %u -> %u (accuracy=%u, dwell=%lu ms)\n",
                   static_cast<unsigned>(current_phase_),
                   static_cast<unsigned>(next_phase),
                   accuracy,
                   static_cast<unsigned long>(phase_dwell_ms));
            stdio_flush();

            current_phase_ = next_phase;
            phase_start_ms_ = now_ms;
        }

        /* Update UI */
        update_phase_dots(current_phase_);
        update_instructions(current_phase_);
        update_accuracy(accuracy);
        update_elapsed(elapsed_ms);

        /* Process LVGL rendering */
        lv_task_handler();

        sleep_ms(POLL_INTERVAL_MS);
    }
}
