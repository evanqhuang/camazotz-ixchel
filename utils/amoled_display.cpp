/*
 * AMOLED_Display Implementation - LVGL-based display output on AMOLED
 */

#include "utils/amoled_display.hpp"
#include "utils/lvgl_port.hpp"
#include "config.h"

#include <cstdio>
#include <cstring>

bool AMOLED_Display::init() {
    if (!driver_.init()) {
        return false;
    }

    if (!lvgl_port_init(driver_)) {
        return false;
    }

    /* Set dark background */
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    driver_.set_brightness(DISPLAY_DEFAULT_BRIGHTNESS);

    /* Initial render */
    lv_task_handler();

    return true;
}

static lv_color_t severity_color(DisplaySeverity sev) {
    switch (sev) {
        case DisplaySeverity::Info:    return lv_color_make(0x00, 0xFF, 0x00);  /* Green */
        case DisplaySeverity::Warning: return lv_color_make(0xFF, 0xFF, 0x00);  /* Yellow */
        case DisplaySeverity::Error:   return lv_color_make(0xFF, 0x80, 0x00);  /* Orange */
        case DisplaySeverity::Fatal:   return lv_color_make(0xFF, 0x00, 0x00);  /* Red */
    }
    return lv_color_white();
}

lv_obj_t *AMOLED_Display::add_label(const char *text, lv_color_t color) {
    scroll_if_needed();

    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, color, LV_PART_MAIN);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_pos(label, MARGIN_X, cursor_y_);
    lv_obj_set_width(label, static_cast<lv_coord_t>(DISPLAY_WIDTH - 2 * MARGIN_X));
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);

    if (label_count_ < MAX_LABELS) {
        labels_[label_count_++] = label;
    }

    cursor_y_ = static_cast<int16_t>(cursor_y_ + LINE_HEIGHT);
    return label;
}

void AMOLED_Display::scroll_if_needed() {
    /* Reserve space for progress bar at bottom */
    int16_t max_y = static_cast<int16_t>(DISPLAY_HEIGHT - 60);

    if (cursor_y_ < max_y) {
        return;
    }

    /* Remove oldest label to make room */
    if (label_count_ > 0) {
        lv_obj_del(labels_[0]);

        /* Shift array down */
        for (uint8_t i = 1; i < label_count_; ++i) {
            labels_[i - 1U] = labels_[i];
            lv_obj_set_y(labels_[i - 1U],
                         static_cast<lv_coord_t>(MARGIN_Y + static_cast<int16_t>((i - 1U)) * LINE_HEIGHT));
        }
        label_count_--;
        labels_[label_count_] = nullptr;

        cursor_y_ = static_cast<int16_t>(MARGIN_Y + static_cast<int16_t>(label_count_) * LINE_HEIGHT);
    }
}

void AMOLED_Display::ensure_progress_widgets() {
    if (progress_bar_ != nullptr) {
        return;
    }

    lv_obj_t *scr = lv_scr_act();

    /* Progress bar near the bottom */
    progress_bar_ = lv_bar_create(scr);
    lv_obj_set_size(progress_bar_,
                    static_cast<lv_coord_t>(DISPLAY_WIDTH - 2 * MARGIN_X * 4),
                    14);
    lv_obj_set_pos(progress_bar_, MARGIN_X * 4,
                   static_cast<lv_coord_t>(DISPLAY_HEIGHT - 40));
    lv_bar_set_range(progress_bar_, 0, 100);
    lv_bar_set_value(progress_bar_, 0, LV_ANIM_OFF);

    /* Percentage label below bar */
    progress_label_ = lv_label_create(scr);
    lv_label_set_text(progress_label_, "0%");
    lv_obj_set_style_text_color(progress_label_, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(progress_label_, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_pos(progress_label_, MARGIN_X * 4,
                   static_cast<lv_coord_t>(DISPLAY_HEIGHT - 22));
}

void AMOLED_Display::show_status(const char *sensor, const char *msg) {
    char buf[80];
    snprintf(buf, sizeof(buf), "[%s] %s", sensor, msg);
    add_label(buf, lv_color_make(0x00, 0xFF, 0x00));
    lv_task_handler();
}

void AMOLED_Display::show_error(const char *sensor, const char *msg, DisplaySeverity sev) {
    char buf[80];
    snprintf(buf, sizeof(buf), "[%s] %s", sensor, msg);
    add_label(buf, severity_color(sev));
    lv_task_handler();
}

void AMOLED_Display::show_progress(const char *sensor, uint8_t pct) {
    ensure_progress_widgets();

    char buf[40];
    snprintf(buf, sizeof(buf), "%s: %u%%", sensor, static_cast<unsigned>(pct));
    lv_label_set_text(progress_label_, buf);
    lv_bar_set_value(progress_bar_, static_cast<int32_t>(pct), LV_ANIM_OFF);
    lv_task_handler();
}

void AMOLED_Display::clear() {
    lv_obj_clean(lv_scr_act());

    for (uint8_t i = 0; i < MAX_LABELS; ++i) {
        labels_[i] = nullptr;
    }
    label_count_ = 0;
    cursor_y_ = MARGIN_Y;
    progress_bar_ = nullptr;
    progress_label_ = nullptr;

    /* Re-apply dark background */
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    lv_task_handler();
}

void AMOLED_Display::set_brightness(uint8_t percent) {
    driver_.set_brightness(percent);
}

void AMOLED_Display::task_handler() {
    lvgl_port_task_handler();
}
