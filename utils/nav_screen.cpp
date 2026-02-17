/*
 * Nav_Screen Implementation - LVGL navigation display
 */

#include "utils/nav_screen.hpp"
#include "config.h"

#include <cstdio>
#include <cmath>

/* Color palette */
static const lv_color_t COLOR_CYAN   = LV_COLOR_MAKE(0x00, 0xFF, 0xFF);
static const lv_color_t COLOR_LBLUE  = LV_COLOR_MAKE(0x80, 0xC0, 0xFF);
static const lv_color_t COLOR_GREEN  = LV_COLOR_MAKE(0x00, 0xFF, 0x00);
static const lv_color_t COLOR_YELLOW = LV_COLOR_MAKE(0xFF, 0xFF, 0x00);
static const lv_color_t COLOR_RED    = LV_COLOR_MAKE(0xFF, 0x00, 0x00);
static const lv_color_t COLOR_GRAY   = LV_COLOR_MAKE(0x80, 0x80, 0x80);
static const lv_color_t COLOR_WHITE  = LV_COLOR_MAKE(0xFF, 0xFF, 0xFF);
static const lv_color_t COLOR_DIM    = LV_COLOR_MAKE(0x20, 0x20, 0x20);

void Nav_Screen::create() {
    if (screen_ != nullptr) return;

    screen_ = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(screen_, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen_, LV_OPA_COVER, LV_PART_MAIN);

    create_map_area();
    create_info_panel();
    create_status_bar();
    create_alert_overlay();
    create_tare_popup();
}

void Nav_Screen::activate() {
    if (screen_ != nullptr) {
        lv_scr_load(screen_);
    }
}

void Nav_Screen::create_map_area() {
    map_container_ = lv_obj_create(screen_);
    lv_obj_remove_style_all(map_container_);
    lv_obj_set_pos(map_container_, 0, 0);
    lv_obj_set_size(map_container_, MAP_W, MAP_H);
    lv_obj_set_style_bg_color(map_container_, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(map_container_, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(map_container_, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(map_container_, COLOR_DIM, LV_PART_MAIN);

    /* Crosshair lines */
    static lv_point_t h_pts[2];
    h_pts[0] = {0, MAP_H / 2};
    h_pts[1] = {MAP_W, MAP_H / 2};

    crosshair_h_ = lv_line_create(map_container_);
    lv_line_set_points(crosshair_h_, h_pts, 2);
    lv_obj_set_style_line_color(crosshair_h_, COLOR_DIM, LV_PART_MAIN);
    lv_obj_set_style_line_width(crosshair_h_, 1, LV_PART_MAIN);

    static lv_point_t v_pts[2];
    v_pts[0] = {MAP_W / 2, 0};
    v_pts[1] = {MAP_W / 2, MAP_H};

    crosshair_v_ = lv_line_create(map_container_);
    lv_line_set_points(crosshair_v_, v_pts, 2);
    lv_obj_set_style_line_color(crosshair_v_, COLOR_DIM, LV_PART_MAIN);
    lv_obj_set_style_line_width(crosshair_v_, 1, LV_PART_MAIN);

    /* Position label at center */
    pos_label_ = lv_label_create(map_container_);
    lv_label_set_text(pos_label_, "(0.0, 0.0)");
    lv_obj_set_style_text_color(pos_label_, COLOR_WHITE, LV_PART_MAIN);
    lv_obj_set_style_text_font(pos_label_, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_pos(pos_label_, MAP_W / 2 + 4, MAP_H / 2 + 4);

    /* Trail polyline — behind crosshairs, dim teal, 2px */
    trail_line_ = lv_line_create(map_container_);
    lv_obj_set_style_line_color(trail_line_, LV_COLOR_MAKE(0x00, 0x99, 0x99), LV_PART_MAIN);
    lv_obj_set_style_line_width(trail_line_, 2, LV_PART_MAIN);
    lv_obj_set_style_line_opa(trail_line_, LV_OPA_70, LV_PART_MAIN);

    /* Start position marker — green 7x7 circle, initially hidden */
    start_marker_ = lv_obj_create(map_container_);
    lv_obj_remove_style_all(start_marker_);
    lv_obj_set_size(start_marker_, 7, 7);
    lv_obj_set_style_radius(start_marker_, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_color(start_marker_, COLOR_GREEN, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(start_marker_, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_add_flag(start_marker_, LV_OBJ_FLAG_HIDDEN);

    /* Current position dot — cyan 7x7 circle, initially hidden */
    pos_dot_ = lv_obj_create(map_container_);
    lv_obj_remove_style_all(pos_dot_);
    lv_obj_set_size(pos_dot_, 7, 7);
    lv_obj_set_style_radius(pos_dot_, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_color(pos_dot_, COLOR_CYAN, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(pos_dot_, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_add_flag(pos_dot_, LV_OBJ_FLAG_HIDDEN);

    /* Z-order: trail line at back, then crosshairs, then markers on top */
    lv_obj_move_to_index(trail_line_, 0);
}

void Nav_Screen::create_info_panel() {
    /* HDG section */
    hdg_title_ = lv_label_create(screen_);
    lv_label_set_text(hdg_title_, "HDG");
    lv_obj_set_style_text_color(hdg_title_, COLOR_GRAY, LV_PART_MAIN);
    lv_obj_set_style_text_font(hdg_title_, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_pos(hdg_title_, PANEL_X, 4);

    hdg_value_ = lv_label_create(screen_);
    lv_label_set_text(hdg_value_, "---.-");
    lv_obj_set_style_text_color(hdg_value_, COLOR_CYAN, LV_PART_MAIN);
    lv_obj_set_style_text_font(hdg_value_, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_pos(hdg_value_, PANEL_X, 22);

    /* DEPTH section */
    depth_title_ = lv_label_create(screen_);
    lv_label_set_text(depth_title_, "DEPTH");
    lv_obj_set_style_text_color(depth_title_, COLOR_GRAY, LV_PART_MAIN);
    lv_obj_set_style_text_font(depth_title_, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_pos(depth_title_, PANEL_X, 60);

    depth_value_ = lv_label_create(screen_);
    lv_label_set_text(depth_value_, "---.- m");
    lv_obj_set_style_text_color(depth_value_, COLOR_LBLUE, LV_PART_MAIN);
    lv_obj_set_style_text_font(depth_value_, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_pos(depth_value_, PANEL_X, 78);

    /* DIST section */
    dist_title_ = lv_label_create(screen_);
    lv_label_set_text(dist_title_, "DIST");
    lv_obj_set_style_text_color(dist_title_, COLOR_GRAY, LV_PART_MAIN);
    lv_obj_set_style_text_font(dist_title_, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_pos(dist_title_, PANEL_X, 116);

    dist_value_ = lv_label_create(screen_);
    lv_label_set_text(dist_value_, "0.0 m");
    lv_obj_set_style_text_color(dist_value_, COLOR_WHITE, LV_PART_MAIN);
    lv_obj_set_style_text_font(dist_value_, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_pos(dist_value_, PANEL_X, 134);

    /* BAT section */
    bat_title_ = lv_label_create(screen_);
    lv_label_set_text(bat_title_, "BAT");
    lv_obj_set_style_text_color(bat_title_, COLOR_GRAY, LV_PART_MAIN);
    lv_obj_set_style_text_font(bat_title_, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_pos(bat_title_, PANEL_X, 172);

    bat_value_ = lv_label_create(screen_);
    lv_label_set_text(bat_value_, "---%");
    lv_obj_set_style_text_color(bat_value_, COLOR_GREEN, LV_PART_MAIN);
    lv_obj_set_style_text_font(bat_value_, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_pos(bat_value_, PANEL_X, 190);
}

void Nav_Screen::create_status_bar() {
    status_bar_ = lv_obj_create(screen_);
    lv_obj_remove_style_all(status_bar_);
    lv_obj_set_pos(status_bar_, 0, STATUS_Y);
    lv_obj_set_size(status_bar_, SCREEN_W, STATUS_H);
    lv_obj_set_style_bg_color(status_bar_, LV_COLOR_MAKE(0x10, 0x10, 0x10), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(status_bar_, LV_OPA_COVER, LV_PART_MAIN);

    static constexpr int16_t STAT_Y = 8;
    static constexpr int16_t STAT_SPACING = 70;

    enc_status_ = lv_label_create(status_bar_);
    lv_label_set_text(enc_status_, "ENC:--");
    lv_obj_set_style_text_font(enc_status_, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_pos(enc_status_, 4, STAT_Y);

    imu_status_ = lv_label_create(status_bar_);
    lv_label_set_text(imu_status_, "IMU:--");
    lv_obj_set_style_text_font(imu_status_, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_pos(imu_status_, 4 + STAT_SPACING, STAT_Y);

    dep_status_ = lv_label_create(status_bar_);
    lv_label_set_text(dep_status_, "DEP:--");
    lv_obj_set_style_text_font(dep_status_, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_pos(dep_status_, 4 + STAT_SPACING * 2, STAT_Y);

    sd_status_ = lv_label_create(status_bar_);
    lv_label_set_text(sd_status_, "SD:--");
    lv_obj_set_style_text_font(sd_status_, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_pos(sd_status_, 4 + STAT_SPACING * 3, STAT_Y);
}

void Nav_Screen::create_alert_overlay() {
    alert_box_ = lv_obj_create(screen_);
    lv_obj_remove_style_all(alert_box_);
    lv_obj_set_pos(alert_box_, 20, 180);
    lv_obj_set_size(alert_box_, 240, 60);
    lv_obj_set_style_bg_color(alert_box_, COLOR_RED, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(alert_box_, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(alert_box_, 6, LV_PART_MAIN);
    lv_obj_add_flag(alert_box_, LV_OBJ_FLAG_HIDDEN);

    alert_label_ = lv_label_create(alert_box_);
    lv_label_set_text(alert_label_, "");
    lv_obj_set_style_text_color(alert_label_, COLOR_WHITE, LV_PART_MAIN);
    lv_obj_set_style_text_font(alert_label_, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_center(alert_label_);
}

void Nav_Screen::create_tare_popup() {
    tare_box_ = lv_obj_create(screen_);
    lv_obj_remove_style_all(tare_box_);
    lv_obj_set_pos(tare_box_, 60, 380);
    lv_obj_set_size(tare_box_, 160, 36);
    lv_obj_set_style_bg_opa(tare_box_, LV_OPA_80, LV_PART_MAIN);
    lv_obj_set_style_radius(tare_box_, 4, LV_PART_MAIN);
    lv_obj_add_flag(tare_box_, LV_OBJ_FLAG_HIDDEN);

    tare_label_ = lv_label_create(tare_box_);
    lv_label_set_text(tare_label_, "");
    lv_obj_set_style_text_color(tare_label_, COLOR_WHITE, LV_PART_MAIN);
    lv_obj_set_style_text_font(tare_label_, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_center(tare_label_);
}

void Nav_Screen::update_trail(float wx, float wy) {
    trail_.add_point(wx, wy);

    uint16_t pt_count = trail_.count();
    if (pt_count < 2) {
        return;
    }

    TrailViewport vp = trail_.compute_viewport(MAP_W, MAP_H);

    /* Remap all points to pixel coordinates */
    for (uint16_t i = 0; i < pt_count; i++) {
        TrailPoint pt = trail_.point_at(i);
        int16_t pixel_x = 0;
        int16_t pixel_y = 0;
        trail_.world_to_pixel(pt.x, pt.y, vp, pixel_x, pixel_y);
        trail_pixels_[i].x = pixel_x;
        trail_pixels_[i].y = pixel_y;
    }

    lv_line_set_points(trail_line_, trail_pixels_, pt_count);

    /* Position start marker at world origin (0, 0) */
    int16_t start_px = 0;
    int16_t start_py = 0;
    trail_.world_to_pixel(0.0f, 0.0f, vp, start_px, start_py);
    lv_obj_set_pos(start_marker_, start_px - 3, start_py - 3);
    lv_obj_clear_flag(start_marker_, LV_OBJ_FLAG_HIDDEN);

    /* Position current dot at current world position */
    int16_t cur_px = 0;
    int16_t cur_py = 0;
    trail_.world_to_pixel(wx, wy, vp, cur_px, cur_py);
    lv_obj_set_pos(pos_dot_, cur_px - 3, cur_py - 3);
    lv_obj_clear_flag(pos_dot_, LV_OBJ_FLAG_HIDDEN);
}

void Nav_Screen::update(float heading_deg, float depth_m, float pos_x, float pos_y,
                        float total_dist_m, uint8_t status_flags, bool sd_ok,
                        uint8_t battery_pct, bool on_battery, uint32_t now_ms) {
    /* Throttle to 5Hz */
    if ((now_ms - last_update_ms_) < UPDATE_INTERVAL_MS) {
        /* Still check tare auto-hide */
        if (!lv_obj_has_flag(tare_box_, LV_OBJ_FLAG_HIDDEN) &&
            (now_ms - tare_show_ms_) >= TARE_DISPLAY_MS) {
            lv_obj_add_flag(tare_box_, LV_OBJ_FLAG_HIDDEN);
        }
        return;
    }
    last_update_ms_ = now_ms;

    /* Heading */
    char buf[24];
    snprintf(buf, sizeof(buf), "%.1f", static_cast<double>(heading_deg));
    lv_label_set_text(hdg_value_, buf);

    /* Depth */
    snprintf(buf, sizeof(buf), "%.1f m", static_cast<double>(depth_m));
    lv_label_set_text(depth_value_, buf);

    /* Distance */
    snprintf(buf, sizeof(buf), "%.1f m", static_cast<double>(total_dist_m));
    lv_label_set_text(dist_value_, buf);

    /* Position */
    char pos_buf[32];
    snprintf(pos_buf, sizeof(pos_buf), "(%.1f, %.1f)",
             static_cast<double>(pos_x), static_cast<double>(pos_y));
    lv_label_set_text(pos_label_, pos_buf);

    /* Update trail map */
    update_trail(pos_x, pos_y);

    /* Sensor status */
    update_sensor_status(enc_status_, "ENC",
                         (status_flags & NAV_FLAG_ENCODER_ESTIMATED) != 0,
                         (status_flags & NAV_FLAG_ENCODER_LOST) != 0);
    update_sensor_status(imu_status_, "IMU",
                         (status_flags & NAV_FLAG_IMU_ESTIMATED) != 0,
                         (status_flags & NAV_FLAG_IMU_LOST) != 0);
    update_sensor_status(dep_status_, "DEP",
                         (status_flags & NAV_FLAG_DEPTH_VIRTUAL) != 0,
                         (status_flags & NAV_FLAG_NAV_CRITICAL) != 0);
    update_sd_status(sd_ok);
    update_battery_status(battery_pct, on_battery);

    /* Auto-hide tare popup */
    if (!lv_obj_has_flag(tare_box_, LV_OBJ_FLAG_HIDDEN) &&
        (now_ms - tare_show_ms_) >= TARE_DISPLAY_MS) {
        lv_obj_add_flag(tare_box_, LV_OBJ_FLAG_HIDDEN);
    }
}

void Nav_Screen::show_critical_alert(const char *msg) {
    lv_obj_set_style_bg_color(alert_box_, COLOR_RED, LV_PART_MAIN);
    lv_label_set_text(alert_label_, msg);
    lv_obj_clear_flag(alert_box_, LV_OBJ_FLAG_HIDDEN);
}

void Nav_Screen::hide_critical_alert() {
    lv_obj_add_flag(alert_box_, LV_OBJ_FLAG_HIDDEN);
}

void Nav_Screen::show_tare_status(bool success, uint32_t now_ms) {
    lv_obj_set_style_bg_color(tare_box_,
                              success ? COLOR_GREEN : COLOR_RED, LV_PART_MAIN);
    lv_label_set_text(tare_label_, success ? "Tare OK" : "Tare FAIL");
    lv_obj_clear_flag(tare_box_, LV_OBJ_FLAG_HIDDEN);
    tare_show_ms_ = now_ms;
}

void Nav_Screen::show_tare_locked(uint32_t now_ms) {
    lv_obj_set_style_bg_color(tare_box_, COLOR_YELLOW, LV_PART_MAIN);
    lv_label_set_text(tare_label_, "Tare LOCKED");
    lv_obj_clear_flag(tare_box_, LV_OBJ_FLAG_HIDDEN);
    tare_show_ms_ = now_ms;
}

void Nav_Screen::show_ready_alert() {
    lv_obj_set_style_bg_color(alert_box_, LV_COLOR_MAKE(0x00, 0x60, 0xCC), LV_PART_MAIN);
    lv_label_set_text(alert_label_, "Press to Start");
    lv_obj_clear_flag(alert_box_, LV_OBJ_FLAG_HIDDEN);
}

void Nav_Screen::reset_trail() {
    trail_.clear();
    lv_obj_add_flag(start_marker_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(pos_dot_, LV_OBJ_FLAG_HIDDEN);
    lv_line_set_points(trail_line_, nullptr, 0);
}

void Nav_Screen::update_sensor_status(lv_obj_t *label, const char *prefix,
                                      bool estimated, bool lost) {
    char buf[16];
    if (lost) {
        snprintf(buf, sizeof(buf), "%s:LOST", prefix);
        lv_obj_set_style_text_color(label, COLOR_RED, LV_PART_MAIN);
    } else if (estimated) {
        snprintf(buf, sizeof(buf), "%s:EST", prefix);
        lv_obj_set_style_text_color(label, COLOR_YELLOW, LV_PART_MAIN);
    } else {
        snprintf(buf, sizeof(buf), "%s:OK", prefix);
        lv_obj_set_style_text_color(label, COLOR_GREEN, LV_PART_MAIN);
    }
    lv_label_set_text(label, buf);
}

void Nav_Screen::update_sd_status(bool sd_ok) {
    if (sd_ok) {
        lv_label_set_text(sd_status_, "SD:OK");
        lv_obj_set_style_text_color(sd_status_, COLOR_GREEN, LV_PART_MAIN);
    } else {
        lv_label_set_text(sd_status_, "SD:ERR");
        lv_obj_set_style_text_color(sd_status_, COLOR_RED, LV_PART_MAIN);
    }
}

void Nav_Screen::update_battery_status(uint8_t pct, bool on_battery) {
    char buf[8];
    if (!on_battery) {
        /* USB power — show cyan "USB" */
        lv_label_set_text(bat_value_, "USB");
        lv_obj_set_style_text_color(bat_value_, COLOR_CYAN, LV_PART_MAIN);
    } else {
        snprintf(buf, sizeof(buf), "%u%%", pct);
        lv_label_set_text(bat_value_, buf);

        /* Color code by percentage */
        if (pct <= 10) {
            lv_obj_set_style_text_color(bat_value_, COLOR_RED, LV_PART_MAIN);
        } else if (pct <= 30) {
            lv_obj_set_style_text_color(bat_value_, COLOR_YELLOW, LV_PART_MAIN);
        } else {
            lv_obj_set_style_text_color(bat_value_, COLOR_GREEN, LV_PART_MAIN);
        }
    }
}
