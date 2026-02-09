/*
 * Nav_Screen - LVGL navigation display for 280x456 AMOLED
 * Shows heading, depth, distance, sensor status, and alerts
 */

#ifndef NAV_SCREEN_HPP
#define NAV_SCREEN_HPP

#include "types.h"
#include "lvgl.h"
#include <cstdint>

class Nav_Screen {
public:
    void create();
    void activate();

    void update(float heading_deg, float depth_m, float pos_x, float pos_y,
                float total_dist_m, uint8_t status_flags, bool sd_ok,
                uint32_t now_ms);

    void show_critical_alert(const char *msg);
    void hide_critical_alert();
    void show_tare_status(bool success, uint32_t now_ms);

private:
    static constexpr uint32_t UPDATE_INTERVAL_MS = 200;  /* 5Hz throttle */
    static constexpr uint32_t TARE_DISPLAY_MS = 2000;    /* Auto-hide tare after 2s */

    /* Layout constants */
    static constexpr int16_t MAP_W = 178;
    static constexpr int16_t MAP_H = 425;
    static constexpr int16_t PANEL_X = 180;
    static constexpr int16_t PANEL_W = 100;
    static constexpr int16_t STATUS_Y = 426;
    static constexpr int16_t STATUS_H = 30;
    static constexpr int16_t SCREEN_W = 280;

    /* Screen root (separate from boot screen) */
    lv_obj_t *screen_ = nullptr;

    /* Map container */
    lv_obj_t *map_container_ = nullptr;
    lv_obj_t *crosshair_h_ = nullptr;
    lv_obj_t *crosshair_v_ = nullptr;
    lv_obj_t *pos_label_ = nullptr;

    /* Info panel labels */
    lv_obj_t *hdg_title_ = nullptr;
    lv_obj_t *hdg_value_ = nullptr;
    lv_obj_t *depth_title_ = nullptr;
    lv_obj_t *depth_value_ = nullptr;
    lv_obj_t *dist_title_ = nullptr;
    lv_obj_t *dist_value_ = nullptr;

    /* Status bar */
    lv_obj_t *status_bar_ = nullptr;
    lv_obj_t *enc_status_ = nullptr;
    lv_obj_t *imu_status_ = nullptr;
    lv_obj_t *dep_status_ = nullptr;
    lv_obj_t *sd_status_ = nullptr;

    /* Alert overlay */
    lv_obj_t *alert_box_ = nullptr;
    lv_obj_t *alert_label_ = nullptr;

    /* Tare popup */
    lv_obj_t *tare_box_ = nullptr;
    lv_obj_t *tare_label_ = nullptr;
    uint32_t tare_show_ms_ = 0;

    /* Throttle state */
    uint32_t last_update_ms_ = 0;

    void create_map_area();
    void create_info_panel();
    void create_status_bar();
    void create_alert_overlay();
    void create_tare_popup();

    void update_sensor_status(lv_obj_t *label, const char *prefix,
                              bool estimated, bool lost);
    void update_sd_status(bool sd_ok);
};

#endif /* NAV_SCREEN_HPP */
