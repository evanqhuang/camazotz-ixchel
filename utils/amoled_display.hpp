/*
 * AMOLED_Display - Display_Interface implementation using LVGL on AMOLED
 * Renders calibration status, errors, and progress via LVGL labels/bars
 */

#ifndef AMOLED_DISPLAY_HPP
#define AMOLED_DISPLAY_HPP

#include "display_interface.hpp"
#include "drivers/amoled_driver.hpp"
#include "lvgl.h"

#include <cstdint>

class AMOLED_Display final : public Display_Interface {
public:
    bool init();

    void show_status(const char *sensor, const char *msg) override;
    void show_error(const char *sensor, const char *msg, DisplaySeverity sev) override;
    void show_progress(const char *sensor, uint8_t pct) override;
    void clear() override;

    AMOLED_Driver &driver() { return driver_; }
    void set_brightness(uint8_t percent);
    void task_handler();

private:
    static constexpr uint8_t MAX_LABELS = 24;
    static constexpr int16_t LINE_HEIGHT = 18;
    static constexpr int16_t MARGIN_X = 4;
    static constexpr int16_t MARGIN_Y = 4;

    AMOLED_Driver driver_;
    int16_t cursor_y_ = MARGIN_Y;
    lv_obj_t *labels_[MAX_LABELS] = {};
    uint8_t label_count_ = 0;
    lv_obj_t *progress_bar_ = nullptr;
    lv_obj_t *progress_label_ = nullptr;

    lv_obj_t *add_label(const char *text, lv_color_t color);
    void ensure_progress_widgets();
    void scroll_if_needed();
};

#endif // AMOLED_DISPLAY_HPP
