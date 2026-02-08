/*
 * LVGL Port Implementation - Connects LVGL to AMOLED_Driver
 */

#include "utils/lvgl_port.hpp"
#include "drivers/amoled_driver.hpp"
#include "config.h"

#include "lvgl.h"
#include "pico/stdlib.h"

#include <cstring>

/* Module-level state */
static AMOLED_Driver *s_driver = nullptr;
static lv_disp_draw_buf_t s_disp_buf;
static lv_disp_drv_t s_disp_drv;
static lv_color_t s_buf[DISPLAY_WIDTH * 48];  /* ~26KB partial draw buffer */
static struct repeating_timer s_tick_timer;

static bool tick_cb(struct repeating_timer * /* t */) {
    lv_tick_inc(5);
    return true;
}

static void flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
    if (s_driver == nullptr) {
        lv_disp_flush_ready(drv);
        return;
    }

    uint16_t x1 = static_cast<uint16_t>(area->x1);
    uint16_t y1 = static_cast<uint16_t>(area->y1);
    uint16_t x2 = static_cast<uint16_t>(area->x2 + 1);
    uint16_t y2 = static_cast<uint16_t>(area->y2 + 1);

    s_driver->set_window(x1, y1, x2, y2);

    uint32_t pixel_count = static_cast<uint32_t>(x2 - x1) * static_cast<uint32_t>(y2 - y1);
    uint32_t byte_count = pixel_count * 2U;  /* RGB565 = 2 bytes/pixel */

    s_driver->transfer_pixels(reinterpret_cast<const uint8_t *>(color_p), byte_count);

    lv_disp_flush_ready(drv);
}

bool lvgl_port_init(AMOLED_Driver &driver) {
    s_driver = &driver;

    lv_init();

    lv_disp_draw_buf_init(&s_disp_buf, s_buf, nullptr, DISPLAY_WIDTH * 48);

    lv_disp_drv_init(&s_disp_drv);
    s_disp_drv.flush_cb = flush_cb;
    s_disp_drv.draw_buf = &s_disp_buf;
    s_disp_drv.hor_res = static_cast<lv_coord_t>(DISPLAY_WIDTH);
    s_disp_drv.ver_res = static_cast<lv_coord_t>(DISPLAY_HEIGHT);
    lv_disp_drv_register(&s_disp_drv);

    /* 5ms repeating timer for LVGL tick */
    add_repeating_timer_ms(5, tick_cb, nullptr, &s_tick_timer);

    return true;
}

void lvgl_port_task_handler() {
    lv_task_handler();
}
