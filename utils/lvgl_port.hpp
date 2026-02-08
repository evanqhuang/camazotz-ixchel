/*
 * LVGL Port - Bridge between LVGL display driver and AMOLED_Driver hardware
 * Manages draw buffer, flush callback, and tick timer
 */

#ifndef LVGL_PORT_HPP
#define LVGL_PORT_HPP

class AMOLED_Driver;

bool lvgl_port_init(AMOLED_Driver &driver);
void lvgl_port_task_handler();

#endif // LVGL_PORT_HPP
