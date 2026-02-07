/*
 * Stdio_Display - USB printf-based display implementation
 * Outputs calibration status to serial console via USB CDC
 */

#ifndef STDIO_DISPLAY_HPP
#define STDIO_DISPLAY_HPP

#include "display_interface.hpp"

class Stdio_Display final : public Display_Interface {
public:
    void show_status(const char *sensor, const char *msg) override;
    void show_error(const char *sensor, const char *msg, DisplaySeverity sev) override;
    void show_progress(const char *sensor, uint8_t pct) override;
    void clear() override;
};

#endif // STDIO_DISPLAY_HPP
