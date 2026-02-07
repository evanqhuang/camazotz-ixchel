/*
 * Display_Interface - Abstract display output for calibration status
 * Decouples calibration logic from display hardware (stdio/AMOLED)
 */

#ifndef DISPLAY_INTERFACE_HPP
#define DISPLAY_INTERFACE_HPP

#include <cstdint>

enum class DisplaySeverity : uint8_t { Info, Warning, Error, Fatal };

class Display_Interface {
public:
    virtual ~Display_Interface() = default;
    virtual void show_status(const char *sensor, const char *msg) = 0;
    virtual void show_error(const char *sensor, const char *msg, DisplaySeverity sev) = 0;
    virtual void show_progress(const char *sensor, uint8_t pct) = 0;
    virtual void clear() = 0;
};

#endif // DISPLAY_INTERFACE_HPP
