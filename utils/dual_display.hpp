/*
 * Dual_Display - Composite display that forwards calls to two Display_Interface implementations
 * Used to output simultaneously to AMOLED and USB CDC serial
 */

#ifndef DUAL_DISPLAY_HPP
#define DUAL_DISPLAY_HPP

#include "display_interface.hpp"

class Dual_Display final : public Display_Interface {
public:
    Dual_Display(Display_Interface &a, Display_Interface &b) : a_(a), b_(b) {}

    void show_status(const char *sensor, const char *msg) override {
        a_.show_status(sensor, msg);
        b_.show_status(sensor, msg);
    }

    void show_error(const char *sensor, const char *msg, DisplaySeverity sev) override {
        a_.show_error(sensor, msg, sev);
        b_.show_error(sensor, msg, sev);
    }

    void show_progress(const char *sensor, uint8_t pct) override {
        a_.show_progress(sensor, pct);
        b_.show_progress(sensor, pct);
    }

    void clear() override {
        a_.clear();
        b_.clear();
    }

private:
    Display_Interface &a_;
    Display_Interface &b_;
};

#endif // DUAL_DISPLAY_HPP
