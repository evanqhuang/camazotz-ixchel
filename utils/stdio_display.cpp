/*
 * Stdio_Display Implementation - USB printf-based display output
 */

#include "utils/stdio_display.hpp"

#include <cstdio>

static const char *severity_label(DisplaySeverity sev) {
    switch (sev) {
        case DisplaySeverity::Info:    return "INFO";
        case DisplaySeverity::Warning: return "WARN";
        case DisplaySeverity::Error:   return "ERROR";
        case DisplaySeverity::Fatal:   return "FATAL";
    }
    return "UNKNOWN";
}

void Stdio_Display::show_status(const char *sensor, const char *msg) {
    printf("[INFO]  %s: %s\n", sensor, msg);
}

void Stdio_Display::show_error(const char *sensor, const char *msg, DisplaySeverity sev) {
    printf("[%s] %s: %s\n", severity_label(sev), sensor, msg);
}

void Stdio_Display::show_progress(const char *sensor, uint8_t pct) {
    printf("[INFO]  %s: %u%% complete\n", sensor, static_cast<unsigned>(pct));
}

void Stdio_Display::clear() {
    printf("\033[2J\033[H");
}
