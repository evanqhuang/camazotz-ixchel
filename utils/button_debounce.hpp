/*
 * Button Debounce - Pure logic, no GPIO dependency
 * Testable on host without hardware
 */

#ifndef BUTTON_DEBOUNCE_HPP
#define BUTTON_DEBOUNCE_HPP

#include <cstdint>

struct DebounceState {
    bool stable_state = false;    /* true = pressed */
    bool last_raw = false;        /* Last raw reading (true = pressed) */
    uint32_t last_change_ms = 0;  /* Timestamp of last raw state change */
};

/*
 * Check for a debounced press event.
 *
 * @param now_ms       Current timestamp in milliseconds
 * @param pressed      true if the button is physically pressed (caller inverts GPIO if needed)
 * @param debounce_ms  Minimum stable interval before accepting a state change
 * @param state        Persistent debounce state (caller-owned)
 * @return true on a single debounced released->pressed transition
 */
bool debounce_check(uint32_t now_ms, bool pressed, uint32_t debounce_ms,
                    DebounceState &state);

#endif /* BUTTON_DEBOUNCE_HPP */
