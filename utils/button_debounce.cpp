/*
 * Button Debounce Implementation
 */

#include "utils/button_debounce.hpp"

bool debounce_check(uint32_t now_ms, bool pressed, uint32_t debounce_ms,
                    DebounceState &state) {
    /* Detect raw state change — restart debounce timer */
    if (pressed != state.last_raw) {
        state.last_raw = pressed;
        state.last_change_ms = now_ms;
        return false;
    }

    /* Check if debounce interval has elapsed (unsigned subtraction handles wrap) */
    uint32_t elapsed = now_ms - state.last_change_ms;
    if (elapsed < debounce_ms) {
        return false;
    }

    /* Raw state has been stable long enough — check for transition */
    bool prev_stable = state.stable_state;
    state.stable_state = pressed;

    /* Fire event only on released->pressed transition */
    return (!prev_stable && pressed);
}
