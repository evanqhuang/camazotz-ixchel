/*
 * Power Button - Long-press detection state machine
 * Pure logic, no GPIO dependency. Testable on host.
 */

#ifndef POWER_BUTTON_HPP
#define POWER_BUTTON_HPP

#include <cstdint>

enum class ButtonAction : uint8_t {
    None,
    Tare,          /* Short press released (< feedback threshold) */
    ShowFeedback,  /* Held past feedback threshold (fires once) */
    HideFeedback,  /* Released after feedback but before shutdown */
    Shutdown       /* Held past shutdown threshold */
};

struct PowerButtonState {
    bool was_pressed = false;
    uint32_t press_start_ms = 0;
    bool feedback_shown = false;
};

/*
 * Update power button state machine each tick.
 *
 * @param now_ms             Current timestamp in milliseconds
 * @param debounced_pressed  true if button is stably pressed (from DebounceState.stable_state)
 * @param feedback_ms        Hold duration to show "power off" feedback (e.g. 1500)
 * @param shutdown_ms        Hold duration to trigger shutdown (e.g. 3000)
 * @param state              Persistent state (caller-owned)
 * @return Action to take this tick
 */
ButtonAction power_button_update(uint32_t now_ms, bool debounced_pressed,
                                  uint32_t feedback_ms, uint32_t shutdown_ms,
                                  PowerButtonState &state);

#endif /* POWER_BUTTON_HPP */
