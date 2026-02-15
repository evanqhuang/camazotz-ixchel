/*
 * Power Button Implementation
 */

#include "utils/power_button.hpp"

ButtonAction power_button_update(uint32_t now_ms, bool debounced_pressed,
                                  uint32_t feedback_ms, uint32_t shutdown_ms,
                                  PowerButtonState &state) {
    /* New press detected */
    if (!state.was_pressed && debounced_pressed) {
        state.was_pressed = true;
        state.press_start_ms = now_ms;
        state.feedback_shown = false;
        return ButtonAction::None;
    }

    /* Button held down */
    if (state.was_pressed && debounced_pressed) {
        /* Unsigned subtraction handles uint32_t wrap */
        uint32_t elapsed = now_ms - state.press_start_ms;

        /* Shutdown threshold reached */
        if (elapsed >= shutdown_ms) {
            return ButtonAction::Shutdown;
        }

        /* Feedback threshold reached for first time */
        if (elapsed >= feedback_ms && !state.feedback_shown) {
            state.feedback_shown = true;
            return ButtonAction::ShowFeedback;
        }

        return ButtonAction::None;
    }

    /* Button released */
    if (state.was_pressed && !debounced_pressed) {
        bool had_shown_feedback = state.feedback_shown;
        state.was_pressed = false;
        state.feedback_shown = false;

        if (had_shown_feedback) {
            return ButtonAction::HideFeedback;
        }
        return ButtonAction::Tare;
    }

    /* Default: not pressed and was not pressed */
    return ButtonAction::None;
}
