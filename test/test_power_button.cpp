/*
 * Power Button Unit Tests
 * Tests state machine logic for short press (tare), long press (shutdown), and feedback
 */

#include "utils/power_button.hpp"
#include <gtest/gtest.h>

class PowerButtonTest : public ::testing::Test {
protected:
    PowerButtonState state = {};
    static constexpr uint32_t FEEDBACK_MS = 1500;
    static constexpr uint32_t SHUTDOWN_MS = 3000;

    ButtonAction update(uint32_t t, bool pressed) {
        return power_button_update(t, pressed, FEEDBACK_MS, SHUTDOWN_MS, state);
    }
};

TEST_F(PowerButtonTest, NoPressAlwaysNone) {
    /* Button never pressed — no events should fire */
    for (uint32_t t = 0; t < 5000; t += 100) {
        EXPECT_EQ(update(t, false), ButtonAction::None);
    }
}

TEST_F(PowerButtonTest, ShortPressAndReleaseTriggersTare) {
    /* Press at t=0, hold for 500ms (< feedback), release -> Tare */
    EXPECT_EQ(update(0, true), ButtonAction::None);     /* press detected */
    EXPECT_EQ(update(200, true), ButtonAction::None);   /* held */
    EXPECT_EQ(update(500, true), ButtonAction::None);   /* still held */
    EXPECT_EQ(update(600, false), ButtonAction::Tare);  /* released before feedback */
}

TEST_F(PowerButtonTest, PressStartReturnsNone) {
    /* First press should return None since we don't know if it's short or long */
    EXPECT_EQ(update(0, true), ButtonAction::None);
    /* Only after release do we know it was a short press */
    EXPECT_EQ(update(100, false), ButtonAction::Tare);
}

TEST_F(PowerButtonTest, HoldPastFeedbackShowsFeedback) {
    /* Press at t=0, hold past feedback threshold -> ShowFeedback fires once */
    EXPECT_EQ(update(0, true), ButtonAction::None);
    EXPECT_EQ(update(1000, true), ButtonAction::None);  /* not yet at threshold */
    EXPECT_EQ(update(1500, true), ButtonAction::ShowFeedback);  /* at threshold */
    EXPECT_EQ(update(1600, true), ButtonAction::None);  /* already shown, no re-fire */
    EXPECT_EQ(update(2000, true), ButtonAction::None);  /* still held, no re-fire */
}

TEST_F(PowerButtonTest, ReleaseAfterFeedbackHidesFeedback) {
    /* Press, hold past feedback, release before shutdown -> HideFeedback (not Tare) */
    EXPECT_EQ(update(0, true), ButtonAction::None);
    EXPECT_EQ(update(1500, true), ButtonAction::ShowFeedback);
    EXPECT_EQ(update(2000, true), ButtonAction::None);   /* held between feedback and shutdown */
    EXPECT_EQ(update(2500, false), ButtonAction::HideFeedback);  /* release -> hide, not tare */
}

TEST_F(PowerButtonTest, HoldPastShutdownTriggersShutdown) {
    /* Press at t=0, hold through shutdown threshold -> Shutdown */
    EXPECT_EQ(update(0, true), ButtonAction::None);
    EXPECT_EQ(update(1500, true), ButtonAction::ShowFeedback);
    EXPECT_EQ(update(2000, true), ButtonAction::None);
    EXPECT_EQ(update(3000, true), ButtonAction::Shutdown);  /* at shutdown threshold */
}

TEST_F(PowerButtonTest, ShutdownFiresRepeatedly) {
    /* After reaching shutdown threshold, Shutdown continues to fire while held */
    EXPECT_EQ(update(0, true), ButtonAction::None);
    EXPECT_EQ(update(1500, true), ButtonAction::ShowFeedback);
    EXPECT_EQ(update(3000, true), ButtonAction::Shutdown);
    EXPECT_EQ(update(3100, true), ButtonAction::Shutdown);  /* still fires */
    EXPECT_EQ(update(3500, true), ButtonAction::Shutdown);  /* continues firing */
    EXPECT_EQ(update(4000, true), ButtonAction::Shutdown);
}

TEST_F(PowerButtonTest, MultiplePressReleaseCycles) {
    /* First press/release -> Tare, second press/release -> Tare (state resets) */
    /* First cycle */
    EXPECT_EQ(update(0, true), ButtonAction::None);
    EXPECT_EQ(update(500, true), ButtonAction::None);
    EXPECT_EQ(update(600, false), ButtonAction::Tare);

    /* Idle period */
    EXPECT_EQ(update(700, false), ButtonAction::None);
    EXPECT_EQ(update(800, false), ButtonAction::None);

    /* Second cycle */
    EXPECT_EQ(update(1000, true), ButtonAction::None);
    EXPECT_EQ(update(1400, true), ButtonAction::None);
    EXPECT_EQ(update(1500, false), ButtonAction::Tare);  /* state fully reset */
}

TEST_F(PowerButtonTest, TimestampWrapAround) {
    /* Press near UINT32_MAX, hold past wrap -> correct elapsed via unsigned subtraction */
    uint32_t near_max = UINT32_MAX - 500;

    EXPECT_EQ(update(near_max, true), ButtonAction::None);  /* press at near_max */

    /* Wrap around: near_max + 1600 wraps to ~1099 */
    uint32_t wrapped_feedback = near_max + 1600;  /* elapsed = 1600ms */
    EXPECT_EQ(update(wrapped_feedback, true), ButtonAction::ShowFeedback);

    /* Continue to shutdown threshold: near_max + 3100 */
    uint32_t wrapped_shutdown = near_max + 3100;  /* elapsed = 3100ms */
    EXPECT_EQ(update(wrapped_shutdown, true), ButtonAction::Shutdown);
}

TEST_F(PowerButtonTest, ExactFeedbackBoundary) {
    /* Hold exactly at FEEDBACK_MS -> ShowFeedback */
    EXPECT_EQ(update(0, true), ButtonAction::None);
    EXPECT_EQ(update(1499, true), ButtonAction::None);  /* 1ms before threshold */
    EXPECT_EQ(update(1500, true), ButtonAction::ShowFeedback);  /* exactly at threshold */
}

TEST_F(PowerButtonTest, ExactShutdownBoundary) {
    /* Hold exactly at SHUTDOWN_MS -> Shutdown */
    EXPECT_EQ(update(0, true), ButtonAction::None);
    EXPECT_EQ(update(1500, true), ButtonAction::ShowFeedback);
    EXPECT_EQ(update(2999, true), ButtonAction::None);  /* 1ms before shutdown */
    EXPECT_EQ(update(3000, true), ButtonAction::Shutdown);  /* exactly at threshold */
}

TEST_F(PowerButtonTest, VeryShortTap) {
    /* Press and release in rapid succession -> Tare */
    EXPECT_EQ(update(100, true), ButtonAction::None);   /* press */
    EXPECT_EQ(update(110, false), ButtonAction::Tare);  /* release after 10ms */
}

TEST_F(PowerButtonTest, FeedbackNotReShownAfterRelease) {
    /* Press, get ShowFeedback, release (HideFeedback), press again -> ShowFeedback fires again */
    /* First press cycle */
    EXPECT_EQ(update(0, true), ButtonAction::None);
    EXPECT_EQ(update(1500, true), ButtonAction::ShowFeedback);
    EXPECT_EQ(update(2000, false), ButtonAction::HideFeedback);

    /* Idle period */
    EXPECT_EQ(update(2100, false), ButtonAction::None);

    /* Second press cycle — feedback_shown flag was reset on release */
    EXPECT_EQ(update(2500, true), ButtonAction::None);
    EXPECT_EQ(update(4000, true), ButtonAction::ShowFeedback);  /* fires again (state reset) */
    EXPECT_EQ(update(4100, true), ButtonAction::None);  /* no re-fire */
}

TEST_F(PowerButtonTest, HoldJustBeforeFeedback) {
    /* Hold at t=1499 -> None (not yet at threshold) */
    EXPECT_EQ(update(0, true), ButtonAction::None);
    EXPECT_EQ(update(1499, true), ButtonAction::None);  /* 1ms before feedback */
    EXPECT_EQ(update(1500, true), ButtonAction::ShowFeedback);  /* now at threshold */
}

TEST_F(PowerButtonTest, StateIsolationBetweenInstances) {
    /* Two PowerButtonState instances are fully independent */
    PowerButtonState state_a = {};
    PowerButtonState state_b = {};

    /* Press A, hold past feedback */
    ButtonAction a1 = power_button_update(0, true, FEEDBACK_MS, SHUTDOWN_MS, state_a);
    ButtonAction b1 = power_button_update(0, false, FEEDBACK_MS, SHUTDOWN_MS, state_b);
    EXPECT_EQ(a1, ButtonAction::None);
    EXPECT_EQ(b1, ButtonAction::None);

    ButtonAction a2 = power_button_update(1500, true, FEEDBACK_MS, SHUTDOWN_MS, state_a);
    ButtonAction b2 = power_button_update(1500, false, FEEDBACK_MS, SHUTDOWN_MS, state_b);
    EXPECT_EQ(a2, ButtonAction::ShowFeedback);  /* A reaches feedback */
    EXPECT_EQ(b2, ButtonAction::None);          /* B still idle */

    /* Now press B */
    ButtonAction b3 = power_button_update(2000, true, FEEDBACK_MS, SHUTDOWN_MS, state_b);
    EXPECT_EQ(b3, ButtonAction::None);  /* B press starts */

    /* A reaches shutdown, B still before feedback */
    ButtonAction a3 = power_button_update(3000, true, FEEDBACK_MS, SHUTDOWN_MS, state_a);
    ButtonAction b4 = power_button_update(3000, true, FEEDBACK_MS, SHUTDOWN_MS, state_b);
    EXPECT_EQ(a3, ButtonAction::Shutdown);  /* A at shutdown (3000ms from t=0) */
    EXPECT_EQ(b4, ButtonAction::None);      /* B at 1000ms elapsed, not yet feedback */
}

TEST_F(PowerButtonTest, ReleaseBeforeFeedbackAlwaysTare) {
    /* Release at various points before feedback threshold -> always Tare */
    uint32_t test_points[] = {10, 100, 500, 1000, 1499};

    for (uint32_t release_time : test_points) {
        PowerButtonState test_state = {};
        ButtonAction press = power_button_update(0, true, FEEDBACK_MS, SHUTDOWN_MS, test_state);
        EXPECT_EQ(press, ButtonAction::None);

        if (release_time > 0) {
            ButtonAction hold = power_button_update(release_time - 1, true, FEEDBACK_MS, SHUTDOWN_MS, test_state);
            EXPECT_EQ(hold, ButtonAction::None);
        }

        ButtonAction release = power_button_update(release_time, false, FEEDBACK_MS, SHUTDOWN_MS, test_state);
        EXPECT_EQ(release, ButtonAction::Tare) << "Expected Tare at release_time=" << release_time;
    }
}

TEST_F(PowerButtonTest, ReleaseAfterFeedbackBeforeShutdownAlwaysHide) {
    /* Release at various points after feedback but before shutdown -> always HideFeedback */
    uint32_t test_points[] = {1501, 1600, 2000, 2500, 2999};

    for (uint32_t release_time : test_points) {
        PowerButtonState test_state = {};
        power_button_update(0, true, FEEDBACK_MS, SHUTDOWN_MS, test_state);
        ButtonAction feedback = power_button_update(FEEDBACK_MS, true, FEEDBACK_MS, SHUTDOWN_MS, test_state);
        EXPECT_EQ(feedback, ButtonAction::ShowFeedback);

        ButtonAction release = power_button_update(release_time, false, FEEDBACK_MS, SHUTDOWN_MS, test_state);
        EXPECT_EQ(release, ButtonAction::HideFeedback) << "Expected HideFeedback at release_time=" << release_time;
    }
}

TEST_F(PowerButtonTest, NotPressedAfterReleaseMeansNone) {
    /* After release, continued not-pressed state returns None */
    EXPECT_EQ(update(0, true), ButtonAction::None);
    EXPECT_EQ(update(500, false), ButtonAction::Tare);  /* release */

    /* Continued not-pressed state */
    EXPECT_EQ(update(600, false), ButtonAction::None);
    EXPECT_EQ(update(700, false), ButtonAction::None);
    EXPECT_EQ(update(1000, false), ButtonAction::None);
}
