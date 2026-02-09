/*
 * Button Debounce Unit Tests
 * Tests pure debounce logic without GPIO dependencies
 */

#include "utils/button_debounce.hpp"
#include <gtest/gtest.h>

class ButtonDebounceTest : public ::testing::Test {
protected:
    DebounceState state = {};
    static constexpr uint32_t DEBOUNCE_MS = 50;
};

TEST_F(ButtonDebounceTest, NoPressMeansNoEvent) {
    /* Button never pressed — no events should fire */
    for (uint32_t t = 0; t < 1000; t += 10) {
        EXPECT_FALSE(debounce_check(t, false, DEBOUNCE_MS, state));
    }
}

TEST_F(ButtonDebounceTest, CleanPressReturnsTrue) {
    /* Stable press held beyond debounce interval fires exactly once */
    EXPECT_FALSE(debounce_check(0, false, DEBOUNCE_MS, state));   /* released */
    EXPECT_FALSE(debounce_check(10, true, DEBOUNCE_MS, state));   /* pressed, timer starts */
    EXPECT_FALSE(debounce_check(30, true, DEBOUNCE_MS, state));   /* still bouncing window */
    EXPECT_TRUE(debounce_check(70, true, DEBOUNCE_MS, state));    /* 60ms > 50ms, fires */
    EXPECT_FALSE(debounce_check(80, true, DEBOUNCE_MS, state));   /* no re-fire while held */
    EXPECT_FALSE(debounce_check(200, true, DEBOUNCE_MS, state));  /* still held, no event */
}

TEST_F(ButtonDebounceTest, BounceFilteredOut) {
    /* Rapid oscillation within debounce window produces no event */
    EXPECT_FALSE(debounce_check(0, false, DEBOUNCE_MS, state));
    EXPECT_FALSE(debounce_check(10, true, DEBOUNCE_MS, state));
    EXPECT_FALSE(debounce_check(20, false, DEBOUNCE_MS, state));  /* bounce back */
    EXPECT_FALSE(debounce_check(30, true, DEBOUNCE_MS, state));   /* bounce again */
    EXPECT_FALSE(debounce_check(40, false, DEBOUNCE_MS, state));  /* bounce back */
    /* Never stable long enough — no event */
    EXPECT_FALSE(debounce_check(60, false, DEBOUNCE_MS, state));
}

TEST_F(ButtonDebounceTest, ReleaseAndRepress) {
    /* Two distinct presses produce two events */
    /* First press */
    EXPECT_FALSE(debounce_check(0, true, DEBOUNCE_MS, state));
    EXPECT_TRUE(debounce_check(60, true, DEBOUNCE_MS, state));

    /* Release */
    EXPECT_FALSE(debounce_check(100, false, DEBOUNCE_MS, state));
    EXPECT_FALSE(debounce_check(160, false, DEBOUNCE_MS, state));  /* stable release */

    /* Second press */
    EXPECT_FALSE(debounce_check(200, true, DEBOUNCE_MS, state));
    EXPECT_TRUE(debounce_check(260, true, DEBOUNCE_MS, state));
}

TEST_F(ButtonDebounceTest, RapidBouncingNeverFires) {
    /* Alternating every 20ms with 50ms debounce = no events */
    for (uint32_t t = 0; t < 1000; t += 20) {
        bool pressed = ((t / 20) % 2) == 1;
        EXPECT_FALSE(debounce_check(t, pressed, DEBOUNCE_MS, state))
            << "Unexpected event at t=" << t << " pressed=" << pressed;
    }
}

TEST_F(ButtonDebounceTest, ZeroDebounceImmediateFire) {
    /* 0ms debounce: first sample detects raw change, second confirms stability */
    EXPECT_FALSE(debounce_check(0, true, 0, state));  /* raw change detected */
    EXPECT_TRUE(debounce_check(0, true, 0, state));   /* stable, fires */
    EXPECT_FALSE(debounce_check(1, true, 0, state));  /* no re-fire while held */
}

TEST_F(ButtonDebounceTest, TimestampWrapAround) {
    /* uint32_t overflow arithmetic works correctly */
    uint32_t near_max = UINT32_MAX - 20;
    EXPECT_FALSE(debounce_check(near_max, true, DEBOUNCE_MS, state));

    /* Wrap around: near_max + 60 wraps to ~38 */
    uint32_t wrapped = near_max + 60;  /* overflows naturally */
    EXPECT_TRUE(debounce_check(wrapped, true, DEBOUNCE_MS, state));
}

TEST_F(ButtonDebounceTest, PressExactlyAtDebounceThresholdFires) {
    /* elapsed == debounce_ms fires because condition is strict less-than (elapsed < debounce_ms) */
    EXPECT_FALSE(debounce_check(0, true, DEBOUNCE_MS, state));    /* raw change */
    EXPECT_TRUE(debounce_check(50, true, DEBOUNCE_MS, state));    /* exactly 50ms: fires */
}

TEST_F(ButtonDebounceTest, BouncyReleaseNoSpuriousEvent) {
    /* Stable press, bouncy release, then clean repress — no spurious event during bounce */
    EXPECT_FALSE(debounce_check(0, true, DEBOUNCE_MS, state));
    EXPECT_TRUE(debounce_check(60, true, DEBOUNCE_MS, state));     /* first press fires */

    /* Bouncy release: oscillates between pressed/released */
    EXPECT_FALSE(debounce_check(100, false, DEBOUNCE_MS, state));  /* start release */
    EXPECT_FALSE(debounce_check(110, true, DEBOUNCE_MS, state));   /* bounce back */
    EXPECT_FALSE(debounce_check(120, false, DEBOUNCE_MS, state));  /* bounce again */
    EXPECT_FALSE(debounce_check(130, true, DEBOUNCE_MS, state));   /* bounce */
    EXPECT_FALSE(debounce_check(140, false, DEBOUNCE_MS, state));  /* settling */
    EXPECT_FALSE(debounce_check(200, false, DEBOUNCE_MS, state));  /* stable release */

    /* New clean press after stable release */
    EXPECT_FALSE(debounce_check(250, true, DEBOUNCE_MS, state));   /* raw change */
    EXPECT_TRUE(debounce_check(310, true, DEBOUNCE_MS, state));    /* second press fires */
}

TEST_F(ButtonDebounceTest, MultipleIndependentStates) {
    /* Two DebounceState instances are fully independent */
    DebounceState state_a = {};
    DebounceState state_b = {};

    EXPECT_FALSE(debounce_check(0, true, DEBOUNCE_MS, state_a));
    EXPECT_FALSE(debounce_check(0, false, DEBOUNCE_MS, state_b));  /* B not pressed */

    EXPECT_TRUE(debounce_check(60, true, DEBOUNCE_MS, state_a));   /* A fires */
    EXPECT_FALSE(debounce_check(60, false, DEBOUNCE_MS, state_b)); /* B still idle */

    /* Now press B */
    EXPECT_FALSE(debounce_check(100, true, DEBOUNCE_MS, state_b));
    EXPECT_TRUE(debounce_check(160, true, DEBOUNCE_MS, state_b));  /* B fires */
}
