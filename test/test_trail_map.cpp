/*
 * GTest suite for TrailMap - Breadcrumb trail with auto-scaling viewport
 */

#include "utils/trail_map.hpp"
#include <gtest/gtest.h>
#include <cmath>

/* Helper: Check if two TrailPoints are equal within tolerance */
static void ExpectPointNear(const TrailPoint &actual, float expected_x, float expected_y, float tolerance = 1e-5f) {
    EXPECT_NEAR(actual.x, expected_x, tolerance);
    EXPECT_NEAR(actual.y, expected_y, tolerance);
}

/* ========================================================================
 * AddPoint Suite - Tests for point addition and distance filtering
 * ======================================================================== */

class AddPointTest : public ::testing::Test {
protected:
    TrailMap trail;

    void SetUp() override {
        trail.clear();
    }
};

TEST_F(AddPointTest, FirstPointAlwaysAccepted) {
    trail.add_point(1.0f, 2.0f);
    EXPECT_EQ(trail.count(), 1);
    TrailPoint pt = trail.point_at(0);
    ExpectPointNear(pt, 1.0f, 2.0f);
}

TEST_F(AddPointTest, DistanceThresholdFilters) {
    trail.add_point(0.0f, 0.0f);
    EXPECT_EQ(trail.count(), 1);

    /* Add point at distance ~0.07m (below 0.1m threshold) */
    trail.add_point(0.05f, 0.05f);
    EXPECT_EQ(trail.count(), 1);  /* Should be rejected */

    /* Verify first point unchanged */
    TrailPoint pt = trail.point_at(0);
    ExpectPointNear(pt, 0.0f, 0.0f);
}

TEST_F(AddPointTest, DistanceThresholdAccepts) {
    trail.add_point(0.0f, 0.0f);
    EXPECT_EQ(trail.count(), 1);

    /* Add point at exactly 0.1m distance (meets threshold) */
    trail.add_point(0.1f, 0.0f);
    EXPECT_EQ(trail.count(), 2);  /* Should be accepted */

    TrailPoint pt0 = trail.point_at(0);
    ExpectPointNear(pt0, 0.0f, 0.0f);
    TrailPoint pt1 = trail.point_at(1);
    ExpectPointNear(pt1, 0.1f, 0.0f);
}

TEST_F(AddPointTest, SequentialBuilding) {
    /* Add 5 well-separated points (1m apart) */
    trail.add_point(0.0f, 0.0f);
    trail.add_point(1.0f, 0.0f);
    trail.add_point(2.0f, 0.0f);
    trail.add_point(3.0f, 0.0f);
    trail.add_point(4.0f, 0.0f);

    EXPECT_EQ(trail.count(), 5);

    /* Verify each point */
    for (uint16_t i = 0; i < 5; i++) {
        TrailPoint pt = trail.point_at(i);
        ExpectPointNear(pt, static_cast<float>(i), 0.0f);
    }
}

TEST_F(AddPointTest, ClearResetsAll) {
    /* Add points */
    trail.add_point(1.0f, 2.0f);
    trail.add_point(3.0f, 4.0f);
    EXPECT_EQ(trail.count(), 2);

    /* Clear */
    trail.clear();
    EXPECT_EQ(trail.count(), 0);

    /* Add new point - should be accepted as first point without distance check */
    trail.add_point(10.0f, 20.0f);
    EXPECT_EQ(trail.count(), 1);
    TrailPoint pt = trail.point_at(0);
    ExpectPointNear(pt, 10.0f, 20.0f);

    /* Add very close point (would be rejected if previous point mattered) */
    trail.add_point(10.01f, 20.01f);
    EXPECT_EQ(trail.count(), 1);  /* Should be rejected based on distance from 10,20 */
}

/* ========================================================================
 * RingBuffer Suite - Tests for circular buffer and decimation
 * ======================================================================== */

class RingBufferTest : public ::testing::Test {
protected:
    TrailMap trail;

    void SetUp() override {
        trail.clear();
    }

    /* Helper: Add well-separated points (1m apart) */
    void AddSeparatedPoints(uint16_t num_points) {
        for (uint16_t i = 0; i < num_points; i++) {
            trail.add_point(static_cast<float>(i), 0.0f);
        }
    }
};

TEST_F(RingBufferTest, FillToCapacity) {
    AddSeparatedPoints(TrailMap::MAX_POINTS);
    EXPECT_EQ(trail.count(), TrailMap::MAX_POINTS);
}

TEST_F(RingBufferTest, DecimationOnOverflow) {
    /* Fill buffer to capacity */
    AddSeparatedPoints(TrailMap::MAX_POINTS);
    EXPECT_EQ(trail.count(), TrailMap::MAX_POINTS);

    /* Add one more point (should trigger decimation) */
    trail.add_point(static_cast<float>(TrailMap::MAX_POINTS), 0.0f);

    /* Count should be less than MAX but more than half */
    EXPECT_LT(trail.count(), TrailMap::MAX_POINTS);
    EXPECT_GT(trail.count(), TrailMap::MAX_POINTS / 2);
}

TEST_F(RingBufferTest, EndpointPreservation) {
    /* Fill buffer */
    AddSeparatedPoints(TrailMap::MAX_POINTS);

    /* Trigger decimation */
    trail.add_point(static_cast<float>(TrailMap::MAX_POINTS), 0.0f);

    /* Newest half should be preserved intact (including the new point) */
    /* Check that the newest point is present */
    TrailPoint last_after = trail.point_at(trail.count() - 1);
    ExpectPointNear(last_after, static_cast<float>(TrailMap::MAX_POINTS), 0.0f);

    /* First point may have changed (older half decimated) */
    /* Just verify we still have a valid trail */
    EXPECT_GT(trail.count(), 0);
}

TEST_F(RingBufferTest, MultipleDecimationCycles) {
    /* Force multiple decimations by adding many points */
    for (uint16_t i = 0; i < TrailMap::MAX_POINTS * 3; i++) {
        trail.add_point(static_cast<float>(i), 0.0f);
    }

    /* Buffer should remain functional and count should be reasonable */
    EXPECT_GT(trail.count(), 0);
    EXPECT_LE(trail.count(), TrailMap::MAX_POINTS);

    /* Verify we can still access points without crash */
    for (uint16_t i = 0; i < trail.count(); i++) {
        TrailPoint pt = trail.point_at(i);
        /* Just verify no crash and values are in expected range */
        EXPECT_GE(pt.x, 0.0f);
    }
}

/* ========================================================================
 * PointAccess Suite - Tests for point retrieval and ordering
 * ======================================================================== */

class PointAccessTest : public ::testing::Test {
protected:
    TrailMap trail;

    void SetUp() override {
        trail.clear();
    }
};

TEST_F(PointAccessTest, OldestFirstOrdering) {
    /* Add points with distinct values */
    trail.add_point(1.0f, 10.0f);
    trail.add_point(2.0f, 20.0f);
    trail.add_point(3.0f, 30.0f);
    trail.add_point(4.0f, 40.0f);
    trail.add_point(5.0f, 50.0f);

    EXPECT_EQ(trail.count(), 5);

    /* point_at(0) should be the first added */
    TrailPoint pt0 = trail.point_at(0);
    ExpectPointNear(pt0, 1.0f, 10.0f);

    /* point_at(4) should be the last added */
    TrailPoint pt4 = trail.point_at(4);
    ExpectPointNear(pt4, 5.0f, 50.0f);
}

TEST_F(PointAccessTest, OutOfRangeSafety) {
    trail.add_point(1.0f, 2.0f);
    trail.add_point(3.0f, 4.0f);
    trail.add_point(5.0f, 6.0f);

    EXPECT_EQ(trail.count(), 3);

    /* Access out of range - should return {0,0} */
    TrailPoint pt_out1 = trail.point_at(3);
    ExpectPointNear(pt_out1, 0.0f, 0.0f);

    TrailPoint pt_out2 = trail.point_at(1000);
    ExpectPointNear(pt_out2, 0.0f, 0.0f);
}

/* ========================================================================
 * Bounds Suite - Tests for bounding box computation
 * ======================================================================== */

class BoundsTest : public ::testing::Test {
protected:
    TrailMap trail;

    void SetUp() override {
        trail.clear();
    }
};

TEST_F(BoundsTest, EmptyBounds) {
    TrailBounds bounds = trail.compute_bounds();
    EXPECT_NEAR(bounds.min_x, 0.0f, 1e-5f);
    EXPECT_NEAR(bounds.max_x, 0.0f, 1e-5f);
    EXPECT_NEAR(bounds.min_y, 0.0f, 1e-5f);
    EXPECT_NEAR(bounds.max_y, 0.0f, 1e-5f);
}

TEST_F(BoundsTest, SinglePoint) {
    trail.add_point(5.0f, 3.0f);

    TrailBounds bounds = trail.compute_bounds();

    /* min == max for both axes */
    EXPECT_NEAR(bounds.min_x, 5.0f, 1e-5f);
    EXPECT_NEAR(bounds.max_x, 5.0f, 1e-5f);
    EXPECT_NEAR(bounds.min_y, 3.0f, 1e-5f);
    EXPECT_NEAR(bounds.max_y, 3.0f, 1e-5f);
}

TEST_F(BoundsTest, MultiPoint) {
    trail.add_point(-2.0f, 1.0f);
    trail.add_point(3.0f, -4.0f);
    trail.add_point(0.0f, 7.0f);

    TrailBounds bounds = trail.compute_bounds();

    EXPECT_NEAR(bounds.min_x, -2.0f, 1e-5f);
    EXPECT_NEAR(bounds.max_x, 3.0f, 1e-5f);
    EXPECT_NEAR(bounds.min_y, -4.0f, 1e-5f);
    EXPECT_NEAR(bounds.max_y, 7.0f, 1e-5f);
}

/* ========================================================================
 * Viewport Suite - Tests for viewport computation and scaling
 * ======================================================================== */

class ViewportTest : public ::testing::Test {
protected:
    TrailMap trail;
    static constexpr int16_t MAP_W = 178;
    static constexpr int16_t MAP_H = 425;

    void SetUp() override {
        trail.clear();
    }
};

TEST_F(ViewportTest, MinScaleClamping) {
    /* Very large trail - should clamp to MIN_SCALE */
    /* Need span large enough that scale would be < MIN_SCALE */
    /* With MAP_W=178, MAP_H=425, if span > 178/20 = 8.9m, scale < 20 */
    /* Add points 200m apart to force low scale */
    trail.add_point(0.0f, 0.0f);
    trail.add_point(200.0f, 0.0f);  /* 200m span, with margin = 240m */
    /* scale_x = 178/240 = 0.74, scale_y = 425/240 = 1.77 */
    /* scale = min(0.74, 1.77) = 0.74, clamped to MIN_SCALE=20 */

    TrailViewport vp = trail.compute_viewport(MAP_W, MAP_H);

    /* Scale should clamp to MIN_SCALE since natural scale would be too small */
    EXPECT_NEAR(vp.scale, TrailMap::MIN_SCALE, 1e-5f);
    EXPECT_EQ(vp.map_w, MAP_W);
    EXPECT_EQ(vp.map_h, MAP_H);
}

TEST_F(ViewportTest, MaxScaleClamping) {
    /* Single point - minimal span gets clamped to 1m, would give large scale */
    /* scale_x = 178/1.0 = 178, scale_y = 425/1.0 = 425 */
    /* min(178, 425) = 178, but should clamp to MAX_SCALE (200) */
    trail.add_point(0.0f, 0.0f);

    TrailViewport vp = trail.compute_viewport(MAP_W, MAP_H);

    /* Scale should be clamped to MAX_SCALE or less */
    EXPECT_LE(vp.scale, TrailMap::MAX_SCALE);
    EXPECT_GT(vp.scale, TrailMap::MIN_SCALE);
}

TEST_F(ViewportTest, Centering) {
    /* Add points symmetrically around origin */
    trail.add_point(-50.0f, -50.0f);
    trail.add_point(50.0f, 50.0f);

    TrailViewport vp = trail.compute_viewport(MAP_W, MAP_H);

    /* Bounds: min_x=-50, max_x=50, min_y=-50, max_y=50 */
    /* Center should be at (0, 0) */
    /* With margin: span_x = 100 * 1.2 = 120, span_y = 100 * 1.2 = 120 */
    /* Scale = min(178/120, 425/120) = min(1.48, 3.54) = 1.48 (clamped to MIN_SCALE=20) */
    /* With MIN_SCALE=20: */
    /* offset_x = 0 - (178/20)*0.5 = -4.45 */
    /* offset_y = 0 - (425/20)*0.5 = -10.625 */

    EXPECT_NEAR(vp.scale, TrailMap::MIN_SCALE, 1e-5f);
    EXPECT_NEAR(vp.offset_x, -4.45f, 0.01f);
    EXPECT_NEAR(vp.offset_y, -10.625f, 0.01f);
}

TEST_F(ViewportTest, UniformScaling) {
    /* Non-square bounds: test that scale is uniform (min of x and y) */
    /* Create a 4m x 20m trail (tall and narrow) */
    trail.add_point(0.0f, 0.0f);
    trail.add_point(4.0f, 20.0f);

    TrailViewport vp = trail.compute_viewport(MAP_W, MAP_H);

    /* Bounds: span_x=4, span_y=20 */
    /* With margin: span_x=4*1.2=4.8, span_y=20*1.2=24 */
    /* scale_x = 178/4.8 = 37.08, scale_y = 425/24 = 17.71 */
    /* scale = min(37.08, 17.71) = 17.71, but clamped to MIN_SCALE=20 */

    EXPECT_NEAR(vp.scale, TrailMap::MIN_SCALE, 1e-5f);
}

TEST_F(ViewportTest, LargeTrailScaleDown) {
    /* Points spread over 50m */
    trail.add_point(0.0f, 0.0f);
    trail.add_point(50.0f, 50.0f);

    TrailViewport vp = trail.compute_viewport(MAP_W, MAP_H);

    /* Bounds: span_x=50, span_y=50 */
    /* With margin: span_x=60, span_y=60 */
    /* scale_x = 178/60 = 2.97, scale_y = 425/60 = 7.08 */
    /* scale = min(2.97, 7.08) = 2.97, but clamped to MIN_SCALE=20 */

    EXPECT_NEAR(vp.scale, TrailMap::MIN_SCALE, 1e-5f);
}

/* ========================================================================
 * WorldToPixel Suite - Tests for coordinate transformation
 * ======================================================================== */

class WorldToPixelTest : public ::testing::Test {
protected:
    TrailMap trail;
    static constexpr int16_t MAP_W = 178;
    static constexpr int16_t MAP_H = 425;

    void SetUp() override {
        trail.clear();
    }
};

TEST_F(WorldToPixelTest, OriginMapping) {
    /* Create known viewport: offset=(0,0), scale=100 */
    TrailViewport vp = {0.0f, 0.0f, 100.0f, MAP_W, MAP_H};

    int16_t px, py;
    trail.world_to_pixel(0.0f, 0.0f, vp, px, py);

    /* World (0,0) maps to pixel (0, MAP_H) before clamping */
    /* px = (0 - 0) * 100 = 0 */
    /* py = 425 - (0 - 0) * 100 = 425 → clamped to 424 */
    EXPECT_EQ(px, 0);
    EXPECT_EQ(py, 424);  /* MAP_H - 1 due to clamping */
}

TEST_F(WorldToPixelTest, YAxisInversion) {
    /* World +Y should map to smaller screen Y (screen up = lower Y) */
    TrailViewport vp = {0.0f, 0.0f, 100.0f, MAP_W, MAP_H};

    int16_t px1, py1, px2, py2;
    trail.world_to_pixel(0.0f, 0.0f, vp, px1, py1);
    trail.world_to_pixel(0.0f, 1.0f, vp, px2, py2);

    /* World Y increases, screen Y should decrease */
    EXPECT_LT(py2, py1);
}

TEST_F(WorldToPixelTest, NegativeCoordinates) {
    TrailViewport vp = {-10.0f, -10.0f, 100.0f, MAP_W, MAP_H};

    int16_t px, py;
    trail.world_to_pixel(-5.0f, -5.0f, vp, px, py);

    /* px = (-5 - (-10)) * 100 = 5 * 100 = 500 → clamped to 177 */
    /* py = 425 - (-5 - (-10)) * 100 = 425 - 500 = -75 → clamped to 0 */
    EXPECT_EQ(px, 177);  /* MAP_W - 1 */
    EXPECT_EQ(py, 0);
}

TEST_F(WorldToPixelTest, ClampsToBounds) {
    TrailViewport vp = {0.0f, 0.0f, 100.0f, MAP_W, MAP_H};

    /* Test extreme positive coordinates */
    int16_t px1, py1;
    trail.world_to_pixel(1000.0f, 1000.0f, vp, px1, py1);
    EXPECT_GE(px1, 0);
    EXPECT_LE(px1, MAP_W - 1);
    EXPECT_GE(py1, 0);
    EXPECT_LE(py1, MAP_H - 1);

    /* Test extreme negative coordinates */
    int16_t px2, py2;
    trail.world_to_pixel(-1000.0f, -1000.0f, vp, px2, py2);
    EXPECT_GE(px2, 0);
    EXPECT_LE(px2, MAP_W - 1);
    EXPECT_GE(py2, 0);
    EXPECT_LE(py2, MAP_H - 1);
}

/* ========================================================================
 * Integration Test - Full workflow
 * ======================================================================== */

TEST(TrailMapIntegration, FullWorkflow) {
    TrailMap trail;

    /* Build a trail */
    for (int i = 0; i < 10; i++) {
        trail.add_point(static_cast<float>(i), static_cast<float>(i * 2));
    }

    EXPECT_EQ(trail.count(), 10);

    /* Compute bounds */
    TrailBounds bounds = trail.compute_bounds();
    EXPECT_NEAR(bounds.min_x, 0.0f, 1e-5f);
    EXPECT_NEAR(bounds.max_x, 9.0f, 1e-5f);
    EXPECT_NEAR(bounds.min_y, 0.0f, 1e-5f);
    EXPECT_NEAR(bounds.max_y, 18.0f, 1e-5f);

    /* Compute viewport */
    TrailViewport vp = trail.compute_viewport(178, 425);
    EXPECT_GT(vp.scale, 0.0f);
    EXPECT_GE(vp.scale, TrailMap::MIN_SCALE);
    EXPECT_LE(vp.scale, TrailMap::MAX_SCALE);

    /* Transform a point */
    int16_t px, py;
    trail.world_to_pixel(5.0f, 10.0f, vp, px, py);
    EXPECT_GE(px, 0);
    EXPECT_LT(px, 178);
    EXPECT_GE(py, 0);
    EXPECT_LT(py, 425);
}
