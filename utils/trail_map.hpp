/*
 * TrailMap - Breadcrumb trail with auto-scaling viewport
 * Pure math/data layer, no LVGL dependency
 */

#ifndef TRAIL_MAP_HPP
#define TRAIL_MAP_HPP

#include <cstdint>

struct TrailPoint {
    float x;
    float y;
};

struct TrailBounds {
    float min_x;
    float max_x;
    float min_y;
    float max_y;
};

struct TrailViewport {
    float offset_x;   /* World X of pixel (0,0) */
    float offset_y;   /* World Y of pixel (0, map_h) — bottom of screen */
    float scale;       /* Pixels per meter */
    int16_t map_w;
    int16_t map_h;
};

class TrailMap {
public:
    static constexpr uint16_t MAX_POINTS = 256;
    static constexpr float MIN_DISTANCE_SQ = 0.01f;   /* 0.1m squared */
    static constexpr float MIN_SCALE = 20.0f;          /* px/m */
    static constexpr float MAX_SCALE = 200.0f;         /* px/m */
    static constexpr float MARGIN_FACTOR = 0.10f;      /* 10% margin */

    void add_point(float wx, float wy);
    TrailBounds compute_bounds() const;
    TrailViewport compute_viewport(int16_t map_w, int16_t map_h) const;

    /* Map world coords to pixel coords. Y-axis inverted: world +Y = screen up */
    void world_to_pixel(float wx, float wy, const TrailViewport &vp,
                        int16_t &px_out, int16_t &py_out) const;

    void decimate();
    TrailPoint point_at(uint16_t logical_index) const;
    uint16_t count() const { return count_; }
    void clear();

private:
    TrailPoint points_[MAX_POINTS] = {};
    uint16_t head_ = 0;       /* Next write index */
    uint16_t count_ = 0;      /* Number of stored points */
    float last_x_ = 0.0f;
    float last_y_ = 0.0f;

    uint16_t ring_index(uint16_t logical) const;
};

#endif /* TRAIL_MAP_HPP */
