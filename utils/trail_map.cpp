/*
 * TrailMap - Breadcrumb trail with auto-scaling viewport
 * Implementation
 */

#include "utils/trail_map.hpp"
#include <algorithm>

void TrailMap::add_point(float wx, float wy) {
    if (count_ == 0) {
        points_[head_] = {wx, wy};
        head_ = (head_ + 1) % MAX_POINTS;
        count_ = 1;
        last_x_ = wx;
        last_y_ = wy;
        return;
    }

    float dx = wx - last_x_;
    float dy = wy - last_y_;
    float dist_sq = dx * dx + dy * dy;

    if (dist_sq < MIN_DISTANCE_SQ) {
        return;
    }

    if (count_ == MAX_POINTS) {
        decimate();
    }

    points_[head_] = {wx, wy};
    head_ = (head_ + 1) % MAX_POINTS;
    if (count_ < MAX_POINTS) {
        count_++;
    }
    last_x_ = wx;
    last_y_ = wy;
}

TrailBounds TrailMap::compute_bounds() const {
    if (count_ == 0) {
        return {0.0f, 0.0f, 0.0f, 0.0f};
    }

    TrailPoint first = point_at(0);
    float min_x = first.x;
    float max_x = first.x;
    float min_y = first.y;
    float max_y = first.y;

    for (uint16_t i = 1; i < count_; i++) {
        TrailPoint pt = point_at(i);
        min_x = std::min(min_x, pt.x);
        max_x = std::max(max_x, pt.x);
        min_y = std::min(min_y, pt.y);
        max_y = std::max(max_y, pt.y);
    }

    return {min_x, max_x, min_y, max_y};
}

TrailViewport TrailMap::compute_viewport(int16_t map_w, int16_t map_h) const {
    TrailBounds bounds = compute_bounds();

    float span_x = bounds.max_x - bounds.min_x;
    float span_y = bounds.max_y - bounds.min_y;

    span_x += span_x * MARGIN_FACTOR * 2.0f;
    span_y += span_y * MARGIN_FACTOR * 2.0f;

    if (span_x < 1.0f) {
        span_x = 1.0f;
    }
    if (span_y < 1.0f) {
        span_y = 1.0f;
    }

    float scale_x = static_cast<float>(map_w) / span_x;
    float scale_y = static_cast<float>(map_h) / span_y;
    float scale = std::min(scale_x, scale_y);

    scale = std::max(MIN_SCALE, std::min(MAX_SCALE, scale));

    float center_x = (bounds.min_x + bounds.max_x) * 0.5f;
    float center_y = (bounds.min_y + bounds.max_y) * 0.5f;

    float offset_x = center_x - (static_cast<float>(map_w) / scale) * 0.5f;
    float offset_y = center_y - (static_cast<float>(map_h) / scale) * 0.5f;

    return {offset_x, offset_y, scale, map_w, map_h};
}

void TrailMap::world_to_pixel(float wx, float wy, const TrailViewport &vp,
                               int16_t &px_out, int16_t &py_out) const {
    float px_f = (wx - vp.offset_x) * vp.scale;
    float py_f = static_cast<float>(vp.map_h) - (wy - vp.offset_y) * vp.scale;

    int16_t px = static_cast<int16_t>(px_f);
    int16_t py = static_cast<int16_t>(py_f);

    px_out = std::max(static_cast<int16_t>(0), std::min(px, static_cast<int16_t>(vp.map_w - 1)));
    py_out = std::max(static_cast<int16_t>(0), std::min(py, static_cast<int16_t>(vp.map_h - 1)));
}

void TrailMap::decimate() {
    if (count_ == 0) {
        return;
    }

    uint16_t half_count = count_ / 2;
    TrailPoint temp[MAX_POINTS];
    uint16_t write_idx = 0;

    for (uint16_t i = 0; i < half_count; i += 2) {
        temp[write_idx++] = point_at(i);
    }

    for (uint16_t i = half_count; i < count_; i++) {
        temp[write_idx++] = point_at(i);
    }

    for (uint16_t i = 0; i < write_idx; i++) {
        points_[i] = temp[i];
    }

    head_ = write_idx % MAX_POINTS;
    count_ = write_idx;
}

TrailPoint TrailMap::point_at(uint16_t logical_index) const {
    if (logical_index >= count_) {
        return {0.0f, 0.0f};
    }

    uint16_t ring_pos = ring_index(logical_index);
    return points_[ring_pos];
}

uint16_t TrailMap::ring_index(uint16_t logical) const {
    return (head_ + MAX_POINTS - count_ + logical) % MAX_POINTS;
}

void TrailMap::clear() {
    head_ = 0;
    count_ = 0;
    last_x_ = 0.0f;
    last_y_ = 0.0f;
}
