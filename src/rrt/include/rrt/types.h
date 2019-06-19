#pragma once

#include "rrt/utils.h"

#include <cstdint>
#include <cmath>

namespace jp {

struct Point {
    double x = 0;
    double y = 0;
};

struct Line {
    Point a{};
    Point b{};
};

struct Rectangle {
    Point bottom_left{};
    Point top_right{};
};

inline bool operator==(const Point &a, const Point &b) {
    return equal(a.x, b.x) && equal(a.y, b.y);
}

inline bool operator!=(const Point &a, const Point &b) {
    return !(operator==(a, b));
}

inline bool intersects(const Line &a, const Line &b) {
    enum class Orientation { Left, Right, Collinear };

    auto orientation = [](const auto &a, const auto &b, const auto &c) {
        const auto area = (b.x - a.x) * (c.y - a.y) -
                          (c.x - a.x) * (b.y - a.y);
        if (equal(area, 0)) {
            return Orientation::Collinear;
        } else if (area > 0) {
            return Orientation::Left;
        } else {
            return Orientation::Right;
        }
    };

    const auto orientation1 = orientation(a.a, a.b, b.a);
    const auto orientation2 = orientation(a.a, a.b, b.b);
    const bool intersected  = (orientation1 == Orientation::Left  && orientation2 == Orientation::Right) ||
                              (orientation1 == Orientation::Right && orientation2 == Orientation::Left);
    return intersected;
}

inline bool intersects(const Line &a, const Rectangle &b) {
    const Line left{b.bottom_left, {b.bottom_left.x, b.top_right.y}};
    const Line top{{b.bottom_left.x, b.top_right.y}, b.top_right};
    const Line right{b.top_right, {b.top_right.x, b.bottom_left.y}};
    const Line bottom{{b.top_right.x, b.bottom_left.x}, b.bottom_left};
    return intersects(a, left) || intersects(a, top) || intersects(a, right) || intersects(a, bottom);
}

inline bool in_bounding_box(const Point &p, const Rectangle &rectangle) {
    const bool x_inside = (p.x >= rectangle.bottom_left.x) && (p.x <= rectangle.top_right.x);
    const bool y_inside = (p.y >= rectangle.bottom_left.y) && (p.y <= rectangle.top_right.y);
    return x_inside && y_inside;
}

inline double point_distance2(const Point &a, const Point &b) {
    return (b.x - a.x) * (b.x - a.x) +
           (b.y - a.y) * (b.y - a.y);
}

inline double point_distance(const Point &a, const Point &b) {
    return std::sqrt(point_distance2(a, b));
}

} // namespace jp
