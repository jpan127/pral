#pragma once

#include <cstdint>
#include <cmath>

namespace jp {

inline bool equal(const double a, const double b) {
    constexpr double kEpsilon = 1e-10;
    return (std::abs(a - b) <= kEpsilon);
}

} // namespace jp
