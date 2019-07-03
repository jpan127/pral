#pragma once

#include "common/pi.h"

#include <cmath>

namespace pral {

struct Pose {
    double x = 0;
    double y = 0;
    double orientation = 0;
};

inline double truncate_radians(const double degrees) {
    const auto modulo = std::fmod(degrees, k2Pi);
    return (modulo < 0) ? modulo + k2Pi : modulo;
}

} // namespace pral
