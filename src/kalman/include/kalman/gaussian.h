#pragma once

namespace jp {

struct Gaussian {
    double mean = 0;
    double covariance = 0;
};

Gaussian operator*(const Gaussian &a, const Gaussian &b) {
    return {};
}

} // namespace jp
