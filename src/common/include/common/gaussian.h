#pragma once

#include "common/pi.h"
#include "common/sampler.h"

#include <cassert>
#include <numeric>
#include <random>

namespace pral {

class Gaussian : public Sampler<double, std::normal_distribution<double>, 1> {
  public:
    Gaussian(const double mean, const double standard_deviation) :
        Sampler(mean, standard_deviation) {}
};

inline double random_gaussian(const double mean, const double standard_deviation) {
    Gaussian g(mean, standard_deviation);
    return g.sample();
}

inline double univariate_gausian_pdf(const double value, const double mean, const double standard_deviation) {
    assert(standard_deviation > std::numeric_limits<double>::epsilon());

    auto square = [](const auto x) { return x * x; };
    const auto variance = square(standard_deviation);
    const auto exponent = (-0.5) * square(value - mean) / variance;
    const auto e_exponent = std::exp(exponent);
    const auto normalizing_constant = 1.0 / std::sqrt(k2Pi * variance);
    return normalizing_constant * e_exponent;
}

} // namespace pral
