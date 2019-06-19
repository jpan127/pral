#pragma once

#include "rrt/types.h"

#include <random>

namespace jp {

class Sampler {
  public:
    Sampler(const double min_x, const double max_x,
            const double min_y, const double max_y) :
            generator_(rd_()),
            x_distribution_(min_x, max_x),
            y_distribution_(min_y, max_y) {
    }

    Point sample() {
        return {
            .x = x_distribution_(generator_),
            .y = y_distribution_(generator_),
        };
    }

  private:
    std::random_device rd_{};
    std::mt19937 generator_;
    std::uniform_real_distribution<double> x_distribution_;
    std::uniform_real_distribution<double> y_distribution_;
};

} // namespace jp
