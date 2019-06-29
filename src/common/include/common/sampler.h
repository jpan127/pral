#pragma once

#include <array>
#include <queue>
#include <random>
#include <type_traits>
#include <utility>
#include <vector>

namespace jp {

template <typename T, typename Distribution, size_t Samples = 1>
class Sampler {
  public:
    static_assert(Samples > 0);

    template <std::size_t N = Samples, std::enable_if_t<N == 1> * = nullptr>
    Sampler(const T min, const T max) : generator_(rd_()) {
        distributions_.emplace_back(min, max);
    }

    template <std::size_t N = Samples, std::enable_if_t<N != 1> * = nullptr>
    Sampler(const std::array<T, Samples> &mins,
            const std::array<T, Samples> &maxs) : generator_(rd_()) {
        distributions_.reserve(Samples);
        for (size_t ii = 0; ii < Samples; ii++) {
            distributions_.emplace_back(mins[ii], maxs[ii]);
        }
    }

    template <std::size_t N = Samples, std::enable_if_t<N == 1> * = nullptr>
    T sample() {
        return distributions_[0](generator_);
    }

    template <std::size_t N = Samples, std::enable_if_t<N != 1> * = nullptr>
    std::array<T, Samples> sample() {
        std::array<T, Samples> samples{};
        for (size_t ii = 0; ii < Samples; ii++) {
            samples[ii] = distributions_[ii](generator_);
        }
        return samples;
    }

  private:
    std::random_device rd_{};
    std::mt19937 generator_;
    std::vector<Distribution> distributions_;
};

template <size_t Samples = 1,
          typename Real = double,
          std::enable_if_t<std::is_floating_point_v<Real>> * = nullptr>
using RealSampler = Sampler<Real, std::uniform_real_distribution<Real>, Samples>;

template <size_t Samples = 1,
          typename Integer = size_t,
          std::enable_if_t<std::is_integral_v<Integer>> * = nullptr>
using IntSampler = Sampler<Integer, std::uniform_int_distribution<Integer>, Samples>;

template <typename T, std::enable_if_t<std::is_floating_point_v<T>> * = nullptr>
T random(const T max) {
    RealSampler<1> sampler(0, max - std::numeric_limits<double>::epsilon());
    return sampler.sample();
}

template <typename T, std::enable_if_t<std::is_integral_v<T>> * = nullptr>
T random(const T max) {
    IntSampler<1> sampler(0, max - 1);
    return sampler.sample();
}

/// An efficient way to resample the indices of a array without creating a new array
/// \param indices Array of indices to be chosen from [v], with or without replacement
/// \param v       Array to be modified
template <typename T, std::size_t N>
void resampling_in_place(std::array<size_t, N> &indices, std::array<T, N> &container) {
    assert(!indices.empty());
    assert(indices.size() == container.size());
    assert(indices.back() < container.size());

    // Sort indices so it counts upwards consecutively
    std::sort(indices.begin(), indices.end());

    std::queue<size_t> replaceables;        /// Indices to be overwritten
    std::queue<size_t> pending_replacement; /// Indices to be copied to a new index

    auto iterator = indices.cbegin();
    for (size_t ii = 0; ii < container.size(); ii++) {
        // No more sampled indices, the rest of the indices can be replaced
        if (iterator == indices.cend()) {
            for (size_t jj = ii; jj < container.size(); jj++) {
                replaceables.push(jj);
            }
            break;
        }

        auto current_sampled_index = *iterator;

        if (current_sampled_index == ii) {
            // Same as array index, which means first occurrence of this sampled index
            // Leave as is in the array and continue
            iterator++;
        } else if (current_sampled_index < ii) {
            while (iterator != indices.cend() && current_sampled_index < ii) {
                // Duplicate found, try to insert in previous index, otherwise pend replacement
                pending_replacement.push(current_sampled_index);
                iterator++;
                current_sampled_index = *iterator;
            }

            if (current_sampled_index == ii) {
                iterator++;
            } else {
                replaceables.push(ii);
            }
        } else {
            // Index is not sampled, mark for replacement
            // Don't advance iterator because its index has not been handled yet
            replaceables.push(ii);
        }
    }

    // If there is anything left in [indices] they must be duplicates of the last index
    while (iterator != indices.cend()) {
        pending_replacement.push(*iterator);
        iterator++;
    }

    assert(pending_replacement.size() == replaceables.size());

    // Add all objects pending replacement
    while (!pending_replacement.empty()) {
        const auto to_be_replaced_index = replaceables.front();
        const auto to_be_copied_index = pending_replacement.front();
        replaceables.pop();
        pending_replacement.pop();
        container[to_be_replaced_index] = container[to_be_copied_index];
    }
}

} // namespace jp
