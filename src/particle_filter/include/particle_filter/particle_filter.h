#pragma once

#include "common/gaussian.h"
#include "common/pose.h"
#include "common/sampler.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <numeric>

namespace pral {

template <size_t NumParticles, size_t NumMeasurements>
class ParticleFilter {
    static constexpr uint8_t kNumSamples = 3;

  public:
    using Particle = Pose;

    /// Parameters
    struct Params {
        double min_x = 0;
        double max_x = 0;
        double min_y = 0;
        double max_y = 0;
        double orientation_noise = 0;
        double distance_noise = 0;
        double bearing_noise = 0;
        double ego_length = 0;
    };

    /// Constructor
    ParticleFilter(const Params &params) : params_(params) {
        // Samples random values for each particle
        RealSampler<kNumSamples> sampler(
            std::array<double, kNumSamples>{params_.min_x, params_.min_y,    0},
            std::array<double, kNumSamples>{params_.max_x, params_.max_y, k2Pi}
        );

        // Initialize particles with random poses
        for (size_t ii = 0; ii < NumParticles; ii++) {
            const auto samples = sampler.sample();
            particles_[ii] = {
                .x = samples[0],
                .y = samples[1],
                .orientation = samples[2],
            };
        }
    }

    /// Updates the filter
    /// \param real_measurements A set of measurements taken from sensor(s)
    /// \param orientation       New orientation of the ego
    /// \param distance          Distance the ego expected to have traveled
    void step(const std::array<double, NumMeasurements> &real_measurements, const double orientation, const double distance) {
        predict(orientation, distance);
        update(real_measurements);
        resample();
    }

    const std::array<Particle, NumParticles> &particles() const {
        return particles_;
    }

    Particle estimate() const {
        Particle estimated{};
        for (const auto &particle : particles_) {
            estimated.x += particle.x;
            estimated.y += particle.y;
            estimated.orientation += truncate_radians(particle.orientation - particles_[0].orientation + kPi);
            estimated.orientation += particles_[0].orientation - kPi;
        }

        estimated.x /= NumParticles;
        estimated.y /= NumParticles;
        estimated.orientation /= NumParticles;
        return estimated;
    }

  protected:
    /// Current set of parameters
    const Params params_;

    /// Container of existing particles
    std::array<Particle, NumParticles> particles_{};

    /// Container of weights of each particle
    std::array<double, NumParticles> weights_{};

    /// Allow particles to localize with known objects
    virtual std::array<double, NumMeasurements> sense(const Particle &particle, const bool noisy = false) const = 0;

    /// Simpler model in which the particle changes orientation then moves straight by the distance
    void simple_dynamic_model(const double orientation, const double distance, Particle &particle) const {
        const auto orientation_with_noise = orientation + random_gaussian(0, params_.orientation_noise);
        const auto distance_with_noise = distance + random_gaussian(0, params_.distance_noise);
        particle.orientation += orientation_with_noise;
        particle.orientation = truncate_radians(particle.orientation);
        particle.y += std::sin(particle.orientation) * distance_with_noise;
        particle.x += std::cos(particle.orientation) * distance_with_noise;
    }

    /// A more complex model in which the particle pivots against a turning radius
    /// with respect to the length of the ego
    void better_dynamic_model(const double orientation, const double distance, Particle &particle) const {
        const auto orientation_with_noise = orientation + random_gaussian(0, params_.orientation_noise);
        const auto distance_with_noise = distance + random_gaussian(0, params_.distance_noise);
        const auto beta = std::tan(orientation_with_noise) * distance_with_noise / params_.ego_length;

        if (beta < 0.001) {
            particle.x += distance_with_noise * std::cos(particle.orientation);
            particle.y += distance_with_noise * std::sin(particle.orientation);
        } else {
            const auto turning_radius = distance_with_noise / beta;
            const auto cx = particle.x - std::sin(particle.orientation) * turning_radius;
            const auto cy = particle.y + std::cos(particle.orientation) * turning_radius;
            particle.x = cx + std::sin(particle.orientation + beta) * turning_radius;
            particle.y = cy - std::cos(particle.orientation + beta) * turning_radius;
        }

        particle.orientation += beta;
        particle.orientation = truncate_radians(particle.orientation);
    }

    /// With the given orientation and distance the ego has expected to have moved, move each particle with the same dynamics
    void predict(const double orientation, const double distance) {
        for (auto &particle : particles_) {
            simple_dynamic_model(orientation, distance, particle);
        }
    }

    /// For each particle, localize with known objects, then calculate the error between those measurements and the
    /// real measurements from the sensors, and multiply each error together to form the weight / probability of each particle
    void update(const std::array<double, NumMeasurements> &real_measurements) {
        constexpr bool kWithoutNoise{true};

        for (size_t ii = 0; ii < NumParticles; ii++) {
            const auto &particle = particles_[ii];
            auto &weight = weights_[ii];
            weight = 1.0;

            // Calculate the bearings of each particle to each known object
            const auto correct_measurements = sense(particle, kWithoutNoise);

            for (std::size_t ii = 0; ii < correct_measurements.size(); ii++) {
                const auto correct = correct_measurements[ii];
                const auto real = real_measurements[ii];
                const auto error = truncate_radians(std::abs(real - correct));
                const auto multiplier = univariate_gausian_pdf(error, 0, params_.bearing_noise);
                weight *= multiplier;
            }
        }
    }

    /// Resample NumParticles particles based on their weight using Thrun's low variable sampling with replacement algorithm
    void resample() {
        const auto max_weight = *(std::max_element(weights_.cbegin(), weights_.cend()));
        std::array<size_t, NumParticles> indices{};
        size_t index = random(NumParticles);
        double beta = 0;

        // Sample indices to keep
        for (size_t ii = 0; ii < NumParticles; ii++) {
            beta += random(2 * max_weight);
            while (beta > weights_[index]) {
                beta -= weights_[index];
                index = (index + 1) % NumParticles;
            }

            indices[ii] = index;
        }

        // Modify [particles_] in place
        resampling_in_place(indices, particles_);
    }
};

} // namespace pral
