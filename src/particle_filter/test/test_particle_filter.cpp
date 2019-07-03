#include "catch.hpp"

#include "particle_filter/particle_filter.h"

#include <fstream>

using namespace pral;

constexpr size_t kNumParticles = 1000;
constexpr size_t kNumMeasurements = 4;
class ParticleFilter2D : public ParticleFilter<kNumParticles, kNumMeasurements> {
  public:
    ParticleFilter2D(const Params &params) : ParticleFilter(params) {}

    std::array<double, kNumMeasurements> sense(const Particle &particle, const bool noisy = false) const override {
        static constexpr std::array<Pose, kNumMeasurements> kLandmarks{{
            {.y =   0, .x = 100},
            {.y =   0, .x =   0},
            {.y = 100, .x =   0},
            {.y = 100, .x = 100},
        }};

        std::array<double, kNumMeasurements> bearings{};
        for (size_t ii = 0; ii < kNumMeasurements; ii++) {
            const auto &landmark = kLandmarks[ii];
            bearings[ii] = std::atan2(landmark.y - particle.y, landmark.x - particle.x) - particle.orientation;
            if (noisy) {
                bearings[ii] += random_gaussian(0, params_.bearing_noise);
            }
            bearings[ii] = truncate_radians(bearings[ii]);
        }

        return bearings;
    }
};

TEST_CASE("2D", "ParticleFilter") {
    ParticleFilter2D pf({
        .min_x = 0,
        .max_x = 100,
        .min_y = 0,
        .max_y = 100,
        .orientation_noise = 0.1,
        .distance_noise = 5,
        .bearing_noise = 0.1,
        .ego_length = 20,
    });

    constexpr size_t kNumSteps = 8;

    static constexpr std::array<std::array<double, kNumMeasurements>, kNumSteps> kMeasurements{{
        {{ 4.746936, 3.859782, 3.045217, 2.045506 }},
        {{ 3.510067, 2.916300, 2.146394, 1.598332 }},
        {{ 2.972469, 2.407489, 1.588474, 1.611094 }},
        {{ 1.906178, 1.193329, 0.619356, 0.807930 }},
        {{ 1.352825, 0.662233, 0.144927, 0.799090 }},
        {{ 0.856150, 0.214590, 5.651497, 1.062401 }},
        {{ 0.194460, 5.660382, 4.761072, 2.471682 }},
        {{ 5.717342, 4.736780, 3.909599, 2.342536 }},
    }};

    static constexpr std::array<std::pair<double, double>, kNumSteps> kMotions{{
        { k2Pi / 10 , 20 },
        { k2Pi / 10 , 20 },
        { k2Pi / 10 , 20 },
        { k2Pi / 10 , 20 },
        { k2Pi / 10 , 20 },
        { k2Pi / 10 , 20 },
        { k2Pi / 10 , 20 },
        { k2Pi / 10 , 20 },
    }};

    auto export_particles = [&](const auto path) {
        std::ofstream csv(path);
        const auto &particles = pf.particles();
        for (const auto &particle : particles) {
            csv << particle.x << "," << particle.y << std::endl;
        }
    };

    export_particles("build/test_particle_filter_before.txt");

    for (size_t ii = 0; ii < kNumSteps; ii++) {
        const auto &measurements = kMeasurements[ii];
        const auto &motions = kMotions[ii];
        const auto orientation = motions.first;
        const auto distance = motions.second;
        pf.step(measurements, orientation, distance);
    }

    pf.estimate();

    export_particles("build/test_particle_filter_after.txt");
}
