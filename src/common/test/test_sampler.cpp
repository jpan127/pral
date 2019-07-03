#include "catch.hpp"

#include "common/sampler.h"

using namespace pral;

constexpr size_t kNumSamples = 100;

TEST_CASE("Real", "Sampler") {
    constexpr double kMax{999'999.999};

    SECTION("Object") {
        RealSampler<1> s(0, kMax);
        std::vector<double> samples;
        for (size_t ii = 0; ii < kNumSamples; ii++) {
            samples.push_back(s.sample());
        }
        std::sort(samples.begin(), samples.end());
        REQUIRE(std::adjacent_find(samples.begin(), samples.end()) == samples.end());
    }

    SECTION("FreeFunction") {
        std::vector<double> samples;
        for (size_t ii = 0; ii < kNumSamples; ii++) {
            samples.push_back(random(kMax));
        }
        std::sort(samples.begin(), samples.end());
        REQUIRE(std::adjacent_find(samples.begin(), samples.end()) == samples.end());
    }
}

TEST_CASE("Int", "Sampler") {
    constexpr double kMax{999'999'999};

    SECTION("Object") {
        IntSampler<1> s(0, kMax);
        std::vector<size_t> samples;
        for (size_t ii = 0; ii < kNumSamples; ii++) {
            samples.push_back(s.sample());
        }
        std::sort(samples.begin(), samples.end());
        REQUIRE(std::adjacent_find(samples.begin(), samples.end()) == samples.end());
    }

    SECTION("FreeFunction") {
        std::vector<size_t> samples;
        for (size_t ii = 0; ii < kNumSamples; ii++) {
            samples.push_back(random(kMax));
        }
        std::sort(samples.begin(), samples.end());
        REQUIRE(std::adjacent_find(samples.begin(), samples.end()) == samples.end());
    }
}

TEST_CASE("Resampling", "Sampler") {
    constexpr size_t N{10};

    std::array<size_t, N> values{{0, 1, 2, 3, 4,
                                  5, 6, 7, 8, 9}};

    SECTION("A") {
        constexpr std::array<size_t, N> kCorrect{{1, 1, 1, 2, 7,
                                                  7, 7, 7, 7, 9}};

        std::array<size_t, N> indices = kCorrect;

        resampling_in_place(indices, values);

        std::sort(values.begin(), values.end());
        REQUIRE(std::equal(values.begin(), values.end(), kCorrect.begin()));
    }

    SECTION("B") {
        constexpr std::array<size_t, N> kCorrect{{0, 1, 2, 3, 4,
                                                  5, 6, 7, 8, 9}};

        std::array<size_t, N> indices = kCorrect;

        resampling_in_place(indices, values);

        std::sort(values.begin(), values.end());
        REQUIRE(std::equal(values.begin(), values.end(), kCorrect.begin()));
    }

    SECTION("C") {
        constexpr std::array<size_t, N> kCorrect{{0, 0, 0, 0, 0,
                                                  0, 0, 0, 0, 0}};
        std::array<size_t, N> indices = kCorrect;

        resampling_in_place(indices, values);

        std::sort(values.begin(), values.end());
        REQUIRE(std::equal(values.begin(), values.end(), kCorrect.begin()));
    }

    SECTION("D") {
        constexpr std::array<size_t, N> kCorrect{{9, 9, 9, 9, 9,
                                                  9, 9, 9, 9, 9}};
        std::array<size_t, N> indices = kCorrect;

        resampling_in_place(indices, values);

        std::sort(values.begin(), values.end());
        REQUIRE(std::equal(values.begin(), values.end(), kCorrect.begin()));
    }
}
