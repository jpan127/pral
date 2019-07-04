#include <iostream>
#include "catch.hpp"

#include "common/sampler.h"
#include "kd_tree/kd_tree.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <unordered_set>

using namespace pral;
using KDT = KDTree<3, size_t>;

constexpr size_t kUpperBound{100};
constexpr size_t kLowerBound{000};
constexpr size_t kNumPoints{20};
constexpr std::array<KDT::Coordinates, kNumPoints> kValues{{
    { 31 , 28 , 33 },
    { 99 , 22 , 98 },
    { 79 , 18 , 72 },
    { 91 , 67 , 95 },
    { 25 , 90 , 47 },
    { 53 , 68 , 83 },
    { 89 , 99 , 79 },
    { 11 , 60 , 7  },
    { 1  , 8  , 53 },
    { 44 , 15 , 44 },
    { 56 , 45 , 40 },
    { 40 , 53 , 76 },
    { 76 , 17 , 25 },
    { 83 , 66 , 89 },
    { 95 , 67 , 1  },
    { 47 , 86 , 56 },
    { 33 , 73 , 11 },
    { 98 , 39 , 31 },
    { 7  , 9  , 91 },
    { 72 , 44 , 99 },
}};

double point_distance(const KDT::Coordinates &a, const KDT::Coordinates &b) {
    double distance = 0;
    distance += (a[0] - b[0]) * (a[0] - b[0]);
    distance += (a[1] - b[1]) * (a[1] - b[1]);
    distance += (a[2] - b[2]) * (a[2] - b[2]);
    return distance;
}

TEST_CASE("3D", "[KDTree]") {
    static constexpr size_t kMiddleBound = (kUpperBound - kLowerBound) / 2;
    static constexpr KDT::Coordinates kMiddlePoint{{kMiddleBound, kMiddleBound}};

    auto print_points = [](const auto &points) {
        using namespace std;
        cout << "-------------------------\n";
        cout << "Failed with these points:\n";
        for (const auto &point : points) {
            cout << point[0] << "," << point[1] << "," << point[2] << endl;
        }
        cout << "-------------------------\n";
    };

    auto execute = [](auto &kdt, const auto &points) {
        // Calculate sqaured distances the middle point has from every other point
        std::array<std::pair<KDT::Coordinates, double>, kNumPoints> distances{};
        for (size_t ii = 0; ii < kNumPoints; ii++) {
            const auto &point = points[ii];
            auto &pair = distances[ii];
            auto &distance = pair.second;

            pair.first = point;
            distance = point_distance(point, kMiddlePoint);
        }

        std::sort(distances.begin(), distances.end(), [](auto &a, auto &b) { return a.second < b.second; });

        for (size_t num_neighbors = 1; num_neighbors < kNumPoints + 1; num_neighbors++) {
            const auto knn = kdt.knn(kMiddlePoint, num_neighbors);
            const bool correct = std::equal(
                knn.begin(),
                knn.end(),
                distances.begin(),
                [](auto &a, auto &b) { return a == b.first; }
            );

            if (!correct) {
                CHECK(correct);
                return false;
            }
        }

        return true;
    };

    SECTION("Deterministic") {
        KDT kdt(kValues.begin(), kValues.end());
        execute(kdt, kValues);
    }

    SECTION("Nondeterministic") {
        IntSampler<> sampler(kLowerBound, kUpperBound);
        std::array<KDT::Coordinates, kNumPoints> samples{};
        std::unordered_set<double> distances;
        for (auto &sample : samples) {
            // Make sure that all samples are different distances away
            // Otherwise validating the neighbors requires the correct order
            while (true) {
                const KDT::Coordinates point{{
                    sampler.sample(),
                    sampler.sample(),
                    sampler.sample(),
                }};
                const auto distance = point_distance(point, kMiddlePoint);
                if (distances.find(distance) == distances.end()) {
                    sample = point;
                    distances.insert(distance);
                    break;
                }
            }
        }

        KDT kdt(samples.begin(), samples.end());
        if (!execute(kdt, samples)) {
            print_points(samples);
        }
    }
}
