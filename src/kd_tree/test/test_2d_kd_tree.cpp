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
using KDT = KDTree<2, size_t>;

constexpr size_t kUpperBound{100};
constexpr size_t kLowerBound{000};
constexpr size_t kNumPoints{60};
constexpr std::array<KDT::Coordinates, kNumPoints> kValues{{
    { 1  , 8  },
    { 7  , 9  },
    { 11 , 60 },
    { 25 , 90 },
    { 31 , 28 },
    { 33 , 73 },
    { 40 , 53 },
    { 44 , 15 },
    { 47 , 86 },
    { 53 , 68 },
    { 56 , 45 },
    { 72 , 44 },
    { 76 , 17 },
    { 79 , 18 },
    { 83 , 66 },
    { 89 , 99 },
    { 91 , 67 },
    { 95 , 67 },
    { 98 , 39 },
    { 99 , 22 },

    { 53 , 68 },
    { 91 , 67 },
    { 7  , 9  },
    { 47 , 86 },
    { 33 , 73 },
    { 11 , 60 },
    { 76 , 17 },
    { 44 , 15 },
    { 56 , 45 },
    { 83 , 66 },
    { 40 , 53 },
    { 99 , 22 },
    { 25 , 90 },
    { 72 , 44 },
    { 89 , 99 },
    { 79 , 18 },
    { 95 , 67 },
    { 1  , 8  },
    { 31 , 28 },
    { 98 , 39 },

    { 83  , 9  },
    { 51  , 21 },
    { 14  , 4  },
    { 75  , 15 },
    { 41  , 77 },
    { 85  , 65 },
    { 67  , 66 },
    { 20  , 39 },
    { 82  , 2  },
    { 13  , 86 },
    { 35  , 76 },
    { 94  , 44 },
    { 3   , 82 },
    { 94  , 58 },
    { 19  , 80 },
    { 99  , 40 },
    { 40  , 20 },
    { 100 , 11 },
    { 56  , 65 },
    { 74  , 22 },
}};

void to_file(const KDT &tree, const std::string &fname) {
    std::filesystem::create_directory("data");

    std::ofstream file(fname);

    const auto &root = tree.root();
    if (!root) {
        return;
    }

    std::queue<const KDT::Node *> q;
    q.push(root.get());

    while (!q.empty()) {
        const auto current = q.front();
        q.pop();

        file << current->point[0]  << ","
             << current->point[1]  << ","
             << current->dimension << "\n";

        if (current->left) {
            q.push(current->left.get());
        }

        if (current->right) {
            q.push(current->right.get());
        }
    }
}

double point_distance(const KDT::Coordinates &a, const KDT::Coordinates &b) {
    double distance = 0;
    distance += (a[0] - b[0]) * (a[0] - b[0]);
    distance += (a[1] - b[1]) * (a[1] - b[1]);
    return distance;
}

TEST_CASE("2D", "[KDTree]") {
    static constexpr size_t kMiddleBound = (kUpperBound - kLowerBound) / 2;
    static constexpr KDT::Coordinates kMiddlePoint{{kMiddleBound, kMiddleBound}};

    auto create = [](const auto &points, const bool dynamic) {
        if (dynamic) {
            return KDT(points.begin(), points.end());
        }

        KDT kdt;
        for (const auto &point : points) {
            kdt.insert({{point[0], point[1]}});
        }
        kdt.rebuild();
        return kdt;
    };

    auto print_points = [](const auto &points) {
        using namespace std;
        cout << "-------------------------\n";
        cout << "Failed with these points:\n";
        for (const auto &point : points) {
            cout << point[0] << "," << point[1] << endl;
        }
        cout << "-------------------------\n";
    };

    auto execute = [](auto &kdt, const auto &points) {
        to_file(kdt, "data/2d_kdtree.txt");

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
        SECTION("Static") {
            auto kdt = create(kValues, false);
            execute(kdt, kValues);
        }
        SECTION("Dynamic") {
            auto kdt = create(kValues, true);
            execute(kdt, kValues);
        }
    }

    SECTION("Nondeterministic") {
        IntSampler<> sampler(kLowerBound, kUpperBound);
        std::array<KDT::Coordinates, kNumPoints> samples{};
        std::unordered_set<double> distances;
        for (auto &sample : samples) {
            // Make sure that all samples are different distances away
            // Otherwise validating the neighbors requires the correct order
            while (true) {
                const KDT::Coordinates point{{sampler.sample(),sampler.sample()}};
                const auto distance = point_distance(point, kMiddlePoint);
                if (distances.find(distance) == distances.end()) {
                    sample = point;
                    distances.insert(distance);
                    break;
                }
            }
        }

        SECTION("Static") {
            auto kdt = create(samples, false);
            if (!execute(kdt, samples)) {
                print_points(samples);
            }
        }
        SECTION("Dynamic") {
            auto kdt = create(samples, true);
            if (!execute(kdt, samples)) {
                print_points(samples);
            }
        }
    }
}

TEST_CASE("2D Dynamic Benchmark", "[!hide]") {
    constexpr size_t kNumSamples{10'000};
    constexpr size_t kNumIterations = 100;

    IntSampler<> sampler(kLowerBound, kUpperBound);
    std::vector<KDT::Coordinates> samples{};
    for (size_t ii = 0; ii < kNumSamples; ii++) {
        samples.push_back({{sampler.sample(),sampler.sample()}});
    }

    double average = 0;
    KDT kdt;

    for (size_t iteration = 0; iteration < kNumIterations; iteration++) {
        const auto start = std::chrono::high_resolution_clock::now();
        {
            for (const auto &sample : samples) {
                kdt.insert(sample);
            }
        }
        const auto end = std::chrono::high_resolution_clock::now();
        average += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    }

    // Average, then convert to seconds
    average /= kNumIterations;
    average /= 1'000'000'000;
    std::cout << "Average : " << average << std::endl;
}

TEST_CASE("2D Static Benchmark", "[!hide]") {
    constexpr size_t kNumSamples{10'000};

    IntSampler<> sampler(kLowerBound, kUpperBound);
    std::vector<KDT::Coordinates> samples{};
    for (size_t ii = 0; ii < kNumSamples; ii++) {
        samples.push_back({{sampler.sample(),sampler.sample()}});
    }

    const auto start = std::chrono::high_resolution_clock::now();
    {
        KDT kdt(samples.begin(), samples.end());
        REQUIRE(kdt.root());
    }
    const auto end = std::chrono::high_resolution_clock::now();
    double time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    time /= 1'000'000'000;
    std::cout << "time : " << time << std::endl;
}
