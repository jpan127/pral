#include "rrt/rrt.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace {

constexpr std::size_t kMaxSteps{100'000};
constexpr std::size_t kStepsPerIteration{5'000};

constexpr double kMaxX = 10'000;
constexpr double kMaxY = 10'000;
constexpr jp::Rectangle kObstacleA{
    .top_right    = { .x = 4'000 , .y = 4'000 },
    .bottom_left  = { .x = 1'000 , .y = 1'000 },
};
constexpr jp::Rectangle kObstacleB{
    .top_right    = { .x = 9'000 , .y = 9'000 },
    .bottom_left  = { .x = 6'000 , .y = 6'000 },
};

} // namespace

int main(int, char**) {
    using namespace jp;
    using namespace jp::rrt;

    auto in_obstacle = [](const Point &p) {
        if (p.x >= kMaxX || p.y >= kMaxY) {
            return true;
        }

        if (in_bounding_box(p, kObstacleA) || in_bounding_box(p, kObstacleB)) {
            return true;
        }

        return false;
    };

    constexpr Point kStart{0, 0};
    constexpr Point kEnd{9'999,9'999};

    std::filesystem::create_directory("data");

    RRT rrt{};
    rrt.init(in_obstacle, kStart, kEnd);

    uint64_t steps = 0;
    while (!rrt.solve(kStepsPerIteration) && steps < kMaxSteps) {
        const auto &nodes = rrt.nodes();
        steps += kStepsPerIteration;

        const std::string name = "data/" + std::to_string(steps) + "_" + std::to_string(nodes.size()) + "_plot.txt";
        std::ofstream file(name);
        for (const auto &n : nodes) {
            const auto &other = nodes[n.parent_index];
            file << other.p.x << ","
                 << other.p.y << ","
                 << n.p.x     << ","
                 << n.p.y     << "\n";
        }

        std::cout << "Stepped : " << steps << std::endl;
    }

    return 0;
}
