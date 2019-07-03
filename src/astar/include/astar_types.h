#pragma once

#include <limits>
#include <unordered_set>

namespace pral {

template <typename DataType>
struct AStarNode {
    uint64_t id = 0;
    DataType data{};
    struct {
        std::unordered_set<uint64_t> in;
        std::unordered_set<uint64_t> out;
    } links;
    bool visited = false;
};

template <typename DataType>
struct AStarLink {
    uint64_t id = 0;
    double cost = 0;
    DataType data{};
    struct {
        uint64_t first = 0;
        uint64_t last = 0;
    } nodes;
};

} // namespace pral
