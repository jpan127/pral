#include "astar.h"

#include "catch.hpp"

#include <array>

using namespace jp::astar;

constexpr bool LAVA = false;
constexpr uint8_t kMapDimension = 10;

using HotLavaMap = std::array<std::array<bool, kMapDimension>, kMapDimension>;

uint64_t create_node_id(const uint8_t row, const uint8_t col) {
    return (static_cast<uint64_t>(row) << 32  | col);
}

struct Data{};
class HotLavaNoHeuristic : public AStarGraph<Data, Data> {
  public:
    using AStarGraph::Node;
    using AStarGraph::Link;

    HotLavaNoHeuristic(const HotLavaMap &map) {
        generate(map);
    }

    const Node *lookup_node(const uint64_t id) const override {
        assert(nodes_.find(id) != nodes_.end());
        return &nodes_.at(id);
    }

    const Link *lookup_link(const Node *, const uint64_t id) const override {
        return &links_.at(id);
    }

  protected:
    std::unordered_map<uint64_t, Node> nodes_;
    std::unordered_map<uint64_t, Link> links_;

    uint64_t create_link(const uint8_t x, const uint8_t y) {
        static uint64_t counter = 0;
        links_[counter] = {
            .id = counter,
            .cost = 1,
            .nodes.last = create_node_id(x, y),
        };
        return counter++;
    }

    void generate(const HotLavaMap &map) {
        for (std::size_t row = 0; row < kMapDimension; row++) {
            for (std::size_t col = 0; col < kMapDimension; col++) {
                if (map[row][col]) {
                    const auto id = create_node_id(row, col);
                    Node node{.id = id};
                    // Up
                    if (row > 0 && map[row - 1][col]) {
                        node.links.out.insert(create_link(row - 1, col));
                    }

                    // Down
                    if (row < (kMapDimension - 1) && map[row + 1][col]) {
                        node.links.out.insert(create_link(row + 1, col));
                    }

                    // Left
                    if (col > 0 && map[row][col - 1]) {
                        node.links.out.insert(create_link(row, col - 1));
                    }

                    // Right
                    if (col < (kMapDimension - 1) && map[row][col + 1]) {
                        node.links.out.insert(create_link(row, col + 1));
                    }

                    nodes_[id] = node;
                }
            }
        }
    }
};

class HotLavaWithHeuristic : public HotLavaNoHeuristic {
  public:
    HotLavaWithHeuristic(const HotLavaMap &map) : HotLavaNoHeuristic(map){}

    std::vector<uint64_t> solve(const uint64_t start_id, const uint64_t end_id) override {
        start_x_ = start_id >> 32;
        start_y_ = start_id & 0xFF;
        end_x_ = end_id >> 32;
        end_y_ = end_id & 0xFF;

        return AStarGraph::solve(start_id, end_id);
    }

    double heuristic(const Node *node, const Link *) const override {
        const auto x = node->id >> 32;
        const auto y = node->id & 0xFF;
        return (std::abs((int)x - (int)end_x_)) +
               (std::abs((int)y - (int)end_y_));
    }

  private:
    uint8_t start_x_ = 0;
    uint8_t start_y_ = 0;
    uint8_t end_x_ = 0;
    uint8_t end_y_ = 0;
};

TEST_CASE("NoHeuristic", "AStar") {
    static constexpr HotLavaMap kOneWallOfLava = {{
        { 1 , 1 , 1 , 1 , 1 ,    1 , 1 , 1 , 1 , 1 },
        { 1 , 1 , 1 , 1 , 1 , LAVA , 1 , 1 , 1 , 1 },
        { 1 , 1 , 1 , 1 , 1 , LAVA , 1 , 1 , 1 , 1 },
        { 1 , 1 , 1 , 1 , 1 , LAVA , 1 , 1 , 1 , 1 },
        { 1 , 1 , 1 , 1 , 1 , LAVA , 1 , 1 , 1 , 1 },
        { 1 , 1 , 1 , 1 , 1 , LAVA , 1 , 1 , 1 , 1 },
        { 1 , 1 , 1 , 1 , 1 , LAVA , 1 , 1 , 1 , 1 },
        { 1 , 1 , 1 , 1 , 1 , LAVA , 1 , 1 , 1 , 1 },
        { 1 , 1 , 1 , 1 , 1 , LAVA , 1 , 1 , 1 , 1 },
        { 1 , 1 , 1 , 1 , 1 ,    1 , 1 , 1 , 1 , 1 },
    }};

    HotLavaNoHeuristic dumb_graph(kOneWallOfLava);
    HotLavaWithHeuristic smart_graph(kOneWallOfLava);

    auto run_test = [&](const auto start_x, const auto start_y, const auto end_x, const auto end_y) {
        const auto start_node_id = create_node_id(start_x, start_y);
        const auto end_node_id = create_node_id(end_x, end_y);
        const auto dumb_route = dumb_graph.solve(start_node_id, end_node_id);
        const auto smart_route = smart_graph.solve(start_node_id, end_node_id);

        REQUIRE(dumb_route.size() == smart_route.size());
        return dumb_route.size();
    };

    REQUIRE(1U == run_test(
        3, 1,
        3, 2
    ));

    REQUIRE(2U == run_test(
        3, 1,
        3, 3
    ));

    REQUIRE(3U == run_test(
        3, 1,
        3, 4
    ));

    REQUIRE(11U == run_test(
        3, 1,
        3, 6
    ));

    REQUIRE(9U == run_test(
        9, 0,
        9, 9
    ));

    REQUIRE(18U == run_test(
        9, 0,
        0, 9
    ));

    REQUIRE(12U == run_test(
        0, 9,
        5, 2
    ));
}