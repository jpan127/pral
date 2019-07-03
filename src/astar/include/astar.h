#pragma once

#include "astar_types.h"

#include <algorithm>
#include <cassert>
#include <queue>
#include <unordered_map>
#include <vector>

namespace pral {

template <typename NodeType, typename LinkType>
class AStarGraph {
    struct PQNode {
        double distance = 0;
        uint64_t node_id = 0;
        struct Compare {
            bool operator()(const PQNode &a, const PQNode &b) const {
                return a.distance > b.distance;
            }
        };
    };

    using PQueue = std::priority_queue<PQNode, std::vector<PQNode>, typename PQNode::Compare>;

  public:
    using Node = AStarNode<NodeType>;
    using Link = AStarLink<LinkType>;
    struct Parents {
        uint64_t parent_node_id = 0;
        uint64_t parent_link_id = 0;
    };

    virtual std::vector<uint64_t> solve(const uint64_t start_id, const uint64_t end_id) {
        std::unordered_map<uint64_t, Parents> parents;
        std::unordered_map<uint64_t, double> distance;
        PQueue queue;

        distance[start_id] = 0;
        parents[start_id] = {start_id, 0};
        queue.push({0, start_id});

        while (!queue.empty()) {
            const uint64_t current_node_id = queue.top().node_id;
            queue.pop();

            if (end_id == current_node_id) {
                break;
            }

            const auto current_node = lookup_node(current_node_id);
            if (!current_node) {
                return {};
            }

            if (current_node->visited) {
                continue;
            }

            const auto &successor_link_ids = current_node->links.out;
            for (const auto successor_link_id : successor_link_ids) {
                const auto link = lookup_link(current_node, successor_link_id);
                if (!link) {
                    return {};
                }

                const double link_distance = link->cost;
                const uint64_t other_node_id = get_dest_node_id(*link);

                const bool distance_exists = distance.find(other_node_id) != distance.end();
                const double new_distance = distance[current_node_id] + link_distance;
                const bool new_shortest_path = (distance_exists) ? (new_distance < distance[other_node_id]) : (true);

                if (new_shortest_path) {
                    const double new_distance_with_heuristic = new_distance + heuristic(current_node, link);
                    distance[other_node_id] = new_distance;
                    parents[other_node_id] = {current_node_id, successor_link_id};
                    queue.push({new_distance_with_heuristic, other_node_id});
                }
            }
        }

        const bool found_end_node = distance.find(end_id) != distance.end();
        return (found_end_node) ? (get_route(start_id, end_id, parents)) : (std::vector<uint64_t>{});
    }

  protected:
    virtual double heuristic(const Node *, const Link *) const {
        return 0;
    }

    virtual const Node *lookup_node(const uint64_t id) const = 0;

    virtual const Link *lookup_link(const Node *current_node, const uint64_t id) const = 0;

    virtual uint64_t get_dest_node_id(const Link &link) const {
        return link.nodes.last;
    }

  private:
    std::vector<uint64_t> get_route(const uint64_t start_id, const uint64_t end_id,
                                    const std::unordered_map<uint64_t, Parents> &parents) const {
        std::vector<uint64_t> path;
        uint64_t current_id = end_id;

        while (current_id != start_id) {
            const auto iterator = parents.find(current_id);
            if (iterator == parents.end()) {
                return {};
            }

            current_id = iterator->second.parent_node_id;
            path.push_back(iterator->second.parent_link_id);
        }

        std::reverse(path.begin(), path.end());
        return path;
    }
};

} // namespace pral
