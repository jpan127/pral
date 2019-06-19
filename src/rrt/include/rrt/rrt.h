#pragma once

#include "rrt/types.h"
#include "rrt/sampler.h"

#include <cassert>
#include <functional>
#include <random>
#include <vector>
#include <memory>

namespace jp::rrt {

/// An implementation of RRT*
template <std::size_t StepSize = 30,
          std::size_t PruneRadius = 200,
          std::size_t MaxX = 10'000,
          std::size_t MaxY = 10'000>
class RRT {
    static constexpr double kMaxX = MaxX;
    static constexpr double kMaxY = MaxY;
    static constexpr double kMinX = 0;
    static constexpr double kMinY = 0;
    static constexpr double kMinimumGain = StepSize;
    static constexpr double kPruneRadius = PruneRadius;
    static constexpr double kStepSize = StepSize;
    static constexpr double kTerminatingRadius = PruneRadius;

    struct Node {
        Point p{};                      /// 2D position of this node
        double distance = 0;            /// Total distance from start node
        std::size_t parent_index = 0;   /// Index of parent node in [nodes_]
    };

  public:
    /// Initializes the state of this object for exploration
    /// \param in_obstacle External function that determines if a sampled point is within an obstacle
    /// \param start       Starting point
    /// \param end         Target point
    /// \return            True if initialization is successful
    bool init(std::function<bool(const Point &)> &&in_obstacle, const Point &start, const Point &end) {
        in_obstacle_ = std::move(in_obstacle);
        nodes_.clear();
        nodes_.push_back({.p = start});
        target_ = end;
        return true;
    }

    /// Attempts to solve by performing steps and checking if the goal has been reached
    /// \param num_steps Number of steps to run for
    /// \return          True if the goal has been reached
    bool solve(const uint64_t num_steps) {
        for (std::size_t s = 0; s < num_steps; s++) {
            if (step()) {
                return true;
            }
        }

        return false;
    }

    /// \return The list of existing nodes in the tree
    const std::vector<Node> &nodes() const {
        return nodes_;
    }

  protected:
    /// Searches for nodes in a radius from the provided node and determines if any linkage can be optimized
    /// \param center The node whose neighbors will be evaluated
    /// \param index  The index of the node in [nodes_]
    void prune(const Node &center, const std::size_t index) {
        constexpr double kPruneRadiusSquared = kPruneRadius * kPruneRadius;
        for (auto &node : nodes_) {
            const auto distance_squared = point_distance2(node.p, center.p);
            if (distance_squared < kPruneRadiusSquared) {
                const auto distance = std::sqrt(distance_squared);
                if (center.distance + distance + kMinimumGain < node.distance) {
                    node.parent_index = index;
                    node.distance = center.distance + distance;
                }
            }
        }
    }

  private:
    /// Callback to determine if a point is within an obstacle
    std::function<bool(const Point &)> in_obstacle_;

    /// List of existing nodes in the tree
    std::vector<Node> nodes_;

    /// 2D point of the goal
    Point target_{};

    /// Random sampler
    std::unique_ptr<Sampler> sampler_ = std::make_unique<Sampler>(kMinX, kMaxX, kMinY, kMaxY);

    /// Determine which of the nodes in the tree is the closest to the provided point
    /// \return Index of the closest node in [nodes_]
    std::size_t find_nearest_node(const Point &p) const {
        double minimum_distance = std::numeric_limits<double>::max();
        std::size_t index = 0;

        for (std::size_t ii = 0; ii < nodes_.size(); ii++) {
            const auto &node = nodes_[ii];
            if (const auto distance = point_distance2(node.p, p); distance < minimum_distance) {
                minimum_distance = distance;
                index = ii;
            }
        }

        return index;
    }

    /// Determines where to place the new node that is along the line from the nearest node to the sampled point
    /// \param nearest_node The existing node that is closest to the sample point
    /// \param sample       The 2D point of the random sample in which the line is derived from
    /// \return             A new node with a point along the line from nearest node to the sample, bounded by the step size
    Node determine_best_placement(const Node &nearest_node, const Point &sample) const {
        const auto dist = point_distance(nearest_node.p, sample);
        return {
            .p.x = nearest_node.p.x + (kStepSize / dist) * (sample.x - nearest_node.p.x),
            .p.y = nearest_node.p.y + (kStepSize / dist) * (sample.y - nearest_node.p.y),
        };
    }

    /// Executes one iteration of tree expansion
    /// \return True if the goal has been reached in this iteration
    bool step() {
        constexpr double kTerminatingRadiusSquared = kTerminatingRadius * kTerminatingRadius;

        while (true) {
            const auto sample = sampler_->sample();
            const auto nearest_node_index = find_nearest_node(sample);
            const auto &existing_node = nodes_[nearest_node_index];
            if (sample == existing_node.p) {
                continue;
            }

            auto new_node = determine_best_placement(existing_node, sample);
            if (in_obstacle_(new_node.p)) {
                continue;
            }

            const auto dist = point_distance(existing_node.p, new_node.p);
            new_node.distance = existing_node.distance + dist;
            new_node.parent_index = nearest_node_index;
            nodes_.push_back(new_node);

            prune(new_node, nodes_.size() - 1);

            return (point_distance2(new_node.p, target_) <= kTerminatingRadiusSquared);
        }
    }
};

} // namespace jp::rrt
