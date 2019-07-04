#pragma once

#include "common/static_heap.h"

#include <array>
#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <memory>

namespace pral {

template <size_t NumDimensions, typename T = double>
class KDTree {
  public:
    using Coordinates = std::array<T, NumDimensions>;

    struct Node {
        Coordinates point{};
        size_t dimension = 0;
        std::unique_ptr<Node> left{};
        std::unique_ptr<Node> right{};
    };

    using NodePtr = std::unique_ptr<Node>;

    /// Constructor
    /// Empty tree
    KDTree() = default;

    /// Constructor
    /// Build tree optimally, always using median as hyperplane
    template <typename ConstIterator,
              std::enable_if_t<std::is_same_v<typename std::iterator_traits<ConstIterator>::value_type, Coordinates>> * = nullptr>
    KDTree(ConstIterator begin, ConstIterator end) {
        std::vector<NodePtr> nodes;
        nodes.reserve(std::distance(begin, end));

        while (begin != end) {
            nodes.push_back(std::make_unique<Node>(Node{.point = *begin}));
            size_++;
            begin++;
        }

        rebuild(root_, 0, nodes.begin(), nodes.end());
    }

    /// Inserts one point / node into the tree
    void insert(const Coordinates &point) {
        // First node, make into root
        if (!root_) {
            root_ = std::make_unique<Node>(Node{.point = point, .dimension = 0});
            return;
        }

        Node *current = root_.get();

        // Traverse down the tree
        while (current) {
            const auto current_dimension = current->dimension;
            const auto new_hyperplane = point[current_dimension];
            const auto current_hyperplane = current->point[current_dimension];

            // Check which side of the current hyperplane the new point lies on
            if (new_hyperplane < current_hyperplane) {
                if (!current->left) {
                    current->left = std::make_unique<Node>(Node{.point = point, .dimension = next(current_dimension)});
                    break;
                }

                current = current->left.get();
            } else {
                if (!current->right) {
                    current->right = std::make_unique<Node>(Node{.point = point, .dimension = next(current_dimension)});
                    break;
                }

                current = current->right.get();
            }
        }

        size_++;
    }

    /// Rebuilds the entire tree as optimally as possible
    void rebuild() {
        if (!root_) {
            return;
        }

        std::vector<NodePtr> nodes;
        tree_to_vector(nodes);
        rebuild(root_, 0, nodes.begin(), nodes.end());
    }

    /// Finds the K nearest neighbors from the center
    /// \param center The point whos neighbors will be found
    /// \param num    K, the number of neighbors to search for
    std::vector<Coordinates> knn(const Coordinates &center, const size_t num) {
        using PQNode = std::pair<const Node *, double>;
        struct Comparator {
            bool operator()(const PQNode &a, const PQNode &b) const {
                return a.second > b.second;
            }
        };

        if (0 == num) {
            throw std::runtime_error("Can not find 0 nearest neighbors...");
        }
        if (!root_) {
            return {};
        }

        using MinHeap = StaticPriorityQueue<PQNode, std::vector<PQNode>, Comparator>;
        MinHeap pq(num);

        std::queue<const Node *> q;
        q.push(root_.get());

        while (!q.empty()) {
            const auto current = q.front();
            q.pop();

            // Add current node to pqueue
            const auto distance_kd = distance2(center, current->point);
            pq.push({current, distance_kd});

            const size_t dimension = current->dimension;
            auto has_potential_neighbors = [&] {
                const auto distance_1d = distance1d_2(center[dimension], current->point[dimension]);
                return (!pq.full()) ||
                       (!pq.last().has_value()) || (distance_1d < pq.last().value().second);
            };

            const bool left_side = center[dimension] < current->point[dimension];
            if (left_side) {
                if (current->left) {
                    q.push(current->left.get());
                }

                if (current->right) {
                    if (has_potential_neighbors()) {
                        q.push(current->right.get());
                    }
                }
            } else {
                if (current->right) {
                    q.push(current->right.get());
                }

                if (current->left) {
                    if (has_potential_neighbors()) {
                        q.push(current->left.get());
                    }
                }
            }
        }

        // Convert priority queue to vector
        std::vector<Coordinates> neighbors;
        while (!pq.empty()) {
            const auto &pair = pq.top();
            const auto node = pair.first;
            neighbors.push_back(node->point);
            pq.pop();
        }
        return neighbors;
    }

    /// Returns the root of the tree
    const std::unique_ptr<Node> &root() const {
        return root_;
    }

  protected:
    /// Calculates 1 dimensional euclidean distance squared
    virtual double distance1d_2(const T a, const T b) const {
        T diff{};

        if constexpr (std::is_floating_point_v<T>) {
            diff = a - b;
        } else {
            if constexpr (std::is_unsigned_v<T>) {
                diff = std::abs(static_cast<ssize_t>(a - b));
            } else {
                diff = a - b;
            }
        }

        return diff * diff;
    }

    /// Calculates euclidean distance squared of two points
    virtual double distance2(const Coordinates &a, const Coordinates &b) const {
        double d = 0;
        for (size_t ii = 0; ii < NumDimensions; ii++) {
            d += distance1d_2(a[ii], b[ii]);
        }
        return d;
    }

  private:
    /// Pointer to root node
    std::unique_ptr<Node> root_{};

    /// Current size of the tree
    size_t size_ = 0;

    /// Determines what the next dimension is
    size_t next(const size_t dimension) const {
        return (dimension + 1) % NumDimensions;
    }

    /// Transforms the tree into a vector of nodes
    /// The nodes are moved from so the tree is unusable after this operation
    void tree_to_vector(std::vector<NodePtr> &nodes) {
        nodes.reserve(size_);

        std::queue<NodePtr> q;
        q.push(std::move(root_));

        while (!q.empty()) {
            auto current = std::move(q.front());
            q.pop();

            if (current->left) {
                q.push(std::move(current->left));
            }

            if (current->right) {
                q.push(std::move(current->right));
            }

            nodes.push_back(std::move(current));
        }
    }

    /// Helper for rebuilding the tree
    /// \param current   The current node, the remaining nodes will be split on this node
    /// \param dimension The dimension of the current node
    /// \param begin     Begin iterator of the nodes list
    /// \param end       End iterator of the nodes list
    void rebuild(NodePtr &current,
                const size_t dimension,
                typename std::vector<NodePtr>::iterator begin,
                typename std::vector<NodePtr>::iterator end) {
        const auto size = std::distance(begin, end);
        if (size == 0) {
            return;
        }
        if (size == 1) {
            current = std::move(*begin);
            current->dimension = dimension;
            return;
        }

        std::sort(begin, end, [&dimension](const auto &a, const auto &b) {
            return a->point[dimension] < b->point[dimension];
        });

        const auto median_index = size / 2;
        current = std::move(*(begin + median_index));
        current->dimension = dimension;

        rebuild(current->left, next(dimension), begin, begin + median_index);
        rebuild(current->right, next(dimension), begin + median_index + 1, end);
    }
};

} // namespace pral
