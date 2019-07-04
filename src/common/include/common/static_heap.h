#pragma once

#include <optional>
#include <memory>
#include <queue>
#include <vector>

namespace pral {

struct PopLastPolicy {
    template <typename PQType, typename Type>
    bool operator()(PQType &q, const Type &, std::optional<Type> &) const {
        // Pop everything from the queue and store into the vector
        std::vector<Type> v;
        while (!q.empty()) {
            v.push_back(q.top());
            q.pop();
        }

        // Remove the last element
        v.pop_back();

        // Push everything else back in
        for (const auto &element : v) {
            q.push(element);
        }

        return true;
    }
};

struct PopLastIfBetterPolicy {
    template <typename PQType, typename Type>
    bool operator()(PQType &q, const Type &value, std::optional<Type> &last) const {
        typename PQType::value_compare comparator{};
        bool should_push = false;

        // Check to see if the work is necessary first
        if (last.has_value()) {
            if (!comparator(last.value(), value)) {
                return false;
            }
        }

        // Pop everything from the queue and store into the vector
        std::vector<Type> v;
        while (!q.empty()) {
            v.push_back(q.top());
            q.pop();
        }

        // Remove the last element if the new element would be better
        // Better as in it would fit between the last and the second last element
        if (comparator(v.back(), value)) {
            v.pop_back();
            should_push = true;
        }

        // Set new last
        if (comparator(v.back(), value)) {
            last = v.back();
        } else {
            last = value;
        }

        // Push everything else back in
        for (const auto &element : v) {
            q.push(element);
        }

        return should_push;
    }
};

/// This container is a wrapper around [std::priority_queue] but with a fixed size
/// When the queue reaches the max size and attempts to push an element into the queue
/// the queue will pop the last element and will push the new element
/// Therefore whenever the queue is full and a push is executed, the operation is
/// O(2N) * O(std::priority_queue::push)
/// N = MaxSize
/// So this container is inefficient for handling scenarios where there are a lot more
/// push operations than the MaxSize
/// \tparam Type       Type of the element in the queue
/// \tparam Container  The underlying storage structure
/// \tparam Comparator How elements are compared for sorting,
///                    std::less for max heap, std::greater for min heap
template <typename Type,
          typename Container  = std::vector<Type>,
          typename Comparator = std::less<typename Container::value_type>,
          typename DropPolicy = PopLastIfBetterPolicy>
class StaticPriorityQueue {
  public:
    using PQType = std::priority_queue<Type, Container, Comparator>;

    /// Constructor
    StaticPriorityQueue(const size_t max_size) : max_size_(max_size) {
        if (0 == max_size) {
            throw std::runtime_error("Can not maintain a queue of 0");
        }
        container_.reserve(max_size);
    }

    /// Destructor
    ~StaticPriorityQueue() = default;

    const Type &top() const {
        return q_->top();
    }

    bool empty() const {
        return q_->empty();
    }

    bool full() const {
        return size() == max_size_;
    }

    std::size_t size() const {
        return q_->size();
    }

    const std::optional<Type> &last() const {
        return last_;
    }

    void push(const Type &value) {
        bool should_push = true;

        if (full()) {
            auto &q = *(q_.get());
            should_push = drop_policy_(q, value, last_);
        }

        if (should_push) {
            q_->push(value);
        }
    }

    void pop() {
        q_->pop();
    }

    template <typename ... Args>
    void emplace(Args && ... args) {
        q_->emplace(std::forward<Args>(args)...);
    }

  private:
    const size_t max_size_;
    DropPolicy drop_policy_{};
    Container container_{};
    std::optional<Type> last_{};
    std::unique_ptr<PQType> q_ = std::make_unique<PQType>(Comparator(), std::move(container_));
};

template <typename Type,
          typename Comparator = std::greater<Type>>
using StaticMinHeap = StaticPriorityQueue<Type, std::vector<Type>, Comparator>;

template <typename Type,
          typename Comparator = std::less<Type>>
using StaticMaxHeap = StaticPriorityQueue<Type, std::vector<Type>, Comparator>;

} // namespace pral
