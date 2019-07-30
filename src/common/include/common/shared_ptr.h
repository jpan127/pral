#pragma once

#include <atomic>
#include <cstdint>

namespace pral {

template <typename Type>
class SharedPtr {
  public:
    /// Default constructor
    SharedPtr() = default;
    SharedPtr(std::nullptr_t) {}

    /// Constructor
    template <typename ConvertibleType>
    explicit SharedPtr(ConvertibleType *ptr) {
        if (ptr) {
            ctx_ = new Context;
            ctx_->value = ptr;
        }
    }

    /// Copy constructor
    SharedPtr(const SharedPtr &other) {
        copy(other);
    }

    /// Copy constructor, convertible pointer
    template <typename ConvertibleType>
    SharedPtr(const SharedPtr<ConvertibleType> &other) {
        copy(other);
    }

    /// Move constructor
    template <typename ConvertibleType>
    SharedPtr(SharedPtr<ConvertibleType> &&other) noexcept {
        swap(*this, other);
    }

    /// Destructor
    ~SharedPtr() {
        reset();
    }

    /// Copy assignment operator
    template <typename ConvertibleType>
    SharedPtr &operator=(const SharedPtr<ConvertibleType> other) {
        swap(*this, other);
        return *this;
    }

    /// Move assignment operator
    template <typename ConvertibleType>
    SharedPtr &operator=(SharedPtr<ConvertibleType> &&other) {
        reset();
        swap(*this, other);
        return *this;
    }

    void reset() {
        if (valid()) {
            if (ctx_->ref_count.fetch_sub(1) == 1) {
                delete ctx_;
            }

            ctx_ = nullptr;
        }
    }

    template <typename ConvertibleType>
    void reset(ConvertibleType *ptr) {
        reset();

        if (!ptr) {
            return;
        }

        ctx_ = new Context;
        ctx_->value = ptr;
    }

    Type *get() { return ctx_->value; }
    const Type *get() const { return ctx_->value; }

    Type &operator*() const { return *ctx_->value; }
    Type *operator->() const { return ctx_->value; }

    operator bool() const noexcept {
        return valid();
    }

    bool valid() const noexcept {
        return ctx_ != nullptr;
    }

    uint64_t use_count() const noexcept {
        return ctx_ ? ctx_->ref_count.load() : 0;
    }

    friend void swap(SharedPtr &a, SharedPtr &b) {
        std::swap(a.ctx_, b.ctx_);
    }

  private:
    struct Context {
        Type *value = nullptr;
        std::atomic<uint64_t> ref_count = 1;
    };

    Context *ctx_ = nullptr;

    template <typename ConvertibleType>
    void copy(const SharedPtr<ConvertibleType> &other) {
        // If different instances
        if (ctx_ != other.ctx_) {
            // Clean up self
            reset();
        }

        // Copy and increment ref count
        ctx_ = other.ctx_;
        if (ctx_) {
            ctx_->ref_count++;
        }
    }
};

template <typename Type, typename ... Args>
SharedPtr<Type> make_shared(Args && ... args) {
    return SharedPtr<Type>(new Type(std::forward<Args>(args)...));
}

} // namespace pral
