#include "catch.hpp"

#include "common/shared_ptr.h"

using namespace pral;

TEST_CASE("Null", "SharedPtr") {
    SharedPtr<int> sp = nullptr;
    REQUIRE(!bool(sp));
    REQUIRE(sp.use_count() == 0);
}

TEST_CASE("Basic", "SharedPtr") {
    SharedPtr<int> sp(new int(5));
    REQUIRE(bool(sp));
    REQUIRE(sp.use_count() == 1);
    REQUIRE(*sp == 5);
    (*sp)++;
    REQUIRE(*sp == 6);
}

TEST_CASE("Polymorphic", "SharedPtr") {
    struct Base {
        Base(int x) : x(x){}
        virtual ~Base() = default;
        int x = 0;
    };

    struct Child : public Base {
        Child(int x) : Base(x) {}
        ~Child() = default;
    };

    SECTION("Assignment") {
        SharedPtr<Base> a;
        auto ptr = new Child(127);
        a.reset(ptr);
        REQUIRE(bool(a));
        REQUIRE(a.use_count() == 1);
        REQUIRE(a.get() == ptr);
        REQUIRE(a.operator->() == ptr);
        REQUIRE(a->x == 127);
    }
    SECTION("Construction") {
        SharedPtr<Base> a(new Child(127));
        REQUIRE(bool(a));
        REQUIRE(a.use_count() == 1);
        REQUIRE(a->x == 127);
    }
}

TEST_CASE("Copy", "SharedPtr") {
    SharedPtr<int> a(new int(5));
    REQUIRE(a.use_count() == 1);

    SECTION("Assignment") {
        auto b = a;
        REQUIRE(bool(a));
        REQUIRE(bool(b));
        REQUIRE(a.use_count() == 2);
        REQUIRE(b.use_count() == 2);
        REQUIRE(*a == 5);
        REQUIRE(*b == 5);
        (*a)++;
        REQUIRE(*a == 6);
        REQUIRE(*b == 6);

        a.reset();
        REQUIRE(b.use_count() == 1);
    }
    SECTION("Construction") {
        SharedPtr<int> b(a);
        REQUIRE(bool(a));
        REQUIRE(bool(b));
        REQUIRE(a.use_count() == 2);
        REQUIRE(b.use_count() == 2);
        REQUIRE(*a == 5);
        REQUIRE(*b == 5);
        (*a)++;
        REQUIRE(*a == 6);
        REQUIRE(*b == 6);

        a.reset();
        REQUIRE(b.use_count() == 1);
    }
}

TEST_CASE("Move", "SharedPtr") {
    SharedPtr<int> a(new int(5));
    REQUIRE(a.use_count() == 1);

    SECTION("Assignment") {
        auto b = std::move(a);
        REQUIRE(!bool(a));
        REQUIRE(bool(b));
        REQUIRE(a.use_count() == 0);
        REQUIRE(b.use_count() == 1);

        a.reset();
        REQUIRE(b.use_count() == 1);
    }
    SECTION("Construction") {
        SharedPtr<int> b(std::move(a));
        REQUIRE(!bool(a));
        REQUIRE(bool(b));
        REQUIRE(a.use_count() == 0);
        REQUIRE(b.use_count() == 1);

        a.reset();
        REQUIRE(b.use_count() == 1);
    }
}

TEST_CASE("Factory", "SharedPtr") {
    auto sp = pral::make_shared<int>(5);
    REQUIRE(bool(sp));
    REQUIRE(sp.use_count() == 1);
    REQUIRE(*sp == 5);
    (*sp)++;
    REQUIRE(*sp == 6);
}
