#include "catch.hpp"

#include "common/plog.h"

#include <array>
#include <string>

using namespace pral;
using namespace pral::plog;

template <typename ... Types>
void log_all_levels(Types && ... args) {
    debug(std::forward<Types>(args)...);
    info(std::forward<Types>(args)...);
    warning(std::forward<Types>(args)...);
    error(std::forward<Types>(args)...);
}

TEST_CASE("plog", "plog") {
    constexpr std::array<LogLevel, LogLevel::kNumLogLevels> kLevels{{
        Debug,
        Info,
        Warning,
        Error,
    }};

    for (const auto level : kLevels) {
        set_level(level);
        REQUIRE(level == get_level());
    }

    set_level(Debug);

    log_all_levels(
        false,
        (uint32_t)127,
        (char)'?',
        (int)-127,
        (float)0.127,
        (double)0.127,
        "literal",
        std::string("std_string")
    );
}
