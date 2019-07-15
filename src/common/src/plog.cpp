#include "common/plog.h"

#include <atomic>
#include <iostream>
#include <mutex>

namespace pral::plog {

static std::atomic<LogLevel> global_log_level = LogLevel::Info;
static std::mutex global_lock;

void set_level(const LogLevel level) {
    global_log_level = level;
}

LogLevel get_level() {
    return global_log_level;
}

namespace detail {

void output(const std::string &msg) {
    std::scoped_lock lock(global_lock);
    std::cout << msg;
}

} // namespace detail

} // namespace pral::plog
