#pragma once

#include <sstream>

namespace pral {

enum LogLevel {
    Debug,
    Info,
    Warning,
    Error,
    kNumLogLevels,
};

namespace plog {

/// Change the global logging level
void set_level(const LogLevel level);

/// Check what the current global logging level is
LogLevel get_level();

namespace detail {

/// Enum to string conversion
const char *string(const LogLevel level) {
    switch (level) {
    case Debug   : return "Debug";
    case Info    : return "Info";
    case Warning : return "Warning";
    case Error   : return "Error";
    default      : throw std::runtime_error("Invalid enum");
    }
}

/// Outputs to stdio while locking the output mutex
void output(const std::string &msg);

/// Converts log level and arguments into a log message, then outputs it
template <typename ... Types>
void log(const LogLevel level, Types && ... args) {
    if (get_level() <= level) {
        std::stringstream ss;
        ss << "[" << string(level) << "] ";
        ((ss << args << " "), ...) << '\n';
        output(ss.str());
    }
}

} // namespace detail

template <typename ... Types> void debug(Types && ... args) { detail::log(LogLevel::Debug, std::forward<Types>(args)...); }
template <typename ... Types> void info(Types && ... args) { detail::log(LogLevel::Info, std::forward<Types>(args)...); }
template <typename ... Types> void warning(Types && ... args) { detail::log(LogLevel::Warning, std::forward<Types>(args)...); }
template <typename ... Types> void error(Types && ... args) { detail::log(LogLevel::Error, std::forward<Types>(args)...); }

} // namespace plog

} // namespace pral
