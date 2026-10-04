#pragma once

#include "logger_utils.hpp"
#include <cstdlib>
#include <string>

namespace aurora {

inline std::string default_log_folder() {
    const char* home = std::getenv("HOME");
    return std::string(home ? home : ".") + "/.aurora/log";
}

inline void init_logger(spdlog::level::level_enum level = spdlog::level::info) {
    aurora::logger::init(default_log_folder(), level);
}

} // namespace aurora
