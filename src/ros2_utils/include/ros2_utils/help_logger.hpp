/**
 * @file help_logger.hpp
 * @author Pandu Surya Tantra (pandustantra@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-28
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef HELP_LOGGER_HPP_
#define HELP_LOGGER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

class HelpLogger
{
private:
    bool _is_initialized = false;

    std::shared_ptr<spdlog::logger> _logger;

public:
    HelpLogger();

    bool is_initialized();

    bool init();

    std::string to_string(const char *fmt, va_list args);

    void debug(const char *fmt, ...);
    void info(const char *fmt, ...);
    void warn(const char *fmt, ...);
    void error(const char *fmt, ...);
    void fatal(const char *fmt, ...);
};

#endif // HELP_LOGGER_HPP_