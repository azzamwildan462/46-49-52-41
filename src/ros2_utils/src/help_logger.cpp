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

#include "ros2_utils/help_logger.hpp"

HelpLogger::HelpLogger() {}

bool HelpLogger::is_initialized()
{
    return _is_initialized;
}

bool HelpLogger::init()
{
    if (_is_initialized)
    {
        return true;
    }

    if (spdlog::get("logger") != nullptr)
    {
        _logger = spdlog::get("logger");

        _is_initialized = true;
        return true;
    }

    _logger = spdlog::stdout_color_st("logger", spdlog::color_mode::always);
    _logger->set_pattern("\033[34m[%H:%M:%S.%e]\033[0m %^[%l]%$ %v");

    _is_initialized = true;
    return true;
}

std::string
HelpLogger::to_string(const char *fmt, va_list args)
{
    if (!_is_initialized)
    {
        return "";
    }

    va_list args_copy;
    va_copy(args_copy, args);

    int size = vsnprintf(nullptr, 0, fmt, args_copy);
    if (size < 0)
    {
        return "";
    }

    std::string str(size, '\0');
    vsnprintf(&str[0], size + 1, fmt, args);

    return str;
}

void HelpLogger::debug(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    _logger->debug(to_string(fmt, args));
    va_end(args);
}

void HelpLogger::info(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    _logger->info(to_string(fmt, args));
    va_end(args);
}

void HelpLogger::warn(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    _logger->warn(to_string(fmt, args));
    va_end(args);
}

void HelpLogger::error(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    _logger->error(to_string(fmt, args));
    va_end(args);
}

void HelpLogger::fatal(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    _logger->critical(to_string(fmt, args));
    va_end(args);
}