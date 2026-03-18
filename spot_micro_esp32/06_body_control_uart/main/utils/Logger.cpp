#include "Logger.hpp"
#include <cstdio>
#include <cstring>
#include <ctime>
#include <inttypes.h>

namespace Robot {
namespace Utils {

Logger& Logger::getInstance() {
    static Logger instance;
    return instance;
}

void Logger::setLogLevel(LogLevel level) {
    current_level_ = level;
}

void Logger::debug(const char* format, ...) {
    if (current_level_ <= LogLevel::DEBUG) {
        va_list args;
        va_start(args, format);
        logInternal(LogLevel::DEBUG, nullptr, format, args);
        va_end(args);
    }
}

void Logger::info(const char* format, ...) {
    if (current_level_ <= LogLevel::INFO) {
        va_list args;
        va_start(args, format);
        logInternal(LogLevel::INFO, nullptr, format, args);
        va_end(args);
    }
}

void Logger::warn(const char* format, ...) {
    if (current_level_ <= LogLevel::WARN) {
        va_list args;
        va_start(args, format);
        logInternal(LogLevel::WARN, nullptr, format, args);
        va_end(args);
    }
}

void Logger::error(const char* format, ...) {
    if (current_level_ <= LogLevel::ERROR) {
        va_list args;
        va_start(args, format);
        logInternal(LogLevel::ERROR, nullptr, format, args);
        va_end(args);
    }
}

void Logger::fatal(const char* format, ...) {
    if (current_level_ <= LogLevel::FATAL) {
        va_list args;
        va_start(args, format);
        logInternal(LogLevel::FATAL, nullptr, format, args);
        va_end(args);
    }
}

void Logger::log(LogLevel level, const char* format, ...) {
    if (current_level_ <= level) {
        va_list args;
        va_start(args, format);
        logInternal(level, nullptr, format, args);
        va_end(args);
    }
}

void Logger::logWithTag(LogLevel level, const char* tag, const char* format, ...) {
    if (current_level_ <= level) {
        va_list args;
        va_start(args, format);
        logInternal(level, tag, format, args);
        va_end(args);
    }
}

void Logger::logInternal(LogLevel level, const char* tag, const char* format, va_list args) {
    // 构建日志消息
    char buffer[512];  // 限制单条日志最大长度
    int offset = 0;

    // 添加颜色代码 (如果启用)
    if (color_output_) {
        int written = snprintf(buffer + offset, sizeof(buffer) - offset, "%s", getLevelColor(level));
        if (written > 0) offset += written;
    }

    // 添加时间戳 (如果启用) - 使用简单的计数器替代FreeRTOS
    if (show_timestamp_) {
        static uint32_t counter = 0;
        counter++;
        int written = snprintf(buffer + offset, sizeof(buffer) - offset, "[%6" PRIu32 "] ", counter);
        if (written > 0) offset += written;
    }

    // 添加日志级别
    int written = snprintf(buffer + offset, sizeof(buffer) - offset, "%s: ", getLevelName(level));
    if (written > 0) offset += written;

    // 添加标签 (如果提供)
    if (tag) {
        written = snprintf(buffer + offset, sizeof(buffer) - offset, "[%s] ", tag);
        if (written > 0) offset += written;
    }

    // 添加用户消息
    written = vsnprintf(buffer + offset, sizeof(buffer) - offset, format, args);
    if (written > 0) offset += written;

    // 添加颜色重置代码 (如果启用)
    if (color_output_) {
        written = snprintf(buffer + offset, sizeof(buffer) - offset, "\033[0m");
        if (written > 0) offset += written;
    }

    // 添加换行符
    if (offset < sizeof(buffer) - 1) {
        buffer[offset++] = '\n';
        buffer[offset] = '\0';
    }

    // 输出日志
    output(buffer);
}

const char* Logger::getLevelName(LogLevel level) {
    switch (level) {
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO:  return "INFO ";
        case LogLevel::WARN:  return "WARN ";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::FATAL: return "FATAL";
        default: return "UNKNW";
    }
}

const char* Logger::getLevelColor(LogLevel level) {
    switch (level) {
        case LogLevel::DEBUG: return "\033[36m";  // 青色
        case LogLevel::INFO:  return "\033[32m";  // 绿色
        case LogLevel::WARN:  return "\033[33m";  // 黄色
        case LogLevel::ERROR: return "\033[31m";  // 红色
        case LogLevel::FATAL: return "\033[35m";  // 紫色
        default: return "\033[0m";
    }
}

std::string Logger::getTimestamp() const {
    static uint32_t counter = 0;
    counter++;
    char timestamp[16];
    snprintf(timestamp, sizeof(timestamp), "[%6" PRIu32 "]", counter);
    return std::string(timestamp);
}

void Logger::output(const char* message) {
    switch (target_) {
        case LogTarget::UART:
        case LogTarget::CONSOLE:
            printf("%s", message);
            fflush(stdout);  // 确保立即输出
            break;
        default:
            break;
    }
}

} // namespace Utils
} // namespace Robot