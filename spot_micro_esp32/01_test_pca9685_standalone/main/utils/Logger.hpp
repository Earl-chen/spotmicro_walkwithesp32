#pragma once

#include <string>
#include <cstdarg>

namespace Robot {
namespace Utils {

/**
 * @brief 日志级别枚举
 */
enum class LogLevel : int {
    DEBUG = 0,   // 调试信息
    INFO = 1,    // 一般信息
    WARN = 2,    // 警告信息
    ERROR = 3,   // 错误信息
    FATAL = 4    // 致命错误
};

/**
 * @brief 日志输出目标
 */
enum class LogTarget {
    UART = 0,    // UART输出
    CONSOLE = 1  // 控制台输出 (相同，但便于扩展)
};

/**
 * @brief 轻量级日志系统
 *
 * 专为ESP32设计的简洁日志系统，支持不同日志级别和格式化输出
 */
class Logger {
public:
    /**
     * @brief 获取Logger单例
     */
    static Logger& getInstance();

    /**
     * @brief 设置日志级别
     * @param level 最低输出级别
     */
    void setLogLevel(LogLevel level);

    /**
     * @brief 获取当前日志级别
     */
    LogLevel getLogLevel() const { return current_level_; }

    /**
     * @brief 设置日志输出目标
     */
    void setLogTarget(LogTarget target) { target_ = target; }

    /**
     * @brief 启用/禁用时间戳
     */
    void setTimestamp(bool enable) { show_timestamp_ = enable; }

    /**
     * @brief 启用/禁用颜色输出
     */
    void setColorOutput(bool enable) { color_output_ = enable; }

    /**
     * @brief 记录调试信息
     */
    void debug(const char* format, ...);

    /**
     * @brief 记录一般信息
     */
    void info(const char* format, ...);

    /**
     * @brief 记录警告信息
     */
    void warn(const char* format, ...);

    /**
     * @brief 记录错误信息
     */
    void error(const char* format, ...);

    /**
     * @brief 记录致命错误
     */
    void fatal(const char* format, ...);

    /**
     * @brief 通用日志记录方法
     */
    void log(LogLevel level, const char* format, ...);

    /**
     * @brief 带标签的日志记录
     */
    void logWithTag(LogLevel level, const char* tag, const char* format, ...);

    /**
     * @brief 获取日志级别名称
     */
    static const char* getLevelName(LogLevel level);

    /**
     * @brief 获取日志级别颜色代码 (ANSI)
     */
    static const char* getLevelColor(LogLevel level);

private:
    Logger() = default;
    ~Logger() = default;

    // 禁用拷贝和赋值
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    LogLevel current_level_ = LogLevel::INFO;
    LogTarget target_ = LogTarget::UART;
    bool show_timestamp_ = true;
    bool color_output_ = false;  // ESP32默认关闭颜色

    /**
     * @brief 内部日志输出方法
     */
    void logInternal(LogLevel level, const char* tag, const char* format, va_list args);

    /**
     * @brief 获取当前时间戳字符串
     */
    std::string getTimestamp() const;

    /**
     * @brief 输出到指定目标
     */
    void output(const char* message);
};

} // namespace Utils
} // namespace Robot

// 便捷宏定义
#define LOG_DEBUG(format, ...) Robot::Utils::Logger::getInstance().debug(format, ##__VA_ARGS__)
#define LOG_INFO(format, ...)  Robot::Utils::Logger::getInstance().info(format, ##__VA_ARGS__)
#define LOG_WARN(format, ...)  Robot::Utils::Logger::getInstance().warn(format, ##__VA_ARGS__)
#define LOG_ERROR(format, ...) Robot::Utils::Logger::getInstance().error(format, ##__VA_ARGS__)
#define LOG_FATAL(format, ...) Robot::Utils::Logger::getInstance().fatal(format, ##__VA_ARGS__)

// 带标签的日志宏
#define LOG_TAG_DEBUG(tag, format, ...) Robot::Utils::Logger::getInstance().logWithTag(Robot::Utils::LogLevel::DEBUG, tag, format, ##__VA_ARGS__)
#define LOG_TAG_INFO(tag, format, ...)  Robot::Utils::Logger::getInstance().logWithTag(Robot::Utils::LogLevel::INFO, tag, format, ##__VA_ARGS__)
#define LOG_TAG_WARN(tag, format, ...)  Robot::Utils::Logger::getInstance().logWithTag(Robot::Utils::LogLevel::WARN, tag, format, ##__VA_ARGS__)
#define LOG_TAG_ERROR(tag, format, ...) Robot::Utils::Logger::getInstance().logWithTag(Robot::Utils::LogLevel::ERROR, tag, format, ##__VA_ARGS__)
#define LOG_TAG_FATAL(tag, format, ...) Robot::Utils::Logger::getInstance().logWithTag(Robot::Utils::LogLevel::FATAL, tag, format, ##__VA_ARGS__)