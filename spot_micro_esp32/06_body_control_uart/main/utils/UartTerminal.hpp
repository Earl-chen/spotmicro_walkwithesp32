/**
 * @file UartTerminal.hpp
 * @brief UART终端交互工具类 - 统一的命令行交互接口
 *
 * 该类提供了一个通用的UART终端接口，支持：
 * - 自动UART初始化和配置
 * - 命令行输入读取和处理
 * - 基于回调函数的命令解析
 * - 可定制的提示符和帮助信息
 * - 内置通用命令（help, exit等）
 *
 * 使用方式：
 * 1. 继承UartTerminal或使用回调函数
 * 2. 实现命令处理逻辑
 * 3. 调用startInteractiveMode()开始交互
 */

#pragma once

#include <string>
#include <functional>
#include <map>
#include <vector>
#include <memory>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/uart.h>
#include <esp_log.h>

namespace Robot {
namespace Utils {

/**
 * @brief 命令处理函数类型
 * @param command 完整的命令字符串
 * @param args 解析后的参数列表
 * @return true表示命令已处理，false表示未识别的命令
 */
using CommandHandler = std::function<bool(const std::string& command, const std::vector<std::string>& args)>;

/**
 * @brief 命令信息结构
 */
struct CommandInfo {
    std::string description;    // 命令描述
    std::string usage;         // 使用方法
    CommandHandler handler;    // 处理函数

    CommandInfo() = default;
    CommandInfo(const std::string& desc, const std::string& use, CommandHandler h)
        : description(desc), usage(use), handler(h) {}
};

/**
 * @brief UART终端配置
 */
struct UartTerminalConfig {
    uart_port_t uart_port = UART_NUM_0;
    int baud_rate = 115200;
    uart_word_length_t data_bits = UART_DATA_8_BITS;
    uart_parity_t parity = UART_PARITY_DISABLE;
    uart_stop_bits_t stop_bits = UART_STOP_BITS_1;
    uart_hw_flowcontrol_t flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    size_t rx_buffer_size = 1024;
    size_t tx_buffer_size = 0;
    int queue_size = 0;
    uint32_t read_timeout_ms = 100;
    uint32_t input_timeout_ms = 30000; // 30秒输入超时
};

/**
 * @brief UART终端交互工具类
 */
class UartTerminal {
public:
    /**
     * @brief 构造函数
     * @param config UART配置
     * @param prompt 提示符前缀
     * @param welcome_message 欢迎信息
     */
    explicit UartTerminal(const UartTerminalConfig& config = UartTerminalConfig{},
                         const std::string& prompt = "terminal",
                         const std::string& welcome_message = "UART Terminal Ready");

    /**
     * @brief 析构函数
     */
    virtual ~UartTerminal();

    /**
     * @brief 初始化UART
     * @return true成功，false失败
     */
    bool init();

    /**
     * @brief 反初始化UART
     */
    void deinit();

    /**
     * @brief 注册命令处理器
     * @param command 命令名称
     * @param description 命令描述
     * @param usage 使用方法
     * @param handler 处理函数
     */
    void registerCommand(const std::string& command,
                        const std::string& description,
                        const std::string& usage,
                        CommandHandler handler);

    /**
     * @brief 设置默认命令处理器（处理未注册的命令）
     * @param handler 处理函数
     */
    void setDefaultHandler(CommandHandler handler);

    /**
     * @brief 开始交互模式
     * @param blocking 是否阻塞模式，false则创建新任务
     */
    void startInteractiveMode(bool blocking = true);

    /**
     * @brief 停止交互模式
     */
    void stopInteractiveMode();

    /**
     * @brief 设置提示符
     * @param prompt 新的提示符
     */
    void setPrompt(const std::string& prompt) { prompt_ = prompt; }

    /**
     * @brief 输出信息
     * @param message 要输出的信息
     */
    void print(const std::string& message);

    /**
     * @brief 输出信息并换行
     * @param message 要输出的信息
     */
    void println(const std::string& message);

protected:
    /**
     * @brief 从UART读取一行输入
     * @param buffer 输入缓冲区
     * @param buffer_size 缓冲区大小
     * @param timeout_ms 超时时间（毫秒）
     * @return true成功读取到一行，false超时或错误
     */
    bool readLine(char* buffer, size_t buffer_size, uint32_t timeout_ms);

    /**
     * @brief 解析命令字符串
     * @param input 输入字符串
     * @param command 解析出的命令
     * @param args 解析出的参数列表
     */
    void parseCommand(const std::string& input, std::string& command, std::vector<std::string>& args);

    /**
     * @brief 处理命令
     * @param input 输入字符串
     * @return true命令已处理，false需要退出
     */
    bool processCommand(const std::string& input);

    /**
     * @brief 显示帮助信息
     */
    void showHelp();

    /**
     * @brief 交互循环任务
     */
    void interactiveTask();

    /**
     * @brief 静态任务入口
     * @param pvParameters 任务参数（UartTerminal指针）
     */
    static void interactiveTaskEntry(void* pvParameters);

private:
    UartTerminalConfig config_;
    std::string prompt_;
    std::string welcome_message_;
    bool initialized_;
    bool running_;

    std::map<std::string, CommandInfo> commands_;
    CommandHandler default_handler_;

    TaskHandle_t task_handle_;

    static const char* TAG;
};

} // namespace Utils
} // namespace Robot