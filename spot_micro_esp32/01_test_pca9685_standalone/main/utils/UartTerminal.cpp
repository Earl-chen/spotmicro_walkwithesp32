/**
 * @file UartTerminal.cpp
 * @brief UART终端交互工具类实现
 */

#include "UartTerminal.hpp"
#include <sstream>
#include <algorithm>
#include <cstring>

namespace Robot {
namespace Utils {

const char* UartTerminal::TAG = "UartTerminal";

UartTerminal::UartTerminal(const UartTerminalConfig& config,
                          const std::string& prompt,
                          const std::string& welcome_message)
    : config_(config)
    , prompt_(prompt)
    , welcome_message_(welcome_message)
    , initialized_(false)
    , running_(false)
    , task_handle_(nullptr) {

    // 注册内置命令
    registerCommand("help", "显示帮助信息", "help",
                   [this](const std::string&, const std::vector<std::string>&) {
                       showHelp();
                       return true;
                   });

    registerCommand("exit", "退出终端", "exit",
                   [this](const std::string&, const std::vector<std::string>&) {
                       println("退出终端...");
                       return false; // 返回false表示退出
                   });

    registerCommand("clear", "清屏", "clear",
                   [this](const std::string&, const std::vector<std::string>&) {
                       print("\033[2J\033[H"); // ANSI清屏命令
                       return true;
                   });
}

UartTerminal::~UartTerminal() {
    deinit();
}

bool UartTerminal::init() {
    if (initialized_) {
        return true;
    }

    uart_config_t uart_config = {
        .baud_rate = config_.baud_rate,
        .data_bits = config_.data_bits,
        .parity = config_.parity,
        .stop_bits = config_.stop_bits,
        .flow_ctrl = config_.flow_ctrl,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(config_.uart_port, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART参数配置失败: %s", esp_err_to_name(ret));
        return false;
    }

    ret = uart_set_pin(config_.uart_port, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART引脚配置失败: %s", esp_err_to_name(ret));
        return false;
    }

    ret = uart_driver_install(config_.uart_port, config_.rx_buffer_size, config_.tx_buffer_size,
                             config_.queue_size, nullptr, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART驱动安装失败: %s", esp_err_to_name(ret));
        return false;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "UART终端初始化成功 (端口: %d, 波特率: %d)", config_.uart_port, config_.baud_rate);
    return true;
}

void UartTerminal::deinit() {
    if (!initialized_) {
        return;
    }

    stopInteractiveMode();
    uart_driver_delete(config_.uart_port);
    initialized_ = false;
    ESP_LOGI(TAG, "UART终端已反初始化");
}

void UartTerminal::registerCommand(const std::string& command,
                                  const std::string& description,
                                  const std::string& usage,
                                  CommandHandler handler) {
    commands_[command] = CommandInfo(description, usage, handler);
    ESP_LOGD(TAG, "注册命令: %s", command.c_str());
}

void UartTerminal::setDefaultHandler(CommandHandler handler) {
    default_handler_ = handler;
}

void UartTerminal::startInteractiveMode(bool blocking) {
    if (!initialized_) {
        ESP_LOGE(TAG, "UART未初始化，无法启动交互模式");
        return;
    }

    if (running_) {
        ESP_LOGW(TAG, "交互模式已在运行");
        return;
    }

    running_ = true;

    if (blocking) {
        interactiveTask();
    } else {
        xTaskCreate(interactiveTaskEntry, "uart_terminal", 4096, this, 5, &task_handle_);
    }
}

void UartTerminal::stopInteractiveMode() {
    if (!running_) {
        return;
    }

    running_ = false;

    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }

    ESP_LOGI(TAG, "交互模式已停止");
}

void UartTerminal::print(const std::string& message) {
    if (!initialized_) {
        return;
    }
    uart_write_bytes(config_.uart_port, message.c_str(), message.length());
}

void UartTerminal::println(const std::string& message) {
    print(message + "\n");
}

bool UartTerminal::readLine(char* buffer, size_t buffer_size, uint32_t timeout_ms) {
    if (!initialized_ || !buffer || buffer_size == 0) {
        return false;
    }

    int pos = 0;
    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while (pos < buffer_size - 1) {
        uint8_t data;
        int len = uart_read_bytes(config_.uart_port, &data, 1, pdMS_TO_TICKS(config_.read_timeout_ms));

        if (len > 0) {
            if (data == '\n' || data == '\r') {
                buffer[pos] = '\0';
                return pos > 0;
            } else if (data >= 32 && data <= 126) { // 可打印字符
                buffer[pos++] = data;
                uart_write_bytes(config_.uart_port, &data, 1); // 回显
            } else if (data == 8 || data == 127) { // 退格键
                if (pos > 0) {
                    pos--;
                    const char backspace[] = "\b \b";
                    uart_write_bytes(config_.uart_port, backspace, 3);
                }
            }
        }

        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (current_time - start_time > timeout_ms) {
            break;
        }

        if (!running_) {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    buffer[pos] = '\0';
    return false;
}

void UartTerminal::parseCommand(const std::string& input, std::string& command, std::vector<std::string>& args) {
    std::istringstream iss(input);
    std::string token;

    args.clear();

    if (iss >> command) {
        while (iss >> token) {
            args.push_back(token);
        }
    }
}

bool UartTerminal::processCommand(const std::string& input) {
    if (input.empty()) {
        return true;
    }

    std::string command;
    std::vector<std::string> args;
    parseCommand(input, command, args);

    // 查找注册的命令
    auto it = commands_.find(command);
    if (it != commands_.end()) {
        return it->second.handler(input, args);
    }

    // 尝试默认处理器
    if (default_handler_) {
        return default_handler_(input, args);
    }

    // 未识别的命令
    println("未识别的命令: " + command + "，输入 'help' 查看帮助");
    return true;
}

void UartTerminal::showHelp() {
    println("\n=== 可用命令 ===");

    for (const auto& pair : commands_) {
        const std::string& cmd = pair.first;
        const CommandInfo& info = pair.second;
        println("  " + cmd + " - " + info.description);
        if (!info.usage.empty()) {
            println("    用法: " + info.usage);
        }
    }

    println("");
}

void UartTerminal::interactiveTask() {
    // 显示欢迎信息
    println("\n" + welcome_message_);
    println("输入 'help' 查看可用命令，输入 'exit' 退出");

    char input_buffer[256];

    while (running_) {
        print(prompt_ + "> ");

        if (readLine(input_buffer, sizeof(input_buffer), config_.input_timeout_ms)) {
            println(""); // 换行

            std::string input(input_buffer);
            // 移除首尾空白字符
            input.erase(0, input.find_first_not_of(" \t"));
            input.erase(input.find_last_not_of(" \t") + 1);

            if (!input.empty()) {
                if (!processCommand(input)) {
                    break; // 命令处理器返回false表示退出
                }
            }
        } else {
            // 超时或无输入，继续循环
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    running_ = false;
}

void UartTerminal::interactiveTaskEntry(void* pvParameters) {
    auto* terminal = static_cast<UartTerminal*>(pvParameters);
    terminal->interactiveTask();
    vTaskDelete(nullptr);
}

} // namespace Utils
} // namespace Robot