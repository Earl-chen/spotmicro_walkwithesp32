# UartTerminal 使用指南

## 概述

`UartTerminal` 是一个统一的UART终端交互工具类，用于替代项目中重复的UART初始化、命令解析和交互循环代码。

## 设计优势

### 原有问题
- 每个测试文件都有重复的UART初始化代码
- 相似的 `read_line_from_uart` 函数
- 重复的命令解析逻辑
- 难以维护和扩展

### 解决方案
- **统一UART管理**: 自动初始化和配置
- **命令注册机制**: 基于回调函数的命令处理
- **可扩展架构**: 继承或组合方式使用
- **内置通用命令**: help、exit、clear等

## 使用方式

### 方式1: 继承UartTerminal（推荐）

```cpp
class MyTestTerminal : public UartTerminal {
public:
    MyTestTerminal() : UartTerminal(
        UartTerminalConfig{},
        "mytest",
        "我的测试终端") {
        registerCommands();
    }

private:
    void registerCommands() {
        registerCommand("test", "测试命令", "test <参数>",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleTestCommand(args);
                       });
    }

    bool handleTestCommand(const std::vector<std::string>& args) {
        println("执行测试命令");
        return true; // 返回true继续，false退出
    }
};

void main() {
    MyTestTerminal terminal;
    if (terminal.init()) {
        terminal.startInteractiveMode(true);
    }
}
```

### 方式2: 组合使用

```cpp
void main() {
    UartTerminal terminal(UartTerminalConfig{}, "demo", "演示终端");

    // 注册命令
    terminal.registerCommand("demo", "演示命令", "demo",
                           [](const std::string& input, const std::vector<std::string>& args) {
                               printf("这是演示命令\n");
                               return true;
                           });

    // 设置默认处理器（处理未注册的命令）
    terminal.setDefaultHandler([](const std::string& input, const std::vector<std::string>& args) {
        printf("未知命令: %s\n", input.c_str());
        return true;
    });

    if (terminal.init()) {
        terminal.startInteractiveMode(true);
    }
}
```

## 配置选项

```cpp
struct UartTerminalConfig {
    uart_port_t uart_port = UART_NUM_0;           // UART端口
    int baud_rate = 115200;                       // 波特率
    uart_word_length_t data_bits = UART_DATA_8_BITS;
    uart_parity_t parity = UART_PARITY_DISABLE;
    uart_stop_bits_t stop_bits = UART_STOP_BITS_1;
    uart_hw_flowcontrol_t flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    size_t rx_buffer_size = 1024;                 // 接收缓冲区大小
    size_t tx_buffer_size = 0;                    // 发送缓冲区大小
    int queue_size = 0;                           // 队列大小
    uint32_t read_timeout_ms = 100;               // 单次读取超时
    uint32_t input_timeout_ms = 30000;            // 输入等待超时
};
```

## 重构步骤

### 1. 现有文件重构

将现有的测试文件按以下步骤重构：

1. **移除重复代码**
   - 删除 `uart_init` 函数
   - 删除 `read_line_from_uart` 函数
   - 删除主要的交互循环

2. **提取命令逻辑**
   - 将命令处理函数改为接受 `std::vector<std::string>` 参数
   - 返回 `bool` 类型（true继续，false退出）

3. **创建终端类**
   - 继承 `UartTerminal`
   - 在构造函数中注册所有命令
   - 实现命令处理函数

### 2. 命令处理函数签名

```cpp
// 原来的函数
void process_command(const char* input) {
    if (sscanf(input, "set %d %f", &servo_id, &angle) == 2) {
        // 处理设置命令
    }
}

// 重构后的函数
bool handleSetCommand(const std::vector<std::string>& args) {
    if (args.size() >= 2) {
        int servo_id = std::atoi(args[0].c_str());
        float angle = std::atof(args[1].c_str());
        // 处理设置命令
    }
    return true;
}
```

## 迁移映射

### 原有文件 → 新架构

| 原有文件 | 重构后类名 | 主要功能 |
|---------|------------|----------|
| `test_joint_controller.cpp` | `JointControllerTerminal` | 关节控制测试 |
| `test_servo_angle_control.cpp` | `ServoControlTerminal` | 舵机角度控制 |
| `test_pca9685_driver.cpp` | `PCA9685Terminal` | PCA9685驱动测试 |
| `pca9685Test.cpp` | `PCA9685TestTerminal` | 综合PCA9685测试 |

## 内置命令

所有终端都自动包含以下命令：

- `help` - 显示帮助信息
- `exit` - 退出终端
- `clear` - 清屏

## 优势总结

1. **代码复用**: 消除重复的UART处理代码
2. **统一接口**: 所有测试使用相同的交互模式
3. **易于扩展**: 新增命令只需注册回调函数
4. **更好维护**: 集中管理UART相关功能
5. **类型安全**: 使用C++标准容器和字符串

## 注意事项

1. 确保在CMakeLists.txt中包含新的源文件
2. 命令处理函数应该处理参数不足的情况
3. 使用 `println()` 而不是 `printf()` 进行输出
4. 命令名称应该简短且直观