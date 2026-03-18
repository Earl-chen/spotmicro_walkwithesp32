/**
 * @file test_pca9685_driver_refactored.cpp
 * @brief PCA9685硬件驱动交互式测试程序 - 使用UartTerminal重构版本
 *
 * ⚠️ 安全警告 ⚠️
 * 本测试程序使用实际校准的舵机PWM值。
 * 不同舵机有不同的校准范围，请勿使用统一的PWM值！
 * 错误的PWM值可能导致舵机撞击限位或机械损坏。
 *
 * 实际校准数据来源: body_control/main/config/ConfigManager.cpp
 */

#include "../drivers/PCA9685Driver.hpp"
#include "../utils/UartTerminal.hpp"
#include <stdio.h>
#include <string.h>
#include <cstdlib>

using namespace Robot::Drivers;
using namespace Robot::Utils;

/**
 * @brief 实际舵机校准配置
 *
 * 每个舵机都有独立的校准值，中位值(90°)在615-685 PWM范围内。
 * 警告: 不要使用350作为中位值！这是错误的！
 */
struct ServoCalibration {
    uint16_t min_pwm;    // 0度位置
    uint16_t center_pwm; // 90度位置 (中位)
    uint16_t max_pwm;    // 180度位置
};

// 实际校准数据 - 来自 body_control 项目的 ConfigManager.cpp
static const ServoCalibration SERVO_CALIBRATION[12] = {
    // 左前腿 (舵机0,1,2)
    {170, 615, 1060},  // 舵机0: 170→615→1060
    {170, 625, 1080},  // 舵机1: 170→625→1080
    {170, 625, 1080},  // 舵机2: 170→625→1080
    // 右前腿 (舵机3,4,5)
    {240, 685, 1130},  // 舵机3: 240→685→1130
    {170, 630, 1090},  // 舵机4: 170→630→1090
    {185, 635, 1085},  // 舵机5: 185→635→1085
    // 左后腿 (舵机6,7,8)
    {190, 655, 1120},  // 舵机6: 190→655→1120
    {200, 655, 1110},  // 舵机7: 200→655→1110
    {195, 655, 1110},  // 舵机8: 195→655→1110
    // 右后腿 (舵机9,10,11)
    {210, 670, 1130},  // 舵机9: 210→670→1130
    {180, 630, 1080},  // 舵机10: 180→630→1080
    {180, 630, 1080},  // 舵机11: 180→630→1080
};

// 获取舵机的安全中位PWM值
uint16_t getServoCenterPWM(int servo_id) {
    if (servo_id >= 0 && servo_id < 12) {
        return SERVO_CALIBRATION[servo_id].center_pwm;
    }
    return 630; // 默认安全值
}

// 全局变量
static std::shared_ptr<PCA9685Driver> pca9685_driver;
static int current_servo = 0;

/**
 * @brief PCA9685测试终端类
 */
class PCA9685TestTerminal : public UartTerminal {
public:
    PCA9685TestTerminal() : UartTerminal(
        UartTerminalConfig{},
        "PCA9685[" + std::to_string(current_servo) + "]",
        // 更详细的欢迎信息
        "\n"
        "╔══════════════════════════════════════════════════════════════╗\n"
        "║     01_PCA9685舵机驱动独立测试程序 (序列号: 01)              ║\n"
        "╠══════════════════════════════════════════════════════════════╣\n"
        "║  硬件: ESP32 + PCA9685 (I2C地址: 0x40)                       ║\n"
        "║  引脚: SDA=GPIO21, SCL=GPIO22                                ║\n"
        "║  舵机: 12个舵机 (四足机器人×4腿×3关节)                        ║\n"
        "╠══════════════════════════════════════════════════════════════╣\n"
        "║  ⚠️  安全警告:                                                ║\n"
        "║  - 每个舵机有独立的校准范围，请勿使用统一的PWM值!             ║\n"
        "║  - 中位PWM范围: 615-685 (非350!)                             ║\n"
        "║  - 首次测试请断开机械结构，单独测试舵机响应                   ║\n"
        "╚══════════════════════════════════════════════════════════════╝\n"
        "\n"
        "📖 快速入门:\n"
        "  1. 输入 's 0'       - 选择舵机0\n"
        "  2. 输入 'cal'       - 查看舵机0的校准参数\n"
        "  3. 输入 'test'      - 运行舵机0的测试序列\n"
        "  4. 输入 'scan'      - 扫描所有舵机连接状态\n"
        "  5. 输入 'help'      - 查看所有可用命令\n"
        "\n"
        "💡 提示: 直接输入PWM数值(如630)可快速设置当前舵机位置") {

        registerCommands();
    }

private:
    void registerCommands() {
        // 选择舵机命令
        registerCommand("s", "选择舵机 (0-11)", "s <舵机编号>\n"
                       "  示例: s 0   - 选择舵机0\n"
                       "        s 5   - 选择舵机5",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleSelectServo(args);
                       });

        // 设置PWM命令
        registerCommand("pwm", "设置当前舵机PWM值", "pwm <PWM值>\n"
                       "  示例: pwm 630   - 设置到中位\n"
                       "  注意: 不同舵机有不同范围，请先用'cal'查看",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleSetPWM(args);
                       });

        // 读取PWM命令
        registerCommand("read", "读取舵机当前PWM值", "read [舵机编号]\n"
                       "  示例: read     - 读取当前舵机\n"
                       "        read 3   - 读取舵机3的PWM值",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleReadPWM(args);
                       });

        // 频率设置命令
        registerCommand("freq", "设置PWM频率 (50-1000Hz)", "freq <频率>\n"
                       "  示例: freq 50    - 标准舵机频率\n"
                       "        freq 100   - 高速舵机",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleSetFrequency(args);
                       });

        // 全部关闭命令
        registerCommand("off", "关闭所有舵机 (PWM=0)", "off\n"
                       "  将所有舵机的PWM设置为0，使舵机失去扭矩",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleAllOff(args);
                       });

        // 测试序列命令
        registerCommand("test", "运行舵机测试序列", "test [舵机编号]\n"
                       "  示例: test    - 测试当前舵机\n"
                       "        test 5  - 测试舵机5\n"
                       "  动作: min→center→max→center→min",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleTestSequence(args);
                       });

        // 扫描命令
        registerCommand("scan", "扫描所有12个舵机连接状态", "scan\n"
                       "  对每个舵机设置中位PWM并尝试读取响应\n"
                       "  用于检测舵机是否正常连接",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleScanServos(args);
                       });

        // 状态命令
        registerCommand("status", "显示系统状态", "status",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           showSystemStatus();
                           return true;
                       });

        // 设置默认处理器（处理纯数字输入作为PWM值）
        setDefaultHandler([this](const std::string& input, const std::vector<std::string>& args) {
            // 尝试解析为PWM值
            char* endptr;
            int pwm_value = std::strtol(input.c_str(), &endptr, 10);
            const auto& cal = SERVO_CALIBRATION[current_servo];
            // 使用实际校准范围进行检查
            if (*endptr == '\0' && pwm_value >= cal.min_pwm && pwm_value <= cal.max_pwm) {
                return setPWM(current_servo, pwm_value);
            } else if (*endptr == '\0') {
                println("⚠️ PWM值超出范围! 当前舵机" + std::to_string(current_servo) +
                       " 有效范围: " + std::to_string(cal.min_pwm) + "-" + std::to_string(cal.max_pwm));
                println("  提示: 输入 'cal' 查看校准参数");
                return true;
            } else {
                println("未知命令: " + input + "，输入 'help' 查看帮助");
                return true;
            }
        });
    }

    bool handleSelectServo(const std::vector<std::string>& args) {
        if (args.empty()) {
            println("错误: 缺少舵机ID参数");
            return true;
        }

        int servo_id = std::atoi(args[0].c_str());
        if (servo_id >= 0 && servo_id < 12) {
            current_servo = servo_id;
            println("[OK] 选择舵机" + std::to_string(servo_id));
            updatePrompt();
            showCurrentServoInfo();
        } else {
            println("[ERROR] 无效的舵机ID: " + std::to_string(servo_id) + " (范围: 0-11)");
        }
        return true;
    }

    bool handleSetPWM(const std::vector<std::string>& args) {
        if (args.empty()) {
            println("错误: 缺少PWM值参数");
            return true;
        }

        int pwm_value = std::atoi(args[0].c_str());
        return setPWM(current_servo, pwm_value);
    }

    bool handleReadPWM(const std::vector<std::string>& args) {
        int servo_id = current_servo;
        if (!args.empty()) {
            servo_id = std::atoi(args[0].c_str());
            if (servo_id < 0 || servo_id >= 12) {
                println("[ERROR] 无效的舵机ID: " + std::to_string(servo_id));
                return true;
            }
        }

        uint16_t pwm_value;
        PCA9685Error result = pca9685_driver->getPWM(servo_id, pwm_value);
        if (result == PCA9685Error::SUCCESS) {
            println("[READ] 舵机" + std::to_string(servo_id) + " PWM值: " + std::to_string(pwm_value));
        } else {
            println("[ERROR] 读取舵机" + std::to_string(servo_id) + " PWM值失败");
        }
        return true;
    }

    bool handleSetFrequency(const std::vector<std::string>& args) {
        if (args.empty()) {
            println("错误: 缺少频率参数");
            return true;
        }

        int frequency = std::atoi(args[0].c_str());
        if (frequency < 50 || frequency > 1000) {
            println("[ERROR] 频率超出范围: " + std::to_string(frequency) + "Hz (范围: 50-1000Hz)");
            return true;
        }

        println("[FREQ] 设置PWM频率到 " + std::to_string(frequency) + "Hz...");
        PCA9685Error result = pca9685_driver->setFrequency(frequency);
        if (result == PCA9685Error::SUCCESS) {
            println("[OK] 频率设置成功");
        } else {
            println("[ERROR] 频率设置失败");
        }
        return true;
    }

    bool handleAllOff(const std::vector<std::string>& args) {
        println("[OFF] 关闭所有舵机...");

        for (int i = 0; i < 12; i++) {
            PCA9685Error result = pca9685_driver->setPWM(i, 0);
            if (result == PCA9685Error::SUCCESS) {
                println("  舵机" + std::to_string(i) + ": [OFF]");
            } else {
                println("  舵机" + std::to_string(i) + ": [ERROR]");
            }
        }

        println("[OFF] 完成");
        return true;
    }

    bool handleTestSequence(const std::vector<std::string>& args) {
        int test_servo = current_servo;
        if (!args.empty()) {
            test_servo = std::atoi(args[0].c_str());
            if (test_servo < 0 || test_servo >= 12) {
                println("[ERROR] 无效的舵机ID: " + std::to_string(test_servo));
                return true;
            }
        }

        println("[TEST] 开始测试序列 - 舵机" + std::to_string(test_servo) + ":");

        // 使用实际校准数据，而非固定的200/350/500
        const auto& cal = SERVO_CALIBRATION[test_servo];
        int test_pwm_values[] = {
            (int)cal.min_pwm,
            (int)cal.center_pwm,
            (int)cal.max_pwm,
            (int)cal.center_pwm,
            (int)cal.min_pwm
        };
        const char* test_descriptions[] = {"0度(min)", "90度(center)", "180度(max)", "90度(center)", "0度(min)"};

        for (int i = 0; i < 5; i++) {
            println("  -> PWM " + std::to_string(test_pwm_values[i]) + " (" + test_descriptions[i] + ")");
            setPWM(test_servo, test_pwm_values[i]);
            vTaskDelay(pdMS_TO_TICKS(1000)); // 延迟1秒
        }

        println("[TEST] 测试序列完成");
        return true;
    }

    bool handleScanServos(const std::vector<std::string>& args) {
        println("[SCAN] 扫描所有舵机连接状态:");
        println("==========================================");

        for (int i = 0; i < 12; i++) {
            // 使用实际校准的中位值测试响应
            uint16_t center_pwm = getServoCenterPWM(i);
            PCA9685Error result = pca9685_driver->setPWM(i, center_pwm);
            if (result == PCA9685Error::SUCCESS) {
                vTaskDelay(pdMS_TO_TICKS(100));
                uint16_t read_value;
                PCA9685Error read_result = pca9685_driver->getPWM(i, read_value);
                if (read_result == PCA9685Error::SUCCESS) {
                    println("舵机" + std::to_string(i) + ": [OK] PWM=" + std::to_string(read_value));
                } else {
                    println("舵机" + std::to_string(i) + ": [WARN] 写入成功但读取失败");
                }
            } else {
                println("舵机" + std::to_string(i) + ": [ERROR] 无响应");
            }
        }

        println("[SCAN] 扫描完成");
        return true;
    }

    bool setPWM(int servo_id, int pwm_value) {
        if (servo_id < 0 || servo_id >= 12) {
            println("[ERROR] 无效的舵机ID: " + std::to_string(servo_id));
            return true;
        }

        if (pwm_value < 0 || pwm_value > 4095) {
            println("[ERROR] PWM值超出范围: " + std::to_string(pwm_value) + " (范围: 0-4095，推荐: 200-500)");
            return true;
        }

        println("[PWM] 舵机" + std::to_string(servo_id) + " 设置PWM到 " + std::to_string(pwm_value) + "...");

        PCA9685Error result = pca9685_driver->setPWM(servo_id, pwm_value);
        if (result == PCA9685Error::SUCCESS) {
            println(" [OK]");
        } else {
            println(" [ERROR] 设置失败");
        }

        return true;
    }

    void updatePrompt() {
        setPrompt("PCA9685[" + std::to_string(current_servo) + "]");
    }

    void showCurrentServoInfo() {
        println("\n[INFO] 当前选择舵机: " + std::to_string(current_servo));

        uint16_t pwm_value;
        PCA9685Error result = pca9685_driver->getPWM(current_servo, pwm_value);
        if (result == PCA9685Error::SUCCESS) {
            println("  当前PWM值: " + std::to_string(pwm_value));
        } else {
            println("  当前PWM值: 读取失败");
        }
    }

    void showSystemStatus() {
        println("\n[STATUS] PCA9685系统状态:");
        println("==========================");

        // 显示设备信息
        println("设备地址: 0x40");
        println("PWM频率: 100Hz (默认)");
        println("当前选择舵机: " + std::to_string(current_servo));

        // 显示活跃的舵机
        println("\n活跃舵机状态:");
        bool has_active = false;
        for (int i = 0; i < 12; i++) {
            uint16_t pwm_value;
            PCA9685Error result = pca9685_driver->getPWM(i, pwm_value);
            if (result == PCA9685Error::SUCCESS && pwm_value > 0) {
                println("  舵机" + std::to_string(i) + ": PWM=" + std::to_string(pwm_value));
                has_active = true;
            }
        }

        if (!has_active) {
            println("  无活跃舵机");
        }

        println("\nPWM参考值 (当前舵机" + std::to_string(current_servo) + "):");
        const auto& cal = SERVO_CALIBRATION[current_servo];
        println("  " + std::to_string(cal.min_pwm) + " ≈ 0度");
        println("  " + std::to_string(cal.center_pwm) + " ≈ 90度 (中位)");
        println("  " + std::to_string(cal.max_pwm) + " ≈ 180度");
        println("\n⚠️ 注意: 不同舵机有不同的校准值!");
        println("  请使用 'cal' 命令查看当前舵机校准参数");
    }
};

/**
 * @brief 主测试函数 - 重构版本
 */
void test_pca9685_driver_refactored() {
    printf("\n[START] PCA9685硬件驱动交互式测试\n");
    printf("=====================================\n");

    // 1. 创建PCA9685驱动
    printf("\n[INIT] 初始化PCA9685 (100Hz频率)...\n");
    pca9685_driver = std::make_shared<PCA9685Driver>(0x40);

    PCA9685Error init_result = pca9685_driver->init(100);
    if (init_result != PCA9685Error::SUCCESS) {
        printf("[ERROR] PCA9685初始化失败，错误代码: %d\n", static_cast<int>(init_result));
        printf("请检查:\n");
        printf("  - I2C接线 (SDA=GPIO21, SCL=GPIO22)\n");
        printf("  - PCA9685模块电源\n");
        printf("  - I2C地址是否为0x40\n");
        return;
    }
    printf("[OK] PCA9685初始化成功\n");

    // 2. 设置所有舵机到安全中位 (使用实际校准值)
    printf("\n[INIT] 设置所有舵机到安全中位...\n");
    printf("⚠️ 注意: 使用实际校准的中位PWM值 (615-685)，而非统一的350!\n");
    for (int i = 0; i < 12; i++) {
        uint16_t center_pwm = getServoCenterPWM(i);
        printf("  舵机%d -> PWM=%d (校准中位)\n", i, center_pwm);
        pca9685_driver->setPWM(i, center_pwm);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    printf("[OK] 舵机初始化完成\n");

    // 3. 创建并启动终端
    PCA9685TestTerminal terminal;

    if (!terminal.init()) {
        printf("[ERROR] UART终端初始化失败\n");
        return;
    }
    printf("[OK] UART终端初始化成功\n\n");

    // 4. 开始交互模式
    terminal.startInteractiveMode(true); // 阻塞模式

    printf("[END] PCA9685硬件驱动测试结束\n");
}