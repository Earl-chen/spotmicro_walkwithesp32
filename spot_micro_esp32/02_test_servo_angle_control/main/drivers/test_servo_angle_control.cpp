/**
 * @file test_servo_angle_control.cpp
 * @brief 舵机角度控制测试程序 - 使用ServoDriver进行角度控制
 *
 * 功能：
 * - 角度→PWM转换测试
 * - 0-180°全范围角度控制
 * - 单舵机/所有舵机控制
 * - 快速测试序列
 *
 * 硬件配置：
 * - PCA9685 I2C地址: 0x40
 * - I2C引脚: SDA=GPIO21, SCL=GPIO22
 * - 舵机数量: 12个 (四足机器人)
 *
 * 安全警告：
 * - 使用实际校准的舵机PWM值 (非默认值)
 * - 首次运行建议使用90度中位测试
 * - 避免舵机到达物理限位
 */

#include "../drivers/PCA9685Driver.hpp"
#include "../drivers/ServoDriver.hpp"
#include "../config/ConfigManager.hpp"
#include "../utils/UartTerminal.hpp"
#include <stdio.h>
#include <string.h>
#include <cstdlib>

using namespace Robot;
using namespace Robot::Drivers;
using namespace Robot::Config;
using namespace Robot::Utils;
using namespace Robot::Controllers;

// 全局变量
static std::shared_ptr<ConfigManager> config_manager;
static std::shared_ptr<PCA9685Driver> pca9685_driver;
static std::shared_ptr<ServoDriver> servo_driver;
static int current_servo = 0;

// 舵机校准数据 (实际测量值)
// 格式: {min_pwm (0度), max_pwm (180度)}
static const struct {
    uint16_t min_pwm;
    uint16_t max_pwm;
    uint16_t center_pwm;  // 计算值，仅供参考
} SERVO_CALIBRATION[12] = {
    {170, 1060, 615},   // 舵机0: 左前腿-髋关节侧摆
    {170, 1080, 625},   // 舵机1: 左前腿-髋关节俯仰
    {170, 1080, 625},   // 舵机2: 左前腿-膝关节俯仰
    {240, 1130, 685},   // 舵机3: 右前腿-髋关节侧摆
    {170, 1090, 630},   // 舵机4: 右前腿-髋关节俯仰
    {185, 1085, 635},   // 舵机5: 右前腿-膝关节俯仰
    {190, 1120, 655},   // 舵机6: 左后腿-髋关节侧摆
    {180, 1080, 630},   // 舵机7: 左后腿-髋关节俯仰 (实际center≈655)
    {195, 1110, 652},   // 舵机8: 左后腿-膝关节俯仰
    {210, 1130, 670},   // 舵机9: 右后腿-髋关节侧摆
    {180, 1080, 630},   // 舵机10: 右后腿-髋关节俯仰
    {180, 1080, 630}    // 舵机11: 右后腿-膝关节俯仰
};

// 腿部和关节名称
static const char* getLegName(int servo_id) {
    if (servo_id >= 0 && servo_id < 3) return "左前腿";
    if (servo_id >= 3 && servo_id < 6) return "右前腿";
    if (servo_id >= 6 && servo_id < 9) return "左后腿";
    if (servo_id >= 9 && servo_id < 12) return "右后腿";
    return "未知";
}

static const char* getJointName(int servo_id) {
    int joint = servo_id % 3;
    switch (joint) {
        case 0: return "髋关节侧摆";
        case 1: return "髋关节俯仰";
        case 2: return "膝关节俯仰";
        default: return "未知";
    }
}

/**
 * @brief 舵机角度控制终端类
 */
class ServoControlTerminal : public UartTerminal {
public:
    ServoControlTerminal() : UartTerminal(
        UartTerminalConfig{},
        "servo[" + std::to_string(current_servo) + "]",
        "ServoDriver角度控制测试终端") {

        registerCommands();
    }

private:
    void registerCommands() {
        // 选择舵机命令
        registerCommand("s", "选择舵机 (0-11)", "s <servo_id>",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleSelectServo(args);
                       });

        // 设置角度命令
        registerCommand("angle", "设置当前舵机角度", "angle <0-180>",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleSetAngle(args);
                       });

        // 快速测试命令
        registerCommand("test", "快速测试单个舵机", "test [servo_id]",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleQuickTest(args);
                       });

        // 所有舵机设置角度
        registerCommand("all", "所有舵机设置角度", "all <angle>",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleSetAllServos(args);
                       });

        // 显示配置信息
        registerCommand("config", "显示舵机配置", "config [servo_id]",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleShowConfig(args);
                       });

        // 显示概览
        registerCommand("overview", "显示所有舵机概览", "overview",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           showAllServosOverview();
                           return true;
                       });

        // 扫描测试
        registerCommand("scan", "扫描单个舵机全范围", "scan <servo_id>",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleScanServo(args);
                       });

        // 设置默认处理器（处理纯数字输入作为角度）
        setDefaultHandler([this](const std::string& input, const std::vector<std::string>& args) {
            // 尝试解析为角度值
            char* endptr;
            float angle = std::strtof(input.c_str(), &endptr);
            if (*endptr == '\0' && angle >= 0.0f && angle <= 180.0f) {
                return setServoAngle(current_servo, angle);
            } else {
                println("未知命令: " + input + "，输入 'help' 查看帮助");
                return true;
            }
        });
    }

    bool handleSelectServo(const std::vector<std::string>& args) {
        if (args.empty()) {
            println("用法: s <servo_id>");
            println("  servo_id: 0-11");
            return true;
        }

        int servo_id = std::atoi(args[0].c_str());
        if (servo_id >= 0 && servo_id < 12) {
            current_servo = servo_id;
            println("[OK] 选择舵机" + std::to_string(servo_id) + " (" +
                   std::string(getLegName(servo_id)) + " - " +
                   std::string(getJointName(servo_id)) + ")");
            updatePrompt();
            showCurrentServoInfo();
        } else {
            println("[ERROR] 无效的舵机ID: " + std::to_string(servo_id) + " (范围: 0-11)");
        }
        return true;
    }

    bool handleSetAngle(const std::vector<std::string>& args) {
        if (args.empty()) {
            println("用法: angle <0-180>");
            println("  设置当前舵机到指定角度");
            return true;
        }

        float angle = std::atof(args[0].c_str());
        return setServoAngle(current_servo, angle);
    }

    bool handleQuickTest(const std::vector<std::string>& args) {
        int test_servo = current_servo;
        if (!args.empty()) {
            test_servo = std::atoi(args[0].c_str());
            if (test_servo < 0 || test_servo >= 12) {
                println("[ERROR] 无效的舵机ID: " + std::to_string(test_servo));
                return true;
            }
        }

        println("[TEST] 开始快速测试舵机" + std::to_string(test_servo) + ":");
        println("  测试序列: 0° -> 90° -> 180° -> 90°");

        float test_angles[] = {0.0f, 90.0f, 180.0f, 90.0f};
        for (int i = 0; i < 4; i++) {
            println("  -> " + std::to_string(static_cast<int>(test_angles[i])) + "度");
            setServoAngle(test_servo, test_angles[i]);
            vTaskDelay(pdMS_TO_TICKS(1000)); // 延迟1秒
        }

        println("[TEST] 快速测试完成");
        return true;
    }

    bool handleSetAllServos(const std::vector<std::string>& args) {
        if (args.empty()) {
            println("用法: all <angle>");
            println("  angle: 0-180度，设置所有舵机到同一角度");
            println("  注意: 建议先使用90度测试安全性");
            return true;
        }

        float angle = std::atof(args[0].c_str());
        if (angle < 0.0f || angle > 180.0f) {
            println("[ERROR] 角度超出范围: " + std::to_string(angle) + "度 (范围: 0-180度)");
            return true;
        }

        println("[ALL] 设置所有舵机到 " + std::to_string(static_cast<int>(angle)) + "度...");

        for (int i = 0; i < 12; i++) {
            ServoError result = servo_driver->setAngle(i, angle);
            if (result == ServoError::SUCCESS) {
                println("  舵机" + std::to_string(i) + ": [OK]");
            } else {
                println("  舵机" + std::to_string(i) + ": [ERROR] " +
                       std::string(ServoDriver::getErrorString(result)));
            }
            vTaskDelay(pdMS_TO_TICKS(50)); // 小延迟避免电流冲击
        }

        println("[ALL] 完成");
        return true;
    }

    bool handleShowConfig(const std::vector<std::string>& args) {
        int servo_id = current_servo;
        if (!args.empty()) {
            servo_id = std::atoi(args[0].c_str());
            if (servo_id < 0 || servo_id >= 12) {
                println("[ERROR] 无效的舵机ID: " + std::to_string(servo_id));
                return true;
            }
        }

        const auto& calib = SERVO_CALIBRATION[servo_id];
        println("\n[CONFIG] 舵机" + std::to_string(servo_id) + " 配置信息:");
        println("  腿部位置: " + std::string(getLegName(servo_id)));
        println("  关节类型: " + std::string(getJointName(servo_id)));
        println("  PWM范围: " + std::to_string(calib.min_pwm) + " (0°) - " +
               std::to_string(calib.center_pwm) + " (90°) - " +
               std::to_string(calib.max_pwm) + " (180°)");

        // 计算当前PWM值
        uint16_t current_pwm;
        float current_angle;
        if (servo_driver->getAngle(servo_id, current_angle) == ServoError::SUCCESS &&
            servo_driver->angleToPWM(servo_id, current_angle, current_pwm) == ServoError::SUCCESS) {
            println("  当前状态: " + std::to_string(static_cast<int>(current_angle)) +
                   "° -> PWM " + std::to_string(current_pwm));
        }

        return true;
    }

    bool handleScanServo(const std::vector<std::string>& args) {
        if (args.empty()) {
            println("用法: scan <servo_id>");
            println("  扫描指定舵机的0-180度全范围");
            println("  警告: 确保舵机没有物理阻挡!");
            return true;
        }

        int servo_id = std::atoi(args[0].c_str());
        if (servo_id < 0 || servo_id >= 12) {
            println("[ERROR] 无效的舵机ID: " + std::to_string(servo_id));
            return true;
        }

        println("[SCAN] 开始扫描舵机" + std::to_string(servo_id) + "全范围...");
        println("  从0度到180度，每10度一步");

        for (int angle = 0; angle <= 180; angle += 10) {
            uint16_t pwm;
            servo_driver->angleToPWM(servo_id, static_cast<float>(angle), pwm);
            println("  " + std::to_string(angle) + "° -> PWM " + std::to_string(pwm));
            servo_driver->setAngle(servo_id, static_cast<float>(angle));
            vTaskDelay(pdMS_TO_TICKS(500)); // 每步延迟500ms
        }

        // 返回中位
        servo_driver->setAngle(servo_id, 90.0f);
        println("[SCAN] 完成，已返回90度中位");
        return true;
    }

    bool setServoAngle(int servo_id, float angle) {
        if (servo_id < 0 || servo_id >= 12) {
            println("[ERROR] 无效的舵机ID: " + std::to_string(servo_id));
            return true;
        }

        if (angle < 0.0f || angle > 180.0f) {
            println("[ERROR] 角度超出范围: " + std::to_string(angle) + "度 (范围: 0-180度)");
            return true;
        }

        // 获取PWM值
        uint16_t pwm_value;
        servo_driver->angleToPWM(servo_id, angle, pwm_value);

        println("[SET] 舵机" + std::to_string(servo_id) + " (" +
               std::string(getLegName(servo_id)) + "-" +
               std::string(getJointName(servo_id)) + ") -> " +
               std::to_string(static_cast<int>(angle)) + "° (PWM: " +
               std::to_string(pwm_value) + ")");

        ServoError result = servo_driver->setAngle(servo_id, angle);
        if (result == ServoError::SUCCESS) {
            println(" [OK]");
        } else {
            println(" [ERROR] " + std::string(ServoDriver::getErrorString(result)));
        }

        return true;
    }

    void updatePrompt() {
        setPrompt("servo[" + std::to_string(current_servo) + "]");
    }

    void showCurrentServoInfo() {
        const auto& calib = SERVO_CALIBRATION[current_servo];
        println("\n[INFO] 当前舵机信息:");
        println("  ID: " + std::to_string(current_servo));
        println("  位置: " + std::string(getLegName(current_servo)) + " - " +
               std::string(getJointName(current_servo)));
        println("  PWM: " + std::to_string(calib.min_pwm) + "-" +
               std::to_string(calib.center_pwm) + "-" + std::to_string(calib.max_pwm));
    }

    void showAllServosOverview() {
        println("\n[OVERVIEW] 舵机配置概览:");
        println("============================================================");
        println(" ID  | 腿部   | 关节         | PWM范围 (0°-90°-180°)");
        println("-----|--------|--------------|---------------------------");

        const char* legs[] = {"左前", "右前", "左后", "右后"};
        const char* joints[] = {"髋侧摆", "髋俯仰", "膝俯仰"};

        for (int i = 0; i < 12; i++) {
            int leg_idx = i / 3;
            int joint_idx = i % 3;
            const auto& calib = SERVO_CALIBRATION[i];

            char line[80];
            snprintf(line, sizeof(line), " %2d  | %-6s | %-12s | %4d - %4d - %4d",
                    i, legs[leg_idx], joints[joint_idx],
                    calib.min_pwm, calib.center_pwm, calib.max_pwm);
            println(line);
        }
        println("");
    }
};

/**
 * @brief 打印欢迎信息
 */
static void printWelcomeMessage() {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║     02_test_servo_angle_control - 舵机角度控制测试        ║\n");
    printf("╠════════════════════════════════════════════════════════════╣\n");
    printf("║  硬件配置:                                                 ║\n");
    printf("║    - PCA9685 I2C地址: 0x40                                 ║\n");
    printf("║    - I2C引脚: SDA=GPIO21, SCL=GPIO22                       ║\n");
    printf("║    - 舵机数量: 12个 (四足机器人)                           ║\n");
    printf("╠════════════════════════════════════════════════════════════╣\n");
    printf("║  功能说明:                                                 ║\n");
    printf("║    - 角度→PWM转换测试 (使用实际校准数据)                   ║\n");
    printf("║    - 0-180°全范围角度控制                                  ║\n");
    printf("║    - 单舵机/所有舵机控制                                   ║\n");
    printf("║    - 快速测试序列                                          ║\n");
    printf("╠════════════════════════════════════════════════════════════╣\n");
    printf("║  安全警告:                                                 ║\n");
    printf("║    ⚠ 使用实际舵机校准数据 (非默认值)                       ║\n");
    printf("║    ⚠ 首次运行建议使用90度中位测试                          ║\n");
    printf("║    ⚠ 避免舵机到达物理限位                                  ║\n");
    printf("║    ⚠ 使用 'all' 命令前确保所有舵机可安全移动               ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n");
}

/**
 * @brief 打印快速开始指南
 */
static void printQuickStart() {
    printf("\n");
    printf("┌─────────────────────────────────────────────────────────────┐\n");
    printf("│  快速开始指南                                               │\n");
    printf("├─────────────────────────────────────────────────────────────┤\n");
    printf("│  1. 查看所有舵机配置:                                       │\n");
    printf("│     > overview                                              │\n");
    printf("│                                                             │\n");
    printf("│  2. 选择舵机:                                               │\n");
    printf("│     > s 0          (选择舵机0)                              │\n");
    printf("│                                                             │\n");
    printf("│  3. 设置角度 (直接输入数字):                                │\n");
    printf("│     > 90            (设置到90度)                            │\n");
    printf("│     > angle 45      (使用angle命令)                         │\n");
    printf("│                                                             │\n");
    printf("│  4. 快速测试单个舵机:                                       │\n");
    printf("│     > test 0        (测试舵机0: 0°→90°→180°→90°)            │\n");
    printf("│                                                             │\n");
    printf("│  5. 扫描舵机全范围:                                         │\n");
    printf("│     > scan 0        (0°→180°，每10度一步)                   │\n");
    printf("│                                                             │\n");
    printf("│  6. 设置所有舵机 (谨慎使用):                                │\n");
    printf("│     > all 90        (所有舵机到90度)                        │\n");
    printf("└─────────────────────────────────────────────────────────────┘\n");
    printf("\n");
}

/**
 * @brief 主测试函数
 */
void test_servo_angle_control() {
    // 打印欢迎信息
    printWelcomeMessage();

    printf("[START] 舵机角度控制测试初始化...\n");
    printf("====================================\n");

    // 1. 创建配置管理器
    printf("\n[CONFIG] 创建配置管理器...\n");
    config_manager = std::make_shared<ConfigManager>();
    printf("[OK] 配置管理器创建成功\n");

    // 2. 创建PCA9685硬件驱动
    printf("\n[INIT] 初始化PCA9685...\n");
    pca9685_driver = std::make_shared<PCA9685Driver>(0x40);
    if (pca9685_driver->init(100) != PCA9685Error::SUCCESS) {
        printf("[ERROR] PCA9685初始化失败\n");
        printf("  请检查:\n");
        printf("  1. I2C接线是否正确 (SDA=GPIO21, SCL=GPIO22)\n");
        printf("  2. PCA9685供电是否正常\n");
        printf("  3. I2C地址是否正确 (0x40)\n");
        return;
    }
    printf("[OK] PCA9685初始化成功 (100Hz)\n");

    // 3. 创建舵机驱动
    printf("\n[INIT] 初始化ServoDriver...\n");
    servo_driver = std::make_shared<ServoDriver>(pca9685_driver, config_manager);
    if (servo_driver->init() != ServoError::SUCCESS) {
        printf("[ERROR] ServoDriver初始化失败\n");
        return;
    }
    printf("[OK] ServoDriver初始化成功\n");

    // 4. 将所有舵机设置到90度中位
    printf("\n[SAFE] 将所有舵机设置到90度中位...\n");
    for (int i = 0; i < 12; i++) {
        servo_driver->setAngle(i, 90.0f);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    printf("[OK] 所有舵机已到中位\n");

    // 5. 打印快速开始指南
    printQuickStart();

    // 6. 创建并启动终端
    ServoControlTerminal terminal;

    if (!terminal.init()) {
        printf("[ERROR] UART终端初始化失败\n");
        return;
    }
    printf("[OK] UART终端初始化成功\n");
    printf("\n输入 'help' 查看所有命令，'overview' 查看舵机配置\n\n");

    // 7. 开始交互模式
    terminal.startInteractiveMode(true); // 阻塞模式

    printf("[END] 舵机角度控制测试结束\n");
}
