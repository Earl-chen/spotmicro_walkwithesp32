/**
 * @file test_joint_controller.cpp
 * @brief 关节控制器测试程序 - 使用UartTerminal交互
 *
 * 功能：
 * - 关节映射验证 (关节角度 ↔ 舵机角度)
 * - 预设姿势 (睡眠/站立)
 * - 单关节角度控制
 * - 四腿协调控制
 *
 * 硬件配置：
 * - PCA9685 I2C地址: 0x40
 * - I2C引脚: SDA=GPIO21, SCL=GPIO22
 * - 舵机数量: 12个 (四足机器人)
 *
 * 机器人坐标系：
 * - X轴: 向前为正
 * - Y轴: 向左为正
 * - Z轴: 向上为正
 *
 * 关节角度范围：
 * - 髋关节侧摆: -90° ~ +90° (绕X轴)
 * - 髋关节俯仰: -90° ~ +90° (绕Y轴)
 * - 膝关节俯仰: -180° ~ 0° (绕Y轴)
 */

#include "../controllers/JointController.hpp"
#include "../controllers/RobotController.hpp"
#include "../config/ConfigManager.hpp"
#include "../utils/UartTerminal.hpp"
#include "../utils/Logger.hpp"
#include <stdio.h>
#include <string.h>
#include <cstdlib>

using namespace Robot;
using namespace Robot::Controllers;
using namespace Robot::Config;
using namespace Robot::Utils;

// 全局变量
static std::shared_ptr<RobotController> robot_controller;
static std::shared_ptr<JointController> joint_controller;
static LegID current_leg = LegID::FRONT_LEFT;
static JointType current_joint = JointType::HIP_ROLL;

// 腿部和关节名称
static const char* getLegName(LegID leg_id) {
    switch (leg_id) {
        case LegID::FRONT_LEFT:  return "左前";
        case LegID::FRONT_RIGHT: return "右前";
        case LegID::BACK_LEFT:   return "左后";
        case LegID::BACK_RIGHT:  return "右后";
        default: return "未知";
    }
}

static const char* getJointName(JointType joint_type) {
    switch (joint_type) {
        case JointType::HIP_ROLL:  return "髋侧摆";
        case JointType::HIP_PITCH: return "髋俯仰";
        case JointType::KNEE_PITCH: return "膝俯仰";
        default: return "未知";
    }
}

static void getJointRange(JointType joint_type, float& min_angle, float& max_angle) {
    switch (joint_type) {
        case JointType::HIP_ROLL:
        case JointType::HIP_PITCH:
            min_angle = -90.0f;
            max_angle = 90.0f;
            break;
        case JointType::KNEE_PITCH:
            min_angle = -180.0f;
            max_angle = 0.0f;
            break;
        default:
            min_angle = 0.0f;
            max_angle = 0.0f;
    }
}

/**
 * @brief 关节控制终端类
 */
class JointControlTerminal : public UartTerminal {
public:
    JointControlTerminal() : UartTerminal(
        UartTerminalConfig{},
        "joint[" + std::string(getLegName(current_leg)) + "-" + std::string(getJointName(current_joint)) + "]",
        "关节控制器测试终端") {

        registerCommands();
    }

private:
    void registerCommands() {
        // 选择腿部
        registerCommand("leg", "选择腿部 (0-3)", "leg <0-3>",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleSelectLeg(args);
                       });

        // 选择关节
        registerCommand("joint", "选择关节 (0-2)", "joint <0-2>",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleSelectJoint(args);
                       });

        // 设置角度
        registerCommand("angle", "设置关节角度", "angle <度>",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleSetAngle(args);
                       });

        // 获取当前角度
        registerCommand("get", "获取当前关节角度", "get",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleGetAngle();
                       });

        // 预设姿势
        registerCommand("pose", "设置预设姿势", "pose <sleep|stand>",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handlePose(args);
                       });

        // 映射验证
        registerCommand("mapping", "显示映射关系验证", "mapping",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return showMappingVerification();
                       });

        // 系统状态
        registerCommand("status", "显示所有关节状态", "status",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return showSystemStatus();
                       });

        // 设置默认处理器（纯数字作为角度）
        setDefaultHandler([this](const std::string& input, const std::vector<std::string>& args) {
            char* endptr;
            float angle = std::strtof(input.c_str(), &endptr);
            if (*endptr == '\0') {
                return setJointAngle(angle);
            }
            println("未知命令: " + input + "，输入 'help' 查看帮助");
            return true;
        });
    }

    bool handleSelectLeg(const std::vector<std::string>& args) {
        if (args.empty()) {
            println("用法: leg <0-3>");
            println("  0=左前, 1=右前, 2=左后, 3=右后");
            return true;
        }

        int id = std::atoi(args[0].c_str());
        if (id >= 0 && id < 4) {
            current_leg = static_cast<LegID>(id);
            println("[OK] 选择腿部: " + std::string(getLegName(current_leg)));
            updatePrompt();
            showCurrentSelection();
        } else {
            println("[ERROR] 无效的腿部ID: " + std::to_string(id) + " (范围: 0-3)");
        }
        return true;
    }

    bool handleSelectJoint(const std::vector<std::string>& args) {
        if (args.empty()) {
            println("用法: joint <0-2>");
            println("  0=髋侧摆, 1=髋俯仰, 2=膝俯仰");
            return true;
        }

        int id = std::atoi(args[0].c_str());
        if (id >= 0 && id < 3) {
            current_joint = static_cast<JointType>(id);
            println("[OK] 选择关节: " + std::string(getJointName(current_joint)));
            updatePrompt();
            showCurrentSelection();
        } else {
            println("[ERROR] 无效的关节ID: " + std::to_string(id) + " (范围: 0-2)");
        }
        return true;
    }

    bool handleSetAngle(const std::vector<std::string>& args) {
        if (args.empty()) {
            println("用法: angle <度>");
            float min_a, max_a;
            getJointRange(current_joint, min_a, max_a);
            println("  角度范围: " + std::to_string(static_cast<int>(min_a)) + "° ~ " +
                   std::to_string(static_cast<int>(max_a)) + "°");
            return true;
        }

        float angle = std::atof(args[0].c_str());
        return setJointAngle(angle);
    }

    bool handleGetAngle() {
        float angle;
        JointError result = joint_controller->getJointAngle(current_leg, current_joint, angle);
        if (result == JointError::SUCCESS) {
            println("[INFO] " + std::string(getLegName(current_leg)) + "-" +
                   std::string(getJointName(current_joint)) + " 当前角度: " +
                   std::to_string(static_cast<int>(angle)) + "°");
        } else {
            println("[ERROR] 读取角度失败: " + std::string(JointController::getErrorString(result)));
        }
        return true;
    }

    bool handlePose(const std::vector<std::string>& args) {
        if (args.empty()) {
            println("用法: pose <sleep|stand>");
            return true;
        }

        if (args[0] == "sleep") {
            println("[POSE] 设置睡眠姿势...");
            JointError result = joint_controller->setSleepPose();
            if (result == JointError::SUCCESS) {
                println("[OK] 睡眠姿势设置成功");
            } else {
                println("[ERROR] 设置失败: " + std::string(JointController::getErrorString(result)));
            }
        } else if (args[0] == "stand") {
            println("[POSE] 设置站立姿势...");
            JointError result = joint_controller->setStandPose();
            if (result == JointError::SUCCESS) {
                println("[OK] 站立姿势设置成功");
            } else {
                println("[ERROR] 设置失败: " + std::string(JointController::getErrorString(result)));
            }
        } else {
            println("[ERROR] 未知姿势: " + args[0] + " (可用: sleep, stand)");
        }
        return true;
    }

    bool setJointAngle(float angle) {
        float min_a, max_a;
        getJointRange(current_joint, min_a, max_a);

        if (angle < min_a || angle > max_a) {
            println("[ERROR] 角度超出范围: " + std::to_string(static_cast<int>(angle)) +
                   "° (范围: " + std::to_string(static_cast<int>(min_a)) + "° ~ " +
                   std::to_string(static_cast<int>(max_a)) + "°)");
            return true;
        }

        println("[SET] " + std::string(getLegName(current_leg)) + "-" +
               std::string(getJointName(current_joint)) + " -> " +
               std::to_string(static_cast<int>(angle)) + "°");

        JointError result = joint_controller->setJointAngle(current_leg, current_joint, angle);
        if (result == JointError::SUCCESS) {
            println(" [OK]");
        } else {
            println(" [ERROR] " + std::string(JointController::getErrorString(result)));
        }
        return true;
    }

    bool showMappingVerification() {
        println("\n[MAPPING] 关节→舵机映射验证:");
        println("================================================");

        const char* legs[] = {"左前", "右前", "左后", "右后"};
        const char* joints[] = {"髋侧摆", "髋俯仰", "膝俯仰"};
        float test_angles[][3] = {{-45.0f, 0.0f, 45.0f}, {-45.0f, 0.0f, 45.0f}, {-90.0f, -45.0f, 0.0f}};

        for (int leg = 0; leg < 4; leg++) {
            println("\n" + std::string(legs[leg]) + ":");

            for (int joint = 0; joint < 3; joint++) {
                int servo_id = leg * 3 + joint;
                std::string line = "  " + std::string(joints[joint]) + " (舵机" + std::to_string(servo_id) + "): ";

                for (int i = 0; i < 3; i++) {
                    float joint_angle = test_angles[joint][i];
                    float servo_angle;

                    JointError result = joint_controller->jointToServoAngle(
                        static_cast<LegID>(leg), static_cast<JointType>(joint),
                        joint_angle, servo_angle);

                    if (result == JointError::SUCCESS) {
                        line += std::to_string(static_cast<int>(joint_angle)) + "°→" +
                               std::to_string(static_cast<int>(servo_angle)) + "° ";
                    } else {
                        line += "ERR ";
                    }
                }
                println(line);
            }
        }
        return true;
    }

    bool showSystemStatus() {
        println("\n[STATUS] 所有关节当前角度:");
        println("================================================");

        const char* legs[] = {"左前", "右前", "左后", "右后"};
        const char* joints[] = {"髋侧摆", "髋俯仰", "膝俯仰"};

        for (int leg = 0; leg < 4; leg++) {
            println("\n" + std::string(legs[leg]) + ":");

            for (int joint = 0; joint < 3; joint++) {
                float angle;
                JointError result = joint_controller->getJointAngle(
                    static_cast<LegID>(leg), static_cast<JointType>(joint), angle);

                if (result == JointError::SUCCESS) {
                    println("  " + std::string(joints[joint]) + ": " +
                           std::to_string(static_cast<int>(angle)) + "°");
                } else {
                    println("  " + std::string(joints[joint]) + ": 读取失败");
                }
            }
        }
        return true;
    }

    void updatePrompt() {
        setPrompt("joint[" + std::string(getLegName(current_leg)) + "-" +
                 std::string(getJointName(current_joint)) + "]");
    }

    void showCurrentSelection() {
        float min_a, max_a;
        getJointRange(current_joint, min_a, max_a);

        println("\n[INFO] 当前选择:");
        println("  腿部: " + std::string(getLegName(current_leg)));
        println("  关节: " + std::string(getJointName(current_joint)));
        println("  范围: " + std::to_string(static_cast<int>(min_a)) + "° ~ " +
               std::to_string(static_cast<int>(max_a)) + "°");
    }
};

/**
 * @brief 打印欢迎信息
 */
static void printWelcomeMessage() {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║       03_test_joint_controller - 关节控制器测试           ║\n");
    printf("╠════════════════════════════════════════════════════════════╣\n");
    printf("║  硬件配置:                                                 ║\n");
    printf("║    - PCA9685 I2C地址: 0x40                                 ║\n");
    printf("║    - I2C引脚: SDA=GPIO21, SCL=GPIO22                       ║\n");
    printf("║    - 舵机数量: 12个 (四足机器人)                           ║\n");
    printf("╠════════════════════════════════════════════════════════════╣\n");
    printf("║  功能说明:                                                 ║\n");
    printf("║    - 关节映射验证 (关节角度 ↔ 舵机角度)                    ║\n");
    printf("║    - 预设姿势 (睡眠/站立)                                  ║\n");
    printf("║    - 单关节角度控制                                        ║\n");
    printf("║    - 四腿协调控制                                          ║\n");
    printf("╠════════════════════════════════════════════════════════════╣\n");
    printf("║  机器人坐标系:                                             ║\n");
    printf("║    - X轴: 向前为正                                         ║\n");
    printf("║    - Y轴: 向左为正                                         ║\n");
    printf("║    - Z轴: 向上为正                                         ║\n");
    printf("╠════════════════════════════════════════════════════════════╣\n");
    printf("║  关节角度范围:                                             ║\n");
    printf("║    - 髋关节侧摆: -90° ~ +90° (绕X轴)                       ║\n");
    printf("║    - 髋关节俯仰: -90° ~ +90° (绕Y轴)                       ║\n");
    printf("║    - 膝关节俯仰: -180° ~ 0° (绕Y轴)                        ║\n");
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
    printf("│  1. 查看映射关系:                                           │\n");
    printf("│     > mapping                                               │\n");
    printf("│                                                             │\n");
    printf("│  2. 选择腿部和关节:                                         │\n");
    printf("│     > leg 0          (选择左前腿)                           │\n");
    printf("│     > joint 1        (选择髋俯仰关节)                       │\n");
    printf("│                                                             │\n");
    printf("│  3. 设置角度 (直接输入数字):                                │\n");
    printf("│     > 45             (设置到45度)                           │\n");
    printf("│     > -30            (设置到-30度)                          │\n");
    printf("│                                                             │\n");
    printf("│  4. 预设姿势:                                               │\n");
    printf("│     > pose sleep      (睡眠姿势)                            │\n");
    printf("│     > pose stand      (站立姿势)                            │\n");
    printf("│                                                             │\n");
    printf("│  5. 查看状态:                                               │\n");
    printf("│     > status          (所有关节角度)                        │\n");
    printf("│     > get             (当前关节角度)                        │\n");
    printf("└─────────────────────────────────────────────────────────────┘\n");
    printf("\n");
}

/**
 * @brief 主测试函数
 */
void test_joint_controller() {
    printWelcomeMessage();

    printf("[START] 关节控制器测试初始化...\n");
    printf("================================\n");

    // 1. 创建机器人控制器
    printf("\n[INIT] 初始化机器人控制器...\n");
    robot_controller = std::make_shared<RobotController>();

    if (!robot_controller->init()) {
        printf("[ERROR] 机器人控制器初始化失败\n");
        return;
    }
    printf("[OK] 机器人控制器初始化成功\n");

    // 2. 切换到关节角度控制模式
    robot_controller->setControlMode(RobotController::ControlMode::JOINT_ANGLE_MODE);
    printf("[OK] 已切换到关节角度控制模式\n");

    // 3. 获取关节控制器
    joint_controller = robot_controller->getJointController();
    if (!joint_controller) {
        printf("[ERROR] 获取关节控制器失败\n");
        return;
    }
    printf("[OK] 关节控制器获取成功\n");

    // 4. 设置站立姿势
    printf("\n[SAFE] 设置初始站立姿势...\n");
    joint_controller->setStandPose();
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("[OK] 初始姿势设置完成\n");

    // 5. 打印快速开始指南
    printQuickStart();

    // 6. 创建并启动终端
    JointControlTerminal terminal;

    if (!terminal.init()) {
        printf("[ERROR] UART终端初始化失败\n");
        return;
    }
    printf("[OK] UART终端初始化成功\n");
    printf("\n输入 'help' 查看所有命令\n\n");

    // 7. 开始交互模式
    terminal.startInteractiveMode(true);

    printf("[END] 关节控制器测试结束\n");
}
