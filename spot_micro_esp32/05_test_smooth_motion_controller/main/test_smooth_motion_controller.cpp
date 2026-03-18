/**
 * @file test_smooth_motion_controller.cpp
 * @brief 平滑运动控制器测试程序 - 姿态插值 + 关节平滑 + 统一运动控制
 *
 * 功能：
 * - 姿态插值: 平滑过渡到目标姿态
 * - 关节平滑: S曲线加减速
 * - 统一运动控制: 协调12个舵机同步运动
 * - 预设动作序列: 站立、坐下、蹲下、俯卧等
 *
 * 硬件配置：
 * - PCA9685 I2C地址: 0x40
 * - I2C引脚: SDA=GPIO21, SCL=GPIO22
 * - 舵机数量: 12个 (四足机器人)
 */

#include "../controllers/JointController.hpp"
#include "../controllers/RobotController.hpp"
#include "../controllers/PoseInterpolator.hpp"
#include "../controllers/JointSmoother.hpp"
#include "../config/ConfigManager.hpp"
#include "../utils/UartTerminal.hpp"
#include "../utils/Logger.hpp"

#include <stdio.h>
#include <string.h>
#include <cmath>
#include <cstdlib>

using namespace Robot;
using namespace Robot::Controllers;
using namespace Robot::Config;
using namespace Robot::Utils;

// 全局变量
static std::shared_ptr<RobotController> robot_controller;
static std::shared_ptr<JointController> joint_controller;
static std::unique_ptr<PoseInterpolator> pose_interpolator;

// 腿部名称
static const char* getLegName(int leg_id) {
    switch (leg_id) {
        case 0: return "左前(LF)";
        case 1: return "右前(RF)";
        case 2: return "左后(LB)";
        case 3: return "右后(RB)";
        default: return "未知";
    }
}

/**
 * @brief 平滑运动控制终端类
 */
class SmoothMotionTerminal : public UartTerminal {
public:
    SmoothMotionTerminal() : UartTerminal(
        UartTerminalConfig{},
        "smooth",
        "平滑运动控制测试终端") {

        registerCommands();
    }

private:
    void registerCommands() {
        // 预设动作
        registerCommand("pose", "执行预设姿势", "pose <sleep|stand>",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handlePose(args);
                       });

        // 平滑测试
        registerCommand("test", "运行平滑运动测试", "test [single|sequence]",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleTest(args);
                       });

        // 显示状态
        registerCommand("status", "显示当前状态", "status",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return showStatus();
                       });

        // 显示详细帮助
        registerCommand("help", "显示详细帮助信息", "help",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return showHelp();
                       });
    }

    bool showHelp() {
        println("");
        println("╔════════════════════════════════════════════════════════════════════╗");
        println("║              平滑运动控制器 - 命令帮助                             ║");
        println("╚════════════════════════════════════════════════════════════════════╝");
        println("");
        println("【预设姿势命令】");
        println("  pose stand    → 站立姿势 - 四腿直立支撑机器人");
        println("  pose sleep    → 睡眠姿势 - 四腿收起, 机器人俯卧");
        println("");
        println("【测试命令】");
        println("  test single   → 单关节平滑测试 - 测试左前腿髋侧摆舵机");
        println("                  舵机会依次移动到: 30° → -30° → 0°");
        println("  test sequence → 姿势序列测试 - 自动执行 stand→sleep→stand");
        println("                  观察机器人姿势切换是否平滑");
        println("");
        println("【状态查看命令】");
        println("  status        → 显示所有12个关节的当前角度");
        println("");
        println("【使用流程建议】");
        println("  1. pose sleep      → 先让机器人俯卧(安全初始状态)");
        println("  2. test sequence   → 观察站立→俯卧→站立动作");
        println("  3. status          → 检查关节角度是否正确");
        println("");
        println("【安全提示】");
        println("  ⚠ 首次测试请确保舵机运动范围内无障碍物");
        println("  ⚠ 观察机器人运动是否正常后再增加测试次数");
        println("");
        return true;
    }

    bool handlePose(const std::vector<std::string>& args) {
        if (args.empty()) {
            println("");
            println("【POSE 预设姿势命令】");
            println("用法: pose <姿势名>");
            println("");
            println("可用姿势:");
            println("  stand → 站立姿势");
            println("    - 四腿直立支撑机器人");
            println("    - 机器人处于正常站立状态");
            println("");
            println("  sleep → 睡眠/俯卧姿势");
            println("    - 四腿收起, 机器人趴在地上");
            println("    - 适合运输和存储");
            println("");
            println("示例:");
            println("  pose stand  → 机器人站立");
            println("  pose sleep  → 机器人俯卧");
            return true;
        }

        std::string pose_name = args[0];
        JointError result = JointError::SUCCESS;

        println("");
        println("┌────────────────────────────────────────┐");
        println("│        执行预设姿势                    │");
        println("└────────────────────────────────────────┘");
        println("");
        println("【姿势】 " + pose_name);

        if (pose_name == "sleep") {
            println("  说明: 睡眠/俯卧姿势");
            println("  动作: 四腿收起, 机器人俯卧");
            result = joint_controller->setSleepPose();
        } else if (pose_name == "stand") {
            println("  说明: 站立姿势");
            println("  动作: 四腿直立支撑");
            result = joint_controller->setStandPose();
        } else {
            println("  ✗ 未知姿势: " + pose_name);
            println("  可用: stand, sleep");
            return true;
        }

        println("");
        if (result == JointError::SUCCESS) {
            println("  ✓ 姿势执行完成");
            println("  输入 'status' 查看当前关节角度");
        } else {
            println("  ✗ 执行失败: " + std::string(JointController::getErrorString(result)));
        }
        return true;
    }

    bool handleTest(const std::vector<std::string>& args) {
        std::string test_type = args.empty() ? "single" : args[0];

        println("");
        println("┌────────────────────────────────────────┐");
        println("│        平滑运动测试                    │");
        println("└────────────────────────────────────────┘");

        if (test_type == "single") {
            println("");
            println("【测试类型】 单关节平滑测试");
            println("【测试目标】 左前腿 髋侧摆舵机 (servo 0)");
            println("【运动轨迹】 0° → 30° → -30° → 0°");
            println("");
            testSingleJointSmooth();
        } else if (test_type == "sequence") {
            println("");
            println("【测试类型】 姿势序列测试");
            println("【运动轨迹】 stand → sleep → stand");
            println("【观察要点】 姿势切换是否平滑");
            println("");
            testPoseSequence();
        } else {
            println("");
            println("用法: test <single|sequence>");
            println("  single   → 单关节平滑测试");
            println("  sequence → 姿势序列测试");
        }
        return true;
    }

    bool showStatus() {
        println("");
        println("┌────────────────────────────────────────┐");
        println("│        四腿关节角度状态                │");
        println("└────────────────────────────────────────┘");
        println("");

        for (int leg = 0; leg < 4; leg++) {
            println("【" + std::string(getLegName(leg)) + "】");

            float angle;
            const char* joint_names[] = {"髋侧摆", "髋俯仰", "膝俯仰"};

            for (int joint = 0; joint < 3; joint++) {
                JointError result = joint_controller->getJointAngle(
                    static_cast<LegID>(leg), static_cast<JointType>(joint), angle);

                if (result == JointError::SUCCESS) {
                    println("  " + std::string(joint_names[joint]) + ": " +
                           std::to_string(static_cast<int>(angle)) + "°");
                }
            }
            println("");
        }
        return true;
    }

    void testSingleJointSmooth() {
        println("  执行测试...");

        // 先到中位
        joint_controller->setJointAngle(LegID::FRONT_LEFT, JointType::HIP_ROLL, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(500));

        // 平滑运动: 0° -> 30° -> -30° -> 0°
        float angles[] = {30.0f, -30.0f, 0.0f};
        for (int i = 0; i < 3; i++) {
            println("    → " + std::to_string(static_cast<int>(angles[i])) + "°");
            joint_controller->setJointAngle(LegID::FRONT_LEFT, JointType::HIP_ROLL, angles[i]);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        println("");
        println("  ✓ 单关节测试完成");
        println("  观察左前腿髋侧摆是否平滑运动");
    }

    void testPoseSequence() {
        println("  执行序列...");

        println("");
        println("  [1/3] 站立姿势");
        joint_controller->setStandPose();
        vTaskDelay(pdMS_TO_TICKS(2000));

        println("  [2/3] 睡眠姿势");
        joint_controller->setSleepPose();
        vTaskDelay(pdMS_TO_TICKS(2000));

        println("  [3/3] 站立姿势");
        joint_controller->setStandPose();
        vTaskDelay(pdMS_TO_TICKS(2000));

        println("");
        println("  ✓ 序列测试完成");
        println("  观察姿势切换是否协调");
    }
};

/**
 * @brief 打印欢迎信息
 */
static void printWelcomeMessage() {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════════════╗\n");
    printf("║        05_test_smooth_motion_controller - 平滑运动控制            ║\n");
    printf("╠════════════════════════════════════════════════════════════════════╣\n");
    printf("║  功能:                                                             ║\n");
    printf("║    pose   - 执行预设姿势 (stand/sleep)                             ║\n");
    printf("║    test   - 运行平滑运动测试                                        ║\n");
    printf("║    status - 显示关节角度                                            ║\n");
    printf("╠════════════════════════════════════════════════════════════════════╣\n");
    printf("║  快速开始:                                                         ║\n");
    printf("║    1. help            → 查看详细帮助                               ║\n");
    printf("║    2. test sequence   → 运行姿势序列测试                           ║\n");
    printf("║    3. status          → 查看关节状态                               ║\n");
    printf("╠════════════════════════════════════════════════════════════════════╣\n");
    printf("║  安全警告:                                                         ║\n");
    printf("║    ⚠ 确保舵机运动范围内无障碍物                                     ║\n");
    printf("║    ⚠ 首次运行请观察运动是否正常                                     ║\n");
    printf("╚════════════════════════════════════════════════════════════════════╝\n");
    printf("\n");
}

/**
 * @brief 主测试函数
 */
void test_smooth_motion_controller() {
    printWelcomeMessage();

    printf("[START] 平滑运动控制器测试初始化...\n");
    printf("====================================\n");

    // 1. 创建机器人控制器
    printf("\n[INIT] 初始化机器人控制器...\n");
    robot_controller = std::make_shared<RobotController>();

    if (!robot_controller->init()) {
        printf("[ERROR] 机器人控制器初始化失败\n");
        return;
    }
    printf("[OK] 机器人控制器初始化成功\n");

    // 2. 获取关节控制器
    joint_controller = robot_controller->getJointController();
    if (!joint_controller) {
        printf("[ERROR] 获取关节控制器失败\n");
        return;
    }
    printf("[OK] 关节控制器获取成功\n");

    // 3. 设置初始站立姿势
    printf("\n[SAFE] 设置初始站立姿势...\n");
    joint_controller->setStandPose();
    vTaskDelay(pdMS_TO_TICKS(1500));
    printf("[OK] 初始姿势设置完成\n");

    // 4. 打印提示
    printf("\n输入 'help' 查看详细命令说明\n\n");

    // 5. 创建并启动终端
    SmoothMotionTerminal terminal;

    if (!terminal.init()) {
        printf("[ERROR] UART终端初始化失败\n");
        return;
    }

    // 6. 开始交互模式
    terminal.startInteractiveMode(true);

    printf("[END] 平滑运动控制器测试结束\n");
}
