/**
 * @file test_kinematics_controller.cpp
 * @brief 运动学控制器测试程序 - IK/FK + 坐标变换 + 四腿协调控制
 *
 * 功能：
 * - 正向运动学 (FK): 输入关节角度 → 计算足端位置
 * - 逆向运动学 (IK): 输入足端位置 → 计算关节角度
 * - 坐标变换: 机体坐标系 ↔ 腿部坐标系
 * - 四腿协调控制
 * - 机体位姿控制
 *
 * 硬件配置：
 * - PCA9685 I2C地址: 0x40
 * - I2C引脚: SDA=GPIO21, SCL=GPIO22
 * - 舵机数量: 12个 (四足机器人)
 */

#include "../kinematics/LegKinematics.hpp"
#include "../kinematics/KinematicsGeometry.hpp"
#include "../kinematics/CoordinateTransform.hpp"
#include "../kinematics/QuadrupedModel.hpp"
#include "../controllers/JointController.hpp"
#include "../controllers/RobotController.hpp"
#include "../config/ConfigManager.hpp"
#include "../utils/UartTerminal.hpp"
#include "../utils/Logger.hpp"

#include <stdio.h>
#include <string.h>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <iomanip>
#include <sstream>
#include <cstdlib>

using namespace Robot;
using namespace Robot::Kinematics;
using namespace Robot::Controllers;
using namespace Robot::Config;
using namespace Robot::Utils;

// 简化类型名称
using Vector3 = CoordinateTransform::Vector3;

// 全局变量
static std::shared_ptr<RobotController> robot_controller;
static std::shared_ptr<JointController> joint_controller;
static std::shared_ptr<QuadrupedModel> quadruped_model;
static int current_leg = 0;  // 0=左前, 1=右前, 2=左后, 3=右后
static Robot::Config::GeometryConfig geometry_config;  // 几何配置

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
 * @brief 运动学测试终端类
 */
class KinematicsTerminal : public UartTerminal {
public:
    KinematicsTerminal() : UartTerminal(
        UartTerminalConfig{},
        "kinematics",
        "运动学控制器测试终端") {

        registerCommands();
    }

private:
    void registerCommands() {
        // 选择腿部
        registerCommand("leg", "选择要操作的腿", "leg <0-3>",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleSelectLeg(args);
                       });

        // 逆向运动学 - 设置足端位置
        registerCommand("ik", "逆向运动学: 足端位置→关节角度", "ik <x> <y> <z>",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleIK(args);
                       });

        // 正向运动学 - 计算足端位置
        registerCommand("fk", "正向运动学: 关节角度→足端位置", "fk <髋侧摆> <髋俯仰> <膝俯仰>",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleFK(args);
                       });

        // 设置机体位姿
        registerCommand("pose", "设置机体位姿(位置+姿态)", "pose <x> <y> <z> [roll] [pitch] [yaw]",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handlePose(args);
                       });

        // 预设动作
        registerCommand("action", "执行预设动作", "action <stand|sit>",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return handleAction(args);
                       });

        // 测试工作空间
        registerCommand("workspace", "显示工作空间边界", "workspace",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return testWorkspace();
                       });

        // 显示腿部状态
        registerCommand("status", "显示所有腿的关节角度", "status",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return showStatus();
                       });

        // 显示详细帮助
        registerCommand("help", "显示详细帮助信息", "help [命令名]",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return showHelp(args);
                       });

        // 运动学验证测试
        registerCommand("test", "运行自动验证测试", "test",
                       [this](const std::string& input, const std::vector<std::string>& args) {
                           return runTests(args);
                       });
    }

    bool showHelp(const std::vector<std::string>& args) {
        if (args.empty()) {
            println("");
            println("╔════════════════════════════════════════════════════════════════════╗");
            println("║                    运动学控制器 - 命令帮助                         ║");
            println("╚════════════════════════════════════════════════════════════════════╝");
            println("");
            println("【腿部选择命令】");
            println("  leg <0-3>       选择要操作的腿");
            println("                  0=左前(LF), 1=右前(RF), 2=左后(LB), 3=右后(RB)");
            println("                  示例: leg 0  → 选择左前腿");
            println("");
            println("【运动学计算命令】");
            println("  ik <x> <y> <z>  逆向运动学 - 根据足端位置计算关节角度");
            println("                  坐标单位: 米, 相对于髋关节原点");
            println("                  示例: ik 0.1 0 -0.15  → 足端在(100mm, 0mm, -150mm)");
            println("");
            println("  fk <a1> <a2> <a3>  正向运动学 - 根据关节角度计算足端位置");
            println("                  角度单位: 度");
            println("                  a1=髋侧摆, a2=髋俯仰, a3=膝俯仰");
            println("                  示例: fk 0 -30 -60  → 计算(0°,-30°,-60°)对应的足端位置");
            println("");
            println("【机体位姿命令】");
            println("  pose <x> <y> <z> [r] [p] [y]  设置整个机体的位置和姿态");
            println("                  位置单位: 米, 姿态单位: 度");
            println("                  示例: pose 0 0 -0.1 5 0 0  → 机体下移100mm, roll倾斜5°");
            println("");
            println("【预设动作命令】");
            println("  action stand    站立姿势 - 四腿直立支撑");
            println("  action sit      坐下姿势 - 前腿伸直, 后腿弯曲");
            println("");
            println("【状态查看命令】");
            println("  status          显示所有腿的当前关节角度");
            println("  workspace       显示腿部工作空间边界");
            println("  test            运行自动验证测试");
            println("");
            println("【坐标系说明】");
            println("  X轴: 向前为正 (机器人前进方向)");
            println("  Y轴: 向左为正 (从上往下看)");
            println("  Z轴: 向上为正 (垂直地面向上)");
            println("  原点: 髋关节中心");
            println("");
            return true;
        }

        // 详细帮助
        std::string cmd = args[0];
        if (cmd == "ik") {
            println("");
            println("【IK 逆向运动学详解】");
            println("");
            println("功能: 根据期望的足端位置, 自动计算所需的关节角度");
            println("");
            println("用法: ik <x> <y> <z>");
            println("  x  - 足端在髋关节前方的距离 (米)");
            println("  y  - 足端在髋关节侧方的距离 (米, 左腿为正)");
            println("  z  - 足端在髋关节下方的距离 (米, 负值表示向下)");
            println("");
            println("示例:");
            println("  ik 0.1 0 -0.15    → 足端在前方100mm, 垂直向下150mm");
            println("  ik 0.05 0.02 -0.12  → 足端在前方50mm, 左侧20mm, 向下120mm");
            println("");
            println("工作空间范围 (参考):");
            println("  X: -50mm ~ 180mm");
            println("  Y: -150mm ~ 150mm");
            println("  Z: -250mm ~ -50mm");
        } else if (cmd == "fk") {
            println("");
            println("【FK 正向运动学详解】");
            println("");
            println("功能: 根据给定的关节角度, 计算足端的实际位置");
            println("");
            println("用法: fk <髋侧摆> <髋俯仰> <膝俯仰>");
            println("  髋侧摆  - 大腿左右摆动的角度 (度, 正值向外)");
            println("  髋俯仰  - 大腿前后摆动的角度 (度, 正值向前抬起)");
            println("  膝俯仰  - 小腿弯曲的角度 (度, 正值伸直)");
            println("");
            println("示例:");
            println("  fk 0 -30 -60    → 计算(0°, -30°, -60°)对应的足端位置");
            println("  fk 10 20 -45    → 计算(10°, 20°, -45°)对应的足端位置");
        } else if (cmd == "pose") {
            println("");
            println("【POSE 机体位姿控制详解】");
            println("");
            println("功能: 控制整个机体的位置和姿态, 所有腿会协调运动");
            println("");
            println("用法: pose <x> <y> <z> [roll] [pitch] [yaw]");
            println("  x,y,z     - 机体中心的位置偏移 (米)");
            println("  roll      - 机体横滚角 (度, 绕X轴旋转)");
            println("  pitch     - 机体俯仰角 (度, 绕Y轴旋转)");
            println("  yaw       - 机体偏航角 (度, 绕Z轴旋转)");
            println("");
            println("示例:");
            println("  pose 0 0 -0.1       → 机体下降100mm");
            println("  pose 0 0 0 10 0 0   → 机体向左倾斜10°");
            println("  pose 0.05 0 -0.1 0 5 0 → 机体前移50mm, 下移100mm, 俯仰5°");
        } else {
            println("未知命令: " + cmd);
            println("输入 'help' 查看所有命令");
        }
        return true;
    }

    bool handleSelectLeg(const std::vector<std::string>& args) {
        if (args.empty()) {
            println("");
            println("【当前选择的腿】");
            println("  腿号: " + std::to_string(current_leg));
            println("  名称: " + std::string(getLegName(current_leg)));
            println("");
            println("用法: leg <0-3>");
            println("  0=左前(LF), 1=右前(RF), 2=左后(LB), 3=右后(RB)");
            return true;
        }

        int id = std::atoi(args[0].c_str());
        if (id >= 0 && id < 4) {
            current_leg = id;
            println("");
            println("✓ 已选择: " + std::string(getLegName(current_leg)));
            println("  现在的IK/FK命令将针对这条腿执行");
            setPrompt("kinematics[" + std::string(getLegName(current_leg)) + "]");
        } else {
            println("");
            println("✗ 错误: 无效的腿部编号");
            println("  有效范围: 0-3");
            println("  0=左前(LF), 1=右前(RF), 2=左后(LB), 3=右后(RB)");
        }
        return true;
    }

    bool handleIK(const std::vector<std::string>& args) {
        if (args.size() < 3) {
            println("");
            println("【IK 逆向运动学计算】");
            println("用法: ik <x> <y> <z>");
            println("  坐标单位: 米 (相对于髋关节)");
            println("  示例: ik 0.1 0 -0.15");
            println("");
            println("输入 'help ik' 查看详细说明");
            return true;
        }

        float x = std::atof(args[0].c_str());
        float y = std::atof(args[1].c_str());
        float z = std::atof(args[2].c_str());

        println("");
        println("┌────────────────────────────────────────┐");
        println("│        IK 逆向运动学计算              │");
        println("└────────────────────────────────────────┘");
        println("");
        println("【输入】");
        println("  腿部: " + std::string(getLegName(current_leg)));
        println("  目标足端位置:");
        println("    X: " + std::to_string(x * 1000.0f) + " mm (前后方向)");
        println("    Y: " + std::to_string(y * 1000.0f) + " mm (左右方向)");
        println("    Z: " + std::to_string(z * 1000.0f) + " mm (垂直方向)");

        if (!quadruped_model) {
            println("");
            println("✗ 错误: 运动学模型未初始化");
            return true;
        }

        // 设置足端位置并计算逆运动学
        Vector3 target_pos(x, y, z);
        bool success = quadruped_model->setLegPosition(current_leg, target_pos);

        println("");
        println("【输出】");
        if (success) {
            // 获取计算出的关节角度
            ThreeJointAngles angles = quadruped_model->getLegJointAngles(current_leg);

            println("  ✓ IK 计算成功!");
            println("");
            println("  计算出的关节角度:");
            println("    髋侧摆 (Hip Side):  " + std::to_string(static_cast<int>(angles.hip_side * 180.0f / M_PI)) + "°");
            println("    髋俯仰 (Hip Pitch): " + std::to_string(static_cast<int>(angles.hip_pitch * 180.0f / M_PI)) + "°");
            println("    膝俯仰 (Knee):      " + std::to_string(static_cast<int>(angles.knee_pitch * 180.0f / M_PI)) + "°");
            println("");
            println("  说明: 这些角度可以让足端精确到达目标位置");
        } else {
            println("  ✗ IK 计算失败!");
            println("");
            println("  可能原因:");
            println("    - 目标位置超出工作空间范围");
            println("    - 位置导致关节角度超限");
            println("    - 几何约束无法满足");
            println("");
            println("  建议: 尝试调整目标位置, 或输入 'workspace' 查看有效范围");
        }
        return true;
    }

    bool handleFK(const std::vector<std::string>& args) {
        if (args.size() < 3) {
            println("");
            println("【FK 正向运动学计算】");
            println("用法: fk <髋侧摆> <髋俯仰> <膝俯仰>");
            println("  角度单位: 度");
            println("  示例: fk 0 -30 -60");
            println("");
            println("输入 'help fk' 查看详细说明");
            return true;
        }

        float hip_side_deg = std::atof(args[0].c_str());
        float hip_pitch_deg = std::atof(args[1].c_str());
        float knee_pitch_deg = std::atof(args[2].c_str());

        println("");
        println("┌────────────────────────────────────────┐");
        println("│        FK 正向运动学计算              │");
        println("└────────────────────────────────────────┘");
        println("");
        println("【输入】");
        println("  腿部: " + std::string(getLegName(current_leg)));
        println("  关节角度:");
        println("    髋侧摆 (Hip Side):  " + std::to_string(static_cast<int>(hip_side_deg)) + "°");
        println("    髋俯仰 (Hip Pitch): " + std::to_string(static_cast<int>(hip_pitch_deg)) + "°");
        println("    膝俯仰 (Knee):      " + std::to_string(static_cast<int>(knee_pitch_deg)) + "°");

        if (!quadruped_model) {
            println("");
            println("✗ 错误: 运动学模型未初始化");
            return true;
        }

        // 获取腿部实例
        Leg* leg = quadruped_model->getLeg(current_leg);
        if (!leg) {
            println("");
            println("✗ 错误: 无法获取腿部实例");
            return true;
        }

        // 设置关节角度（度数版本）并计算正向运动学
        bool success = leg->setJointAnglesDegrees(hip_side_deg, hip_pitch_deg, knee_pitch_deg);

        println("");
        println("【输出】");
        if (success) {
            // 获取计算出的足端位置
            Vector3 foot_pos = leg->getFootPosition();

            println("  ✓ FK 计算成功!");
            println("");
            println("  足端位置 (腿部坐标系):");
            println("    X: " + std::to_string(foot_pos.x * 1000.0f) + " mm (前后方向)");
            println("    Y: " + std::to_string(foot_pos.y * 1000.0f) + " mm (左右方向)");
            println("    Z: " + std::to_string(foot_pos.z * 1000.0f) + " mm (垂直方向)");
            println("");
            println("  说明: 这是给定关节角度下足端的实际位置");
        } else {
            println("  ✗ FK 计算失败!");
            println("");
            println("  可能原因: 关节角度超出有效范围");
        }
        return true;
    }

    bool handlePose(const std::vector<std::string>& args) {
        if (args.size() < 3) {
            println("");
            println("【POSE 机体位姿控制】");
            println("用法: pose <x> <y> <z> [roll] [pitch] [yaw]");
            println("  x,y,z: 机体位置偏移 (米)");
            println("  roll,pitch,yaw: 机体姿态角 (度, 可选)");
            println("");
            println("输入 'help pose' 查看详细说明");
            return true;
        }

        float x = std::atof(args[0].c_str());
        float y = std::atof(args[1].c_str());
        float z = std::atof(args[2].c_str());
        float roll = args.size() > 3 ? std::atof(args[3].c_str()) : 0;
        float pitch = args.size() > 4 ? std::atof(args[4].c_str()) : 0;
        float yaw = args.size() > 5 ? std::atof(args[5].c_str()) : 0;

        println("");
        println("┌────────────────────────────────────────┐");
        println("│        POSE 机体位姿控制              │");
        println("└────────────────────────────────────────┘");
        println("");
        println("【输入】");
        println("  位置偏移:");
        println("    X: " + std::to_string(x * 1000.0f) + " mm");
        println("    Y: " + std::to_string(y * 1000.0f) + " mm");
        println("    Z: " + std::to_string(z * 1000.0f) + " mm");
        println("  姿态角度:");
        println("    Roll:  " + std::to_string(static_cast<int>(roll)) + "°");
        println("    Pitch: " + std::to_string(static_cast<int>(pitch)) + "°");
        println("    Yaw:   " + std::to_string(static_cast<int>(yaw)) + "°");

        if (!quadruped_model) {
            println("");
            println("✗ 错误: 运动学模型未初始化");
            return true;
        }

        // 转换角度为弧度
        Vector3 position(x, y, z);
        Vector3 orientation(roll * M_PI / 180.0f, pitch * M_PI / 180.0f, yaw * M_PI / 180.0f);

        // 更新机体位姿
        bool success = quadruped_model->updateBodyPose(position, orientation);

        println("");
        println("【输出】");
        if (success) {
            println("  ✓ 位姿更新成功!");
            println("");
            println("  各腿新的关节角度:");

            // 获取所有关节角度
            auto all_angles = quadruped_model->getAllJointAngles();
            for (int leg = 0; leg < 4; leg++) {
                println("    " + std::string(getLegName(leg)) + ": (" +
                       std::to_string(static_cast<int>(all_angles[leg].hip_side * 180.0f / M_PI)) + "°, " +
                       std::to_string(static_cast<int>(all_angles[leg].hip_pitch * 180.0f / M_PI)) + "°, " +
                       std::to_string(static_cast<int>(all_angles[leg].knee_pitch * 180.0f / M_PI)) + "°)");
            }
            println("");
            println("  说明: 足端位置保持不变, 机体姿态改变导致关节角度调整");
        } else {
            println("  ✗ 位姿更新失败!");
            println("");
            println("  可能原因: 位姿超出了运动学约束范围");
        }
        return true;
    }

    bool handleAction(const std::vector<std::string>& args) {
        if (args.empty()) {
            println("");
            println("【ACTION 预设动作】");
            println("用法: action <动作名>");
            println("  stand - 站立姿势 (四腿直立支撑)");
            println("  sit   - 坐下姿势 (前腿伸直, 后腿弯曲)");
            return true;
        }

        println("");
        if (args[0] == "stand") {
            println("【执行动作: 站立】");
            println("  正在设置站立姿势...");
            joint_controller->setStandPose();
            println("  ✓ 站立姿势设置完成");
        } else if (args[0] == "sit") {
            println("【执行动作: 坐下】");
            println("  正在设置坐下姿势...");
            joint_controller->setSleepPose();
            println("  ✓ 坐下姿势设置完成");
        } else {
            println("✗ 未知动作: " + args[0]);
            println("  可用动作: stand, sit");
        }
        return true;
    }

    bool testWorkspace() {
        println("");
        println("┌────────────────────────────────────────┐");
        println("│        工作空间边界                    │");
        println("└────────────────────────────────────────┘");
        println("");
        println("【腿部几何参数】");
        println("  L1 (髋延伸): 60.5 mm");
        println("  L2 (髋垂直): 10.0 mm");
        println("  L3 (大腿):   111.1 mm");
        println("  L4 (小腿):   118.5 mm");
        println("");
        println("【理论工作空间范围】");
        println("  X方向 (前后): -50 mm ~ 180 mm");
        println("  Y方向 (左右): -150 mm ~ 150 mm");
        println("  Z方向 (垂直): -250 mm ~ -50 mm (向下为负)");
        println("");
        println("【建议安全范围】");
        println("  X方向: 0 ~ 150 mm");
        println("  Y方向: -100 ~ 100 mm");
        println("  Z方向: -200 ~ -80 mm");
        println("");
        println("【使用建议】");
        println("  - 位置靠近边界时IK可能失败");
        println("  - Z值为负表示足端在髋关节下方");
        println("  - 实际可用范围取决于机械限位");
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

            // 获取关节角度
            LegID leg_id = static_cast<LegID>(leg);
            float angle;

            for (int joint = 0; joint < 3; joint++) {
                JointType joint_type = static_cast<JointType>(joint);
                const char* joint_names[] = {"髋侧摆", "髋俯仰", "膝俯仰"};

                if (joint_controller->getJointAngle(leg_id, joint_type, angle) == JointError::SUCCESS) {
                    println("  " + std::string(joint_names[joint]) + ": " +
                           std::to_string(static_cast<int>(angle)) + "°");
                }
            }
            println("");
        }
        return true;
    }

    bool runTests(const std::vector<std::string>& args) {
        println("");
        println("┌────────────────────────────────────────┐");
        println("│        运动学验证测试                  │");
        println("└────────────────────────────────────────┘");
        println("");

        // IK 测试
        println("【测试1: IK 逆向运动学】");
        println("  测试位置: (0.1, 0, -0.15) m");

        if (quadruped_model) {
            Vector3 test_pos(0.1f, 0.0f, -0.15f);
            bool ik_success = quadruped_model->setLegPosition(current_leg, test_pos);

            if (ik_success) {
                ThreeJointAngles angles = quadruped_model->getLegJointAngles(current_leg);
                println("  ✓ IK 成功");
                println("    结果角度: (" +
                       std::to_string(static_cast<int>(angles.hip_side * 180.0f / M_PI)) + "°, " +
                       std::to_string(static_cast<int>(angles.hip_pitch * 180.0f / M_PI)) + "°, " +
                       std::to_string(static_cast<int>(angles.knee_pitch * 180.0f / M_PI)) + "°)");
            } else {
                println("  ✗ IK 失败");
            }
        }

        println("");
        println("【测试2: FK 正向运动学】");
        println("  测试角度: (0°, -30°, -60°)");

        if (quadruped_model) {
            Leg* leg = quadruped_model->getLeg(current_leg);
            if (leg) {
                bool fk_success = leg->setJointAnglesDegrees(0, -30, -60);
                if (fk_success) {
                    Vector3 pos = leg->getFootPosition();
                    println("  ✓ FK 成功");
                    println("    结果位置: (" +
                           std::to_string(static_cast<int>(pos.x * 1000.0f)) + "mm, " +
                           std::to_string(static_cast<int>(pos.y * 1000.0f)) + "mm, " +
                           std::to_string(static_cast<int>(pos.z * 1000.0f)) + "mm)");
                } else {
                    println("  ✗ FK 失败");
                }
            }
        }

        println("");
        println("【测试完成】");
        return true;
    }
};

/**
 * @brief 打印欢迎信息
 */
static void printWelcomeMessage() {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════════════╗\n");
    printf("║     04_test_kinematics_controller - 运动学控制器测试              ║\n");
    printf("╠════════════════════════════════════════════════════════════════════╣\n");
    printf("║  功能:                                                             ║\n");
    printf("║    IK - 输入足端位置, 计算关节角度                                  ║\n");
    printf("║    FK - 输入关节角度, 计算足端位置                                  ║\n");
    printf("║    POSE - 控制整个机体的位置和姿态                                  ║\n");
    printf("╠════════════════════════════════════════════════════════════════════╣\n");
    printf("║  快速开始:                                                         ║\n");
    printf("║    1. leg 0              → 选择左前腿                               ║\n");
    printf("║    2. ik 0.1 0 -0.15     → 设置足端位置, 查看计算出的角度           ║\n");
    printf("║    3. fk 0 -30 -60       → 输入角度, 查看足端位置                   ║\n");
    printf("║    4. help               → 查看详细帮助                             ║\n");
    printf("╚════════════════════════════════════════════════════════════════════╝\n");
    printf("\n");
}

/**
 * @brief 主测试函数
 */
void test_kinematics_controller() {
    printWelcomeMessage();

    printf("[START] 运动学控制器测试初始化...\n");
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

    // 3. 初始化四足运动学模型
    printf("\n[INIT] 初始化四足运动学模型...\n");
    quadruped_model = std::make_shared<QuadrupedModel>(geometry_config);
    if (!quadruped_model) {
        printf("[ERROR] QuadrupedModel 创建失败\n");
        return;
    }
    printf("[OK] 四足运动学模型初始化成功\n");

    // 4. 设置站立姿势
    printf("\n[SAFE] 设置初始站立姿势...\n");
    joint_controller->setStandPose();
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("[OK] 初始姿势设置完成\n");

    // 5. 打印提示
    printf("\n输入 'help' 查看详细命令说明\n");
    printf("输入 'test' 运行自动验证测试\n\n");

    // 6. 创建并启动终端
    KinematicsTerminal terminal;

    if (!terminal.init()) {
        printf("[ERROR] UART终端初始化失败\n");
        return;
    }

    // 7. 开始交互模式
    terminal.startInteractiveMode(true);

    printf("[END] 运动学控制器测试结束\n");
}
