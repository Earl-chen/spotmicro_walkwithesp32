/**
 * @file pca9685Test.cpp
 * @brief ESP32四足机器人舵机控制系统 v4.0.0 (模块化重构版本)
 *
 * 使用全新的分层模块化架构
 * 支持命令行交互和现代化的控制接口
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ESP-IDF
#include <driver/i2c.h>
#include <esp_log.h>
#include "driver/uart.h"

// 新的模块化架构
#include "controllers/RobotController.hpp"
#include "utils/Logger.hpp"

// 运动学模块
#include "kinematics/LegKinematics.hpp"
#include "kinematics/KinematicsGeometry.hpp"
#include "kinematics/CoordinateTransform.hpp"

// 平滑运动控制相关
#include "controllers/SmoothMotionController.hpp"
#include "controllers/PoseInterpolator.hpp"
#include "kinematics/QuadrupedModel.hpp"
#include "config/SmoothMotionConfig.hpp"

using namespace Robot;
using namespace Robot::Controllers;
using namespace Robot::Kinematics;

// 全局变量
static std::shared_ptr<RobotController> g_robot_controller_;
static RobotController::ControlMode g_current_mode_ = RobotController::ControlMode::SERVO_ANGLE_MODE;
static int g_selected_servo_ = 0;

// 关节模式下的选择变量
static LegID g_selected_leg_ = LegID::FRONT_LEFT;
static JointType g_selected_joint_ = JointType::HIP_ROLL;

// 运动学相关变量
static std::unique_ptr<SpotLegKinematics> g_left_front_leg_;
static std::unique_ptr<SpotLegKinematics> g_right_front_leg_;
static std::unique_ptr<SpotLegKinematics> g_left_back_leg_;
static std::unique_ptr<SpotLegKinematics> g_right_back_leg_;
static FrameManager g_frame_manager_;

// 腿部状态存储
struct LegState {
    CoordinateTransform::Vector3 foot_position;
    ThreeJointAngles joint_angles;
    bool valid;
};
static std::array<LegState, 4> g_leg_states_;
static std::array<CoordinateTransform::Vector3, 4> g_foot_world_positions_;

// 平滑运动控制相关全局变量
static std::unique_ptr<Robot::Controllers::SmoothMotionController> g_smooth_controller_;
static std::shared_ptr<Robot::Kinematics::QuadrupedModel> g_quadruped_model_;
static bool g_smooth_motion_initialized_ = false;

// UART配置
#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024
static QueueHandle_t uart_queue;

// 函数声明
extern "C" void app_main();  // ESP32入口函数
void uart_init();
void uart_read_task(void *pvParameters);
void init_robot_system();
void parseCommand(const char* input);
void printHelp();
void printStatus();

// 命令处理函数
bool handleSystemCommands(const char* input);
bool handleModeCommands(const char* input);
bool handleServoCommands(const char* input);
bool handleJointSelectionCommands(const char* input);  // 新增
bool handleJointCommands(const char* input);
bool handlePoseCommands(const char* input);
bool handleDirectValueCommand(const char* input);
bool handleDirectServoCommand(const char* input);
bool handleDiagnosisCommands(const char* input);  // 新增：硬件诊断命令

// 平滑运动控制函数
bool handleSmoothMotionCommands(const char* input);  // 新增：平滑运动命令
void initSmoothMotionSystem();

// 运动学控制函数
bool handleKinematicsCommands(const char* input);
void initKinematicsSystem();
void resetLegStates();
SpotLegKinematics* getLegKinematics(int leg_id);
CoordinateTransform::Vector3 getHipOffset(int leg_id);
void updateFootWorldPositions();
void controlServos();
KinematicsResult calculateForwardKinematics(int leg_id, const ThreeJointAngles& joint_angles);
KinematicsResult calculateInverseKinematics(int leg_id, const CoordinateTransform::Vector3& foot_pos);

extern "C" void app_main() {
    LOG_INFO("=== ESP32四足机器人控制系统 v4.0.0 启动 ===");
    LOG_INFO("模块化重构版本 - 分层架构设计");

    // 初始化机器人系统
    init_robot_system();

    // 初始化UART
    uart_init();

    // 创建UART读取任务
    xTaskCreate(uart_read_task, "uart_read_task", 8192, NULL, 10, NULL);

    LOG_INFO("系统初始化完成！输入 'help' 查看可用命令");
    printHelp();

    // 显示初始提示符
    printf("SpotMicro> ");
    fflush(stdout);
}

void init_robot_system() {
    LOG_TAG_INFO("SYSTEM", "初始化机器人控制系统...");
 
    // 2. 创建机器人控制器
    g_robot_controller_ = std::make_shared<RobotController>();

    // 3. 初始化系统
    if (!g_robot_controller_->init()) {
        LOG_TAG_ERROR("SYSTEM", "机器人控制器初始化失败！");
        return;
    }

    // 4. 设置默认控制模式
    g_robot_controller_->setControlMode(g_current_mode_);

    // 5. 初始化运动学系统
    initKinematicsSystem();

    // 6. 设置日志级别
    auto& logger = Utils::Logger::getInstance();
    logger.setLogLevel(Utils::LogLevel::INFO);
    logger.setTimestamp(true);

    LOG_TAG_INFO("SYSTEM", "机器人控制系统初始化成功");
}

void uart_init() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 10, &uart_queue, 0));
}

void uart_read_task(void *pvParameters) {
    char buffer[BUF_SIZE];
    static char command_buffer[256];  // 命令行缓冲区
    static int cmd_pos = 0;           // 当前命令位置

    while (1) {
        int len = uart_read_bytes(UART_NUM, buffer, BUF_SIZE - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            buffer[len] = '\0';

            // 逐字符处理，构建完整命令行
            for (int i = 0; i < len; i++) {
                char c = buffer[i];

                if (c == '\n' || c == '\r') {
                    // 回显换行
                    uart_write_bytes(UART_NUM, "\r\n", 2);

                    // 遇到换行符，处理完整命令
                    if (cmd_pos > 0) {
                        command_buffer[cmd_pos] = '\0';

                        // 处理完整命令
                        parseCommand(command_buffer);

                        // 重置缓冲区
                        cmd_pos = 0;
                        memset(command_buffer, 0, sizeof(command_buffer));
                    }

                    // 显示提示符
                    uart_write_bytes(UART_NUM, "SpotMicro> ", 11);
                } else if (c >= 32 && c <= 126) {  // 可打印字符
                    // 回显字符到终端
                    uart_write_bytes(UART_NUM, &c, 1);

                    // 添加字符到命令缓冲区
                    if (cmd_pos < sizeof(command_buffer) - 1) {
                        command_buffer[cmd_pos++] = c;
                    } else {
                        // 缓冲区满，重置
                        LOG_TAG_WARN("UART", "命令行过长，重置缓冲区");
                        cmd_pos = 0;
                        memset(command_buffer, 0, sizeof(command_buffer));
                    }
                } else if (c == 8 || c == 127) {  // 退格键处理
                    if (cmd_pos > 0) {
                        cmd_pos--;
                        command_buffer[cmd_pos] = '\0';
                        // 发送退格回显: 退格+空格+退格
                        uart_write_bytes(UART_NUM, "\b \b", 3);
                    }
                }
                // 忽略其他控制字符
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void parseCommand(const char* input) {
    LOG_TAG_INFO("CMD", "处理完整命令: '%s' (长度: %d)", input, strlen(input));

    if (!g_robot_controller_) {
        LOG_TAG_ERROR("CMD", "机器人控制器未初始化");
        return;
    }

    // 按优先级依次尝试各种命令类型
    if (handleSystemCommands(input))    return;  // help, status
    if (handleDiagnosisCommands(input)) return;  // 硬件诊断命令 (新增)
    if (handleSmoothMotionCommands(input)) return;  // 平滑运动命令 (新增)
    if (handleModeCommands(input))      return;  // pwm, angle, joint, kinematics
    if (handleKinematicsCommands(input)) return;  // pose命令 (运动学控制)
    if (handleServoCommands(input))     return;  // s <id>
    if (handleJointSelectionCommands(input)) return;  // leg <id>, joint <id>
    if (handlePoseCommands(input))      return;  // sleep0, sleep1
    if (handleJointCommands(input))     return;  // j <leg> <joint> <angle>
    if (handleDirectValueCommand(input)) return;  // <value>
    if (handleDirectServoCommand(input)) return;  // <servo_id> <value>

    // 未识别的命令
    LOG_TAG_WARN("CMD", "未识别的命令: %s", input);
    LOG_TAG_INFO("CMD", "输入 'help' 查看可用命令");
}

void printHelp() {
    printf("\n=== ESP32四足机器人控制系统 v4.0.0 帮助 ===\n");
    printf("🏗️ 新模块化架构，支持分层控制\n\n");

    printf("📋 模式切换命令:\n");
    printf("  pwm        - PWM直接控制模式\n");
    printf("  angle      - 舵机角度控制模式\n");
    printf("  joint      - 关节角度控制模式\n");
    printf("  kinematics - 运动学控制模式\n\n");

    printf("🎮 舵机控制命令:\n");
    printf("  s <ID>     - 选择舵机 (0-11)\n");
    printf("  <数值>     - 控制当前选中的舵机\n");
    printf("  <ID> <值>  - 直接控制指定舵机\n\n");

    printf("🦿 关节控制命令:\n");
    printf("  j <腿> <关节> <角度> - 控制特定关节\n");
    printf("    腿: 0=左前, 1=右前, 2=左后, 3=右后\n");
    printf("    关节: 0=髋侧摆, 1=髋俯仰, 2=膝俯仰\n");
    printf("  leg <腿ID>          - 选择当前操作的腿部\n");
    printf("  joint <关节ID>      - 选择当前操作的关节\n");
    printf("  <角度>              - 设置当前选中关节的角度\n\n");

    printf("🏃 预设姿势命令:\n");
    printf("  sleep0     - 趴下姿势\n");
    printf("  sleep1     - 站立姿势\n\n");

    printf("🦾 运动学控制命令:\n");
    printf("  pose <x> <y> <z> <roll> <pitch> <yaw> - 设置机体位姿\n");
    printf("    位置单位: 米, 角度单位: 度\n");
    printf("    示例: pose 0 0 0.15 0 5 0 (高度15cm，俯仰5度)\n\n");

    printf("🌊 平滑运动控制命令:\n");
    printf("  smooth_init                 - 初始化平滑运动系统\n");
    printf("  action_cy [beta]            - 圆锥连续动作 (beta:5-45度，默认30)\n");
    printf("  pose6 <x> <y> <z> <r> <p> <y> - 6DOF平滑姿态控制\n\n");

    printf("ℹ️ 系统命令:\n");
    printf("  help       - 显示此帮助信息\n");
    printf("  status     - 显示系统状态\n");
    printf("  diag       - 硬件诊断和测试\n");
    printf("  read <ch>  - 读取指定通道PWM值\n");
    printf("  scan       - 扫描I2C设备\n\n");

    printf("💡 新架构特性:\n");
    printf("  - 分层模块化设计\n");
    printf("  - 统一配置管理\n");
    printf("  - 分级日志系统\n");
    printf("  - 完整错误处理\n");
    printf("==========================================\n\n");
}

void printStatus() {
    printf("\n=== 系统状态 ===\n");

    if (!g_robot_controller_) {
        printf("❌ 机器人控制器未初始化\n");
        return;
    }

    // 显示当前模式
    const char* mode_names[] = {"PWM模式", "舵机角度模式", "关节角度模式", "运动学模式"};
    printf("🎮 当前控制模式: %s\n", mode_names[static_cast<int>(g_current_mode_)]);
    printf("🎯 当前选中舵机: %d\n", g_selected_servo_);

    // 显示各个组件状态
    auto hw_driver = g_robot_controller_->getHardwareDriver();
    auto servo_driver = g_robot_controller_->getServoDriver();
    auto joint_controller = g_robot_controller_->getJointController();
    auto config_manager = g_robot_controller_->getConfigManager();

    printf("\n📊 组件状态:\n");
    printf("  硬件驱动 (PCA9685): %s\n",
           (hw_driver && hw_driver->getStatus() == Drivers::PCA9685Status::INITIALIZED) ? "✅ 正常" : "❌ 异常");
    printf("  舵机驱动: %s\n",
           (servo_driver && servo_driver->isInitialized()) ? "✅ 正常" : "❌ 异常");
    printf("  关节控制器: %s\n",
           (joint_controller && joint_controller->isInitialized()) ? "✅ 正常" : "❌ 异常");
    printf("  配置管理器: %s\n",
           (config_manager && config_manager->validateConfig()) ? "✅ 正常" : "❌ 异常");

    if (hw_driver) {
        printf("\n⚙️ 硬件信息:\n");
        printf("  I2C地址: 0x%02X\n", hw_driver->getI2CAddress());
        printf("  PWM频率: %d Hz\n", hw_driver->getFrequency());
    }

    if (config_manager) {
        printf("\n📋 配置信息:\n");
        printf("  配置版本: %d\n", config_manager->getConfig().config_version);
        printf("  机器人名称: %s\n", config_manager->getConfig().robot_name.c_str());
    }

    printf("==================\n\n");
}

// ==================== 命令处理函数实现 ====================

bool handleSystemCommands(const char* input) {
    if (strcmp(input, "help") == 0) {
        printHelp();
        return true;
    }

    if (strcmp(input, "status") == 0) {
        printStatus();
        return true;
    }

    return false;
}

bool handleModeCommands(const char* input) {
    if (strcmp(input, "pwm") == 0) {
        g_current_mode_ = RobotController::ControlMode::PWM_MODE;
        g_robot_controller_->setControlMode(g_current_mode_);
        LOG_TAG_INFO("MODE", "切换到PWM直接控制模式");
        return true;
    }

    if (strcmp(input, "angle") == 0) {
        g_current_mode_ = RobotController::ControlMode::SERVO_ANGLE_MODE;
        g_robot_controller_->setControlMode(g_current_mode_);
        LOG_TAG_INFO("MODE", "切换到舵机角度控制模式");
        return true;
    }

    if (strcmp(input, "joint") == 0) {
        g_current_mode_ = RobotController::ControlMode::JOINT_ANGLE_MODE;
        g_robot_controller_->setControlMode(g_current_mode_);
        LOG_TAG_INFO("MODE", "切换到关节角度控制模式");
        return true;
    }

    if (strcmp(input, "kinematics") == 0) {
        g_current_mode_ = RobotController::ControlMode::JOINT_ANGLE_MODE;  // 使用关节角度模式代替
        g_robot_controller_->setControlMode(g_current_mode_);
        LOG_TAG_INFO("MODE", "切换到运动学控制模式");
        return true;
    }

    return false;
}

bool handleServoCommands(const char* input) {
    if (input[0] == 's' && input[1] == ' ') {
        int servo_id = atoi(&input[2]);
        if (servo_id >= 0 && servo_id < 12) {
            g_selected_servo_ = servo_id;
            LOG_TAG_INFO("SERVO", "选中舵机 %d", servo_id);
        } else {
            LOG_TAG_ERROR("CMD", "无效的舵机ID: %d", servo_id);
        }
        return true;
    }
    return false;
}

bool handlePoseCommands(const char* input) {
    if (strcmp(input, "sleep0") == 0) {
        auto joint_controller = g_robot_controller_->getJointController();
        if (joint_controller) {
            JointError result = joint_controller->setSleepPose();
            LOG_TAG_INFO("POSE", "趴下姿势: %s", JointController::getErrorString(result));
        }
        return true;
    }

    if (strcmp(input, "sleep1") == 0) {
        auto joint_controller = g_robot_controller_->getJointController();
        if (joint_controller) {
            JointError result = joint_controller->setStandPose();
            LOG_TAG_INFO("POSE", "站立姿势: %s", JointController::getErrorString(result));
        }
        return true;
    }

    return false;
}

bool handleJointCommands(const char* input) {
    if (input[0] == 'j' && input[1] == ' ') {
        int leg_id, joint_type;
        float angle;
        if (sscanf(&input[2], "%d %d %f", &leg_id, &joint_type, &angle) == 3) {
            if (leg_id >= 0 && leg_id < 4 && joint_type >= 0 && joint_type < 3) {
                auto joint_controller = g_robot_controller_->getJointController();
                if (joint_controller) {
                    JointError result = joint_controller->setJointAngle(
                        static_cast<LegID>(leg_id),
                        static_cast<JointType>(joint_type),
                        angle);

                    if (result == JointError::SUCCESS) {
                        // 显示详细的转换结果（类似test_joint_accuracy.cpp）
                        float servo_angle;
                        JointError conv_result = joint_controller->jointToServoAngle(
                            static_cast<LegID>(leg_id),
                            static_cast<JointType>(joint_type),
                            angle, servo_angle);

                        int servo_id = joint_controller->getServoId(
                            static_cast<LegID>(leg_id),
                            static_cast<JointType>(joint_type));

                        if (conv_result == JointError::SUCCESS) {
                            LOG_TAG_INFO("JOINT", "✅ 腿%d关节%d: %.2f° -> 舵机%d %.2f°",
                                        leg_id, joint_type, angle, servo_id, servo_angle);
                        } else {
                            LOG_TAG_INFO("JOINT", "✅ 腿%d关节%d设置为%.2f°",
                                        leg_id, joint_type, angle);
                        }
                    } else {
                        LOG_TAG_ERROR("JOINT", "❌ 腿%d关节%d设置失败: %s",
                                     leg_id, joint_type, JointController::getErrorString(result));
                    }
                } else {
                    LOG_TAG_ERROR("CMD", "❌ JointController不可用");
                }
            } else {
                LOG_TAG_ERROR("CMD", "❌ 无效参数: 腿部ID(0-3): %d, 关节类型(0-2): %d", leg_id, joint_type);
            }
        } else {
            LOG_TAG_ERROR("CMD", "❌ 关节命令格式错误，正确格式: j <腿> <关节> <角度>");
        }
        return true;
    }
    return false;
}

bool handleDirectValueCommand(const char* input) {
    char* endptr;
    float value = strtof(input, &endptr);

    if (endptr != input && *endptr == '\0') {
        // 是一个有效的数值
        switch (g_current_mode_) {
            case RobotController::ControlMode::PWM_MODE: {
                auto hw_driver = g_robot_controller_->getHardwareDriver();
                if (hw_driver) {
                    uint16_t pwm_value = static_cast<uint16_t>(value);
                    auto result = hw_driver->setPWM(g_selected_servo_, pwm_value);
                    LOG_TAG_INFO("PWM", "舵机%d PWM设置为%d: %s",
                                g_selected_servo_, pwm_value,
                                Drivers::PCA9685Driver::getErrorString(result));
                }
                break;
            }

            case RobotController::ControlMode::SERVO_ANGLE_MODE: {
                auto servo_driver = g_robot_controller_->getServoDriver();
                if (servo_driver) {
                    ServoError result = servo_driver->setAngle(g_selected_servo_, value);
                    LOG_TAG_INFO("SERVO", "舵机%d角度设置为%.2f°: %s",
                                g_selected_servo_, value,
                                ServoDriver::getErrorString(result));
                }
                break;
            }

            case RobotController::ControlMode::JOINT_ANGLE_MODE: {
                // 在关节模式下，支持直接输入角度控制当前选中的关节
                auto joint_controller = g_robot_controller_->getJointController();
                if (joint_controller) {
                    JointError result = joint_controller->setJointAngle(
                        g_selected_leg_, g_selected_joint_, value);

                    if (result == JointError::SUCCESS) {
                        // 显示详细的转换结果
                        float servo_angle;
                        JointError conv_result = joint_controller->jointToServoAngle(
                            g_selected_leg_, g_selected_joint_, value, servo_angle);

                        int servo_id = joint_controller->getServoId(g_selected_leg_, g_selected_joint_);

                        if (conv_result == JointError::SUCCESS) {
                            LOG_TAG_INFO("JOINT", "✅ 当前关节: %.2f° -> 舵机%d %.2f°",
                                        value, servo_id, servo_angle);
                        } else {
                            LOG_TAG_INFO("JOINT", "✅ 当前关节设置为%.2f°", value);
                        }
                    } else {
                        LOG_TAG_ERROR("JOINT", "❌ 关节设置失败: %s",
                                     JointController::getErrorString(result));
                    }
                } else {
                    LOG_TAG_WARN("CMD", "❌ JointController不可用");
                }
                break;
            }
        }
        return true;
    }
    return false;
}

bool handleDirectServoCommand(const char* input) {
    int servo_id;
    float servo_value;
    if (sscanf(input, "%d %f", &servo_id, &servo_value) == 2) {
        if (servo_id >= 0 && servo_id < 12) {
            switch (g_current_mode_) {
                case RobotController::ControlMode::PWM_MODE: {
                    auto hw_driver = g_robot_controller_->getHardwareDriver();
                    if (hw_driver) {
                        uint16_t pwm_value = static_cast<uint16_t>(servo_value);
                        auto result = hw_driver->setPWM(servo_id, pwm_value);
                        LOG_TAG_INFO("PWM", "舵机%d PWM设置为%d: %s",
                                    servo_id, pwm_value,
                                    Drivers::PCA9685Driver::getErrorString(result));
                    }
                    break;
                }

                case RobotController::ControlMode::SERVO_ANGLE_MODE: {
                    auto servo_driver = g_robot_controller_->getServoDriver();
                    if (servo_driver) {
                        ServoError result = servo_driver->setAngle(servo_id, servo_value);
                        LOG_TAG_INFO("SERVO", "舵机%d角度设置为%.2f°: %s",
                                    servo_id, servo_value,
                                    ServoDriver::getErrorString(result));
                    }
                    break;
                }

                default:
                    LOG_TAG_WARN("CMD", "当前模式不支持直接舵机控制");
                    break;
            }
        } else {
            LOG_TAG_ERROR("CMD", "无效的舵机ID: %d", servo_id);
        }
        return true;
    }
    return false;
}
// ==================== 硬件诊断命令 ====================

bool handleDiagnosisCommands(const char* input) {
    // 硬件诊断命令
    if (strcmp(input, "diag") == 0) {
        printf("\n=== 🔧 硬件诊断报告 ===\n");

        auto hw_driver = g_robot_controller_->getHardwareDriver();
        if (!hw_driver) {
            printf("❌ 硬件驱动未创建\n");
            return true;
        }

        // 显示驱动状态
        printf("🔌 PCA9685状态: ");
        switch (hw_driver->getStatus()) {
            case Drivers::PCA9685Status::UNINITIALIZED:
                printf("❌ 未初始化\n");
                break;
            case Drivers::PCA9685Status::INITIALIZED:
                printf("✅ 已初始化\n");
                break;
            case Drivers::PCA9685Status::ERROR:
                printf("❌ 错误状态\n");
                break;
        }

        printf("📡 I2C地址: 0x%02X\n", hw_driver->getI2CAddress());
        printf("⚡ PWM频率: %d Hz\n", hw_driver->getFrequency());

        // 测试读取前几个通道的PWM值
        printf("\n📊 当前PWM值读取测试:\n");
        for (int ch = 0; ch < 4; ch++) {
            uint16_t pwm_value;
            auto result = hw_driver->getPWM(ch, pwm_value);
            if (result == Drivers::PCA9685Error::SUCCESS) {
                printf("  通道 %d: PWM = %d\n", ch, pwm_value);
            } else {
                printf("  通道 %d: 读取失败 - %s\n", ch, Drivers::PCA9685Driver::getErrorString(result));
            }
        }

        printf("====================\n\n");
        return true;
    }

    // 读取指定通道PWM值
    if (strncmp(input, "read ", 5) == 0) {
        int channel = atoi(&input[5]);
        if (channel >= 0 && channel < 16) {
            auto hw_driver = g_robot_controller_->getHardwareDriver();
            if (hw_driver) {
                uint16_t pwm_value;
                auto result = hw_driver->getPWM(channel, pwm_value);
                if (result == Drivers::PCA9685Error::SUCCESS) {
                    LOG_TAG_INFO("READ", "通道 %d PWM值: %d", channel, pwm_value);
                } else {
                    LOG_TAG_ERROR("READ", "读取通道 %d 失败: %s", channel,
                                 Drivers::PCA9685Driver::getErrorString(result));
                }
            }
        } else {
            LOG_TAG_ERROR("CMD", "无效的通道号: %d (有效范围: 0-15)", channel);
        }
        return true;
    }

    // I2C设备扫描
    if (strcmp(input, "scan") == 0) {
        printf("\n=== 🔍 I2C设备扫描 ===\n");
        printf("扫描I2C总线上的设备...\n");

        bool found_device = false;
        for (uint8_t addr = 0x08; addr < 0x78; addr++) {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);

            esp_err_t result = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10));
            i2c_cmd_link_delete(cmd);

            if (result == ESP_OK) {
                printf("✅ 发现设备: 0x%02X", addr);
                if (addr == 0x40) printf(" (PCA9685默认地址)");
                printf("\n");
                found_device = true;
            }
        }

        if (!found_device) {
            printf("❌ 未发现任何I2C设备\n");
            printf("请检查:\n");
            printf("  - I2C接线 (SDA:GPIO21, SCL:GPIO22)\n");
            printf("  - 设备供电\n");
            printf("  - 上拉电阻\n");
        }

        printf("========================\n\n");
        return true;
    }

    return false;
}

bool handleJointSelectionCommands(const char* input) {
    // 处理腿部选择命令: leg <0-3>
    if (strncmp(input, "leg ", 4) == 0) {
        int leg_id = atoi(&input[4]);
        if (leg_id >= 0 && leg_id <= 3) {
            g_selected_leg_ = static_cast<LegID>(leg_id);
            const char* leg_names[] = {"左前腿", "右前腿", "左后腿", "右后腿"};
            LOG_TAG_INFO("JOINT", "✅ 选择%s", leg_names[leg_id]);

            // 显示当前选择信息
            auto joint_controller = g_robot_controller_->getJointController();
            if (joint_controller) {
                int servo_id = joint_controller->getServoId(g_selected_leg_, g_selected_joint_);
                const char* joint_names[] = {"髋侧摆", "髋俯仰", "膝俯仰"};
                LOG_TAG_INFO("JOINT", "📊 当前选择: %s %s -> 舵机%d",
                           leg_names[leg_id], joint_names[static_cast<int>(g_selected_joint_)], servo_id);
            }
        } else {
            LOG_TAG_ERROR("CMD", "❌ 无效腿部ID: %d (有效范围: 0-3)", leg_id);
        }
        return true;
    }

    // 处理关节选择命令: joint <0-2>
    if (strncmp(input, "joint ", 6) == 0) {
        int joint_id = atoi(&input[6]);
        if (joint_id >= 0 && joint_id <= 2) {
            g_selected_joint_ = static_cast<JointType>(joint_id);
            const char* joint_names[] = {"髋侧摆", "髋俯仰", "膝俯仰"};
            LOG_TAG_INFO("JOINT", "✅ 选择%s", joint_names[joint_id]);

            // 显示当前选择信息
            auto joint_controller = g_robot_controller_->getJointController();
            if (joint_controller) {
                int servo_id = joint_controller->getServoId(g_selected_leg_, g_selected_joint_);
                const char* leg_names[] = {"左前腿", "右前腿", "左后腿", "右后腿"};
                LOG_TAG_INFO("JOINT", "📊 当前选择: %s %s -> 舵机%d",
                           leg_names[static_cast<int>(g_selected_leg_)], joint_names[joint_id], servo_id);
            }
        } else {
            LOG_TAG_ERROR("CMD", "❌ 无效关节ID: %d (有效范围: 0-2)", joint_id);
        }
        return true;
    }

    return false;
}

// ==================== 运动学控制函数实现 ====================

void initKinematicsSystem() {
    LOG_TAG_INFO("KINEMATICS", "初始化运动学系统...");

    // 初始化四条腿的运动学求解器
    g_left_front_leg_ = std::make_unique<SpotLegKinematics>(true);
    g_right_front_leg_ = std::make_unique<SpotLegKinematics>(false);
    g_left_back_leg_ = std::make_unique<SpotLegKinematics>(true);
    g_right_back_leg_ = std::make_unique<SpotLegKinematics>(false);

    // 初始化腿部状态
    resetLegStates();

    LOG_TAG_INFO("KINEMATICS", "运动学系统初始化完成");
}

void resetLegStates() {
    // 设置默认关节角度（合理的站立姿态）
    ThreeJointAngles default_angles(0.0f, 0.0f, 0.0f);

    for (int i = 0; i < 4; i++) {
        g_leg_states_[i].joint_angles = default_angles;
        g_leg_states_[i].valid = true;

        // 通过正运动学计算脚部位置
        KinematicsResult fk_result = calculateForwardKinematics(i, default_angles);
        if (fk_result.success) {
            auto hip_offset = getHipOffset(i);
            auto foot_hip = g_frame_manager_.transformBodyToHip(fk_result.foot_position, hip_offset);
            g_leg_states_[i].foot_position = foot_hip;
            g_foot_world_positions_[i] = fk_result.foot_position;
        } else {
            g_leg_states_[i].foot_position = CoordinateTransform::Vector3(0.12f, 0.0f, -0.15f);
            auto hip_offset = getHipOffset(i);
            auto foot_body = g_frame_manager_.transformHipToBody(g_leg_states_[i].foot_position, hip_offset);
            g_foot_world_positions_[i] = foot_body;
        }
    }
}

SpotLegKinematics* getLegKinematics(int leg_id) {
    switch (leg_id) {
        case 0: return g_left_front_leg_.get();
        case 1: return g_right_front_leg_.get();
        case 2: return g_left_back_leg_.get();
        case 3: return g_right_back_leg_.get();
        default: return nullptr;
    }
}

CoordinateTransform::Vector3 getHipOffset(int leg_id) {
    SpotMicroGeometry geometry;  // 创建实例
    auto hip_offset = geometry.getHipOffset(static_cast<SpotMicroGeometry::LegIndex>(leg_id));
    return CoordinateTransform::Vector3(hip_offset.x, hip_offset.y, hip_offset.z);
}

void updateFootWorldPositions() {
    for (int leg_id = 0; leg_id < 4; leg_id++) {
        if (g_leg_states_[leg_id].valid) {
            auto foot_hip = g_leg_states_[leg_id].foot_position;
            auto hip_offset = getHipOffset(leg_id);
            auto foot_body = g_frame_manager_.transformHipToBody(foot_hip, hip_offset);
            g_foot_world_positions_[leg_id] = g_frame_manager_.transformBodyToWorld(foot_body);
        }
    }
}

void controlServos() {
    if (!g_robot_controller_) {
        LOG_TAG_ERROR("KINEMATICS", "机器人控制器未初始化");
        return;
    }

    auto joint_controller = g_robot_controller_->getJointController();
    if (!joint_controller) {
        LOG_TAG_ERROR("KINEMATICS", "关节控制器未初始化");
        return;
    }

    int success_count = 0;
    int total_count = 0;

    for (int leg_id = 0; leg_id < 4; leg_id++) {
        if (!g_leg_states_[leg_id].valid) {
            continue;
        }

        auto& angles = g_leg_states_[leg_id].joint_angles;
        float hip_side_deg = angles.hip_side * 180.0f / M_PI;
        float hip_pitch_deg = angles.hip_pitch * 180.0f / M_PI;
        float knee_pitch_deg = angles.knee_pitch * 180.0f / M_PI;

        LegID leg_enum = static_cast<LegID>(leg_id);

        // 控制三个关节
        total_count++;
        if (joint_controller->setJointAngle(leg_enum, JointType::HIP_ROLL, hip_side_deg) == JointError::SUCCESS) {
            success_count++;
        }

        total_count++;
        if (joint_controller->setJointAngle(leg_enum, JointType::HIP_PITCH, hip_pitch_deg) == JointError::SUCCESS) {
            success_count++;
        }

        total_count++;
        if (joint_controller->setJointAngle(leg_enum, JointType::KNEE_PITCH, knee_pitch_deg) == JointError::SUCCESS) {
            success_count++;
        }
    }
}

KinematicsResult calculateForwardKinematics(int leg_id, const ThreeJointAngles& joint_angles) {
    SpotLegKinematics* leg = getLegKinematics(leg_id);
    if (!leg) {
        return KinematicsResult(false, joint_angles, CoordinateTransform::Vector3(), "无效的腿部ID");
    }

    SpotMicroGeometry geometry;  // 创建实例
    auto hip_offset = geometry.getHipOffset(static_cast<SpotMicroGeometry::LegIndex>(leg_id));
    CoordinateTransform::Vector3 hip_pos(hip_offset.x, hip_offset.y, hip_offset.z);

    return leg->forwardKinematics(hip_pos, joint_angles);
}

KinematicsResult calculateInverseKinematics(int leg_id, const CoordinateTransform::Vector3& foot_pos) {
    SpotLegKinematics* leg = getLegKinematics(leg_id);
    if (!leg) {
        return KinematicsResult(false, ThreeJointAngles(), foot_pos, "无效的腿部ID");
    }

    return leg->inverseKinematics(foot_pos);
}

bool handleKinematicsCommands(const char* input) {
    // 解析pose命令: pose <x> <y> <z> <roll> <pitch> <yaw>
    if (strncmp(input, "pose ", 5) == 0) {
        float x, y, z, roll, pitch, yaw;
        int parsed = sscanf(input + 5, "%f %f %f %f %f %f", &x, &y, &z, &roll, &pitch, &yaw);

        if (parsed != 6) {
            LOG_TAG_ERROR("KINEMATICS", "用法: pose <x> <y> <z> <roll> <pitch> <yaw>");
            LOG_TAG_INFO("KINEMATICS", "示例: pose 0 0 0.15 0 5 0");
            return true;
        }

        LOG_TAG_INFO("KINEMATICS", "🔄 开始位姿变换...");

        // 步骤1: 保存当前脚部在世界坐标系中的位置
        updateFootWorldPositions();

        // 步骤2: 设置新的机体位姿
        float roll_rad = roll * M_PI / 180.0f;
        float pitch_rad = pitch * M_PI / 180.0f;
        float yaw_rad = yaw * M_PI / 180.0f;

        CoordinateTransform::Pose6DOF new_body_pose(x, y, z, roll_rad, pitch_rad, yaw_rad);
        g_frame_manager_.setBodyPose(new_body_pose);

        LOG_TAG_INFO("KINEMATICS", "✅ 机体位姿设置成功: pos(%.3f,%.3f,%.3f) rot(%.1f°,%.1f°,%.1f°)",
                     x, y, z, roll, pitch, yaw);

        // 步骤3: 基于脚部世界坐标计算新的关节角度
        for (int leg_id = 0; leg_id < 4; leg_id++) {
            auto foot_world = g_foot_world_positions_[leg_id];
            auto hip_offset = getHipOffset(leg_id);
            auto foot_body = g_frame_manager_.transformWorldToBody(foot_world);
            auto foot_hip = g_frame_manager_.transformBodyToHip(foot_body, hip_offset);

            KinematicsResult result = calculateInverseKinematics(leg_id, foot_hip);

            if (result.success) {
                g_leg_states_[leg_id].foot_position = foot_hip;
                g_leg_states_[leg_id].joint_angles = result.joint_angles;
                g_leg_states_[leg_id].valid = true;
            } else {
                g_leg_states_[leg_id].valid = false;
                LOG_TAG_ERROR("KINEMATICS", "腿%d 逆运动学失败: %s", leg_id, result.error_message);
            }
        }

        // 步骤4: 显示结果
        printf("\n📐 各腿关节角度计算结果:\n");
        const char* leg_names[] = {"左前腿", "右前腿", "左后腿", "右后腿"};
        for (int leg_id = 0; leg_id < 4; leg_id++) {
            if (g_leg_states_[leg_id].valid) {
                auto& angles = g_leg_states_[leg_id].joint_angles;
                int hip_side_deg = (int)(angles.hip_side * 180.0f / M_PI);
                int hip_pitch_deg = (int)(angles.hip_pitch * 180.0f / M_PI);
                int knee_pitch_deg = (int)(angles.knee_pitch * 180.0f / M_PI);

                printf("%s: %d°|%d°|%d°\n", leg_names[leg_id], hip_side_deg, hip_pitch_deg, knee_pitch_deg);
            }
        }

        // 步骤5: 控制舵机运动
        controlServos();

        return true;
    }

    return false;
}

// ===== 平滑运动控制功能实现 =====

/**
 * @brief 初始化平滑运动系统
 */
void initSmoothMotionSystem() {
    printf("🔄 初始化平滑运动控制系统...\n");

    if (!g_robot_controller_) {
        printf("❌ 机器人控制器未初始化\n");
        return;
    }

    // 初始化QuadrupedModel
    g_quadruped_model_ = std::make_shared<Robot::Kinematics::QuadrupedModel>();
    printf("✅ QuadrupedModel初始化完成\n");

    // 初始化SmoothMotionController
    Robot::Controllers::SmoothMotionConfig config;
    config.enable_pose_smoothing = true;
    config.enable_servo_smoothing = true;
    config.pose_interpolation.position_step = 0.005f;  // 5mm
    config.pose_interpolation.angle_step = 0.1f;       // 0.1rad
    config.pose_interpolation.max_steps = 50;
    config.servo_smoothing.angle_step = 2.0f;          // 2°
    config.servo_smoothing.step_delay_ms = 10;

    g_smooth_controller_ = std::make_unique<Robot::Controllers::SmoothMotionController>(g_robot_controller_, config);
    g_smooth_controller_->setQuadrupedModel(g_quadruped_model_);

    if (g_smooth_controller_->initialize()) {
        g_smooth_motion_initialized_ = true;
        printf("✅ 平滑运动控制器初始化成功\n");
    } else {
        printf("❌ 平滑运动控制器初始化失败\n");
    }
}

/**
 * @brief 根据alpha和beta角度计算圆锥运动的roll,pitch,yaw
 */
std::tuple<float, float, float> coneRpyFromAlpha(float alpha, float beta) {
    // yaw = atan2(sin(β) * cos(α), cos(β))
    float yaw = std::atan2(std::sin(beta) * std::cos(alpha), std::cos(beta));
    // pitch = -asin(sin(β) * sin(α))
    float pitch = -std::asin(std::max(-1.0f, std::min(1.0f, std::sin(beta) * std::sin(alpha))));
    // roll在由方向确定的情况下是自由度，设为0便于观察
    float roll = 0.0f;
    return std::make_tuple(roll, pitch, yaw);
}

/**
 * @brief 解析字符串参数为浮点数数组
 */
int parseFloatArgs(const char* input, float* args, int max_args) {
    const char* ptr = input;
    int count = 0;

    // 跳过命令名
    while (*ptr && *ptr != ' ') ptr++;
    while (*ptr == ' ') ptr++;

    while (*ptr && count < max_args) {
        char* endptr;
        args[count] = strtof(ptr, &endptr);
        if (endptr == ptr) break; // 没有有效数字
        count++;
        ptr = endptr;
        while (*ptr == ' ') ptr++; // 跳过空格
    }

    return count;
}

/**
 * @brief 处理平滑运动控制命令
 */
bool handleSmoothMotionCommands(const char* input) {
    // smooth_init 命令
    if (strcmp(input, "smooth_init") == 0) {
        initSmoothMotionSystem();
        return true;
    }

    // action_cy [beta] 命令
    if (strncmp(input, "action_cy", 9) == 0) {
        if (!g_smooth_motion_initialized_) {
            printf("❌ 平滑运动系统未初始化，请先执行 smooth_init\n");
            return true;
        }

        // 解析beta参数
        float beta_degrees = 30.0f; // 默认30度
        const char* ptr = input + 9; // 跳过"action_cy"
        while (*ptr == ' ') ptr++; // 跳过空格

        if (*ptr) {
            char* endptr;
            float user_beta = strtof(ptr, &endptr);
            if (endptr != ptr && user_beta >= 5.0f && user_beta <= 45.0f) {
                beta_degrees = user_beta;
            } else {
                printf("❌ Beta参数无效，应为5-45度之间的数值，使用默认值30度\n");
            }
        }

        printf("🌊 执行圆锥连续动作: action_cy\n");
        printf("📐 半顶角β = %.1f°\n", beta_degrees);

        // 计算圆锥运动起始时的yaw角度，确保连续性
        float initial_yaw = std::atan2(std::sin(beta_degrees * M_PI / 180.0f), std::cos(beta_degrees * M_PI / 180.0f));
        float initial_yaw_degrees = initial_yaw * 180.0f / M_PI;

        // Step 1: 运动到初始位置
        printf("📍 Step 1: 运动到初始位置 (0.05, 0, -0.1, 0, 0, %.1f°)\n", initial_yaw_degrees);
        Robot::Controllers::BodyPose initial_pose(0.05f, 0.0f, -0.1f, 0.0f, 0.0f, initial_yaw);

        if (g_smooth_controller_ && g_smooth_controller_->getMotionState() != Robot::Controllers::MotionState::ERROR) {
            g_smooth_controller_->setTargetPose(initial_pose);
            bool success = g_smooth_controller_->executeMotionToTarget();
            if (!success) {
                printf("⚠️ 平滑控制器执行有问题\n");
            }
        }

        printf("✅ 到达初始位置\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Step 2: 执行圆锥运动
        printf("🌀 Step 2: 开始圆锥运动轨迹 (36个采样点，半顶角β=%.1f°)\n", beta_degrees);

        float beta = beta_degrees * M_PI / 180.0f;
        const int num_samples = 36;
        const float alpha_step = 2.0f * M_PI / num_samples;

        for (int i = 0; i < num_samples; i++) {
            float alpha = i * alpha_step;
            auto [roll, pitch, yaw] = coneRpyFromAlpha(alpha, beta);

            Robot::Controllers::BodyPose cone_pose(0.05f, 0.0f, -0.1f, roll, pitch, yaw);

            printf("🎯 %d/36 alpha=%.1f° roll=%.1f° pitch=%.1f° yaw=%.1f°\n",
                   i + 1, alpha * 180.0f / M_PI, roll * 180.0f / M_PI,
                   pitch * 180.0f / M_PI, yaw * 180.0f / M_PI);

            if (g_smooth_controller_ && g_smooth_controller_->getMotionState() != Robot::Controllers::MotionState::ERROR) {
                g_smooth_controller_->setTargetPose(cone_pose);
                g_smooth_controller_->executeMotionToTarget();
            }
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);
        // Step 3: 回到中性位置
        printf("🏠 Step 3: 回到中性位置 (0.05, 0, -0.1, 0, 0, 0°)\n");
        Robot::Controllers::BodyPose neutral_pose(0.05f, 0.0f, -0.1f, 0.0f, 0.0f, 0.0f);

        if (g_smooth_controller_ && g_smooth_controller_->getMotionState() != Robot::Controllers::MotionState::ERROR) {
            g_smooth_controller_->setTargetPose(neutral_pose);
            bool success = g_smooth_controller_->executeMotionToTarget();
            if (success) {
                printf("✅ 已回到中性位置\n");
            } else {
                printf("⚠️ 回归中性位置时有问题\n");
            }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("🎬 圆锥连续动作完成！\n");
        return true;
    }

    // pose6 <x> <y> <z> <roll> <pitch> <yaw> 命令
    if (strncmp(input, "pose6", 5) == 0) {
        if (!g_smooth_motion_initialized_) {
            printf("❌ 平滑运动系统未初始化，请先执行 smooth_init\n");
            return true;
        }

        float args[6];
        int arg_count = parseFloatArgs(input, args, 6);

        if (arg_count != 6) {
            printf("❌ 用法: pose6 <x> <y> <z> <roll> <pitch> <yaw>\n");
            printf("   位置单位: 米, 角度单位: 度\n");
            printf("   示例: pose6 0 0 -0.12 0 5 0  (高度-0.12m，俯仰5度)\n");
            return true;
        }

        float x = args[0], y = args[1], z = args[2];
        float roll = args[3] * M_PI / 180.0f;
        float pitch = args[4] * M_PI / 180.0f;
        float yaw = args[5] * M_PI / 180.0f;

        printf("🔄 开始平滑位姿变换...\n");
        printf("  目标位置: (%.3f, %.3f, %.3f)\n", x, y, z);
        printf("  目标姿态: (%.1f°, %.1f°, %.1f°)\n", args[3], args[4], args[5]);

        if (g_smooth_controller_ && g_smooth_controller_->getMotionState() != Robot::Controllers::MotionState::ERROR) {
            Robot::Controllers::BodyPose target_pose(x, y, z, roll, pitch, yaw);

            printf("🌊 执行平滑运动控制...\n");
            g_smooth_controller_->setTargetPose(target_pose);
            bool motion_success = g_smooth_controller_->executeMotionToTarget();

            if (motion_success) {
                printf("✅ 平滑姿态控制完成\n");
            } else {
                printf("⚠️ 平滑控制执行有警告\n");
            }
        } else {
            printf("❌ 平滑控制器不可用\n");
        }

        return true;
    }

    return false;
}
