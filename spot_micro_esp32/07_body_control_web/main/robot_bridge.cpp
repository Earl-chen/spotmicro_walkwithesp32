/**
 * @file robot_bridge.cpp
 * @brief 机器人控制桥接层实现
 * 
 * 连接C接口和C++机器人控制系统
 * 提供线程安全的控制接口和错误处理
 * 
 * @author ESP32四足机器人项目组
 * @date 2025-09-26
 */

#include "robot_bridge.h"
#include <cstring>
#include <cmath>
#include <memory>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ESP-IDF
#include <esp_log.h>

// 机器人控制系统
#include "controllers/RobotController.hpp"
#include "controllers/SmoothMotionController.hpp"
#include "controllers/GaitController.hpp"          // 新增：步态控制器
#include "kinematics/QuadrupedModel.hpp"
#include "utils/Logger.hpp"
#include "gait/WalkGait.hpp"                       // 新增：Walk步态算法

using namespace Robot;
using namespace Robot::Controllers;
using namespace Robot::Kinematics;

 // 桥接层管理的控制器实例（内部自维护，避免外部链接依赖）
static std::shared_ptr<RobotController> g_robot_controller_;
static std::unique_ptr<SmoothMotionController> g_smooth_controller_;
static std::shared_ptr<QuadrupedModel> g_quadruped_model_;
static bool g_smooth_motion_initialized_ = false;

// 步态控制器（新增）
static std::unique_ptr<Gait::WalkGait> g_walk_gait_;
static bool g_gait_running_ = false;

// 桥接层内部状态
static bool g_bridge_initialized_ = false;
static robot_pose_state_t g_current_pose_ = {0.05f, 0.0f, -0.10f, 0.0f, 0.0f, 0.0f};
static char g_last_error_[256] = {0};
static SemaphoreHandle_t g_bridge_mutex_ = nullptr;

// 日志标签
static const char* TAG = "ROBOT_BRIDGE";

// 内部函数声明
static void set_last_error(const char* error_msg);
static float clamp_float(float value, float min_val, float max_val);
static bool validate_pose_params(float x, float y, float z, float roll, float pitch, float yaw);
static bool validate_cone_params(float beta_deg);

 // 不再依赖外部初始化函数，桥接层内部实现初始化流程

/**
 * @brief 设置最后的错误信息
 */
static void set_last_error(const char* error_msg) {
    if (error_msg) {
        strncpy(g_last_error_, error_msg, sizeof(g_last_error_) - 1);
        g_last_error_[sizeof(g_last_error_) - 1] = '\0';
        ESP_LOGE(TAG, "%s", error_msg);
    }
}

/**
 * @brief 浮点数限幅函数
 */
static float clamp_float(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/**
 * @brief 验证姿态参数有效性
 */
static bool validate_pose_params(float x, float y, float z, float roll, float pitch, float yaw) {
    // 位置限制 (米)
    if (x < -0.12f || x > 0.12f) {
        set_last_error("X position out of range [-0.12, 0.12]");
        return false;
    }
    if (y < -0.12f || y > 0.12f) {
        set_last_error("Y position out of range [-0.12, 0.12]");
        return false;
    }
    if (z < -0.22f || z > -0.05f) {
        set_last_error("Z position out of range [-0.22, -0.05]");
        return false;
    }
    
    // 角度限制 (度)
    if (roll < -180.0f || roll > 180.0f) {
        set_last_error("Roll angle out of range [-180, 180]");
        return false;
    }
    if (pitch < -180.0f || pitch > 180.0f) {
        set_last_error("Pitch angle out of range [-180, 180]");
        return false;
    }
    if (yaw < -180.0f || yaw > 180.0f) {
        set_last_error("Yaw angle out of range [-180, 180]");
        return false;
    }
    
    return true;
}

/**
 * @brief 验证圆锥动作参数
 */
static bool validate_cone_params(float beta_deg) {
    if (beta_deg < 5.0f || beta_deg > 45.0f) {
        set_last_error("Beta angle out of range [5, 45]");
        return false;
    }
    return true;
}

// ========== 公共接口实现 ==========

extern "C" int robot_bridge_init(void) {
    ESP_LOGI(TAG, "🔄 初始化机器人桥接系统...");

    // 设置日志级别为WARN以减少冗余输出
    ESP_LOGI(TAG, "📝 设置日志级别为WARN以减少舵机调试信息");
    Robot::Utils::Logger::getInstance().setLogLevel(Robot::Utils::LogLevel::WARN);

    // 创建互斥锁
    if (g_bridge_mutex_ == nullptr) {
        g_bridge_mutex_ = xSemaphoreCreateMutex();
        if (g_bridge_mutex_ == nullptr) {
            set_last_error("Failed to create bridge mutex");
            return ROBOT_BRIDGE_ERROR_IO;
        }
    }
    
    // 获取互斥锁
    if (xSemaphoreTake(g_bridge_mutex_, pdMS_TO_TICKS(5000)) != pdTRUE) {
        set_last_error("Failed to acquire bridge mutex");
        return ROBOT_BRIDGE_ERROR_BUSY;
    }
    

        // 初始化机器人控制系统（桥接层内部）
        ESP_LOGI(TAG, "📡 初始化机器人控制系统...");
        g_robot_controller_ = std::make_shared<RobotController>();
        if (!g_robot_controller_ || !g_robot_controller_->init()) {
            set_last_error("Robot controller initialization failed");
            xSemaphoreGive(g_bridge_mutex_);
            return ROBOT_BRIDGE_ERROR_IO;
        }
        g_robot_controller_->setControlMode(RobotController::ControlMode::SERVO_ANGLE_MODE);

        // 初始化平滑运动系统（桥接层内部）
        ESP_LOGI(TAG, "🌊 初始化平滑运动系统...");
        g_quadruped_model_ = std::make_shared<QuadrupedModel>();
        Robot::Controllers::SmoothMotionConfig config;
        config.enable_pose_smoothing = true;
        config.enable_servo_smoothing = true;
        config.pose_interpolation.position_step = 0.005f;  // 5mm
        config.pose_interpolation.angle_step = 0.1f;       // 0.1rad
        config.pose_interpolation.max_steps = 50;
        config.servo_smoothing.angle_step = 2.0f;          // 2°
        config.servo_smoothing.step_delay_ms = 10;

        g_smooth_controller_ = std::make_unique<SmoothMotionController>(g_robot_controller_, config);
        g_smooth_controller_->setQuadrupedModel(g_quadruped_model_);
        if (!g_smooth_controller_->initialize()) {
            set_last_error("Smooth motion system initialization failed");
            xSemaphoreGive(g_bridge_mutex_);
            return ROBOT_BRIDGE_ERROR_IO;
        }
        g_smooth_motion_initialized_ = true;

        // 设置初始姿态
        g_current_pose_.x = 0.05f;
        g_current_pose_.y = 0.0f;
        g_current_pose_.z = -0.10f;
        g_current_pose_.roll = 0.0f;
        g_current_pose_.pitch = 0.0f;
        g_current_pose_.yaw = 0.0f;

        g_bridge_initialized_ = true;
        memset(g_last_error_, 0, sizeof(g_last_error_));
        ESP_LOGI(TAG, "✅ 机器人桥接系统初始化成功");
        

    
    xSemaphoreGive(g_bridge_mutex_);
    return ROBOT_BRIDGE_OK;
}

extern "C" bool robot_bridge_is_initialized(void) {
    return g_bridge_initialized_;
}

extern "C" int robot_bridge_pose6(float x, float y, float z, 
                                  float roll_deg, float pitch_deg, float yaw_deg) {
    if (!g_bridge_initialized_) {
        set_last_error("Bridge not initialized");
        return ROBOT_BRIDGE_ERROR_UNINIT;
    }
    
    // 参数验证
    if (!validate_pose_params(x, y, z, roll_deg, pitch_deg, yaw_deg)) {
        return ROBOT_BRIDGE_ERROR_PARAM;
    }
    
    // 获取互斥锁
    if (xSemaphoreTake(g_bridge_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        set_last_error("Failed to acquire mutex for pose6");
        return ROBOT_BRIDGE_ERROR_BUSY;
    }
    

        // 参数限幅
        x = clamp_float(x, -0.12f, 0.12f);
        y = clamp_float(y, -0.12f, 0.12f);
        z = clamp_float(z, -0.22f, -0.05f);
        roll_deg = clamp_float(roll_deg, -180.0f, 180.0f);
        pitch_deg = clamp_float(pitch_deg, -180.0f, 180.0f);
        yaw_deg = clamp_float(yaw_deg, -180.0f, 180.0f);
        
        // 角度转弧度
        float roll_rad = roll_deg * M_PI / 180.0f;
        float pitch_rad = pitch_deg * M_PI / 180.0f;
        float yaw_rad = yaw_deg * M_PI / 180.0f;
        
        ESP_LOGI(TAG, "🎯 执行pose6: pos(%.3f,%.3f,%.3f) rot(%.1f°,%.1f°,%.1f°)", 
                 x, y, z, roll_deg, pitch_deg, yaw_deg);
        
        // 检查平滑控制器状态
        if (!g_smooth_controller_ || 
            g_smooth_controller_->getMotionState() == MotionState::ERROR) {
            set_last_error("Smooth controller not available or in error state");
            xSemaphoreGive(g_bridge_mutex_);
            return ROBOT_BRIDGE_ERROR_IO;
        }
        
        // 设置目标姿态
        BodyPose target_pose(x, y, z, roll_rad, pitch_rad, yaw_rad);
        g_smooth_controller_->setTargetPose(target_pose);
        
        // 执行平滑运动
        bool success = g_smooth_controller_->executeMotionToTarget();
        
        if (success) {
            // 更新当前姿态状态
            g_current_pose_.x = x;
            g_current_pose_.y = y;
            g_current_pose_.z = z;
            g_current_pose_.roll = roll_deg;
            g_current_pose_.pitch = pitch_deg;
            g_current_pose_.yaw = yaw_deg;
            
            ESP_LOGI(TAG, "✅ pose6执行成功");
        } else {
            set_last_error("Smooth motion execution failed");
            xSemaphoreGive(g_bridge_mutex_);
            return ROBOT_BRIDGE_ERROR_IO;
        }
        

    
    xSemaphoreGive(g_bridge_mutex_);
    return ROBOT_BRIDGE_OK;
}

extern "C" int robot_bridge_action_cone(float beta_deg) {
    if (!g_bridge_initialized_) {
        set_last_error("Bridge not initialized");
        return ROBOT_BRIDGE_ERROR_UNINIT;
    }
    
    // 参数验证
    if (!validate_cone_params(beta_deg)) {
        return ROBOT_BRIDGE_ERROR_PARAM;
    }
    
    // 获取互斥锁
    if (xSemaphoreTake(g_bridge_mutex_, pdMS_TO_TICKS(2000)) != pdTRUE) {
        set_last_error("Failed to acquire mutex for action_cone");
        return ROBOT_BRIDGE_ERROR_BUSY;
    }
    

        ESP_LOGI(TAG, "🌀 执行圆锥动作: beta=%.1f°", beta_deg);
        
        // 检查平滑控制器状态
        if (!g_smooth_controller_ || 
            g_smooth_controller_->getMotionState() == MotionState::ERROR) {
            set_last_error("Smooth controller not available or in error state");
            xSemaphoreGive(g_bridge_mutex_);
            return ROBOT_BRIDGE_ERROR_IO;
        }
        
        float beta_rad = beta_deg * M_PI / 180.0f;
        
        // Step 1: 运动到初始位置
        float initial_yaw = std::atan2(std::sin(beta_rad), std::cos(beta_rad));
        BodyPose initial_pose(0.05f, 0.0f, -0.1f, 0.0f, 0.0f, initial_yaw);
        
        g_smooth_controller_->setTargetPose(initial_pose);
        if (!g_smooth_controller_->executeMotionToTarget()) {
            set_last_error("Failed to reach initial position for cone action");
            xSemaphoreGive(g_bridge_mutex_);
            return ROBOT_BRIDGE_ERROR_IO;
        }
        
        vTaskDelay(pdMS_TO_TICKS(500)); // 短暂停顿
        
        // Step 2: 执行圆锥运动轨迹
        const int num_samples = 36;
        const float alpha_step = 2.0f * M_PI / num_samples;
        
        for (int i = 0; i < num_samples; i++) {
            float alpha = i * alpha_step;
            
            // 计算圆锥运动的roll, pitch, yaw
            float yaw = std::atan2(std::sin(beta_rad) * std::cos(alpha), std::cos(beta_rad));
            float pitch = -std::asin(std::max(-1.0f, std::min(1.0f, std::sin(beta_rad) * std::sin(alpha))));
            float roll = 0.0f;
            
            BodyPose cone_pose(0.05f, 0.0f, -0.1f, roll, pitch, yaw);
            
            g_smooth_controller_->setTargetPose(cone_pose);
            if (!g_smooth_controller_->executeMotionToTarget()) {
                ESP_LOGW(TAG, "⚠️ 圆锥轨迹点%d执行有问题", i + 1);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // 停顿
        
        // Step 3: 回到中性位置
        BodyPose neutral_pose(0.05f, 0.0f, -0.1f, 0.0f, 0.0f, 0.0f);
        g_smooth_controller_->setTargetPose(neutral_pose);
        if (!g_smooth_controller_->executeMotionToTarget()) {
            ESP_LOGW(TAG, "⚠️ 回归中性位置有问题");
        }
        
        // 更新当前姿态为中性位置
        g_current_pose_.x = 0.05f;
        g_current_pose_.y = 0.0f;
        g_current_pose_.z = -0.1f;
        g_current_pose_.roll = 0.0f;
        g_current_pose_.pitch = 0.0f;
        g_current_pose_.yaw = 0.0f;
        
        ESP_LOGI(TAG, "✅ 圆锥动作执行完成");
        

    
    xSemaphoreGive(g_bridge_mutex_);
    return ROBOT_BRIDGE_OK;
}

extern "C" int robot_bridge_get_current_pose(robot_pose_state_t* pose) {
    if (!pose) {
        set_last_error("Null pose pointer");
        return ROBOT_BRIDGE_ERROR_PARAM;
    }
    
    if (!g_bridge_initialized_) {
        set_last_error("Bridge not initialized");
        return ROBOT_BRIDGE_ERROR_UNINIT;
    }
    
    // 获取互斥锁
    if (xSemaphoreTake(g_bridge_mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        set_last_error("Failed to acquire mutex for get_pose");
        return ROBOT_BRIDGE_ERROR_BUSY;
    }
    
    *pose = g_current_pose_;
    
    xSemaphoreGive(g_bridge_mutex_);
    return ROBOT_BRIDGE_OK;
}

extern "C" int robot_bridge_emergency_stop(void) {
    ESP_LOGW(TAG, "🚨 紧急停止触发");
    
    if (!g_bridge_initialized_) {
        return ROBOT_BRIDGE_ERROR_UNINIT;
    }
    
    // 尝试获取互斥锁，但不等待太久
    if (xSemaphoreTake(g_bridge_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
            // 停止当前运动（如果平滑控制器支持停止功能）
            if (g_smooth_controller_) {
                // 这里可以添加停止运动的逻辑
                ESP_LOGI(TAG, "🛑 平滑运动已停止");
            }
        xSemaphoreGive(g_bridge_mutex_);
    }
    
    return ROBOT_BRIDGE_OK;
}

extern "C" const char* robot_bridge_get_last_error(void) {
    return g_last_error_;
}

// ========== 扩展接口实现 ==========

extern "C" int robot_bridge_set_joint_angle(int leg_id, int joint_id, float angle_deg) {
    if (!g_bridge_initialized_) {
        set_last_error("Bridge not initialized");
        return ROBOT_BRIDGE_ERROR_UNINIT;
    }
    
    if (leg_id < 0 || leg_id > 3 || joint_id < 0 || joint_id > 2) {
        set_last_error("Invalid leg_id or joint_id");
        return ROBOT_BRIDGE_ERROR_PARAM;
    }
    
    if (!g_robot_controller_) {
        set_last_error("Robot controller not available");
        return ROBOT_BRIDGE_ERROR_UNINIT;
    }
    
    auto joint_controller = g_robot_controller_->getJointController();
    if (!joint_controller) {
        set_last_error("Joint controller not available");
        return ROBOT_BRIDGE_ERROR_UNINIT;
    }
    
    JointError result = joint_controller->setJointAngle(
        static_cast<LegID>(leg_id),
        static_cast<JointType>(joint_id),
        angle_deg);
    
    if (result == JointError::SUCCESS) {
        ESP_LOGI(TAG, "✅ 关节控制: 腿%d关节%d设置为%.2f°", leg_id, joint_id, angle_deg);
        return ROBOT_BRIDGE_OK;
    } else {
        set_last_error("Joint angle setting failed");
        return ROBOT_BRIDGE_ERROR_IO;
    }
}

extern "C" int robot_bridge_set_servo_angle(int servo_id, float angle_deg) {
    if (!g_bridge_initialized_) {
        set_last_error("Bridge not initialized");
        return ROBOT_BRIDGE_ERROR_UNINIT;
    }
    
    if (servo_id < 0 || servo_id > 11) {
        set_last_error("Invalid servo_id");
        return ROBOT_BRIDGE_ERROR_PARAM;
    }
    
    if (angle_deg < 0.0f || angle_deg > 180.0f) {
        set_last_error("Servo angle out of range [0, 180]");
        return ROBOT_BRIDGE_ERROR_PARAM;
    }
    
    if (!g_robot_controller_) {
        set_last_error("Robot controller not available");
        return ROBOT_BRIDGE_ERROR_UNINIT;
    }
    
    auto servo_driver = g_robot_controller_->getServoDriver();
    if (!servo_driver) {
        set_last_error("Servo driver not available");
        return ROBOT_BRIDGE_ERROR_UNINIT;
    }
    
    ServoError result = servo_driver->setAngle(servo_id, angle_deg);
    
    if (result == ServoError::SUCCESS) {
        ESP_LOGI(TAG, "✅ 舵机控制: 舵机%d设置为%.2f°", servo_id, angle_deg);
        return ROBOT_BRIDGE_OK;
    } else {
        set_last_error("Servo angle setting failed");
        return ROBOT_BRIDGE_ERROR_IO;
    }
}

extern "C" int robot_bridge_set_pwm(int channel, uint16_t pwm_value) {
    if (!g_bridge_initialized_) {
        set_last_error("Bridge not initialized");
        return ROBOT_BRIDGE_ERROR_UNINIT;
    }
    
    if (channel < 0 || channel > 15) {
        set_last_error("Invalid PWM channel");
        return ROBOT_BRIDGE_ERROR_PARAM;
    }
    
    if (pwm_value < 150 || pwm_value > 4095) {
        set_last_error("PWM value out of range [150, 4095]");
        return ROBOT_BRIDGE_ERROR_PARAM;
    }
    
    if (!g_robot_controller_) {
        set_last_error("Robot controller not available");
        return ROBOT_BRIDGE_ERROR_UNINIT;
    }
    
    auto hw_driver = g_robot_controller_->getHardwareDriver();
    if (!hw_driver) {
        set_last_error("Hardware driver not available");
        return ROBOT_BRIDGE_ERROR_UNINIT;
    }
    
    auto result = hw_driver->setPWM(channel, pwm_value);
    
    if (result == Drivers::PCA9685Error::SUCCESS) {
        ESP_LOGI(TAG, "✅ PWM控制: 通道%d设置为%d", channel, pwm_value);
        return ROBOT_BRIDGE_OK;
    } else {
        set_last_error("PWM setting failed");
        return ROBOT_BRIDGE_ERROR_IO;
    }
}

extern "C" int robot_bridge_set_preset_pose(int pose_id) {
    if (!g_bridge_initialized_) {
        set_last_error("Bridge not initialized");
        return ROBOT_BRIDGE_ERROR_UNINIT;
    }
    
    if (!g_robot_controller_) {
        set_last_error("Robot controller not available");
        return ROBOT_BRIDGE_ERROR_UNINIT;
    }
    
    auto joint_controller = g_robot_controller_->getJointController();
    if (!joint_controller) {
        set_last_error("Joint controller not available");
        return ROBOT_BRIDGE_ERROR_UNINIT;
    }
    
    JointError result;
    
    switch (pose_id) {
        case 0: // 趴下姿势
            result = joint_controller->setSleepPose();
            if (result == JointError::SUCCESS) {
                ESP_LOGI(TAG, "✅ 执行趴下姿势");
                return ROBOT_BRIDGE_OK;
            }
            break;
            
        case 1: // 站立姿势
            result = joint_controller->setStandPose();
            if (result == JointError::SUCCESS) {
                ESP_LOGI(TAG, "✅ 执行站立姿势");
                return ROBOT_BRIDGE_OK;
            }
            break;
            
        default:
            set_last_error("Invalid pose_id");
            return ROBOT_BRIDGE_ERROR_PARAM;
    }
    
    set_last_error("Preset pose execution failed");
    return ROBOT_BRIDGE_ERROR_IO;
}

// ========== 步态控制接口实现 ==========

extern "C" int robot_bridge_start_walk_gait(float stride_length, float step_height, float frequency) {
    if (!g_bridge_initialized_) {
        set_last_error("Bridge not initialized");
        return ROBOT_BRIDGE_ERROR_UNINIT;
    }
    
    // 参数验证
    if (stride_length < 0.02f || stride_length > 0.08f) {
        set_last_error("stride_length out of range [0.02, 0.08]");
        return ROBOT_BRIDGE_ERROR_PARAM;
    }
    
    if (step_height < 0.01f || step_height > 0.05f) {
        set_last_error("step_height out of range [0.01, 0.05]");
        return ROBOT_BRIDGE_ERROR_PARAM;
    }
    
    if (frequency < 0.3f || frequency > 1.5f) {
        set_last_error("frequency out of range [0.3, 1.5]");
        return ROBOT_BRIDGE_ERROR_PARAM;
    }
    
    // 创建步态控制器
    if (!g_walk_gait_) {
        g_walk_gait_ = std::make_unique<Gait::WalkGait>(stride_length, step_height, frequency);
    } else {
        g_walk_gait_->set_parameters(stride_length, step_height, frequency);
    }
    
    g_gait_running_ = true;
    
    ESP_LOGI(TAG, "✅ 启动 Walk 步态: 步长=%.1fmm, 步高=%.1fmm, 步频=%.2fHz",
             stride_length * 1000, step_height * 1000, frequency);
    
    return ROBOT_BRIDGE_OK;
}

extern "C" int robot_bridge_stop_gait(void) {
    if (!g_bridge_initialized_) {
        set_last_error("Bridge not initialized");
        return ROBOT_BRIDGE_ERROR_UNINIT;
    }
    
    g_gait_running_ = false;
    
    // 回到站立姿势
    if (g_robot_controller_) {
        auto joint_controller = g_robot_controller_->getJointController();
        if (joint_controller) {
            joint_controller->setStandPose();
        }
    }
    
    ESP_LOGI(TAG, "✅ 停止步态，回到站立姿势");
    
    return ROBOT_BRIDGE_OK;
}

extern "C" void robot_bridge_update_gait(float dt) {
    if (!g_bridge_initialized_ || !g_gait_running_ || !g_walk_gait_) {
        return;
    }
    
    // 更新步态相位
    g_walk_gait_->update(dt);
    
    // 获取所有腿的轨迹
    auto trajectories = g_walk_gait_->get_all_foot_trajectories();
    
    // 应用到腿部控制（通过 IK 转换为关节角度）
    if (g_robot_controller_ && g_quadruped_model_) {
        auto joint_controller = g_robot_controller_->getJointController();
        if (!joint_controller) {
            return;
        }
        
        using namespace Robot::Gait;
        
        // 遍历四条腿
        for (const auto& pair : trajectories) {
            LegIndex leg_index = pair.first;
            const TrajectoryPoint& trajectory = pair.second;
            
            // 获取腿部基准位置（站立姿势）
            // TODO: 需要从 QuadrupedModel 获取每条腿的基准位置
            // 这里暂时使用固定值
            float base_x = 0.0f;
            float base_y = 0.0f;
            float base_z = -0.15f;  // 站立高度 150mm
            
            // 计算目标位置（基准 + 轨迹偏移）
            float target_x = base_x + trajectory.position.x;
            float target_y = base_y + trajectory.position.y;
            float target_z = base_z + trajectory.position.z;
            
            // 使用 IK 计算关节角度
            // TODO: 调用 LegKinematics::inverse_kinematics()
            // 这需要将腿部索引映射到具体的腿对象
            
            // 伪代码：
            // int leg_id = static_cast<int>(leg_index);
            // JointAngles angles = g_quadruped_model_->getLeg(leg_id).inverse_kinematics(target_x, target_y, target_z);
            // joint_controller->setJointAngles(leg_id, angles);
        }
    }
    
    // 调试输出（每1秒输出一次）
    static float debug_timer = 0.0f;
    debug_timer += dt;
    if (debug_timer >= 1.0f) {
        debug_timer = 0.0f;
        
        ESP_LOGI(TAG, "步态运行: 相位=%.2f, 支撑腿=%d",
                 g_walk_gait_->get_global_phase(),
                 g_walk_gait_->count_support_legs());
    }
}

extern "C" int robot_bridge_get_gait_state(robot_gait_state_t* state) {
    if (!state) {
        set_last_error("Null state pointer");
        return ROBOT_BRIDGE_ERROR_PARAM;
    }
    
    if (!g_bridge_initialized_) {
        set_last_error("Bridge not initialized");
        return ROBOT_BRIDGE_ERROR_UNINIT;
    }
    
    if (!g_walk_gait_) {
        state->is_running = false;
        state->global_phase = 0.0f;
        state->stride_length = 0.0f;
        state->step_height = 0.0f;
        state->frequency = 0.0f;
        state->support_legs = 0;
        return ROBOT_BRIDGE_OK;
    }
    
    auto gait_state = g_walk_gait_->get_state();
    
    state->is_running = g_gait_running_;
    state->global_phase = g_walk_gait_->get_global_phase();
    state->stride_length = gait_state.stride_length;
    state->step_height = gait_state.step_height;
    state->frequency = gait_state.frequency;
    state->support_legs = g_walk_gait_->count_support_legs();
    
    return ROBOT_BRIDGE_OK;
}