#pragma once

/**
 * @file SmoothMotionConfig.hpp
 * @brief 平滑运动系统配置参数
 *
 * 基于新增功能.txt中的MOTION_STEP_ANGLE、MOTION_STEP_MOVEMENT、SERVO_STEP_ANGLE等参数
 * 提供集中化的配置管理
 */

#include <cstdint>
#include <algorithm>

namespace Robot {
namespace Controllers {

/**
 * @brief 插值配置参数
 */
struct InterpolationConfig {
    float position_step;    // 位置步长 (m)
    float angle_step;       // 角度步长 (弧度)
    uint32_t max_steps;     // 最大插值步数
    uint32_t step_delay_ms; // 每步延时 (ms)

    InterpolationConfig()
        : position_step(0.002f), angle_step(0.05f), max_steps(1000), step_delay_ms(0) {}
};

/**
 * @brief 舵机平滑配置参数
 */
struct ServoSmoothConfig {
    float angle_step;       // 角度步长 (度)
    uint32_t step_delay_ms; // 每步延时 (ms)
    uint32_t max_steps;     // 最大步数
    bool enable_smoothing;  // 是否启用平滑

    ServoSmoothConfig()
        : angle_step(2.0f), step_delay_ms(0), max_steps(500), enable_smoothing(true) {}
};

/**
 * @brief 平滑运动配置
 */
struct SmoothMotionConfig {
    // 姿态插值配置
    InterpolationConfig pose_interpolation;

    // 舵机平滑配置
    ServoSmoothConfig servo_smoothing;

    // 运动控制配置
    bool enable_pose_smoothing;     // 启用姿态平滑
    bool enable_servo_smoothing;    // 启用舵机平滑
    uint32_t motion_update_rate_ms; // 运动更新频率 (ms)

    SmoothMotionConfig()
        : enable_pose_smoothing(true)
        , enable_servo_smoothing(true)
        , motion_update_rate_ms(20) {}
};

} // namespace Controllers
}

namespace Robot {
namespace Config {

/**
 * @brief 平滑运动配置常量
 *
 * 基于新增功能.txt中的示例参数设计
 */
namespace SmoothMotionConstants {
    // 姿态插值默认参数 (基于iterate_to_position函数)
    static constexpr float DEFAULT_MOTION_STEP_ANGLE = 0.05f;      // 角度步长 (弧度) ≈ 2.86度
    static constexpr float DEFAULT_MOTION_STEP_MOVEMENT = 0.002f;  // 位置步长 (m) = 2mm

    // 舵机平滑默认参数 (基于set_leg_servos_in_steps函数)
    static constexpr float DEFAULT_SERVO_STEP_ANGLE = 2.0f;        // 舵机角度步长 (度)

    // 性能和安全限制
    static constexpr uint32_t DEFAULT_MAX_INTERPOLATION_STEPS = 1000;
    static constexpr uint32_t DEFAULT_MAX_SERVO_STEPS = 500;
    static constexpr uint32_t DEFAULT_STEP_DELAY_MS = 20;
    static constexpr uint32_t DEFAULT_MOTION_UPDATE_RATE_MS = 20;

    // 容差参数
    static constexpr float POSITION_TOLERANCE_M = 0.001f;    // 位置容差 1mm
    static constexpr float ANGLE_TOLERANCE_RAD = 0.01f;      // ≈ 0.57度
    static constexpr float SERVO_ANGLE_TOLERANCE_DEG = 1.0f;
}

/**
 * @brief 姿态插值配置预设
 */
struct PoseInterpolationPresets {
    // 快速模式 - 适用于实时控制
    static constexpr float FAST_POSITION_STEP = 0.005f;     // 5mm
    static constexpr float FAST_ANGLE_STEP = 0.1f;          // 弧度
    static constexpr uint32_t FAST_MAX_STEPS = 200;
    static constexpr uint32_t FAST_DELAY_MS = 10;

    // 标准模式 - 平衡速度和平滑度
    static constexpr float STANDARD_POSITION_STEP = 0.002f; // 2mm
    static constexpr float STANDARD_ANGLE_STEP = 0.05f;     // 弧度
    static constexpr uint32_t STANDARD_MAX_STEPS = 500;
    static constexpr uint32_t STANDARD_DELAY_MS = 20;

    // 平滑模式 - 最大平滑度
    static constexpr float SMOOTH_POSITION_STEP = 0.001f;   // 1mm
    static constexpr float SMOOTH_ANGLE_STEP = 0.02f;       // 弧度
    static constexpr uint32_t SMOOTH_MAX_STEPS = 1000;
    static constexpr uint32_t SMOOTH_DELAY_MS = 30;
};

/**
 * @brief 舵机平滑配置预设
 */
struct ServoSmoothingPresets {
    // 快速模式
    static constexpr float FAST_SERVO_STEP = 5.0f;         // 度
    static constexpr uint32_t FAST_SERVO_MAX_STEPS = 100;
    static constexpr uint32_t FAST_SERVO_DELAY_MS = 5;

    // 标准模式
    static constexpr float STANDARD_SERVO_STEP = 2.0f;     // 度
    static constexpr uint32_t STANDARD_SERVO_MAX_STEPS = 300;
    static constexpr uint32_t STANDARD_SERVO_DELAY_MS = 10;

    // 平滑模式
    static constexpr float SMOOTH_SERVO_STEP = 1.0f;       // 度
    static constexpr uint32_t SMOOTH_SERVO_MAX_STEPS = 500;
    static constexpr uint32_t SMOOTH_SERVO_DELAY_MS = 15;
};

/**
 * @brief 预定义配置模式
 */
enum class SmoothMotionMode {
    FAST,       // 快速模式 - 优先响应速度
    STANDARD,   // 标准模式 - 平衡性能
    SMOOTH,     // 平滑模式 - 优先平滑度
    CUSTOM      // 自定义模式
};

/**
 * @brief 配置工厂类
 */
class SmoothMotionConfigFactory {
public:
    /**
     * @brief 创建姿态插值配置
     */
    static Robot::Controllers::InterpolationConfig createPoseInterpolationConfig(SmoothMotionMode mode) {
        Robot::Controllers::InterpolationConfig config;

        switch (mode) {
            case SmoothMotionMode::FAST:
                config.position_step = PoseInterpolationPresets::FAST_POSITION_STEP;
                config.angle_step = PoseInterpolationPresets::FAST_ANGLE_STEP;
                config.max_steps = PoseInterpolationPresets::FAST_MAX_STEPS;
                config.step_delay_ms = PoseInterpolationPresets::FAST_DELAY_MS;
                break;

            case SmoothMotionMode::STANDARD:
                config.position_step = PoseInterpolationPresets::STANDARD_POSITION_STEP;
                config.angle_step = PoseInterpolationPresets::STANDARD_ANGLE_STEP;
                config.max_steps = PoseInterpolationPresets::STANDARD_MAX_STEPS;
                config.step_delay_ms = PoseInterpolationPresets::STANDARD_DELAY_MS;
                break;

            case SmoothMotionMode::SMOOTH:
                config.position_step = PoseInterpolationPresets::SMOOTH_POSITION_STEP;
                config.angle_step = PoseInterpolationPresets::SMOOTH_ANGLE_STEP;
                config.max_steps = PoseInterpolationPresets::SMOOTH_MAX_STEPS;
                config.step_delay_ms = PoseInterpolationPresets::SMOOTH_DELAY_MS;
                break;

            default: // CUSTOM - 使用默认值
                config.position_step = SmoothMotionConstants::DEFAULT_MOTION_STEP_MOVEMENT;
                config.angle_step = SmoothMotionConstants::DEFAULT_MOTION_STEP_ANGLE;
                config.max_steps = SmoothMotionConstants::DEFAULT_MAX_INTERPOLATION_STEPS;
                config.step_delay_ms = SmoothMotionConstants::DEFAULT_STEP_DELAY_MS;
                break;
        }

        return config;
    }

    /**
     * @brief 创建舵机平滑配置
     */
    static Robot::Controllers::ServoSmoothConfig createServoSmoothConfig(SmoothMotionMode mode) {
        Robot::Controllers::ServoSmoothConfig config;

        switch (mode) {
            case SmoothMotionMode::FAST:
                config.angle_step = ServoSmoothingPresets::FAST_SERVO_STEP;
                config.max_steps = ServoSmoothingPresets::FAST_SERVO_MAX_STEPS;
                config.step_delay_ms = ServoSmoothingPresets::FAST_SERVO_DELAY_MS;
                break;

            case SmoothMotionMode::STANDARD:
                config.angle_step = ServoSmoothingPresets::STANDARD_SERVO_STEP;
                config.max_steps = ServoSmoothingPresets::STANDARD_SERVO_MAX_STEPS;
                config.step_delay_ms = ServoSmoothingPresets::STANDARD_SERVO_DELAY_MS;
                break;

            case SmoothMotionMode::SMOOTH:
                config.angle_step = ServoSmoothingPresets::SMOOTH_SERVO_STEP;
                config.max_steps = ServoSmoothingPresets::SMOOTH_SERVO_MAX_STEPS;
                config.step_delay_ms = ServoSmoothingPresets::SMOOTH_SERVO_DELAY_MS;
                break;

            default: // CUSTOM
                config.angle_step = SmoothMotionConstants::DEFAULT_SERVO_STEP_ANGLE;
                config.max_steps = SmoothMotionConstants::DEFAULT_MAX_SERVO_STEPS;
                config.step_delay_ms = SmoothMotionConstants::DEFAULT_STEP_DELAY_MS;
                break;
        }

        config.enable_smoothing = true;
        return config;
    }

    /**
     * @brief 创建完整的平滑运动配置
     */
    static Robot::Controllers::SmoothMotionConfig createSmoothMotionConfig(SmoothMotionMode mode) {
        Robot::Controllers::SmoothMotionConfig config;

        config.pose_interpolation = createPoseInterpolationConfig(mode);
        config.servo_smoothing = createServoSmoothConfig(mode);

        // 通用设置
        config.enable_pose_smoothing = true;
        config.enable_servo_smoothing = true;

        switch (mode) {
            case SmoothMotionMode::FAST:
                config.motion_update_rate_ms = 10;
                break;
            case SmoothMotionMode::STANDARD:
                config.motion_update_rate_ms = 20;
                break;
            case SmoothMotionMode::SMOOTH:
                config.motion_update_rate_ms = 30;
                break;
            default:
                config.motion_update_rate_ms = SmoothMotionConstants::DEFAULT_MOTION_UPDATE_RATE_MS;
                break;
        }

        return config;
    }

    /**
     * @brief 创建调试模式配置 (禁用平滑功能)
     */
    static Robot::Controllers::SmoothMotionConfig createDebugConfig() {
        Robot::Controllers::SmoothMotionConfig config;

        // 禁用所有平滑功能用于调试
        config.enable_pose_smoothing = false;
        config.enable_servo_smoothing = false;
        config.motion_update_rate_ms = 50;

        return config;
    }

    /**
     * @brief 创建高性能模式配置
     */
    static Robot::Controllers::SmoothMotionConfig createPerformanceConfig() {
        Robot::Controllers::SmoothMotionConfig config;

        // 使用最小的平滑处理以提高响应速度
        config.enable_pose_smoothing = true;
        config.enable_servo_smoothing = false; // 禁用舵机平滑以提高响应速度

        config.pose_interpolation.position_step = 0.003f;  // 3mm
        config.pose_interpolation.angle_step = 0.08f;
        config.pose_interpolation.max_steps = 200;
        config.pose_interpolation.step_delay_ms = 5;

        config.motion_update_rate_ms = 10;

        return config;
    }
};

/**
 * @brief 配置验证器
 */
class SmoothMotionConfigValidator {
public:
    /**
     * @brief 验证配置参数的有效性
     */
    static bool validateConfig(const Robot::Controllers::SmoothMotionConfig& config, const char** error_message = nullptr) {
        // 验证姿态插值参数
        if (config.pose_interpolation.position_step <= 0 || config.pose_interpolation.position_step > 0.050f) {
            if (error_message) *error_message = "Invalid position step (must be 0 < step <= 0.05m = 50mm)";
            return false;
        }

        if (config.pose_interpolation.angle_step <= 0 || config.pose_interpolation.angle_step > 0.5f) {
            if (error_message) *error_message = "Invalid angle step (must be 0 < step <= 0.5 rad)";
            return false;
        }

        // 验证舵机平滑参数
        if (config.servo_smoothing.angle_step <= 0 || config.servo_smoothing.angle_step > 45.0f) {
            if (error_message) *error_message = "Invalid servo step (must be 0 < step <= 45°)";
            return false;
        }

        // 验证时间参数
        if (config.motion_update_rate_ms < 5 || config.motion_update_rate_ms > 1000) {
            if (error_message) *error_message = "Invalid update rate (must be 5-1000 ms)";
            return false;
        }

        if (error_message) *error_message = "Configuration valid";
        return true;
    }

    /**
     * @brief 自动修正配置参数
     */
    static Robot::Controllers::SmoothMotionConfig autoCorrectConfig(const Robot::Controllers::SmoothMotionConfig& config) {
        Robot::Controllers::SmoothMotionConfig corrected = config;

        // 修正姿态插值参数
        corrected.pose_interpolation.position_step =
            std::max(0.0001f, std::min(0.050f, config.pose_interpolation.position_step));
        corrected.pose_interpolation.angle_step =
            std::max(0.005f, std::min(0.5f, config.pose_interpolation.angle_step));

        // 修正舵机平滑参数
        corrected.servo_smoothing.angle_step =
            std::max(0.1f, std::min(45.0f, config.servo_smoothing.angle_step));

        // 修正时间参数
        uint32_t temp_rate = config.motion_update_rate_ms;
        if (temp_rate < 5) temp_rate = 5;
        if (temp_rate > 1000) temp_rate = 1000;
        corrected.motion_update_rate_ms = temp_rate;

        return corrected;
    }
};

} // namespace Config
} // namespace Robot