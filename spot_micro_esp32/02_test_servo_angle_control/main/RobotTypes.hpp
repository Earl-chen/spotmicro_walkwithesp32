#pragma once

#include <array>
#include <cstdint>

/**
 * @brief 机器人通用类型定义
 *
 * 坐标系定义：
 * - X轴：机器人向前为正方向
 * - Y轴：机器人向左为正方向
 * - Z轴：机器人向上为正方向
 */

namespace Robot {

/**
 * @brief 关节类型枚举
 */
enum class JointType : uint8_t {
    HIP_ROLL = 0,    // 髋关节侧摆，绕X轴旋转，范围: -90°~+90°
    HIP_PITCH = 1,   // 髋关节俯仰，绕Y轴旋转，范围: -90°~+90°
    KNEE_PITCH = 2   // 膝关节俯仰，绕Y轴旋转，范围: -180°~0°
};

/**
 * @brief 腿部ID枚举
 */
enum class LegID : uint8_t {
    FRONT_LEFT = 0,   // 左前腿
    FRONT_RIGHT = 1,  // 右前腿
    BACK_LEFT = 2,    // 左后腿
    BACK_RIGHT = 3    // 右后腿
};

/**
 * @brief 关节角度结构体
 */
struct JointAngle {
    float angle;      // 关节角度（度）
    bool isValid;     // 角度是否有效

    JointAngle() : angle(0.0f), isValid(false) {}
    JointAngle(float a) : angle(a), isValid(true) {}
    JointAngle(float a, bool valid) : angle(a), isValid(valid) {}
};

/**
 * @brief 舵机角度结构体
 */
struct ServoAngle {
    float angle;           // 舵机角度（度）
    uint16_t pwm_value;    // 对应的PWM值
    bool isValid;          // 角度是否有效

    ServoAngle() : angle(0.0f), pwm_value(0), isValid(false) {}
    ServoAngle(float a, uint16_t pwm) : angle(a), pwm_value(pwm), isValid(true) {}
    ServoAngle(float a, uint16_t pwm, bool valid) : angle(a), pwm_value(pwm), isValid(valid) {}
};

/**
 * @brief 三关节角度结构体 (用于运动学计算)
 */
struct ThreeJointAngles {
    float hip_side;   // 髋侧摆角度 (弧度)
    float hip_pitch;  // 髋俯仰角度 (弧度)
    float knee_pitch; // 膝俯仰角度 (弧度)

    ThreeJointAngles() : hip_side(0.0f), hip_pitch(0.0f), knee_pitch(0.0f) {}
    ThreeJointAngles(float h_side, float h_pitch, float k_pitch)
        : hip_side(h_side), hip_pitch(h_pitch), knee_pitch(k_pitch) {}
};

/**
 * @brief 机器人常量定义
 */
namespace Constants {
    static constexpr int SERVO_COUNT = 12;         // 舵机总数
    static constexpr int LEG_COUNT = 4;            // 腿数
    static constexpr int JOINTS_PER_LEG = 3;       // 每条腿的关节数

    // 关节角度范围（度）
    static constexpr float HIP_ROLL_MIN = -90.0f;
    static constexpr float HIP_ROLL_MAX = 90.0f;
    static constexpr float HIP_PITCH_MIN = -90.0f;
    static constexpr float HIP_PITCH_MAX = 90.0f;
    static constexpr float KNEE_PITCH_MIN = -180.0f;
    static constexpr float KNEE_PITCH_MAX = 0.0f;

    // 舵机角度范围（度）
    static constexpr float SERVO_ANGLE_MIN = 0.0f;
    static constexpr float SERVO_ANGLE_MAX = 180.0f;
}

/**
 * @brief 类型别名定义
 */
using ServoArray = std::array<ServoAngle, Constants::SERVO_COUNT>;
using JointArray = std::array<JointAngle, Constants::SERVO_COUNT>;
using LegJoints = std::array<JointAngle, Constants::JOINTS_PER_LEG>;
using AllLegJoints = std::array<LegJoints, Constants::LEG_COUNT>;

} // namespace Robot