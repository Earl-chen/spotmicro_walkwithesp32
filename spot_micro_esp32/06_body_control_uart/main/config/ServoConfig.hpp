#pragma once

#include <cstdint>
#include <array>

namespace Robot {
namespace Config {

/**
 * @brief 单个舵机配置结构
 */
struct ServoConfig {
    uint16_t min_pwm;           // 最小PWM值 (0度对应)
    uint16_t max_pwm;           // 最大PWM值 (180度对应)
    float pwm_ratio;            // 预计算的比率 (max_pwm - min_pwm) / MAX_ANGLE
    bool enabled;               // 舵机是否启用

    // 默认构造函数
    ServoConfig()
        : min_pwm(150), max_pwm(600), pwm_ratio(0.0f), enabled(true) {
        calculateRatio();
    }

    // 参数构造函数
    ServoConfig(uint16_t min, uint16_t max, bool en = true)
        : min_pwm(min), max_pwm(max), pwm_ratio(0.0f), enabled(en) {
        calculateRatio();
    }

    // 验证配置有效性
    bool isValid() const {
        return min_pwm < max_pwm && max_pwm <= 4095;
    }

    // 获取PWM范围
    uint16_t getPWMRange() const { return max_pwm - min_pwm; }

    // 计算PWM比率
    void calculateRatio() {
        static constexpr float MAX_ANGLE = 180.0f;
        if (max_pwm > min_pwm) {
            pwm_ratio = static_cast<float>(max_pwm - min_pwm) / MAX_ANGLE;
        } else {
            pwm_ratio = 0.0f;
        }
    }

    // 设置PWM范围并自动计算比率
    void setPWMRange(uint16_t min, uint16_t max) {
        min_pwm = min;
        max_pwm = max;
        calculateRatio();
    }
};

/**
 * @brief 所有舵机配置
 */
struct AllServoConfig {
    static constexpr int SERVO_COUNT = 12;
    std::array<ServoConfig, SERVO_COUNT> servos;

    // 获取指定舵机配置
    const ServoConfig& getServo(int servo_id) const {
        return servos[servo_id];
    }

    // 设置指定舵机配置
    void setServo(int servo_id, const ServoConfig& config) {
        if (servo_id >= 0 && servo_id < SERVO_COUNT) {
            servos[servo_id] = config;
        }
    }

    // 验证所有配置
    bool isValid() const {
        for (const auto& servo : servos) {
            if (!servo.isValid()) return false;
        }
        return true;
    }
};

} // namespace Config
} // namespace Robot