#pragma once

#include "PCA9685Driver.hpp"
#include "../config/ConfigManager.hpp"
#include "../utils/Logger.hpp"
#include <memory>

namespace Robot {
namespace Controllers {

/**
 * @brief 舵机控制错误码
 */
enum class ServoError {
    SUCCESS = 0,           // 成功
    INVALID_SERVO_ID = 1,  // 无效舵机ID
    INVALID_ANGLE = 2,     // 无效角度
    HARDWARE_ERROR = 3,    // 硬件错误
    NOT_INITIALIZED = 4,   // 未初始化
    CONFIG_ERROR = 5       // 配置错误
};

/**
 * @brief 舵机驱动控制器
 *
 * 纯净的舵机角度控制层，不包含业务逻辑
 * 负责角度到PWM的转换和舵机硬件控制
 */
class ServoDriver {
public:
    static constexpr int MAX_SERVOS = 12;  // 实际使用的舵机数量
    static constexpr float MIN_ANGLE = 0.0f;
    static constexpr float MAX_ANGLE = 180.0f;

    /**
     * @brief 构造函数
     * @param hw_driver PCA9685硬件驱动器 (依赖注入)
     * @param config_manager 配置管理器 (依赖注入)
     */
    ServoDriver(std::shared_ptr<Drivers::PCA9685Driver> hw_driver,
                std::shared_ptr<Config::ConfigManager> config_manager);

    /**
     * @brief 析构函数
     */
    ~ServoDriver();

    /**
     * @brief 初始化舵机驱动
     * @return 错误码
     */
    ServoError init();

    /**
     * @brief 反初始化
     */
    void deinit();

    /**
     * @brief 设置单个舵机角度
     * @param servo_id 舵机ID (0-11)
     * @param angle 角度 (0-180度)
     * @return 错误码
     */
    ServoError setAngle(int servo_id, float angle);

    /**
     * @brief 获取单个舵机当前角度
     * @param servo_id 舵机ID (0-11)
     * @param angle 输出角度
     * @return 错误码
     */
    ServoError getAngle(int servo_id, float& angle) const;

    /**
     * @brief 批量设置舵机角度
     * @param angles 角度数组 (最多12个)
     * @param count 舵机数量
     * @return 错误码
     */
    ServoError setAngles(const float* angles, int count);

    /**
     * @brief 批量设置舵机角度 (std::array版本)
     * @param angles 角度数组
     * @return 错误码
     */
    ServoError setAngles(const std::array<float, MAX_SERVOS>& angles);

    /**
     * @brief 获取所有舵机角度
     * @param angles 输出角度数组
     * @return 错误码
     */
    ServoError getAngles(std::array<float, MAX_SERVOS>& angles) const;

    /**
     * @brief 角度转PWM值
     * @param servo_id 舵机ID
     * @param angle 角度
     * @param pwm_value 输出PWM值
     * @return 错误码
     */
    ServoError angleToPWM(int servo_id, float angle, uint16_t& pwm_value) const;

    /**
     * @brief PWM值转角度
     * @param servo_id 舵机ID
     * @param pwm_value PWM值
     * @param angle 输出角度
     * @return 错误码
     */
    ServoError pwmToAngle(int servo_id, uint16_t pwm_value, float& angle) const;

    /**
     * @brief 获取舵机配置
     * @param servo_id 舵机ID
     * @return 舵机配置
     */
    const Config::ServoConfig& getServoConfig(int servo_id) const;

    /**
     * @brief 更新舵机配置
     * @param servo_id 舵机ID
     * @param config 新配置
     * @return 错误码
     */
    ServoError updateServoConfig(int servo_id, const Config::ServoConfig& config);

    /**
     * @brief 验证舵机ID是否有效
     */
    bool isValidServoId(int servo_id) const {
        return servo_id >= 0 && servo_id < MAX_SERVOS;
    }

    /**
     * @brief 验证角度是否有效
     */
    bool isValidAngle(float angle) const {
        return angle >= MIN_ANGLE && angle <= MAX_ANGLE;
    }

    /**
     * @brief 验证角度是否在舵机范围内
     */
    bool isAngleInRange(int servo_id, float angle) const;

    /**
     * @brief 获取舵机角度范围
     * @param servo_id 舵机ID
     * @param min_angle 输出最小角度
     * @param max_angle 输出最大角度
     * @return 错误码
     */
    ServoError getAngleRange(int servo_id, float& min_angle, float& max_angle) const;

    /**
     * @brief 设置舵机启用状态
     * @param servo_id 舵机ID
     * @param enabled 是否启用
     * @return 错误码
     */
    ServoError setServoEnabled(int servo_id, bool enabled);

    /**
     * @brief 获取舵机启用状态
     * @param servo_id 舵机ID
     * @return 是否启用
     */
    bool isServoEnabled(int servo_id) const;

    /**
     * @brief 舵机校准 - 记录角度和PWM对应关系
     * @param servo_id 舵机ID
     * @param angle 角度
     * @param pwm_value 对应的PWM值
     * @return 错误码
     */
    ServoError calibrateServo(int servo_id, float angle, uint16_t pwm_value);

    /**
     * @brief 获取错误描述
     */
    static const char* getErrorString(ServoError error);

    /**
     * @brief 获取初始化状态
     */
    bool isInitialized() const { return initialized_; }

private:
    std::shared_ptr<Drivers::PCA9685Driver> hw_driver_;      // 硬件驱动器
    std::shared_ptr<Config::ConfigManager> config_manager_;  // 配置管理器
    std::array<float, MAX_SERVOS> current_angles_;           // 当前角度缓存
    bool initialized_;                                       // 初始化状态

    /**
     * @brief 验证并限制角度范围
     */
    float clampAngle(int servo_id, float angle) const;

    /**
     * @brief 线性插值计算PWM值
     */
    uint16_t interpolatePWM(const Config::ServoConfig& config, float angle) const;

    /**
     * @brief 反向线性插值计算角度
     */
    float interpolateAngle(const Config::ServoConfig& config, uint16_t pwm_value) const;
};

} // namespace Controllers
} // namespace Robot