#include "ServoDriver.hpp"

namespace Robot {
namespace Controllers {

ServoDriver::ServoDriver(std::shared_ptr<Drivers::PCA9685Driver> hw_driver,
                         std::shared_ptr<Config::ConfigManager> config_manager)
    : hw_driver_(hw_driver)
    , config_manager_(config_manager)
    , current_angles_{}
    , initialized_(false)
{
    LOG_TAG_DEBUG("SERVO", "ServoDriver created");
}

ServoDriver::~ServoDriver() {
    if (initialized_) {
        deinit();
    }
}

ServoError ServoDriver::init() {
    LOG_TAG_INFO("SERVO", "Initializing ServoDriver");

    if (!hw_driver_) {
        LOG_TAG_ERROR("SERVO", "Hardware driver not provided");
        return ServoError::HARDWARE_ERROR;
    }

    if (!config_manager_) {
        LOG_TAG_ERROR("SERVO", "Config manager not provided");
        return ServoError::CONFIG_ERROR;
    }

    // 验证配置有效性
    if (!config_manager_->validateConfig()) {
        LOG_TAG_ERROR("SERVO", "Invalid servo configuration");
        return ServoError::CONFIG_ERROR;
    }

    // 初始化角度缓存为中位值
    current_angles_.fill(90.0f);

    initialized_ = true;
    LOG_TAG_INFO("SERVO", "ServoDriver initialized successfully");
    return ServoError::SUCCESS;
}

void ServoDriver::deinit() {
    if (initialized_) {
        LOG_TAG_INFO("SERVO", "Deinitializing ServoDriver");

        // 将所有舵机设置到安全位置 (90度中位)
        for (int i = 0; i < MAX_SERVOS; i++) {
            setAngle(i, 90.0f);
        }

        initialized_ = false;
        LOG_TAG_DEBUG("SERVO", "ServoDriver deinitialized");
    }
}

ServoError ServoDriver::setAngle(int servo_id, float angle) {
    // 验证参数
    if (!isValidServoId(servo_id)) {
        LOG_TAG_ERROR("SERVO", "Invalid servo ID: %d", servo_id);
        return ServoError::INVALID_SERVO_ID;
    }

    if (!isValidAngle(angle)) {
        LOG_TAG_ERROR("SERVO", "Invalid angle: %.2f for servo %d", angle, servo_id);
        return ServoError::INVALID_ANGLE;
    }

    if (!initialized_) {
        LOG_TAG_ERROR("SERVO", "ServoDriver not initialized");
        return ServoError::NOT_INITIALIZED;
    }

    // 检查舵机是否启用
    if (!isServoEnabled(servo_id)) {
        LOG_TAG_WARN("SERVO", "Servo %d is disabled, ignoring command", servo_id);
        return ServoError::SUCCESS;  // 不报错，但不执行
    }

    // 限制角度范围
    float clamped_angle = clampAngle(servo_id, angle);

    // 直接转换为PWM值
    uint16_t pwm_value;
    ServoError result = angleToPWM(servo_id, clamped_angle, pwm_value);
    if (result != ServoError::SUCCESS) {
        LOG_TAG_ERROR("SERVO", "Failed to convert angle to PWM for servo %d", servo_id);
        return result;
    }

    // 发送到硬件
    Drivers::PCA9685Error hw_result = hw_driver_->setPWM(servo_id, pwm_value);
    if (hw_result != Drivers::PCA9685Error::SUCCESS) {
        LOG_TAG_ERROR("SERVO", "Hardware error setting PWM for servo %d: %s",
                      servo_id, Drivers::PCA9685Driver::getErrorString(hw_result));
        return ServoError::HARDWARE_ERROR;
    }

    // 更新缓存
    current_angles_[servo_id] = angle;

    LOG_TAG_DEBUG("SERVO", "Servo %d angle set to %.2f° (PWM: %d)", servo_id, angle, pwm_value);
    return ServoError::SUCCESS;
}

ServoError ServoDriver::getAngle(int servo_id, float& angle) const {
    if (!isValidServoId(servo_id)) {
        LOG_TAG_ERROR("SERVO", "Invalid servo ID: %d", servo_id);
        return ServoError::INVALID_SERVO_ID;
    }

    if (!initialized_) {
        LOG_TAG_ERROR("SERVO", "ServoDriver not initialized");
        return ServoError::NOT_INITIALIZED;
    }

    angle = current_angles_[servo_id];
    return ServoError::SUCCESS;
}

ServoError ServoDriver::setAngles(const float* angles, int count) {
    if (!angles || count <= 0 || count > MAX_SERVOS) {
        LOG_TAG_ERROR("SERVO", "Invalid parameters for setAngles");
        return ServoError::INVALID_ANGLE;
    }

    LOG_TAG_DEBUG("SERVO", "Setting angles for %d servos", count);

    for (int i = 0; i < count; i++) {
        ServoError result = setAngle(i, angles[i]);
        if (result != ServoError::SUCCESS) {
            LOG_TAG_ERROR("SERVO", "Failed to set angle for servo %d in batch", i);
            return result;
        }
    }

    LOG_TAG_DEBUG("SERVO", "Successfully set angles for all %d servos", count);
    return ServoError::SUCCESS;
}

ServoError ServoDriver::setAngles(const std::array<float, MAX_SERVOS>& angles) {
    return setAngles(angles.data(), MAX_SERVOS);
}

ServoError ServoDriver::getAngles(std::array<float, MAX_SERVOS>& angles) const {
    if (!initialized_) {
        LOG_TAG_ERROR("SERVO", "ServoDriver not initialized");
        return ServoError::NOT_INITIALIZED;
    }

    angles = current_angles_;
    return ServoError::SUCCESS;
}

ServoError ServoDriver::angleToPWM(int servo_id, float angle, uint16_t& pwm_value) const {
    if (!isValidServoId(servo_id)) {
        return ServoError::INVALID_SERVO_ID;
    }

    const auto& config = getServoConfig(servo_id);
    pwm_value = interpolatePWM(config, angle);

    LOG_TAG_DEBUG("SERVO", "Servo %d: %.2f° -> PWM %d", servo_id, angle, pwm_value);
    return ServoError::SUCCESS;
}

ServoError ServoDriver::pwmToAngle(int servo_id, uint16_t pwm_value, float& angle) const {
    if (!isValidServoId(servo_id)) {
        return ServoError::INVALID_SERVO_ID;
    }

    const auto& config = getServoConfig(servo_id);
    angle = interpolateAngle(config, pwm_value);

    LOG_TAG_DEBUG("SERVO", "Servo %d: PWM %d -> %.2f°", servo_id, pwm_value, angle);
    return ServoError::SUCCESS;
}

const Config::ServoConfig& ServoDriver::getServoConfig(int servo_id) const {
    return config_manager_->getServoConfig().getServo(servo_id);
}

ServoError ServoDriver::updateServoConfig(int servo_id, const Config::ServoConfig& config) {
    if (!isValidServoId(servo_id)) {
        return ServoError::INVALID_SERVO_ID;
    }

    if (!config.isValid()) {
        LOG_TAG_ERROR("SERVO", "Invalid servo config for servo %d", servo_id);
        return ServoError::CONFIG_ERROR;
    }

    auto all_servos = config_manager_->getServoConfig();
    all_servos.setServo(servo_id, config);

    if (!config_manager_->setServoConfig(all_servos)) {
        LOG_TAG_ERROR("SERVO", "Failed to update servo config for servo %d", servo_id);
        return ServoError::CONFIG_ERROR;
    }

    LOG_TAG_INFO("SERVO", "Updated config for servo %d", servo_id);
    return ServoError::SUCCESS;
}

bool ServoDriver::isAngleInRange(int servo_id, float angle) const {
    if (!isValidServoId(servo_id)) {
        return false;
    }

    float min_angle, max_angle;
    if (getAngleRange(servo_id, min_angle, max_angle) != ServoError::SUCCESS) {
        return false;
    }

    return angle >= min_angle && angle <= max_angle;
}

ServoError ServoDriver::getAngleRange(int servo_id, float& min_angle, float& max_angle) const {
    if (!isValidServoId(servo_id)) {
        return ServoError::INVALID_SERVO_ID;
    }

    // 基本范围是0-180度，但可以根据配置调整
    min_angle = MIN_ANGLE;
    max_angle = MAX_ANGLE;

    return ServoError::SUCCESS;
}

ServoError ServoDriver::setServoEnabled(int servo_id, bool enabled) {
    if (!isValidServoId(servo_id)) {
        return ServoError::INVALID_SERVO_ID;
    }

    auto config = getServoConfig(servo_id);
    config.enabled = enabled;

    return updateServoConfig(servo_id, config);
}

bool ServoDriver::isServoEnabled(int servo_id) const {
    if (!isValidServoId(servo_id)) {
        return false;
    }

    return getServoConfig(servo_id).enabled;
}

ServoError ServoDriver::calibrateServo(int servo_id, float angle, uint16_t pwm_value) {
    if (!isValidServoId(servo_id)) {
        return ServoError::INVALID_SERVO_ID;
    }

    if (!isValidAngle(angle)) {
        return ServoError::INVALID_ANGLE;
    }

    LOG_TAG_INFO("SERVO", "Calibrating servo %d: %.2f° = PWM %d", servo_id, angle, pwm_value);

    // 根据标定点更新配置
    // 这里可以实现更复杂的多点标定算法
    auto config = getServoConfig(servo_id);

    if (angle <= 1.0f) {  // 接近0度
        config.min_pwm = pwm_value;
    } else if (angle >= 179.0f) {  // 接近180度
        config.max_pwm = pwm_value;
    }

    return updateServoConfig(servo_id, config);
}

const char* ServoDriver::getErrorString(ServoError error) {
    switch (error) {
        case ServoError::SUCCESS:           return "Success";
        case ServoError::INVALID_SERVO_ID:  return "Invalid servo ID";
        case ServoError::INVALID_ANGLE:     return "Invalid angle";
        case ServoError::HARDWARE_ERROR:    return "Hardware error";
        case ServoError::NOT_INITIALIZED:   return "Not initialized";
        case ServoError::CONFIG_ERROR:      return "Configuration error";
        default:                            return "Unknown error";
    }
}

float ServoDriver::clampAngle(int servo_id, float angle) const {
    float min_angle, max_angle;
    if (getAngleRange(servo_id, min_angle, max_angle) != ServoError::SUCCESS) {
        return angle;
    }

    if (angle < min_angle) {
        LOG_TAG_WARN("SERVO", "Servo %d angle %.2f° clamped to %.2f°", servo_id, angle, min_angle);
        return min_angle;
    }

    if (angle > max_angle) {
        LOG_TAG_WARN("SERVO", "Servo %d angle %.2f° clamped to %.2f°", servo_id, angle, max_angle);
        return max_angle;
    }

    return angle;
}

uint16_t ServoDriver::interpolatePWM(const Config::ServoConfig& config, float angle) const {
    // 使用预计算的比率: PWM = min_pwm + angle * ratio
    uint16_t pwm = static_cast<uint16_t>(config.min_pwm + angle * config.pwm_ratio);

    // 确保在硬件范围内
    return std::min(std::max(pwm, static_cast<uint16_t>(0)), static_cast<uint16_t>(4095));
}

float ServoDriver::interpolateAngle(const Config::ServoConfig& config, uint16_t pwm_value) const {
    // 使用预计算的比率: angle = (PWM - min_pwm) / ratio
    if (config.pwm_ratio <= 0.0f) {
        return 90.0f;  // 避免除零，返回中位值
    }

    float angle = static_cast<float>(pwm_value - config.min_pwm) / config.pwm_ratio;

    // 限制在有效范围内
    return std::min(std::max(angle, MIN_ANGLE), MAX_ANGLE);
}

} // namespace Controllers
} // namespace Robot