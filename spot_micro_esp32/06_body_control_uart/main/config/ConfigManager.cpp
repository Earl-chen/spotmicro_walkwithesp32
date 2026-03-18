#include "ConfigManager.hpp"
#include <cstdio>

namespace Robot {
namespace Config {

ConfigManager::ConfigManager() {
    loadConfigs();
}

ConfigManager::~ConfigManager() {
    // 析构函数
}

bool ConfigManager::loadConfigs() {
    // 使用默认构造函数创建默认配置
    config_ = RobotConfig();

    // 使用ActualServoConfig提供的实际测试数据
    printf("ConfigManager: 使用ActualServoConfig的实际测试数据\n");
    config_.servo_config = createActualConfig();

    // 设置关节映射关系
    setupJointMappings();

    last_error_ = "";
    return validateConfig();
}

/**
 * @brief 创建基于实际测试数据的舵机配置
 * @return 配置好的AllServoConfig对象
 */
AllServoConfig ConfigManager::createActualConfig() {
    AllServoConfig config;

    // 左前腿
    config.setServo(0, ServoConfig(170, 1060, true));   // 舵机0: 170→615→1060
    config.setServo(1, ServoConfig(170, 1080, true));   // 舵机1: 170→625→1080
    config.setServo(2, ServoConfig(170, 1080, true));   // 舵机2: 170→625→1080

    // 右前腿
    config.setServo(3, ServoConfig(240, 1130, true));   // 舵机3: 240→685→1130
    config.setServo(4, ServoConfig(170, 1090, true));   // 舵机4: 170→630→1090
    config.setServo(5, ServoConfig(185, 1085, true));   // 舵机5: 185→635→1085

    // 左后腿
    config.setServo(6, ServoConfig(190, 1120, true));   // 舵机6: 190→655→1120
    config.setServo(7, ServoConfig(180, 1080, true));   // 舵机7: 200→655→1110
    config.setServo(8, ServoConfig(195, 1110, true));   // 舵机8: 195→655→1110

    // 右后腿
    config.setServo(9, ServoConfig(210, 1130, true));   // 舵机9: 210→670→1130
    config.setServo(10, ServoConfig(180, 1080, true));  // 舵机10: 180→630→1080
    config.setServo(11, ServoConfig(180, 1080, true));  // 舵机11: 180→630→1080

    return config;
}

void ConfigManager::setupJointMappings() {
    // 根据您提供的精确映射关系设置
    // 映射公式：servo_angle = k * joint_angle + b

    printf("ConfigManager: 设置关节映射关系（基于您的精确映射）\n");

    // 左前腿映射 (舵机0,1,2)
    LegMappingConfig left_front;
    // 髋关节侧摆：90°→0°, 0°→90°, 180°→-90° => servo = -joint + 90
    left_front.setJoint(JointType::HIP_ROLL, LegMappingConfig::JointMapping(-1.0f, 90.0f, -90.0f, 90.0f));
    // 髋关节俯仰：90°→0°, 180°→90°, 0°→-90° => servo = joint + 90
    left_front.setJoint(JointType::HIP_PITCH, LegMappingConfig::JointMapping(1.0f, 90.0f, -90.0f, 90.0f));
    // 膝关节俯仰：180°→0°, 90°→-90°, 0°→-180° => servo = joint + 180
    left_front.setJoint(JointType::KNEE_PITCH, LegMappingConfig::JointMapping(1.0f, 180.0f, -180.0f, 0.0f));
    config_.mapping_config.setLeg(LegID::FRONT_LEFT, left_front);

    // 右前腿映射 (舵机3,4,5)
    LegMappingConfig right_front;
    // 髋关节侧摆：90°→0°, 0°→90°, 180°→-90° => servo = -joint + 90
    right_front.setJoint(JointType::HIP_ROLL, LegMappingConfig::JointMapping(-1.0f, 90.0f, -90.0f, 90.0f));
    // 髋关节俯仰：90°→0°, 0°→90°, 180°→-90° => servo = -joint + 90
    right_front.setJoint(JointType::HIP_PITCH, LegMappingConfig::JointMapping(-1.0f, 90.0f, -90.0f, 90.0f));
    // 膝关节俯仰：0°→0°, 90°→-90°, 180°→-180° => servo = -joint
    right_front.setJoint(JointType::KNEE_PITCH, LegMappingConfig::JointMapping(-1.0f, 0.0f, -180.0f, 0.0f));
    config_.mapping_config.setLeg(LegID::FRONT_RIGHT, right_front);

    // 左后腿映射 (舵机6,7,8)
    LegMappingConfig left_back;
    // 髋关节侧摆：90°→0°, 180°→90°, 0°→-90° => servo = joint + 90
    left_back.setJoint(JointType::HIP_ROLL, LegMappingConfig::JointMapping(1.0f, 90.0f, -90.0f, 90.0f));
    // 髋关节俯仰：90°→0°, 180°→90°, 0°→-90° => servo = joint + 90
    left_back.setJoint(JointType::HIP_PITCH, LegMappingConfig::JointMapping(1.0f, 90.0f, -90.0f, 90.0f));
    // 膝关节俯仰：180°→0°, 90°→-90°, 0°→-180° => servo = joint + 180
    left_back.setJoint(JointType::KNEE_PITCH, LegMappingConfig::JointMapping(1.0f, 180.0f, -180.0f, 0.0f));
    config_.mapping_config.setLeg(LegID::BACK_LEFT, left_back);

    // 右后腿映射 (舵机9,10,11)
    LegMappingConfig right_back;
    // 髋关节侧摆：90°→0°, 180°→90°, 0°→-90° => servo = joint + 90
    right_back.setJoint(JointType::HIP_ROLL, LegMappingConfig::JointMapping(1.0f, 90.0f, -90.0f, 90.0f));
    // 髋关节俯仰：90°→0°, 0°→90°, 180°→-90° => servo = -joint + 90
    right_back.setJoint(JointType::HIP_PITCH, LegMappingConfig::JointMapping(-1.0f, 90.0f, -90.0f, 90.0f));
    // 膝关节俯仰：0°→0°, 90°→-90°, 180°→-180° => servo = -joint
    right_back.setJoint(JointType::KNEE_PITCH, LegMappingConfig::JointMapping(-1.0f, 0.0f, -180.0f, 0.0f));
    config_.mapping_config.setLeg(LegID::BACK_RIGHT, right_back);
}

bool ConfigManager::setConfig(const RobotConfig& config) {
    if (!config.isValid()) {
        setError("Invalid configuration provided");
        return false;
    }

    config_ = config;
    last_error_ = "";
    return true;
}

bool ConfigManager::validateConfig() const {
    if (!config_.isValid()) {
        return false;
    }

    // 额外的验证逻辑
    for (int i = 0; i < AllServoConfig::SERVO_COUNT; i++) {
        const auto& servo = config_.servo_config.getServo(i);
        if (!servo.isValid()) {
            return false;
        }
    }

    return true;
}

bool ConfigManager::setServoConfig(const AllServoConfig& servo_config) {
    if (!servo_config.isValid()) {
        setError("Invalid servo configuration");
        return false;
    }

    // 确保所有舵机配置都重新计算了比率
    AllServoConfig updated_config = servo_config;
    for (int i = 0; i < AllServoConfig::SERVO_COUNT; i++) {
        auto servo = updated_config.servos[i];
        servo.calculateRatio();
        updated_config.setServo(i, servo);
    }

    config_.servo_config = updated_config;
    return true;
}

bool ConfigManager::setMappingConfig(const MappingConfig& mapping_config) {
    if (!mapping_config.isValid()) {
        setError("Invalid mapping configuration");
        return false;
    }
    config_.mapping_config = mapping_config;
    return true;
}

bool ConfigManager::setGeometryConfig(const GeometryConfig& geometry_config) {
    if (!geometry_config.isValid()) {
        setError("Invalid geometry configuration");
        return false;
    }
    config_.geometry_config = geometry_config;
    return true;
}

bool ConfigManager::setControlConfig(const ControlConfig& control_config) {
    if (!control_config.isValid()) {
        setError("Invalid control configuration");
        return false;
    }
    config_.control_config = control_config;
    return true;
}

void ConfigManager::setError(const std::string& error) {
    last_error_ = error;
}

} // namespace Config
} // namespace Robot