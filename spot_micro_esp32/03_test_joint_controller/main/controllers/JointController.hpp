#pragma once

#include "../drivers/ServoDriver.hpp"
#include "../config/ConfigManager.hpp"
#include "../config/MappingConfig.hpp"
#include "../RobotTypes.hpp"
#include "../utils/Logger.hpp"
#include <memory>
#include <array>

namespace Robot {
namespace Controllers {

/**
 * @brief 关节控制错误码
 */
enum class JointError {
    SUCCESS = 0,           // 成功
    INVALID_LEG_ID = 1,    // 无效腿部ID
    INVALID_JOINT_TYPE = 2,// 无效关节类型
    INVALID_ANGLE = 3,     // 无效角度
    SERVO_ERROR = 4,       // 舵机错误
    NOT_INITIALIZED = 5,   // 未初始化
    CONFIG_ERROR = 6       // 配置错误
};

/**
 * @brief 映射关系缓存结构体 - 高性能角度转换
 */
struct CachedJointMapping {
    float k;        // 线性映射系数
    float b;        // 线性映射偏移
    bool enabled;   // 是否启用

    CachedJointMapping() : k(1.0f), b(90.0f), enabled(true) {}
    CachedJointMapping(float slope, float offset, bool en = true) : k(slope), b(offset), enabled(en) {}

    // 内联角度转换 - 高性能
    inline float jointToServo(float joint_deg) const { return k * joint_deg + b; }
    inline float servoToJoint(float servo_deg) const { return (servo_deg - b) / k; }
};

/**
 * @brief 关节控制器
 *
 * 负责关节角度到舵机角度的映射和控制
 * 简化版本，专注于核心功能
 */
class JointController {
public:
    static constexpr int LEG_COUNT = 4;
    static constexpr int JOINTS_PER_LEG = 3;

    /**
     * @brief 构造函数
     * @param servo_driver 舵机驱动器 (依赖注入)
     * @param config_manager 配置管理器 (依赖注入)
     */
    JointController(std::shared_ptr<ServoDriver> servo_driver,
                    std::shared_ptr<Config::ConfigManager> config_manager);

    /**
     * @brief 析构函数
     */
    ~JointController();

    /**
     * @brief 初始化关节控制器
     * @return 错误码
     */
    JointError init();

    /**
     * @brief 反初始化
     */
    void deinit();

    /**
     * @brief 设置单个关节角度
     * @param leg_id 腿部ID
     * @param joint_type 关节类型
     * @param angle 关节角度 (度)
     * @return 错误码
     */
    JointError setJointAngle(LegID leg_id, JointType joint_type, float angle);

    /**
     * @brief 获取单个关节角度
     * @param leg_id 腿部ID
     * @param joint_type 关节类型
     * @param angle 输出关节角度
     * @return 错误码
     */
    JointError getJointAngle(LegID leg_id, JointType joint_type, float& angle) const;

    /**
     * @brief 设置单条腿的所有关节角度
     * @param leg_id 腿部ID
     * @param angles 三个关节的角度
     * @return 错误码
     */
    JointError setLegAngles(LegID leg_id, const ThreeJointAngles& angles);

    /**
     * @brief 获取单条腿的所有关节角度
     * @param leg_id 腿部ID
     * @param angles 输出三个关节的角度
     * @return 错误码
     */
    JointError getLegAngles(LegID leg_id, ThreeJointAngles& angles) const;

    /**
     * @brief 设置所有腿的关节角度
     * @param all_angles 所有腿的关节角度
     * @return 错误码
     */
    JointError setAllJointAngles(const AllLegJoints& all_angles);

    /**
     * @brief 获取所有腿的关节角度
     * @param all_angles 输出所有腿的关节角度
     * @return 错误码
     */
    JointError getAllJointAngles(AllLegJoints& all_angles) const;

    /**
     * @brief 关节角度转舵机角度
     * @param leg_id 腿部ID
     * @param joint_type 关节类型
     * @param joint_angle 关节角度
     * @param servo_angle 输出舵机角度
     * @return 错误码
     */
    JointError jointToServoAngle(LegID leg_id, JointType joint_type,
                                float joint_angle, float& servo_angle) const;

    /**
     * @brief 舵机角度转关节角度
     * @param leg_id 腿部ID
     * @param joint_type 关节类型
     * @param servo_angle 舵机角度
     * @param joint_angle 输出关节角度
     * @return 错误码
     */
    JointError servoToJointAngle(LegID leg_id, JointType joint_type,
                                float servo_angle, float& joint_angle) const;

    /**
     * @brief 高性能关节角度转舵机角度 (使用缓存)
     * @param leg_id 腿部ID
     * @param joint_type 关节类型
     * @param joint_angle 关节角度
     * @param servo_angle 输出舵机角度
     * @return 错误码
     */
    JointError jointToServoAngleFast(LegID leg_id, JointType joint_type,
                                    float joint_angle, float& servo_angle) const;

    /**
     * @brief 高性能舵机角度转关节角度 (使用缓存)
     * @param leg_id 腿部ID
     * @param joint_type 关节类型
     * @param servo_angle 舵机角度
     * @param joint_angle 输出关节角度
     * @return 错误码
     */
    JointError servoToJointAngleFast(LegID leg_id, JointType joint_type,
                                    float servo_angle, float& joint_angle) const;

    /**
     * @brief 获取关节角度限制
     * @param leg_id 腿部ID
     * @param joint_type 关节类型
     * @param min_angle 输出最小角度
     * @param max_angle 输出最大角度
     * @return 错误码
     */
    JointError getJointLimits(LegID leg_id, JointType joint_type,
                             float& min_angle, float& max_angle) const;

    /**
     * @brief 验证关节角度是否在限制范围内
     * @param leg_id 腿部ID
     * @param joint_type 关节类型
     * @param angle 关节角度
     * @return 是否在范围内
     */
    bool isJointAngleValid(LegID leg_id, JointType joint_type, float angle) const;

    /**
     * @brief 限制关节角度在有效范围内
     * @param leg_id 腿部ID
     * @param joint_type 关节类型
     * @param angle 输入角度
     * @return 限制后的角度
     */
    float clampJointAngle(LegID leg_id, JointType joint_type, float angle) const;

    /**
     * @brief 获取舵机ID
     * @param leg_id 腿部ID
     * @param joint_type 关节类型
     * @return 舵机ID
     */
    int getServoId(LegID leg_id, JointType joint_type) const;

    /**
     * @brief 预设姿势 - 趴下
     * @return 错误码
     */
    JointError setSleepPose();

    /**
     * @brief 预设姿势 - 站立
     * @return 错误码
     */
    JointError setStandPose();

    /**
     * @brief 获取错误描述
     */
    static const char* getErrorString(JointError error);

    /**
     * @brief 获取初始化状态
     */
    bool isInitialized() const { return initialized_; }

private:
    std::shared_ptr<ServoDriver> servo_driver_;              // 舵机驱动器
    std::shared_ptr<Config::ConfigManager> config_manager_;  // 配置管理器
    std::array<ThreeJointAngles, LEG_COUNT> current_angles_; // 当前关节角度缓存
    bool initialized_;                                       // 初始化状态

    // 映射关系缓存 - 避免频繁查询ConfigManager (4条腿 x 3个关节)
    std::array<std::array<CachedJointMapping, JOINTS_PER_LEG>, LEG_COUNT> mapping_cache_;
    bool mapping_cached_;    // 缓存是否已构建

    /**
     * @brief 验证腿部ID
     */
    bool isValidLegId(LegID leg_id) const {
        return static_cast<int>(leg_id) >= 0 && static_cast<int>(leg_id) < LEG_COUNT;
    }

    /**
     * @brief 验证关节类型
     */
    bool isValidJointType(JointType joint_type) const {
        return static_cast<int>(joint_type) >= 0 && static_cast<int>(joint_type) < JOINTS_PER_LEG;
    }

    /**
     * @brief 获取映射配置
     */
    const Config::LegMappingConfig::JointMapping&
    getJointMapping(LegID leg_id, JointType joint_type) const;

    /**
     * @brief 应用映射转换 (关节角度 -> 舵机角度)
     */
    float applyJointMapping(const Config::LegMappingConfig::JointMapping& mapping,
                           float joint_angle) const;

    /**
     * @brief 反向映射转换 (舵机角度 -> 关节角度)
     */
    float reverseJointMapping(const Config::LegMappingConfig::JointMapping& mapping,
                             float servo_angle) const;

    /**
     * @brief 构建映射关系缓存 - 初始化时一次性构建
     */
    bool buildMappingCache();

    /**
     * @brief 获取缓存的映射关系 - 高性能访问
     */
    const CachedJointMapping& getCachedMapping(LegID leg_id, JointType joint_type) const;
};

} // namespace Controllers
} // namespace Robot