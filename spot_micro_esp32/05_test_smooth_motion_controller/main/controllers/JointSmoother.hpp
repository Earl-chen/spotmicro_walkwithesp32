#pragma once

#include "../RobotTypes.hpp"
#include "../utils/Logger.hpp"
#include "../config/SmoothMotionConfig.hpp"
#include "../config/ConfigManager.hpp"
#include <memory>
#include <array>

// 前向声明
namespace Robot {
namespace Controllers {
    class JointController;
}
}

namespace Robot {
namespace Controllers {

/**
 * @brief 关节状态结构体
 */
struct JointState {
    float current_angle;    // 当前关节角度（弧度）
    float target_angle;     // 目标关节角度（弧度）
    bool is_moving;         // 是否正在运动
    bool is_valid;          // 状态是否有效

    JointState() : current_angle(0.0f), target_angle(0.0f), is_moving(false), is_valid(false) {}
};



/**
 * @brief 关节平滑器
 *
 * 在关节层面添加平滑处理，处理运动学语义的关节角度
 * 内部自动处理关节角度到舵机角度的转换
 */
class JointSmoother {
private:
    std::shared_ptr<JointController> joint_controller_;  // 关节控制器 (负责角度转换和舵机控制)
    std::shared_ptr<Robot::Config::ConfigManager> config_manager_;  // 配置管理器
    ServoSmoothConfig config_;                   // 平滑配置

    // 所有关节的状态 (4条腿 x 3个关节 = 12个关节)
    std::array<std::array<JointState, Robot::Constants::JOINTS_PER_LEG>, Robot::Constants::LEG_COUNT> joint_states_;

    bool is_smoothing_;      // 是否正在执行平滑运动
    uint32_t total_steps_;   // 总步数计数

    static const char* TAG;

    /**
     * @brief 计算单个关节的角度步长
     */
    float calculateJointStep(Robot::LegID leg_id, Robot::JointType joint_type, float angle_diff) const;

    /**
     * @brief 检查是否所有关节都到达目标
     */
    bool hasAllJointsReachedTarget() const;

    /**
     * @brief 验证腿部ID有效性
     */
    bool isValidLegId(Robot::LegID leg_id) const;

    /**
     * @brief 验证关节类型有效性
     */
    bool isValidJointType(Robot::JointType joint_type) const;

    /**
     * @brief 验证关节角度有效性
     */
    bool isValidJointAngle(Robot::JointType joint_type, float angle) const;

    /**
     * @brief 获取关节状态引用
     */
    JointState& getJointState(Robot::LegID leg_id, Robot::JointType joint_type);
    const JointState& getJointState(Robot::LegID leg_id, Robot::JointType joint_type) const;

public:
    /**
     * @brief 构造函数
     */
    JointSmoother(std::shared_ptr<JointController> joint_controller,
                  std::shared_ptr<Robot::Config::ConfigManager> config_manager);

    /**
     * @brief 构造函数，带配置参数
     */
    JointSmoother(std::shared_ptr<JointController> joint_controller,
                  std::shared_ptr<Robot::Config::ConfigManager> config_manager,
                  const ServoSmoothConfig& config);

    /**
     * @brief 析构函数
     */
    ~JointSmoother();

    /**
     * @brief 初始化关节平滑器
     */
    bool initialize();

    /**
     * @brief 设置单个关节目标角度
     */
    bool setJointAngle(Robot::LegID leg_id, Robot::JointType joint_type, float angle);

    /**
     * @brief 设置单条腿的关节角度
     */
    bool setLegJointAngles(Robot::LegID leg_id, const ThreeJointAngles& angles);

    /**
     * @brief 设置所有腿的关节角度
     */
    bool setAllJointAngles(const AllLegJoints& all_joints);

    /**
     * @brief 从运动学角度设置腿部关节
     */
    bool setLegAnglesFromKinematics(Robot::LegID leg_id, float hip_side, float hip_pitch, float knee_pitch);

    /**
     * @brief 执行一步关节平滑运动
     * @return true: 还需要继续运动，false: 已到达目标
     */
    bool stepJointSmoothing();

    /**
     * @brief 执行完整的平滑运动直到到达目标
     */
    bool smoothToTarget();

    /**
     * @brief 立即跳转所有关节到目标角度
     */
    bool jumpToTarget();

    /**
     * @brief 停止平滑运动
     */
    void stopSmoothing();

    /**
     * @brief 获取当前关节角度
     */
    float getCurrentJointAngle(Robot::LegID leg_id, Robot::JointType joint_type) const;

    /**
     * @brief 获取目标关节角度
     */
    float getTargetJointAngle(Robot::LegID leg_id, Robot::JointType joint_type) const;

    /**
     * @brief 获取单条腿的当前关节角度
     */
    ThreeJointAngles getLegCurrentAngles(Robot::LegID leg_id) const;

    /**
     * @brief 获取单条腿的目标关节角度
     */
    ThreeJointAngles getLegTargetAngles(Robot::LegID leg_id) const;

    /**
     * @brief 获取所有关节当前角度
     */
    AllLegJoints getAllCurrentJoints() const;

    /**
     * @brief 获取所有关节目标角度
     */
    AllLegJoints getAllTargetJoints() const;

    /**
     * @brief 检查是否正在平滑运动
     */
    bool isSmoothing() const { return is_smoothing_; }

    /**
     * @brief 检查特定关节是否在运动
     */
    bool isJointMoving(Robot::LegID leg_id, Robot::JointType joint_type) const;

    /**
     * @brief 检查特定腿是否在运动
     */
    bool isLegMoving(Robot::LegID leg_id) const;

    /**
     * @brief 更新配置
     */
    void setConfig(const ServoSmoothConfig& config);

    /**
     * @brief 获取配置
     */
    const ServoSmoothConfig& getConfig() const { return config_; }

    /**
     * @brief 启用/禁用平滑功能
     */
    void enableSmoothing(bool enable);

    /**
     * @brief 重置所有关节状态
     */
    void reset();

    /**
     * @brief 获取平滑进度 (0.0 - 1.0)
     */
    float getSmoothingProgress() const;

    /**
     * @brief 调试信息输出
     */
    void printStatus() const;

    /**
     * @brief 打印所有关节状态
     */
    void printAllJointStates() const;
};

} // namespace Controllers
} // namespace Robot