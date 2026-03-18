#pragma once

#include "LegKinematics.hpp"
#include "KinematicsGeometry.hpp"
#include "CoordinateTransform.hpp"
#include <memory>

namespace Robot {
namespace Kinematics {

/**
 * @brief 单条腿的高层封装类
 *
 * 封装了单条腿的所有属性和行为，包括：
 * - 腿部物理参数（连杆长度、髋关节偏移等）
 * - 当前状态（关节角度、足端位置）
 * - 运动学计算接口（正逆运动学）
 *
 * 这个类解决了原有代码中腿部逻辑分散、状态管理复杂的问题
 */
class Leg {
public:
    using LegType = SpotMicroGeometry::LegIndex;
    using Vector3 = CoordinateTransform::Vector3;

    /**
     * @brief 构造函数
     * @param leg_type 腿部类型（LEFT_FRONT, RIGHT_FRONT, LEFT_BACK, RIGHT_BACK）
     * @param geometry_config 几何配置参数
     */
    explicit Leg(LegType leg_type, const Robot::Config::GeometryConfig& geometry_config = Robot::Config::GeometryConfig());

    /**
     * @brief 析构函数
     */
    ~Leg() = default;

    // 禁用拷贝构造和赋值，确保对象唯一性
    Leg(const Leg&) = delete;
    Leg& operator=(const Leg&) = delete;

    // 允许移动构造和赋值
    Leg(Leg&&) = default;
    Leg& operator=(Leg&&) = default;

    /**
     * @brief 逆运动学计算
     *
     * 根据足端在腿部坐标系中的目标位置，计算对应的关节角度
     * @param foot_position_in_leg_frame 足端在腿部坐标系中的位置（米）
     * @return 运动学计算结果，包含成功标志、关节角度和错误信息
     */
    KinematicsResult calculateIK(const Vector3& foot_position_in_leg_frame);

    /**
     * @brief 正运动学计算
     *
     * 根据关节角度计算足端位置
     * @param joint_angles 关节角度（弧度）
     * @return 运动学计算结果，包含成功标志、足端位置和错误信息
     */
    KinematicsResult calculateFK(const ThreeJointAngles& joint_angles);

    /**
     * @brief 正运动学计算（基于当前存储的关节角度）
     *
     * 使用内部存储的关节角度计算足端在腿部坐标系中的位置
     * @return 运动学计算结果，包含成功标志、足端位置和错误信息
     */
    KinematicsResult calculateFKFromCurrentAngles();

    /**
     * @brief 获取足端在腿部坐标系中的位置（基于当前角度）
     *
     * 这是一个便捷方法，直接返回基于当前关节角度计算的足端位置
     * @return 足端在腿部坐标系中的位置（米），如果计算失败返回零向量
     */
    Vector3 getFootPositionInLegFrame() const;

    /**
     * @brief 设置关节角度（数组版本）
     *
     * 使用数组形式设置关节角度，便于与舵机控制系统集成
     * @param angles 关节角度数组 [hip_side, hip_pitch, knee_pitch]（弧度）
     * @return 是否设置成功
     */
    bool setJointAngles(const float angles[3]);

    /**
     * @brief 设置关节角度（角度版本）
     *
     * 使用度数设置关节角度，便于用户输入
     * @param hip_side_deg 髋侧摆角度（度）
     * @param hip_pitch_deg 髋俯仰角度（度）
     * @param knee_pitch_deg 膝俯仰角度（度）
     * @return 是否设置成功
     */
    bool setJointAnglesDegrees(float hip_side_deg, float hip_pitch_deg, float knee_pitch_deg);

    /**
     * @brief 设置足端位置
     *
     * 设置足端目标位置并自动计算对应的关节角度
     * @param position 足端在腿部坐标系中的位置（米）
     * @return 是否设置成功
     */
    bool setFootPosition(const Vector3& position);

    /**
     * @brief 设置关节角度
     *
     * 直接设置关节角度并更新足端位置
     * @param angles 关节角度（弧度）
     * @return 是否设置成功
     */
    bool setJointAngles(const ThreeJointAngles& angles);

    /**
     * @brief 获取当前关节角度
     * @return 当前关节角度（弧度）
     */
    const ThreeJointAngles& getJointAngles() const { return current_joint_angles_; }

    /**
     * @brief 获取当前足端位置
     * @return 足端在腿部坐标系中的位置（米）
     */
    const Vector3& getFootPosition() const { return current_foot_position_; }

    /**
     * @brief 获取髋关节偏移
     * @return 髋关节在机体坐标系中的偏移（米）
     */
    CoordinateTransform::Vector3 getHipOffset() const;

    /**
     * @brief 检查当前状态是否有效
     * @return 当前关节角度和足端位置是否有效
     */
    bool isValid() const { return is_valid_; }

    /**
     * @brief 检查给定位置是否可达
     * @param position 待检查的位置（腿部坐标系）
     * @return 位置是否在工作空间内
     */
    bool isPositionReachable(const Vector3& position) const;

    /**
     * @brief 获取腿部类型
     * @return 腿部类型枚举
     */
    LegType getLegType() const { return leg_type_; }

    /**
     * @brief 获取腿部名称（用于调试）
     * @return 腿部名称字符串
     */
    const char* getLegName() const;

    /**
     * @brief 重置到默认状态
     *
     * 将腿部重置为合理的默认姿态（站立状态）
     */
    void resetToDefault();

    /**
     * @brief 获取工作空间边界
     * @return 腿部工作空间边界信息
     */
    SpotLegKinematics::WorkspaceBounds getWorkspaceBounds() const;

    /**
     * @brief 调试输出腿部信息
     */
    void printDebugInfo() const;

private:
    // 腿部标识
    LegType leg_type_;

    // 几何配置
    Robot::Config::GeometryConfig geometry_config_;
    SpotMicroGeometry geometry_;

    // 运动学求解器（封装底层计算逻辑）
    std::unique_ptr<SpotLegKinematics> kinematics_solver_;

    // 当前状态
    ThreeJointAngles current_joint_angles_;  // 当前关节角度
    Vector3 current_foot_position_;          // 当前足端位置（腿部坐标系）
    bool is_valid_;                          // 当前状态是否有效

    /**
     * @brief 更新内部状态
     *
     * 当关节角度或足端位置发生变化时，更新对应的状态
     * @param result 运动学计算结果
     */
    void updateInternalState(const KinematicsResult& result);

    /**
     * @brief 验证关节角度和位置的一致性
     * @return 是否一致
     */
    bool validateConsistency() const;
};

} // namespace Kinematics
} // namespace Robot