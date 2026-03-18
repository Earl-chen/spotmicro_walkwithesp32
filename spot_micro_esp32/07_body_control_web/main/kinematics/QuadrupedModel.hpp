#pragma once

#include "Leg.hpp"
#include "CoordinateTransform.hpp"
#include <array>
#include <memory>

namespace Robot {
namespace Kinematics {

/**
 * @brief 四足机器人模型类
 *
 * 封装了整个四足机器人的协调控制逻辑，包括：
 * - 四条腿的统一管理
 * - 机身位姿控制
 * - 坐标系变换管理
 * - 预设动作执行
 *
 * 这个类解决了原有代码中四腿重复处理、协调逻辑分散的问题
 */
class QuadrupedModel {
public:
    using Vector3 = CoordinateTransform::Vector3;
    using Pose6DOF = CoordinateTransform::Pose6DOF;
    using LegType = SpotMicroGeometry::LegIndex;

    /**
     * @brief 预设姿势类型
     */
    enum class PresetPoseType {
        STAND,   // 标准站立
        SIT,     // 坐下（前腿伸展，后腿收缩）
        CROUCH,  // 蹲下（所有腿收缩）
        TILT     // 前倾（机身倾斜10度）
    };

    /**
     * @brief 四足机器人状态结构
     */
    struct RobotState {
        Pose6DOF body_pose;                                    // 机身位姿
        std::array<ThreeJointAngles, 4> joint_angles;         // 所有关节角度
        std::array<Vector3, 4> foot_positions_in_leg_frame;   // 足端位置（腿部坐标系）
        std::array<Vector3, 4> foot_positions_in_world;       // 足端位置（世界坐标系）
        std::array<bool, 4> leg_validity;                     // 各腿状态有效性
    };

    /**
     * @brief 构造函数
     * @param geometry_config 几何配置参数
     */
    explicit QuadrupedModel(const Robot::Config::GeometryConfig& geometry_config = Robot::Config::GeometryConfig());

    /**
     * @brief 析构函数
     */
    ~QuadrupedModel() = default;

    // 禁用拷贝构造和赋值
    QuadrupedModel(const QuadrupedModel&) = delete;
    QuadrupedModel& operator=(const QuadrupedModel&) = delete;

    // 允许移动构造和赋值
    QuadrupedModel(QuadrupedModel&&) = default;
    QuadrupedModel& operator=(QuadrupedModel&&) = default;

    /**
     * @brief 更新机身位姿
     *
     * 这是核心方法，用于替代原有handlePoseCommand的主要逻辑
     * @param position 机身在世界坐标系中的位置（米）
     * @param orientation 机身姿态角（roll, pitch, yaw，弧度）
     * @return 是否更新成功
     */
    bool updateBodyPose(const Vector3& position, const Vector3& orientation);

    /**
     * @brief 更新机身位姿（6DOF版本）
     * @param pose 6自由度位姿
     * @return 是否更新成功
     */
    bool updateBodyPose(const Pose6DOF& pose);

    /**
     * @brief 设置单腿足端位置
     *
     * 设置指定腿的足端位置（腿部坐标系）
     * @param leg_id 腿部索引（0-3）
     * @param position 足端位置（腿部坐标系，米）
     * @return 是否设置成功
     */
    bool setLegPosition(int leg_id, const Vector3& position);

    /**
     * @brief 获取所有关节角度
     *
     * 返回所有12个关节的角度，按腿部顺序排列
     * @return 关节角度数组
     */
    std::array<ThreeJointAngles, 4> getAllJointAngles() const;

    /**
     * @brief 获取指定腿的关节角度
     * @param leg_id 腿部索引（0-3）
     * @return 指定腿的关节角度，如果无效返回空值
     */
    ThreeJointAngles getLegJointAngles(int leg_id) const;

    /**
     * @brief 执行预设姿势
     * @param pose_type 预设姿势类型
     * @return 是否执行成功
     */
    bool executePresetPose(PresetPoseType pose_type);

    /**
     * @brief 基于给定舵机角度初始化机器人状态
     *
     * 核心功能：通过指定的12个舵机角度设置机器人初始状态，
     * 并计算各腿足端在世界坐标系中的位置
     * @param all_joint_angles 包含所有12个舵机角度的数组（弧度）
     *                        排列顺序：[leg0_hip_side, leg0_hip_pitch, leg0_knee_pitch,
     *                                  leg1_hip_side, leg1_hip_pitch, leg1_knee_pitch,
     *                                  leg2_hip_side, leg2_hip_pitch, leg2_knee_pitch,
     *                                  leg3_hip_side, leg3_hip_pitch, leg3_knee_pitch]
     * @return 是否初始化成功
     */
    bool setInitialStateFromAngles(const float all_joint_angles[12]);

    /**
     * @brief 基于给定舵机角度初始化（度数版本）
     *
     * 便于用户使用度数输入的版本
     * @param all_joint_angles_deg 包含所有12个舵机角度的数组（度）
     * @return 是否初始化成功
     */
    bool setInitialStateFromAnglesDegrees(const float all_joint_angles_deg[12]);

    /**
     * @brief 获取指定腿足端在世界坐标系中的位置
     * @param leg_index 腿部索引（0-3）
     * @return 足端在世界坐标系中的位置（米）
     */
    Vector3 getFootPositionInWorld(int leg_index) const;

    /**
     * @brief 获取所有腿足端在世界坐标系中的位置
     * @return 包含四个足端世界坐标的数组
     */
    std::array<Vector3, 4> getAllFootPositionsInWorld() const;

    /**
     * @brief 更新所有腿的足端世界坐标缓存
     *
     * 基于当前的机身位姿和各腿状态，重新计算并缓存足端世界坐标
     */
    void updateFootWorldPositionsCache();

    /**
     * @brief 打印当前机器人状态信息
     *
     * 包括机身位姿、各腿关节角度、足端位置等详细信息
     */
    void printCurrentState() const;

    /**
     * @brief 获取当前机器人状态
     * @return 完整的机器人状态信息
     */
    RobotState getCurrentState() const;

    /**
     * @brief 检查当前配置是否有效
     * @return 是否所有腿部都处于有效状态
     */
    bool isValidConfiguration() const;

    /**
     * @brief 重置到默认状态
     *
     * 将机器人重置为合理的默认站立姿态
     */
    void resetToDefault();

    /**
     * @brief 获取指定腿的实例
     * @param leg_id 腿部索引（0-3）
     * @return Leg实例指针，无效时返回nullptr
     */
    Leg* getLeg(int leg_id);
    const Leg* getLeg(int leg_id) const;

    /**
     * @brief 获取当前机身位姿
     * @return 机身位姿
     */
    Pose6DOF getBodyPose() const;

    /**
     * @brief 调试输出机器人信息
     */
    void printDebugInfo() const;

    /**
     * @brief 验证腿部索引有效性
     * @param leg_id 腿部索引
     * @return 是否有效
     */
    static bool isValidLegId(int leg_id) { return leg_id >= 0 && leg_id < 4; }

private:
    // 几何配置
    Robot::Config::GeometryConfig geometry_config_;

    // 四条腿的实例
    std::array<std::unique_ptr<Leg>, 4> legs_;

    // 坐标系变换管理器
    FrameManager frame_manager_;

    // 缓存的足部世界坐标（用于位姿控制时保持足端位置）
    std::array<Vector3, 4> cached_foot_world_positions_;

    /**
     * @brief 初始化所有腿部
     */
    void initializeLegs();

    /**
     * @brief 更新缓存的足部世界坐标
     *
     * 保存当前所有足端在世界坐标系中的位置
     */
    void updateCachedFootWorldPositions();

    /**
     * @brief 根据缓存的足部世界坐标重新计算关节角度
     *
     * 当机身位姿改变时，基于保存的足端世界坐标重新计算各腿关节角度
     * @return 是否计算成功
     */
    bool recalculateJointAnglesFromWorldPositions();

    /**
     * @brief 执行站立姿势
     */
    bool executeStandPose();

    /**
     * @brief 执行坐下姿势
     */
    bool executeSitPose();

    /**
     * @brief 执行蹲下姿势
     */
    bool executeCrouchPose();

    /**
     * @brief 执行前倾姿势
     */
    bool executeTiltPose();

    /**
     * @brief 坐标变换：髋关节坐标系 -> 机体坐标系 -> 世界坐标系
     * @param leg_id 腿部索引
     * @param hip_position 髋关节坐标系中的位置
     * @return 世界坐标系中的位置
     */
    Vector3 transformHipToWorld(int leg_id, const Vector3& hip_position) const;

    /**
     * @brief 坐标变换：世界坐标系 -> 机体坐标系 -> 髋关节坐标系
     * @param leg_id 腿部索引
     * @param world_position 世界坐标系中的位置
     * @return 髋关节坐标系中的位置
     */
    Vector3 transformWorldToHip(int leg_id, const Vector3& world_position) const;

    /**
     * @brief 获取腿部名称（用于调试）
     */
    static const char* getLegName(int leg_id);
};

} // namespace Kinematics
} // namespace Robot