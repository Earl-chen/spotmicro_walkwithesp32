#pragma once

#include "KinematicsGeometry.hpp"
#include "CoordinateTransform.hpp"
#include "RobotTypes.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Robot {
namespace Kinematics {

/**
 * @brief 运动学计算结果结构
 */
struct KinematicsResult {
    bool success;                    // 计算是否成功
    ThreeJointAngles joint_angles;   // 关节角度 (弧度)
    CoordinateTransform::Vector3 foot_position;  // 脚部位置 (米)
    const char* error_message;       // 错误信息

    KinematicsResult() : success(false), error_message("") {}
    KinematicsResult(bool s, const ThreeJointAngles& angles, const CoordinateTransform::Vector3& pos, const char* msg = "")
        : success(s), joint_angles(angles), foot_position(pos), error_message(msg) {}
};

/**
 * @brief SpotMicro单腿运动学求解器
 *
 * 与Python版本robots/spot_micro/leg_kinematics.py的算法完全一致
 * 支持正向运动学(FK)和逆向运动学(IK)计算
 */
class SpotLegKinematics {
private:
    // 几何参数
    float l1_, l2_, l3_, l4_;
    bool is_left_;

    static const char* TAG;

    /**
     * @brief 检查关节角度限制
     */
    bool validateJointAngles(const ThreeJointAngles& angles) const;

    /**
     * @brief 检查脚部位置可达性
     */
    bool validateFootPosition(const CoordinateTransform::Vector3& foot_pos) const;

public:
    /**
     * @brief 构造函数
     * @param is_left 是否为左腿
     * @param l1 髋关节延伸段长度 (默认使用几何参数)
     * @param l2 髋关节垂直段长度
     * @param l3 大腿长度
     * @param l4 小腿长度
     */
    SpotLegKinematics(bool is_left,
                      float l1 = SpotMicroGeometry::L1,
                      float l2 = SpotMicroGeometry::L2,
                      float l3 = SpotMicroGeometry::L3,
                      float l4 = SpotMicroGeometry::L4);

    /**
     * @brief 正向运动学计算
     *
     * 根据关节角度计算脚部位置
     * @param hip_offset 髋关节在机体坐标系中的偏移
     * @param joint_angles 关节角度 (弧度)
     * @return 运动学计算结果
     */
    KinematicsResult forwardKinematics(const CoordinateTransform::Vector3& hip_offset,
                                     const ThreeJointAngles& joint_angles) const;

    /**
     * @brief 逆向运动学计算
     *
     * 根据脚部在髋坐标系中的位置计算关节角度
     * @param foot_pos_in_hip 脚部在髋坐标系中的位置 (米)
     * @return 运动学计算结果
     */
    KinematicsResult inverseKinematics(const CoordinateTransform::Vector3& foot_pos_in_hip) const;

    /**
     * @brief 获取腿部工作空间边界
     */
    struct WorkspaceBounds {
        float min_x, max_x;  // X方向范围
        float min_y, max_y;  // Y方向范围
        float min_z, max_z;  // Z方向范围
    };

    WorkspaceBounds getWorkspaceBounds() const;

    /**
     * @brief 获取腿部几何参数
     */
    void getGeometry(float& l1, float& l2, float& l3, float& l4) const {
        l1 = l1_; l2 = l2_; l3 = l3_; l4 = l4_;
    }

    /**
     * @brief 获取腿部类型
     */
    bool isLeftLeg() const { return is_left_; }

    /**
     * @brief 调试输出腿部信息
     */
    void printLegInfo() const;
};

} // namespace Kinematics
} // namespace Robot