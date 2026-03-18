#include "LegKinematics.hpp"
#include <esp_log.h>
#include <cmath>
#include <algorithm>

namespace Robot {
namespace Kinematics {

const char* SpotLegKinematics::TAG = "LegKinematics";

SpotLegKinematics::SpotLegKinematics(bool is_left, const Robot::Config::GeometryConfig& geometry_config)
    : l1_(geometry_config.l1), l2_(geometry_config.l2), l3_(geometry_config.l3), l4_(geometry_config.l4), is_left_(is_left) {

    ESP_LOGI(TAG, "初始化%s腿运动学: L1=%.3f L2=%.3f L3=%.3f L4=%.3f",
             is_left ? "左" : "右", l1_, l2_, l3_, l4_);

    // 验证几何参数
    SpotMicroGeometry geometry(geometry_config);
    if (!geometry.validateGeometry()) {
        ESP_LOGW(TAG, "警告: 几何参数可能不合理");
    }
}

bool SpotLegKinematics::validateJointAngles(const ThreeJointAngles& angles) const {
    // 基于实际舵机限制的角度范围检查
    const float HIP_SIDE_LIMIT = M_PI/2;      // ±90°
    const float HIP_PITCH_LIMIT = M_PI/2;     // ±90°
    const float KNEE_PITCH_MIN = -M_PI;       // -180°
    const float KNEE_PITCH_MAX = 0;           // 0°

    if (std::abs(angles.hip_side) > HIP_SIDE_LIMIT) return false;
    if (std::abs(angles.hip_pitch) > HIP_PITCH_LIMIT) return false;
    if (angles.knee_pitch < KNEE_PITCH_MIN || angles.knee_pitch > KNEE_PITCH_MAX) return false;

    return true;
}

bool SpotLegKinematics::validateFootPosition(const CoordinateTransform::Vector3& foot_pos) const {
    // 基本可达性检查
    float distance = sqrt(foot_pos.x*foot_pos.x + foot_pos.y*foot_pos.y + foot_pos.z*foot_pos.z);
    float max_reach = l3_ + l4_;
    float min_reach = abs(l3_ - l4_);

    return (distance >= min_reach && distance <= max_reach);
}

KinematicsResult SpotLegKinematics::forwardKinematics(const CoordinateTransform::Vector3& hip_offset,
                                                    const ThreeJointAngles& joint_angles) const {
    // 验证输入角度
    if (!validateJointAngles(joint_angles)) {
        return KinematicsResult(false, joint_angles, CoordinateTransform::Vector3(), "关节角度超出限制");
    }

    // 从Python版本leg_kinematics.py的forward()方法移植
    float hip_side_angle = joint_angles.hip_side;
    float hip_pitch_angle = joint_angles.hip_pitch;
    float knee_pitch_angle = joint_angles.knee_pitch;

    // 髋关节延伸段计算
    CoordinateTransform::Vector3 l1_base;
    if (is_left_) {
        l1_base = CoordinateTransform::Vector3(0, l1_, 0);
    } else {
        l1_base = CoordinateTransform::Vector3(0, -l1_, 0);
    }

    CoordinateTransform::Matrix3x3 rot_side = CoordinateTransform::rotationX(hip_side_angle);
    CoordinateTransform::Vector3 extension_pos = hip_offset +
        CoordinateTransform::matrixVectorMultiply(rot_side, l1_base);

    // 髋关节俯仰段计算
    CoordinateTransform::Vector3 l2_base(0, 0, -l2_);
    CoordinateTransform::Vector3 hip_pitch_pos = extension_pos +
        CoordinateTransform::matrixVectorMultiply(rot_side, l2_base);

    // 大腿段计算
    CoordinateTransform::Vector3 l3_base(0, 0, -l3_);
    CoordinateTransform::Matrix3x3 rot_pitch = CoordinateTransform::rotationY(hip_pitch_angle);
    CoordinateTransform::Matrix3x3 rot_side_pitch = CoordinateTransform::matrixMultiply(rot_side, rot_pitch);
    CoordinateTransform::Vector3 knee_pos = hip_pitch_pos +
        CoordinateTransform::matrixVectorMultiply(rot_side_pitch, l3_base);

    // 小腿段计算
    CoordinateTransform::Vector3 l4_base(0, 0, -l4_);
    CoordinateTransform::Matrix3x3 rot_knee = CoordinateTransform::rotationY(knee_pitch_angle);
    CoordinateTransform::Matrix3x3 rot_combined = CoordinateTransform::matrixMultiply(rot_side_pitch, rot_knee);
    CoordinateTransform::Vector3 foot_pos = knee_pos +
        CoordinateTransform::matrixVectorMultiply(rot_combined, l4_base);

    return KinematicsResult(true, joint_angles, foot_pos, "");
}

KinematicsResult SpotLegKinematics::inverseKinematics(const CoordinateTransform::Vector3& foot_pos_in_hip) const {
    // 从Python版本leg_kinematics.py的inverse()方法移植
    float x = foot_pos_in_hip.x;
    float y = foot_pos_in_hip.y;
    float z = foot_pos_in_hip.z;

    // 几何计算
    float H = std::sqrt(y*y + z*z);

    // 检查基本几何约束
    if (H < l1_) {
        return KinematicsResult(false, ThreeJointAngles(), foot_pos_in_hip, "目标点太接近髋关节");
    }

    float G = std::sqrt(H*H - l1_*l1_);
    float F = G - l2_;
    float S = std::sqrt(F*F + x*x);

    // 检查可达性约束
    if (S > (l3_ + l4_)) {
        return KinematicsResult(false, ThreeJointAngles(), foot_pos_in_hip, "目标点超出最大伸展范围");
    }

    if (S < std::abs(l3_ - l4_)) {
        return KinematicsResult(false, ThreeJointAngles(), foot_pos_in_hip, "目标点太接近，无法到达");
    }

    // 髋侧摆角度计算 (与Python版本完全一致)
    float hip_side_angle;
    if (is_left_) {
        hip_side_angle = std::atan2(z, y) + std::acos(l1_ / H);
    } else {
        hip_side_angle = M_PI + std::atan2(z, y) - std::acos(l1_ / H);
    }

    // 髋俯仰角度计算 (与Python版本完全一致)
    float cos_hip = (S*S + l3_*l3_ - l4_*l4_) / (2*S*l3_);
    cos_hip = std::max(-1.0f, std::min(1.0f, cos_hip));  // 数值稳定性
    float hip_pitch_angle = std::acos(cos_hip) - std::atan2(x, F);

    // 膝俯仰角度计算 (与Python版本完全一致)
    float cos_knee = (S*S - l3_*l3_ - l4_*l4_) / (2*l3_*l4_);
    cos_knee = std::max(-1.0f, std::min(1.0f, cos_knee));  // 数值稳定性
    float knee_pitch_angle = -std::acos(cos_knee);

    // 约束角度范围（与Python版本一致）
    hip_side_angle = std::max(-float(M_PI)/2.0f, std::min(float(M_PI)/2.0f, hip_side_angle));
    hip_pitch_angle = std::max(-float(M_PI)/2.0f, std::min(float(M_PI)/2.0f, hip_pitch_angle));
    knee_pitch_angle = std::max(-float(M_PI), std::min(0.0f, knee_pitch_angle));

    ThreeJointAngles result_angles;
    result_angles.hip_side = hip_side_angle;
    result_angles.hip_pitch = hip_pitch_angle;
    result_angles.knee_pitch = knee_pitch_angle;

    // 验证计算结果
    if (!validateJointAngles(result_angles)) {
        return KinematicsResult(false, result_angles, foot_pos_in_hip, "计算得到的关节角度超出限制");
    }

    return KinematicsResult(true, result_angles, foot_pos_in_hip, "");
}

SpotLegKinematics::WorkspaceBounds SpotLegKinematics::getWorkspaceBounds() const {
    WorkspaceBounds bounds;
    // ADD BY CM，大致估算的工作空间边界--> 临时，后续可改为精确计算
    // 基于几何参数估算工作空间
    float max_reach = l2_ + l3_ + l4_;

    bounds.min_x = -max_reach;
    bounds.max_x = max_reach;
    bounds.min_y = -max_reach;
    bounds.max_y = max_reach;
    bounds.min_z = -max_reach;
    bounds.max_z = max_reach;

    return bounds;
}

void SpotLegKinematics::printLegInfo() const {
    ESP_LOGI(TAG, "=== %s腿运动学信息 ===", is_left_ ? "左" : "右");
    ESP_LOGI(TAG, "几何参数: L1=%.3fm L2=%.3fm L3=%.3fm L4=%.3fm", l1_, l2_, l3_, l4_);

    WorkspaceBounds bounds = getWorkspaceBounds();
    ESP_LOGI(TAG, "工作空间: X[%.3f,%.3f] Y[%.3f,%.3f] Z[%.3f,%.3f]",
             bounds.min_x, bounds.max_x, bounds.min_y, bounds.max_y, bounds.min_z, bounds.max_z);

    float max_reach = l3_ + l4_;
    float min_reach = abs(l3_ - l4_);
    ESP_LOGI(TAG, "伸展范围: 最小%.3fm 最大%.3fm", min_reach, max_reach);
}

} // namespace Kinematics
} // namespace Robot