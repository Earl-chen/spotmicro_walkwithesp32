#include "Leg.hpp"
#include "../utils/cpp11_compatibility.hpp"
#include <esp_log.h>
#include <cmath>

namespace Robot {
namespace Kinematics {

static const char* TAG = "Leg";

// 默认站立姿态的关节角度（弧度）
static constexpr float DEFAULT_HIP_SIDE = 0.0f;
static constexpr float DEFAULT_HIP_PITCH = 0.0f;
static constexpr float DEFAULT_KNEE_PITCH = 0.0f;

// 默认足端位置（腿部坐标系，米）- 调整为更合理的站立位置
static constexpr float DEFAULT_FOOT_X = 0.10f;  // 向前10cm
static constexpr float DEFAULT_FOOT_Y = 0.0f;   // 侧向居中
static constexpr float DEFAULT_FOOT_Z = -0.12f; // 向下12cm

Leg::Leg(LegType leg_type, const Robot::Config::GeometryConfig& geometry_config)
    : leg_type_(leg_type), geometry_config_(geometry_config), geometry_(geometry_config_)
    , current_joint_angles_(DEFAULT_HIP_SIDE, DEFAULT_HIP_PITCH, DEFAULT_KNEE_PITCH)
    , current_foot_position_(DEFAULT_FOOT_X, DEFAULT_FOOT_Y, DEFAULT_FOOT_Z)
    , is_valid_(false)
{
    // 根据腿部类型创建运动学求解器
    bool is_left = SpotMicroGeometry::isLeftLeg(leg_type_);
    kinematics_solver_ = std::make_unique<SpotLegKinematics>(is_left, geometry_config_);

    // 初始化为默认状态
    resetToDefault();

    ESP_LOGI(TAG, "Leg %s initialized", getLegName());
}

KinematicsResult Leg::calculateIK(const Vector3& foot_position_in_leg_frame) {
    if (!kinematics_solver_) {
        ESP_LOGE(TAG, "Kinematics solver not initialized");
        return KinematicsResult(false, ThreeJointAngles(), foot_position_in_leg_frame,
                               "Kinematics solver not initialized");
    }

    // 调用底层运动学求解器
    KinematicsResult result = kinematics_solver_->inverseKinematics(foot_position_in_leg_frame);

    if (result.success) {
        ESP_LOGD(TAG, "IK calculation successful for %s", getLegName());
    } else {
        ESP_LOGW(TAG, "IK calculation failed for %s: %s", getLegName(), result.error_message);
    }

    return result;
}

KinematicsResult Leg::calculateFK(const ThreeJointAngles& joint_angles) {
    if (!kinematics_solver_) {
        ESP_LOGE(TAG, "Kinematics solver not initialized");
        return KinematicsResult(false, joint_angles, Vector3(),
                               "Kinematics solver not initialized");
    }

    // 获取髋关节偏移并调用正向运动学
    Vector3 zero(0.0f, 0.0f, 0.0f);
    // 调用底层运动学求解器得到足端在髋关节坐标系中的位置
    KinematicsResult result = kinematics_solver_->forwardKinematics(zero, joint_angles);

    if (result.success) {
        ESP_LOGD(TAG, "FK calculation successful for %s", getLegName());
    } else {
        ESP_LOGW(TAG, "FK calculation failed for %s: %s", getLegName(), result.error_message);
    }

    return result;
}

KinematicsResult Leg::calculateFKFromCurrentAngles() {
    return calculateFK(current_joint_angles_);
}

CoordinateTransform::Vector3 Leg::getFootPositionInLegFrame() const {
    KinematicsResult result = const_cast<Leg*>(this)->calculateFK(current_joint_angles_);

    if (result.success) {
        return result.foot_position;
    } else {
        ESP_LOGW(TAG, "FK calculation failed for %s: %s", getLegName(), result.error_message);
        return CoordinateTransform::Vector3(0.0f, 0.0f, 0.0f);
    }
}

bool Leg::setJointAngles(const float angles[3]) {
    if (!angles) {
        ESP_LOGE(TAG, "Invalid angles array pointer");
        return false;
    }

    ThreeJointAngles joint_angles(angles[0], angles[1], angles[2]);
    return setJointAngles(joint_angles);
}

bool Leg::setJointAnglesDegrees(float hip_side_deg, float hip_pitch_deg, float knee_pitch_deg) {
    // 转换度数为弧度
    float hip_side_rad = hip_side_deg * M_PI / 180.0f;
    float hip_pitch_rad = hip_pitch_deg * M_PI / 180.0f;
    float knee_pitch_rad = knee_pitch_deg * M_PI / 180.0f;

    ThreeJointAngles joint_angles(hip_side_rad, hip_pitch_rad, knee_pitch_rad);
    return setJointAngles(joint_angles);
}

bool Leg::setFootPosition(const Vector3& position) {
    // 先检查位置是否可达
    if (!isPositionReachable(position)) {
        ESP_LOGW(TAG, "Position not reachable for %s: (%.3f, %.3f, %.3f)",
                 getLegName(), position.x, position.y, position.z);
        return false;
    }

    // 执行逆运动学计算
    KinematicsResult result = calculateIK(position);

    if (result.success) {
        // 更新状态
        updateInternalState(result);
        ESP_LOGD(TAG, "Foot position set for %s: (%.3f, %.3f, %.3f)",
                 getLegName(), position.x, position.y, position.z);
        return true;
    } else {
        ESP_LOGW(TAG, "Failed to set foot position for %s: %s",
                 getLegName(), result.error_message);
        return false;
    }
}

bool Leg::setJointAngles(const ThreeJointAngles& angles) {
    // 执行正向运动学验证角度有效性
    KinematicsResult result = calculateFK(angles);

    if (result.success) {
        // 更新状态
        current_joint_angles_ = angles;
        current_foot_position_ = result.foot_position;
        is_valid_ = true;

        ESP_LOGI(TAG, "[setJointAngles] 关节角度给  %s: 设置为 :(%.1f°, %.1f°, %.1f°)",
                 getLegName(),
                 angles.hip_side * 180.0f / M_PI,
                 angles.hip_pitch * 180.0f / M_PI,
                 angles.knee_pitch * 180.0f / M_PI);
        ESP_LOGI(TAG, "[setJointAngles] 足端位置为:(%.3f, %.3f, %.3f)m",
                 current_foot_position_.x,
                 current_foot_position_.y,
                 current_foot_position_.z);
        return true;
    } else {
        ESP_LOGW(TAG, "Invalid joint angles for %s: %s",
                 getLegName(), result.error_message);
        is_valid_ = false;
        return false;
    }
}

CoordinateTransform::Vector3 Leg::getHipOffset() const {
    auto hip_offset = geometry_.getHipOffset(leg_type_);
    return Vector3(hip_offset.x, hip_offset.y, hip_offset.z);
}

bool Leg::isPositionReachable(const Vector3& position) const {
    if (!kinematics_solver_) {
        return false;
    }

    // 获取工作空间边界
    auto bounds = kinematics_solver_->getWorkspaceBounds();

    // 增加小的容差来处理浮点数精度问题
    constexpr float TOLERANCE = 0.001f; // 1mm容差

    // 检查是否在边界内（带容差）
    bool within_bounds = (position.x >= bounds.min_x - TOLERANCE && position.x <= bounds.max_x + TOLERANCE) &&
                        (position.y >= bounds.min_y - TOLERANCE && position.y <= bounds.max_y + TOLERANCE) &&
                        (position.z >= bounds.min_z - TOLERANCE && position.z <= bounds.max_z + TOLERANCE);

    if (!within_bounds) {
        ESP_LOGD(TAG, "Position out of bounds for %s: (%.3f,%.3f,%.3f), bounds: X[%.3f,%.3f] Y[%.3f,%.3f] Z[%.3f,%.3f]",
                 getLegName(), position.x, position.y, position.z,
                 bounds.min_x, bounds.max_x, bounds.min_y, bounds.max_y, bounds.min_z, bounds.max_z);
        return false;
    }

    // 尝试逆运动学计算来验证可达性
    KinematicsResult result = kinematics_solver_->inverseKinematics(position);
    return result.success;
}

const char* Leg::getLegName() const {
    return SpotMicroGeometry::getLegName(leg_type_);
}

void Leg::resetToDefault() {
    ThreeJointAngles default_angles(DEFAULT_HIP_SIDE, DEFAULT_HIP_PITCH, DEFAULT_KNEE_PITCH);

    // 首先尝试使用默认关节角度
    if (setJointAngles(default_angles)) {
        ESP_LOGI(TAG, "Reset %s to default joint angles", getLegName());
        return;
    }
    else {
        ESP_LOGW(TAG, "为 %s 设置默认关节角度失败", getLegName());
    }

    // 如果默认关节角度失败，尝试使用默认足端位置
    Vector3 default_position(DEFAULT_FOOT_X, DEFAULT_FOOT_Y, DEFAULT_FOOT_Z);
    if (setFootPosition(default_position)) {
        ESP_LOGI(TAG, "Reset %s to default foot position", getLegName());
        return;
    }

    // 如果都失败，手动设置一个安全状态
    current_joint_angles_ = default_angles;
    current_foot_position_ = default_position;
    is_valid_ = false;

    ESP_LOGW(TAG, "Failed to reset %s to valid default state", getLegName());
}

SpotLegKinematics::WorkspaceBounds Leg::getWorkspaceBounds() const {
    if (!kinematics_solver_) {
        return SpotLegKinematics::WorkspaceBounds{};
    }
    return kinematics_solver_->getWorkspaceBounds();
}

void Leg::printDebugInfo() const {
    ESP_LOGI(TAG, "=== %s Debug Info ===", getLegName());
    ESP_LOGI(TAG, "Valid: %s", is_valid_ ? "Yes" : "No");

    // 髋关节偏移
    Vector3 hip_offset = getHipOffset();
    ESP_LOGI(TAG, "Hip Offset: (%.3f, %.3f, %.3f) m",
             hip_offset.x, hip_offset.y, hip_offset.z);

    // 关节角度
    ESP_LOGI(TAG, "Joint Angles: (%.1f°, %.1f°, %.1f°)",
             current_joint_angles_.hip_side * 180.0f / M_PI,
             current_joint_angles_.hip_pitch * 180.0f / M_PI,
             current_joint_angles_.knee_pitch * 180.0f / M_PI);

    // 足端位置
    ESP_LOGI(TAG, "Foot Position: (%.3f, %.3f, %.3f) m",
             current_foot_position_.x, current_foot_position_.y, current_foot_position_.z);

    // 工作空间信息
    if (kinematics_solver_) {
        auto bounds = kinematics_solver_->getWorkspaceBounds();
        ESP_LOGI(TAG, "Workspace: X[%.3f,%.3f] Y[%.3f,%.3f] Z[%.3f,%.3f]",
                 bounds.min_x, bounds.max_x, bounds.min_y, bounds.max_y, bounds.min_z, bounds.max_z);
    }
}

void Leg::updateInternalState(const KinematicsResult& result) {
    if (result.success) {
        current_joint_angles_ = result.joint_angles;
        current_foot_position_ = result.foot_position;
        is_valid_ = true;
    } else {
        is_valid_ = false;
    }
}

bool Leg::validateConsistency() const {
    if (!is_valid_ || !kinematics_solver_) {
        return false;
    }

    // 使用正向运动学验证一致性
    Vector3 hip_offset = getHipOffset();
    KinematicsResult fk_result = kinematics_solver_->forwardKinematics(hip_offset, current_joint_angles_);

    if (!fk_result.success) {
        return false;
    }

    // 检查位置差异是否在可接受范围内
    constexpr float TOLERANCE = 0.001f; // 1mm容差
    float diff_x = std::abs(fk_result.foot_position.x - current_foot_position_.x);
    float diff_y = std::abs(fk_result.foot_position.y - current_foot_position_.y);
    float diff_z = std::abs(fk_result.foot_position.z - current_foot_position_.z);

    return (diff_x < TOLERANCE && diff_y < TOLERANCE && diff_z < TOLERANCE);
}

} // namespace Kinematics
} // namespace Robot