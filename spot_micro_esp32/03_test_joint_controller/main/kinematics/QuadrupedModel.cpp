#include "QuadrupedModel.hpp"
#include "../utils/cpp11_compatibility.hpp"
#include <esp_log.h>
#include <cmath>

namespace Robot {
namespace Kinematics {

static const char* TAG = "QuadrupedModel";

// 预设姿势的足端位置定义（腿部坐标系，米）
namespace PresetPositions {
    inline CoordinateTransform::Vector3 getStandPosition() {
        return CoordinateTransform::Vector3{0.10f, 0.0f, -0.12f};  // 调整为更合理的站立位置
    }
    inline CoordinateTransform::Vector3 getSitFrontPosition() {
        return CoordinateTransform::Vector3{0.12f, 0.0f, -0.08f};
    }
    inline CoordinateTransform::Vector3 getSitBackPosition() {
        return CoordinateTransform::Vector3{0.08f, 0.0f, -0.16f};  // 后腿坐下时稍低
    }
    inline CoordinateTransform::Vector3 getCrouchPosition() {
        return CoordinateTransform::Vector3{0.08f, 0.0f, -0.20f};  // 蹲下位置更低
    }

    // 前倾姿势参数
    static constexpr float TILT_PITCH = 0.175f; // 10度前倾（弧度）
}

QuadrupedModel::QuadrupedModel(const Robot::Config::GeometryConfig& geometry_config)
    : geometry_config_(geometry_config) {
    ESP_LOGI(TAG, "Initializing QuadrupedModel with geometry config");

    initializeLegs();
    resetToDefault();

    ESP_LOGI(TAG, "QuadrupedModel initialized successfully");
}

bool QuadrupedModel::updateBodyPose(const Vector3& position, const Vector3& orientation) {
    Pose6DOF pose(position.x, position.y, position.z,
                  orientation.x, orientation.y, orientation.z);
    return updateBodyPose(pose);
}

bool QuadrupedModel::updateBodyPose(const Pose6DOF& pose) {
    ESP_LOGI(TAG, "Updating body pose: pos(%.3f,%.3f,%.3f) rot(%.1f°,%.1f°,%.1f°)",
             pose.position.x, pose.position.y, pose.position.z,
             pose.orientation.x * 180.0f / M_PI,
             pose.orientation.y * 180.0f / M_PI,
             pose.orientation.z * 180.0f / M_PI);

    // 步骤1：保存当前脚部在世界坐标系中的位置
    updateCachedFootWorldPositions();

    // 步骤2：设置新的机身位姿
    frame_manager_.setBodyPose(pose);

    // 步骤3：基于脚部世界坐标重新计算关节角度
    bool success = recalculateJointAnglesFromWorldPositions();

    if (success) {
        ESP_LOGI(TAG, "Body pose updated successfully");
    } else {
        ESP_LOGW(TAG, "Body pose update completed with some leg calculation failures");
    }

    return success;
}

bool QuadrupedModel::setLegPosition(int leg_id, const Vector3& position) {
    if (!isValidLegId(leg_id)) {
        ESP_LOGE(TAG, "Invalid leg ID: %d", leg_id);
        return false;
    }

    Leg* leg = getLeg(leg_id);
    if (!leg) {
        ESP_LOGE(TAG, "Leg %d not initialized", leg_id);
        return false;
    }

    bool success = leg->setFootPosition(position);
    if (success) {
        ESP_LOGD(TAG, "Leg %d position set to (%.3f,%.3f,%.3f)",
                 leg_id, position.x, position.y, position.z);
    } else {
        ESP_LOGW(TAG, "Failed to set leg %d position", leg_id);
    }

    return success;
}

std::array<ThreeJointAngles, 4> QuadrupedModel::getAllJointAngles() const {
    std::array<ThreeJointAngles, 4> all_angles;

    for (int i = 0; i < 4; i++) {
        const Leg* leg = getLeg(i);
        if (leg && leg->isValid()) {
            all_angles[i] = leg->getJointAngles();
        } else {
            // 如果腿部无效，返回零角度
            all_angles[i] = ThreeJointAngles(0.0f, 0.0f, 0.0f);
        }
    }

    return all_angles;
}

ThreeJointAngles QuadrupedModel::getLegJointAngles(int leg_id) const {
    if (!isValidLegId(leg_id)) {
        ESP_LOGE(TAG, "Invalid leg ID: %d", leg_id);
        return ThreeJointAngles(0.0f, 0.0f, 0.0f);
    }

    const Leg* leg = getLeg(leg_id);
    if (leg && leg->isValid()) {
        return leg->getJointAngles();
    } else {
        return ThreeJointAngles(0.0f, 0.0f, 0.0f);
    }
}

bool QuadrupedModel::executePresetPose(PresetPoseType pose_type) {
    switch (pose_type) {
        case PresetPoseType::STAND:
            return executeStandPose();
        case PresetPoseType::SIT:
            return executeSitPose();
        case PresetPoseType::CROUCH:
            return executeCrouchPose();
        case PresetPoseType::TILT:
            return executeTiltPose();
        default:
            ESP_LOGE(TAG, "Unknown preset pose type");
            return false;
    }
}

bool QuadrupedModel::setInitialStateFromAngles(const float all_joint_angles[12]) {
    if (!all_joint_angles) {
        ESP_LOGE(TAG, "Invalid joint angles array pointer");
        return false;
    }

    ESP_LOGI(TAG, "Setting initial state from joint angles (rad)");

    int success_count = 0;

    // 遍历四条腿，每条腿3个关节
    for (int leg_id = 0; leg_id < 4; leg_id++) {
        Leg* leg = getLeg(leg_id);
        if (!leg) {
            ESP_LOGE(TAG, "Leg %d not initialized", leg_id);
            continue;
        }

        // 提取当前腿的3个关节角度
        int base_index = leg_id * 3;
        float leg_angles[3] = {
            all_joint_angles[base_index + 0],     // hip_side
            all_joint_angles[base_index + 1],     // hip_pitch
            all_joint_angles[base_index + 2]      // knee_pitch
        };

        // 设置腿部关节角度
        if (leg->setJointAngles(leg_angles)) {
            success_count++;
            ESP_LOGD(TAG, "Leg %d angles set: (%.2f, %.2f, %.2f) rad",
                     leg_id, leg_angles[0], leg_angles[1], leg_angles[2]);
        } else {
            ESP_LOGW(TAG, "Failed to set angles for leg %d", leg_id);
        }
    }

    // 更新足端世界坐标缓存
    updateFootWorldPositionsCache();

    bool all_success = (success_count == 4);
    ESP_LOGI(TAG, "Initial state set: %d/4 legs successful", success_count);

    return all_success;
}

bool QuadrupedModel::setInitialStateFromAnglesDegrees(const float all_joint_angles_deg[12]) {
    if (!all_joint_angles_deg) {
        ESP_LOGE(TAG, "Invalid joint angles array pointer");
        return false;
    }

    ESP_LOGI(TAG, "Setting initial state from joint angles (degrees)");

    // 转换度数为弧度
    float all_joint_angles_rad[12];
    for (int i = 0; i < 12; i++) {
        all_joint_angles_rad[i] = all_joint_angles_deg[i] * M_PI / 180.0f;
    }

    return setInitialStateFromAngles(all_joint_angles_rad);
}

QuadrupedModel::Vector3 QuadrupedModel::getFootPositionInWorld(int leg_index) const {
    if (!isValidLegId(leg_index)) {
        ESP_LOGE(TAG, "Invalid leg index: %d", leg_index);
        return Vector3(0, 0, 0);
    }

    const Leg* leg = getLeg(leg_index);
    if (!leg || !leg->isValid()) {
        ESP_LOGW(TAG, "Leg %d not valid or not initialized", leg_index);
        return Vector3(0, 0, 0);
    }

    // 获取足端在腿部坐标系中的位置
    Vector3 foot_leg_frame = leg->getFootPositionInLegFrame();

    // 转换到世界坐标系
    Vector3 foot_world = transformHipToWorld(leg_index, foot_leg_frame);

    ESP_LOGD(TAG, "Leg %d foot position in world: (%.3f, %.3f, %.3f)",
             leg_index, foot_world.x, foot_world.y, foot_world.z);

    return foot_world;
}

std::array<QuadrupedModel::Vector3, 4> QuadrupedModel::getAllFootPositionsInWorld() const {
    std::array<Vector3, 4> positions;

    for (int i = 0; i < 4; i++) {
        positions[i] = getFootPositionInWorld(i);
    }

    return positions;
}

void QuadrupedModel::updateFootWorldPositionsCache() {
    ESP_LOGD(TAG, "Updating foot world positions cache");

    for (int i = 0; i < 4; i++) {
        cached_foot_world_positions_[i] = getFootPositionInWorld(i);
    }

    ESP_LOGD(TAG, "Foot world positions cache updated");
}

void QuadrupedModel::printCurrentState() const {
    ESP_LOGI(TAG, "\n=== QuadrupedModel Current State ===");

    // 机身位姿
    Pose6DOF pose = frame_manager_.getBodyPose();
    ESP_LOGI(TAG, "Body Pose:");
    ESP_LOGI(TAG, "  Position: (%.3f, %.3f, %.3f) m",
             pose.position.x, pose.position.y, pose.position.z);
    ESP_LOGI(TAG, "  Orientation: (%.1f°, %.1f°, %.1f°)",
             pose.orientation.x * 180.0f / M_PI,
             pose.orientation.y * 180.0f / M_PI,
             pose.orientation.z * 180.0f / M_PI);

    // 各腿详细状态
    ESP_LOGI(TAG, "\nLeg States:");
    for (int i = 0; i < 4; i++) {
        const Leg* leg = getLeg(i);
        ESP_LOGI(TAG, "  %s:", getLegName(i));

        if (leg && leg->isValid()) {
            // 关节角度
            auto angles = leg->getJointAngles();
            ESP_LOGI(TAG, "    Joint Angles: (%.1f°, %.1f°, %.1f°)",
                     angles.hip_side * 180.0f / M_PI,
                     angles.hip_pitch * 180.0f / M_PI,
                     angles.knee_pitch * 180.0f / M_PI);

            // 足端位置（腿部坐标系）
            auto foot_leg = leg->getFootPositionInLegFrame();
            ESP_LOGI(TAG, "    Foot (Leg Frame): (%.3f, %.3f, %.3f) m",
                     foot_leg.x, foot_leg.y, foot_leg.z);

            // 足端位置（世界坐标系）
            auto foot_world = getFootPositionInWorld(i);
            ESP_LOGI(TAG, "    Foot (World Frame): (%.3f, %.3f, %.3f) m",
                     foot_world.x, foot_world.y, foot_world.z);
        } else {
            ESP_LOGI(TAG, "    Status: Invalid");
        }
    }

    ESP_LOGI(TAG, "====================================\n");
}

QuadrupedModel::RobotState QuadrupedModel::getCurrentState() const {
    RobotState state;

    state.body_pose = frame_manager_.getBodyPose();

    for (int i = 0; i < 4; i++) {
        const Leg* leg = getLeg(i);
        if (leg) {
            state.joint_angles[i] = leg->getJointAngles();
            state.foot_positions_in_leg_frame[i] = leg->getFootPosition();
            state.leg_validity[i] = leg->isValid();

            // 计算足端的世界坐标
            if (leg->isValid()) {
                state.foot_positions_in_world[i] = transformHipToWorld(i, leg->getFootPosition());
            } else {
                state.foot_positions_in_world[i] = Vector3(0, 0, 0);
            }
        } else {
            state.joint_angles[i] = ThreeJointAngles(0, 0, 0);
            state.foot_positions_in_leg_frame[i] = Vector3(0, 0, 0);
            state.foot_positions_in_world[i] = Vector3(0, 0, 0);
            state.leg_validity[i] = false;
        }
    }

    return state;
}

bool QuadrupedModel::isValidConfiguration() const {
    for (int i = 0; i < 4; i++) {
        const Leg* leg = getLeg(i);
        if (!leg || !leg->isValid()) {
            return false;
        }
    }
    return true;
}

void QuadrupedModel::resetToDefault() {
    ESP_LOGI(TAG, "Resetting to default configuration");

    // 重置机身位姿为原点，高度调整为与默认足端位置匹配
    Pose6DOF default_pose(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // 高度0cm
    frame_manager_.setBodyPose(default_pose);

    // 重置所有腿到默认状态
    for (int i = 0; i < 4; i++) {
        Leg* leg = getLeg(i);
        if (leg) {
            leg->resetToDefault();
        }
    }

    ESP_LOGI(TAG, "Reset to default configuration completed");
}

Leg* QuadrupedModel::getLeg(int leg_id) {
    if (!isValidLegId(leg_id)) {
        return nullptr;
    }
    return legs_[leg_id].get();
}

const Leg* QuadrupedModel::getLeg(int leg_id) const {
    if (!isValidLegId(leg_id)) {
        return nullptr;
    }
    return legs_[leg_id].get();
}

QuadrupedModel::Pose6DOF QuadrupedModel::getBodyPose() const {
    return frame_manager_.getBodyPose();
}

void QuadrupedModel::printDebugInfo() const {
    ESP_LOGI(TAG, "=== QuadrupedModel Debug Info ===");

    // 机身位姿
    Pose6DOF pose = frame_manager_.getBodyPose();
    ESP_LOGI(TAG, "Body Pose: pos(%.3f,%.3f,%.3f) rot(%.1f°,%.1f°,%.1f°)",
             pose.position.x, pose.position.y, pose.position.z,
             pose.orientation.x * 180.0f / M_PI,
             pose.orientation.y * 180.0f / M_PI,
             pose.orientation.z * 180.0f / M_PI);

    // 各腿状态
    ESP_LOGI(TAG, "Leg States:");
    for (int i = 0; i < 4; i++) {
        const Leg* leg = getLeg(i);
        if (leg) {
            ESP_LOGI(TAG, "  %s: %s", getLegName(i), leg->isValid() ? "Valid" : "Invalid");
            if (leg->isValid()) {
                auto angles = leg->getJointAngles();
                auto pos = leg->getFootPosition();
                ESP_LOGI(TAG, "    Angles: (%.1f°,%.1f°,%.1f°)",
                         angles.hip_side * 180.0f / M_PI,
                         angles.hip_pitch * 180.0f / M_PI,
                         angles.knee_pitch * 180.0f / M_PI);
                ESP_LOGI(TAG, "    Position: (%.3f,%.3f,%.3f)", pos.x, pos.y, pos.z);
            }
        } else {
            ESP_LOGI(TAG, "  %s: Not initialized", getLegName(i));
        }
    }

    ESP_LOGI(TAG, "Valid Configuration: %s", isValidConfiguration() ? "Yes" : "No");
}

void QuadrupedModel::initializeLegs() {
    ESP_LOGI(TAG, "Initializing legs with geometry config");

    // 创建四条腿的实例，传递几何配置
    legs_[0] = std::make_unique<Leg>(LegType::LEFT_FRONT, geometry_config_);
    legs_[1] = std::make_unique<Leg>(LegType::RIGHT_FRONT, geometry_config_);
    legs_[2] = std::make_unique<Leg>(LegType::LEFT_BACK, geometry_config_);
    legs_[3] = std::make_unique<Leg>(LegType::RIGHT_BACK, geometry_config_);

    ESP_LOGI(TAG, "All legs initialized with geometry config");
}

void QuadrupedModel::updateCachedFootWorldPositions() {
    for (int i = 0; i < 4; i++) {
        const Leg* leg = getLeg(i);
        if (leg && leg->isValid()) {
            // 计算足端在世界坐标系中的位置
            cached_foot_world_positions_[i] = transformHipToWorld(i, leg->getFootPosition());
        }
    }

    // ESP_LOGI(TAG, "当前足端在世界坐标系中的位置为:");
    // for (int i = 0; i < 4; i++) {
    //     ESP_LOGI(TAG, "  Leg %d: (%.3f, %.3 f, %.3f) m",
    //              i,
    //              cached_foot_world_positions_[i].x,
    //              cached_foot_world_positions_[i].y,
    //              cached_foot_world_positions_[i].z);
    //     }
}

bool QuadrupedModel::recalculateJointAnglesFromWorldPositions() {
    int success_count = 0;

    for (int i = 0; i < 4; i++) {
        Leg* leg = getLeg(i);
        if (!leg) {
            continue;
        }

        // 将世界坐标转换为髋关节坐标系
        Vector3 foot_hip = transformWorldToHip(i, cached_foot_world_positions_[i]);

        // 设置新的足端位置（会自动计算关节角度）
        if (leg->setFootPosition(foot_hip)) {
            success_count++;
            // ESP_LOGI(TAG, "腿 %d 关节角度重新计算成功", i);
        } else {
            ESP_LOGW(TAG, "Failed to recalculate joint angles for leg %d", i);
        }
    }

    ESP_LOGI(TAG, "Joint angle recalculation: %d/4 legs successful", success_count);
    return success_count == 4;
}

bool QuadrupedModel::executeStandPose() {
    ESP_LOGI(TAG, "Executing stand pose");

    int success_count = 0;
    for (int i = 0; i < 4; i++) {
        if (setLegPosition(i, PresetPositions::getStandPosition())) {
            success_count++;
        }
    }

    bool success = (success_count == 4);
    ESP_LOGI(TAG, "Stand pose: %d/4 legs positioned successfully", success_count);
    return success;
}

bool QuadrupedModel::executeSitPose() {
    ESP_LOGI(TAG, "Executing sit pose");

    int success_count = 0;

    // 前腿（0,1）使用前腿位置
    for (int i = 0; i < 2; i++) {
        if (setLegPosition(i, PresetPositions::getSitFrontPosition())) {
            success_count++;
        }
    }

    // 后腿（2,3）使用后腿位置
    for (int i = 2; i < 4; i++) {
        if (setLegPosition(i, PresetPositions::getSitBackPosition())) {
            success_count++;
        }
    }

    bool success = (success_count == 4);
    ESP_LOGI(TAG, "Sit pose: %d/4 legs positioned successfully", success_count);
    return success;
}

bool QuadrupedModel::executeCrouchPose() {
    ESP_LOGI(TAG, "Executing crouch pose");

    int success_count = 0;
    for (int i = 0; i < 4; i++) {
        if (setLegPosition(i, PresetPositions::getCrouchPosition())) {
            success_count++;
        }
    }

    bool success = (success_count == 4);
    ESP_LOGI(TAG, "Crouch pose: %d/4 legs positioned successfully", success_count);
    return success;
}

bool QuadrupedModel::executeTiltPose() {
    ESP_LOGI(TAG, "Executing tilt pose (10° forward pitch)");

    // 设置机身前倾10度
    Pose6DOF tilt_pose(0.0f, 0.0f, 0.15f, 0.0f, PresetPositions::TILT_PITCH, 0.0f);
    return updateBodyPose(tilt_pose);
}

QuadrupedModel::Vector3 QuadrupedModel::transformHipToWorld(int leg_id, const Vector3& hip_position) const {
    if (!isValidLegId(leg_id)) {
        return Vector3(0, 0, 0);
    }

    const Leg* leg = getLeg(leg_id);
    if (!leg) {
        return Vector3(0, 0, 0);
    }

    // 获取髋关节偏移
    Vector3 hip_offset = leg->getHipOffset();

    // 髋关节坐标系 -> 机体坐标系 -> 世界坐标系
    Vector3 body_position = frame_manager_.transformHipToBody(hip_position, hip_offset);
    return frame_manager_.transformBodyToWorld(body_position);
}

QuadrupedModel::Vector3 QuadrupedModel::transformWorldToHip(int leg_id, const Vector3& world_position) const {
    if (!isValidLegId(leg_id)) {
        return Vector3(0, 0, 0);
    }

    const Leg* leg = getLeg(leg_id);
    if (!leg) {
        return Vector3(0, 0, 0);
    }

    // 获取髋关节偏移
    Vector3 hip_offset = leg->getHipOffset();

    // 世界坐标系 -> 机体坐标系 -> 髋关节坐标系
    Vector3 body_position = frame_manager_.transformWorldToBody(world_position);
    return frame_manager_.transformBodyToHip(body_position, hip_offset);
}

const char* QuadrupedModel::getLegName(int leg_id) {
    static const char* names[] = {"LEFT_FRONT", "RIGHT_FRONT", "LEFT_BACK", "RIGHT_BACK"};
    if (isValidLegId(leg_id)) {
        return names[leg_id];
    }
    return "INVALID";
}

} // namespace Kinematics
} // namespace Robot