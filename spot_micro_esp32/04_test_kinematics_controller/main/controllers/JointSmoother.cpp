/**
 * @file JointSmoother.cpp
 * @brief 关节平滑器实现
 *
 * 实现关节层面的平滑运动控制，处理运动学语义的关节角度
 * 通过JointController进行角度转换和舵机控制
 */

#include "JointSmoother.hpp"
#include "JointController.hpp"
#include <cmath>
#include <algorithm>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace Robot {
namespace Controllers {

const char* JointSmoother::TAG = "JointSmoother";

JointSmoother::JointSmoother(std::shared_ptr<JointController> joint_controller,
                             std::shared_ptr<Robot::Config::ConfigManager> config_manager)
    : joint_controller_(joint_controller), config_manager_(config_manager), config_(), is_smoothing_(false), total_steps_(0) {
    ESP_LOGI(TAG, "JointSmoother initialized with default config");
}

JointSmoother::JointSmoother(std::shared_ptr<JointController> joint_controller,
                             std::shared_ptr<Robot::Config::ConfigManager> config_manager,
                             const ServoSmoothConfig& config)
    : joint_controller_(joint_controller), config_manager_(config_manager), config_(config), is_smoothing_(false), total_steps_(0) {
    ESP_LOGI(TAG, "JointSmoother initialized with custom config");
    ESP_LOGI(TAG, "Angle step: %.2f degrees, Step delay: %d ms",
             config_.angle_step, config_.step_delay_ms);
}

JointSmoother::~JointSmoother() {
    ESP_LOGI(TAG, "JointSmoother destroyed");
}

bool JointSmoother::initialize() {
    if (!joint_controller_) {
        ESP_LOGE(TAG, "JointController is null, cannot initialize");
        return false;
    }

    if (!joint_controller_->isInitialized()) {
        ESP_LOGE(TAG, "JointController is not initialized");
        return false;
    }

    // 初始化所有关节状态为中性位置
    for (int leg = 0; leg < Robot::Constants::LEG_COUNT; leg++) {
        for (int joint = 0; joint < Robot::Constants::JOINTS_PER_LEG; joint++) {
            joint_states_[leg][joint].current_angle = 0.0f;        // 关节零位（弧度）
            joint_states_[leg][joint].target_angle = 0.0f;         // 目标角度初始化为零位
            joint_states_[leg][joint].is_moving = false;           // 初始不运动
            joint_states_[leg][joint].is_valid = true;             // 标记为有效
        }
    }

    ESP_LOGI(TAG, "JointSmoother initialized successfully");
    return true;
}

bool JointSmoother::isValidLegId(Robot::LegID leg_id) const {
    int leg_id_int = static_cast<int>(leg_id);
    return (leg_id_int < Robot::Constants::LEG_COUNT);
}

bool JointSmoother::isValidJointType(Robot::JointType joint_type) const {
    return (joint_type == Robot::JointType::HIP_ROLL ||
            joint_type == Robot::JointType::HIP_PITCH ||
            joint_type == Robot::JointType::KNEE_PITCH);
}

bool JointSmoother::isValidJointAngle(Robot::JointType joint_type, float angle) const {
    switch (joint_type) {
        case Robot::JointType::HIP_ROLL:
            return (angle >= Robot::Constants::HIP_ROLL_MIN * M_PI / 180.0f &&
                    angle <= Robot::Constants::HIP_ROLL_MAX * M_PI / 180.0f);
        case Robot::JointType::HIP_PITCH:
            return (angle >= Robot::Constants::HIP_PITCH_MIN * M_PI / 180.0f &&
                    angle <= Robot::Constants::HIP_PITCH_MAX * M_PI / 180.0f);
        case Robot::JointType::KNEE_PITCH:
            return (angle >= Robot::Constants::KNEE_PITCH_MIN * M_PI / 180.0f &&
                    angle <= Robot::Constants::KNEE_PITCH_MAX * M_PI / 180.0f);
        default:
            return false;
    }
}

JointState& JointSmoother::getJointState(Robot::LegID leg_id, Robot::JointType joint_type) {
    return joint_states_[static_cast<int>(leg_id)][static_cast<int>(joint_type)];
}

const JointState& JointSmoother::getJointState(Robot::LegID leg_id, Robot::JointType joint_type) const {
    return joint_states_[static_cast<int>(leg_id)][static_cast<int>(joint_type)];
}

float JointSmoother::calculateJointStep(Robot::LegID leg_id, Robot::JointType joint_type, float angle_diff) const {
    if (!isValidLegId(leg_id) || !isValidJointType(joint_type)) return 0.0f;

    // 将配置中的度转换为弧度
    float step_rad = config_.angle_step * M_PI / 180.0f;

    if (std::abs(angle_diff) < step_rad) {
        return angle_diff;  // 直接到达目标
    } else {
        return (angle_diff < 0) ? -step_rad : step_rad;  // 按步长移动
    }
}

bool JointSmoother::hasAllJointsReachedTarget() const {
    float threshold = 0.5f * M_PI / 180.0f; // 0.5度阈值
    for (int leg = 0; leg < Robot::Constants::LEG_COUNT; leg++) {
        for (int joint = 0; joint < Robot::Constants::JOINTS_PER_LEG; joint++) {
            const JointState& state = joint_states_[leg][joint];
            if (state.is_valid && std::abs(state.current_angle - state.target_angle) > threshold) {
                return false;
            }
        }
    }
    return true;
}

bool JointSmoother::setJointAngle(Robot::LegID leg_id, Robot::JointType joint_type, float angle) {
    if (!isValidLegId(leg_id)) {
        ESP_LOGE(TAG, "Invalid leg ID: %d", static_cast<int>(leg_id));
        return false;
    }
    if (!isValidJointType(joint_type)) {
        ESP_LOGE(TAG, "Invalid joint type: %d", static_cast<int>(joint_type));
        return false;
    }
    if (!isValidJointAngle(joint_type, angle)) {
        ESP_LOGE(TAG, "Invalid joint angle: %.2f for leg %d joint %d",
                 angle * 180.0f / M_PI, static_cast<int>(leg_id), static_cast<int>(joint_type));
        return false;
    }
    JointState& state = getJointState(leg_id, joint_type);
    state.target_angle = angle;
    state.is_valid = true;
    state.is_moving = true;
    ESP_LOGD(TAG, "Joint target set: leg %d, joint %d, target %.2f°",
             static_cast<int>(leg_id), static_cast<int>(joint_type), angle * 180.0f / M_PI);
    return true;
}

bool JointSmoother::setLegJointAngles(Robot::LegID leg_id, const ThreeJointAngles& angles) {
    bool success = true;
    success &= setJointAngle(leg_id, Robot::JointType::HIP_ROLL, angles.hip_side);
    success &= setJointAngle(leg_id, Robot::JointType::HIP_PITCH, angles.hip_pitch);
    success &= setJointAngle(leg_id, Robot::JointType::KNEE_PITCH, angles.knee_pitch);
    return success;
}

bool JointSmoother::setAllJointAngles(const AllLegJoints& all_joints) {
    bool success = true;
    for (int leg = 0; leg < Robot::Constants::LEG_COUNT; leg++) {
        for (int joint = 0; joint < Robot::Constants::JOINTS_PER_LEG; joint++) {
            if (all_joints[leg][joint].isValid) {
                // 直接设置目标角度到JointState，避免重复验证
                JointState& state = joint_states_[leg][joint];
                state.target_angle = all_joints[leg][joint].angle;
                state.is_valid = true;
                state.is_moving = (std::abs(state.current_angle - state.target_angle) > 0.5f * M_PI / 180.0f);

                ESP_LOGD(TAG, "Set joint L%dJ%d target: %.2f° (current: %.2f°)",
                         leg, joint,
                         state.target_angle * 180.0f / M_PI,
                         state.current_angle * 180.0f / M_PI);
            }
        }
    }
    return success;
}

bool JointSmoother::setLegAnglesFromKinematics(Robot::LegID leg_id, float hip_side, float hip_pitch, float knee_pitch) {
    ThreeJointAngles angles(hip_side, hip_pitch, knee_pitch);
    return setLegJointAngles(leg_id, angles);
}

bool JointSmoother::stepJointSmoothing() {
    if (!config_.enable_smoothing) {
        return false;  // 禁用平滑时不执行
    }

    bool goal_reached = true;
    float step_rad = config_.angle_step * M_PI / 180.0f;

    for (int leg = 0; leg < Robot::Constants::LEG_COUNT; leg++) {
        for (int joint = 0; joint < Robot::Constants::JOINTS_PER_LEG; joint++) {
            JointState& state = joint_states_[leg][joint];
            if (state.is_valid) {
                float diff = state.target_angle - state.current_angle;
                if (std::abs(diff) > step_rad) {
                    state.current_angle += (diff > 0 ? step_rad : -step_rad);
                    state.is_moving = true;
                    goal_reached = false;
                } else if (std::abs(diff) > 0.5f * M_PI / 180.0f) {
                    state.current_angle = state.target_angle;
                    state.is_moving = false;
                } else {
                    state.is_moving = false;
                }

                // 通过JointController设置关节角度（弧度转度）
                float joint_angle_deg = state.current_angle * 180.0f / M_PI;
                JointError result = joint_controller_->setJointAngle(
                    static_cast<Robot::LegID>(leg),
                    static_cast<Robot::JointType>(joint),
                    joint_angle_deg
                );
                
                if (result != JointError::SUCCESS) {
                    ESP_LOGW(TAG, "Failed to set joint L%dJ%d to %.2f°: %s",
                             leg, joint, joint_angle_deg, JointController::getErrorString(result));
                }
            }
        }
    }

    total_steps_++;
    is_smoothing_ = !goal_reached;

    if (goal_reached) {
        ESP_LOGI(TAG, "All joints reached target in %d steps", total_steps_);
    } else {
        ESP_LOGD(TAG, "Joint smoothing step %d completed", total_steps_);
    }
    return !goal_reached;
}

bool JointSmoother::smoothToTarget() {
    if (!config_.enable_smoothing) {
        return jumpToTarget();  // 禁用平滑时直接跳转
    }

    is_smoothing_ = true;
    total_steps_ = 0;

    ESP_LOGI(TAG, "Starting smooth joint motion to target");

    do {
        if (total_steps_ >= config_.max_steps) {
            ESP_LOGW(TAG, "Maximum smoothing steps reached, forcing completion");
            jumpToTarget();
            break;
        }

        // 延时
        if (config_.step_delay_ms > 0) {
            vTaskDelay(config_.step_delay_ms / portTICK_PERIOD_MS);
        }

    } while (stepJointSmoothing());

    is_smoothing_ = false;
    ESP_LOGI(TAG, "Smooth joint motion completed");
    return true;
}

bool JointSmoother::jumpToTarget() {
    bool success = true;
    for (int leg = 0; leg < Robot::Constants::LEG_COUNT; leg++) {
        for (int joint = 0; joint < Robot::Constants::JOINTS_PER_LEG; joint++) {
            JointState& state = joint_states_[leg][joint];
            if (state.is_valid) {
                state.current_angle = state.target_angle;
                state.is_moving = false;
                
                // 通过JointController设置关节角度（弧度转度）
                float joint_angle_deg = state.current_angle * 180.0f / M_PI;
                JointError result = joint_controller_->setJointAngle(
                    static_cast<Robot::LegID>(leg),
                    static_cast<Robot::JointType>(joint),
                    joint_angle_deg
                );
                
                if (result != JointError::SUCCESS) {
                    ESP_LOGW(TAG, "Failed to set joint L%dJ%d to %.2f°: %s",
                             leg, joint, joint_angle_deg, JointController::getErrorString(result));
                    success = false;
                }
            }
        }
    }
    is_smoothing_ = false;
    ESP_LOGI(TAG, "Jumped directly to target joint positions");
    return success;
}

void JointSmoother::stopSmoothing() {
    is_smoothing_ = false;
    ESP_LOGI(TAG, "Joint smoothing stopped");
}

float JointSmoother::getCurrentJointAngle(Robot::LegID leg_id, Robot::JointType joint_type) const {
    if (!isValidLegId(leg_id) || !isValidJointType(joint_type)) return 0.0f;
    return joint_states_[static_cast<int>(leg_id)][static_cast<int>(joint_type)].current_angle;
}

float JointSmoother::getTargetJointAngle(Robot::LegID leg_id, Robot::JointType joint_type) const {
    if (!isValidLegId(leg_id) || !isValidJointType(joint_type)) return 0.0f;
    return joint_states_[static_cast<int>(leg_id)][static_cast<int>(joint_type)].target_angle;
}

ThreeJointAngles JointSmoother::getLegCurrentAngles(Robot::LegID leg_id) const {
    if (!isValidLegId(leg_id)) return ThreeJointAngles();

    const auto& leg_joints = joint_states_[static_cast<int>(leg_id)];
    return ThreeJointAngles(leg_joints[0].current_angle, leg_joints[1].current_angle, leg_joints[2].current_angle);
}

ThreeJointAngles JointSmoother::getLegTargetAngles(Robot::LegID leg_id) const {
    if (!isValidLegId(leg_id)) return ThreeJointAngles();

    const auto& leg_joints = joint_states_[static_cast<int>(leg_id)];
    return ThreeJointAngles(leg_joints[0].target_angle, leg_joints[1].target_angle, leg_joints[2].target_angle);
}

AllLegJoints JointSmoother::getAllCurrentJoints() const {
    AllLegJoints result;
    for (int leg = 0; leg < Robot::Constants::LEG_COUNT; leg++) {
        for (int joint = 0; joint < Robot::Constants::JOINTS_PER_LEG; joint++) {
            const JointState& state = joint_states_[leg][joint];
            result[leg][joint] = JointAngle(state.current_angle, state.is_valid);
        }
    }
    return result;
}

AllLegJoints JointSmoother::getAllTargetJoints() const {
    AllLegJoints result;
    for (int leg = 0; leg < Robot::Constants::LEG_COUNT; leg++) {
        for (int joint = 0; joint < Robot::Constants::JOINTS_PER_LEG; joint++) {
            const JointState& state = joint_states_[leg][joint];
            result[leg][joint] = JointAngle(state.target_angle, state.is_valid);
        }
    }
    return result;
}

bool JointSmoother::isJointMoving(Robot::LegID leg_id, Robot::JointType joint_type) const {
    if (!isValidLegId(leg_id) || !isValidJointType(joint_type)) return false;
    return joint_states_[static_cast<int>(leg_id)][static_cast<int>(joint_type)].is_moving;
}

bool JointSmoother::isLegMoving(Robot::LegID leg_id) const {
    if (!isValidLegId(leg_id)) return false;

    for (int joint = 0; joint < Robot::Constants::JOINTS_PER_LEG; joint++) {
        if (isJointMoving(leg_id, static_cast<Robot::JointType>(joint))) {
            return true;
        }
    }
    return false;
}

void JointSmoother::setConfig(const ServoSmoothConfig& config) {
    config_ = config;
    ESP_LOGI(TAG, "Configuration updated - Angle step: %.2f, Delay: %d ms",
             config_.angle_step, config_.step_delay_ms);
}

void JointSmoother::enableSmoothing(bool enable) {
    config_.enable_smoothing = enable;
    ESP_LOGI(TAG, "Joint smoothing %s", enable ? "enabled" : "disabled");
}

void JointSmoother::reset() {
    stopSmoothing();
    for (auto& leg : joint_states_) {
        for (auto& joint : leg) {
            joint.current_angle = 0.0f;
            joint.target_angle = 0.0f;
            joint.is_moving = false;
            joint.is_valid = false;
        }
    }
    total_steps_ = 0;
    ESP_LOGI(TAG, "JointSmoother reset");
}

float JointSmoother::getSmoothingProgress() const {
    if (!is_smoothing_ || config_.max_steps == 0) return 1.0f;
    return std::min(1.0f, static_cast<float>(total_steps_) / config_.max_steps);
}

void JointSmoother::printStatus() const {
    ESP_LOGI(TAG, "=== JointSmoother Status ===");
    ESP_LOGI(TAG, "Smoothing: %s, Steps: %d/%d",
             is_smoothing_ ? "ACTIVE" : "INACTIVE", total_steps_, config_.max_steps);
    ESP_LOGI(TAG, "Config: step=%.2f°, delay=%dms, enabled=%s",
             config_.angle_step, config_.step_delay_ms,
             config_.enable_smoothing ? "YES" : "NO");
    ESP_LOGI(TAG, "Progress: %.1f%%", getSmoothingProgress() * 100.0f);
    ESP_LOGI(TAG, "=======================");
}

void JointSmoother::printAllJointStates() const {
    char states_str[512] = {0};
    int offset = 0;

    for (int leg = 0; leg < Robot::Constants::LEG_COUNT && offset < 480; leg++) {
        offset += snprintf(states_str + offset, 480 - offset, "L%d[", leg);
        for (int joint = 0; joint < Robot::Constants::JOINTS_PER_LEG && offset < 480; joint++) {
            const JointState& state = joint_states_[leg][joint];
            if (state.is_valid) {
                offset += snprintf(states_str + offset, 480 - offset,
                                  "J%d:%.1f°->%.1f°%s ", joint,
                                  state.current_angle * 180.0f / M_PI,
                                  state.target_angle * 180.0f / M_PI,
                                  state.is_moving ? "*" : "");
            }
        }
        offset += snprintf(states_str + offset, 480 - offset, "] ");
    }
    ESP_LOGI(TAG, "Joints: %s", states_str);
}

} // namespace Controllers
} // namespace Robot