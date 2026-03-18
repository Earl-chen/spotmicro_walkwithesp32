/**
 * @file PoseInterpolator.cpp
 * @brief 姿态平滑插值器实现
 *
 * 基于新增功能.txt中的iterate_to_position函数逻辑
 * 实现机体姿态的平滑过渡
 */

#include "PoseInterpolator.hpp"
#include <cmath>
#include <algorithm>
#include <esp_log.h>

namespace Robot {
namespace Controllers {

const char* PoseInterpolator::TAG = "PoseInterpolator";

PoseInterpolator::PoseInterpolator()
    : current_pose_(), target_pose_(), config_(), is_moving_(false), current_step_(0) {
    ESP_LOGI(TAG, "PoseInterpolator initialized with default config");
}

PoseInterpolator::PoseInterpolator(const InterpolationConfig& config)
    : current_pose_(), target_pose_(), config_(config), is_moving_(false), current_step_(0) {
    ESP_LOGI(TAG, "PoseInterpolator initialized with custom config");
    ESP_LOGI(TAG, "Position step: %.2f mm, Angle step: %.4f rad",
             config_.position_step, config_.angle_step);
}

PoseInterpolator::~PoseInterpolator() {
    ESP_LOGI(TAG, "PoseInterpolator destroyed");
}

void PoseInterpolator::setCurrentPose(const BodyPose& pose) {
    current_pose_ = pose;
    ESP_LOGD(TAG, "Current pose set: (%.2f,%.2f,%.2f) - (%.4f,%.4f,%.4f)",
             pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
}

void PoseInterpolator::setTargetPose(const BodyPose& pose) {
    target_pose_ = pose;
    is_moving_ = (current_pose_ != target_pose_);
    current_step_ = 0;

    ESP_LOGI(TAG, "设置机器狗的目标姿态: (位置 %.2f,%.2f,%.2f - 姿态 %.4f,%.4f,%.4f)",
             pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
    ESP_LOGI(TAG, "当前》》机器狗的姿态: (位置 %.2f,%.2f,%.2f - 姿态 %.4f,%.4f,%.4f)",
             current_pose_.x, current_pose_.y, current_pose_.z,
             current_pose_.roll, current_pose_.pitch, current_pose_.yaw);
}

float PoseInterpolator::calculateStep(float current, float target, float max_step) const {
    float diff = target - current;
    if (std::abs(diff) < max_step) {
        return diff;  // 直接到达目标
    } else {
        return (diff < 0) ? -max_step : max_step;  // 按最大步长移动
    }
}

bool PoseInterpolator::hasReachedTarget() const {
    const float position_tolerance = config_.position_step;
    const float angle_tolerance = config_.angle_step;

    return (std::abs(current_pose_.x - target_pose_.x) < position_tolerance &&
            std::abs(current_pose_.y - target_pose_.y) < position_tolerance &&
            std::abs(current_pose_.z - target_pose_.z) < position_tolerance &&
            std::abs(current_pose_.roll - target_pose_.roll) < angle_tolerance &&
            std::abs(current_pose_.pitch - target_pose_.pitch) < angle_tolerance &&
            std::abs(current_pose_.yaw - target_pose_.yaw) < angle_tolerance);
}

bool PoseInterpolator::stepInterpolation() {
    if (!is_moving_) {
        return false;  // 已停止运动
    }

    if (current_step_ >= config_.max_steps) {
        ESP_LOGW(TAG, "Maximum interpolation steps reached, forcing completion");
        jumpToTarget();
        return false;
    }

    bool needs_movement = false;

    // 位置插值 (X轴)
    if (target_pose_.x != current_pose_.x) {
        float step = calculateStep(current_pose_.x, target_pose_.x, config_.position_step);
        current_pose_.x += step;
        needs_movement = true;
    }

    // 位置插值 (Y轴)
    if (target_pose_.y != current_pose_.y) {
        float step = calculateStep(current_pose_.y, target_pose_.y, config_.position_step);
        current_pose_.y += step;
        needs_movement = true;
    }

    // 位置插值 (Z轴)
    if (target_pose_.z != current_pose_.z) {
        float step = calculateStep(current_pose_.z, target_pose_.z, config_.position_step);
        current_pose_.z += step;
        needs_movement = true;
    }

    // 姿态插值 (Roll)
    if (target_pose_.roll != current_pose_.roll) {
        float step = calculateStep(current_pose_.roll, target_pose_.roll, config_.angle_step);
        current_pose_.roll += step;
        needs_movement = true;
    }

    // 姿态插值 (Pitch)
    if (target_pose_.pitch != current_pose_.pitch) {
        float step = calculateStep(current_pose_.pitch, target_pose_.pitch, config_.angle_step);
        current_pose_.pitch += step;
        needs_movement = true;
    }

    // 姿态插值 (Yaw)
    if (target_pose_.yaw != current_pose_.yaw) {
        float step = calculateStep(current_pose_.yaw, target_pose_.yaw, config_.angle_step);
        current_pose_.yaw += step;
        needs_movement = true;
    }

    current_step_++;

    // 检查是否到达目标
    if (!needs_movement || hasReachedTarget()) {
        current_pose_ = target_pose_;  // 确保精确到达目标
        is_moving_ = false;
        ESP_LOGI(TAG, "Interpolation completed in %d steps", current_step_);
        return false;
    }

    ESP_LOGD(TAG, "CURRENT (%.2f,%.2f,%.2f - %.4f,%.4f,%.4f) step:%d",
             current_pose_.x, current_pose_.y, current_pose_.z,
             current_pose_.roll, current_pose_.pitch, current_pose_.yaw, current_step_);

    return true;  // 需要继续插值
}

void PoseInterpolator::stopInterpolation() {
    is_moving_ = false;
    ESP_LOGI(TAG, "Interpolation stopped at step %d", current_step_);
}

void PoseInterpolator::jumpToTarget() {
    current_pose_ = target_pose_;
    is_moving_ = false;
    ESP_LOGI(TAG, "Jumped directly to target pose");
}

void PoseInterpolator::setConfig(const InterpolationConfig& config) {
    config_ = config;
    ESP_LOGI(TAG, "Configuration updated - Position step: %.2f, Angle step: %.4f",
             config_.position_step, config_.angle_step);
}

float PoseInterpolator::getProgress() const {
    if (!is_moving_ || config_.max_steps == 0) {
        return 1.0f;
    }
    return std::min(1.0f, static_cast<float>(current_step_) / config_.max_steps);
}

void PoseInterpolator::reset() {
    current_pose_ = BodyPose();
    target_pose_ = BodyPose();
    is_moving_ = false;
    current_step_ = 0;
    ESP_LOGI(TAG, "PoseInterpolator reset to initial state");
}

void PoseInterpolator::printStatus() const {
    ESP_LOGI(TAG, "=== PoseInterpolator Status ===");
    ESP_LOGI(TAG, "Moving: %s, Step: %d/%d",
             is_moving_ ? "YES" : "NO", current_step_, config_.max_steps);
    ESP_LOGI(TAG, "Current: (%.2f,%.2f,%.2f) - (%.4f,%.4f,%.4f)",
             current_pose_.x, current_pose_.y, current_pose_.z,
             current_pose_.roll, current_pose_.pitch, current_pose_.yaw);
    ESP_LOGI(TAG, "Target:  (%.2f,%.2f,%.2f) - (%.4f,%.4f,%.4f)",
             target_pose_.x, target_pose_.y, target_pose_.z,
             target_pose_.roll, target_pose_.pitch, target_pose_.yaw);
    ESP_LOGI(TAG, "Progress: %.1f%%", getProgress() * 100.0f);
    ESP_LOGI(TAG, "========================");
}

} // namespace Controllers
} // namespace Robot