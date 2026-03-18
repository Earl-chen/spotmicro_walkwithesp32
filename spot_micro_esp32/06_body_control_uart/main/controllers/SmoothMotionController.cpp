/**
 * @file SmoothMotionController.cpp
 * @brief 统一平滑运动控制器实现
 */

#include "SmoothMotionController.hpp"
#include "../kinematics/KinematicsGeometry.hpp"
#include <cmath>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace Robot {
namespace Controllers {

const char* SmoothMotionController::TAG = "SmoothMotion";

SmoothMotionController::SmoothMotionController(std::shared_ptr<RobotController> robot_controller)
    : robot_controller_(robot_controller), config_(), current_state_(MotionState::IDLE), motion_step_count_(0) {
    ESP_LOGI(TAG, "SmoothMotionController created");
}

SmoothMotionController::SmoothMotionController(std::shared_ptr<RobotController> robot_controller,
                                              const SmoothMotionConfig& config)
    : robot_controller_(robot_controller), config_(config), current_state_(MotionState::IDLE), motion_step_count_(0) {
    ESP_LOGI(TAG, "SmoothMotionController created with custom config");
}

SmoothMotionController::~SmoothMotionController() {
    ESP_LOGI(TAG, "SmoothMotionController destroyed");
}

bool SmoothMotionController::initialize() {
    if (!robot_controller_) {
        ESP_LOGE(TAG, "RobotController is null");
        return false;
    }

    if (!quadruped_model_) {
        ESP_LOGE(TAG, "QuadrupedModel is null, please call setQuadrupedModel() first");
        return false;
    }

    // 初始化姿态插值器
    if (config_.enable_pose_smoothing) {
        pose_interpolator_ = std::make_unique<PoseInterpolator>(config_.pose_interpolation);
        if (!pose_interpolator_) {
            ESP_LOGE(TAG, "Failed to create PoseInterpolator");
            return false;
        }
    }

    // 初始化关节平滑器
    if (config_.enable_servo_smoothing) {
        // 使用RobotController的JointController
        auto joint_controller = robot_controller_->getJointController();
        if (!joint_controller) {
            ESP_LOGE(TAG, "JointController from RobotController is null");
            return false;
        }

        // 创建JointSmoother (使用RobotController的JointController)
        auto config_manager = robot_controller_->getConfigManager();
        joint_smoother_ = std::make_unique<JointSmoother>(joint_controller, config_manager, config_.servo_smoothing);
        if (!joint_smoother_->initialize()) {
            ESP_LOGE(TAG, "Failed to initialize JointSmoother");
            return false;
        }
    }

    current_state_ = MotionState::IDLE;
    motion_step_count_ = 0;

    ESP_LOGI(TAG, "SmoothMotionController initialized successfully");
    ESP_LOGI(TAG, "Features: PoseSmooth=%s, ServoSmooth=%s",
             config_.enable_pose_smoothing ? "ON" : "OFF",
             config_.enable_servo_smoothing ? "ON" : "OFF");

    return true;
}

void SmoothMotionController::setQuadrupedModel(std::shared_ptr<Robot::Kinematics::QuadrupedModel> model) {
    quadruped_model_ = model;

    if (quadruped_model_) {
        // 设置初始关节角度（全部为 0 度 - 安全的中立位置）
        // 数组顺序：[左前腿3个关节, 右前腿3个关节, 左后腿3个关节, 右后腿3个关节]
        // 每组关节顺序：髋关节侧摆, 髋关节俯仰, 膝关节俯仰
        float initial_angles_deg[12] = {
            // 左前腿 (LEFT_FRONT)
            0.0f, 0.0f, 0.0f,
            // 右前腿 (RIGHT_FRONT)
            0.0f, 0.0f, 0.0f,
            // 左后腿 (LEFT_BACK)
            0.0f, 0.0f, 0.0f,
            // 右后腿 (RIGHT_BACK)
            0.0f, 0.0f, 0.0f
        };

        ESP_LOGI(TAG, "Setting initial joint angles for QuadrupedModel (all 0 degrees)");

        if (quadruped_model_->setInitialStateFromAnglesDegrees(initial_angles_deg)) {
            ESP_LOGI(TAG, "✅ QuadrupedModel initialized with default joint angles successfully");
        } else {
            ESP_LOGW(TAG, "⚠️ Failed to set initial joint angles, QuadrupedModel will use its internal defaults");
        }
    }

    ESP_LOGI(TAG, "QuadrupedModel set successfully");
}

bool SmoothMotionController::setTargetPose(const BodyPose& pose) {
    ESP_LOGI(TAG, "setTargetPose called with: (%.3f,%.3f,%.3f)-(%.3f,%.3f,%.3f)",
             pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);

    if (!pose_interpolator_) {
        ESP_LOGW(TAG, "PoseInterpolator not initialized, creating one with config");
        // 如果没有插值器，创建一个带配置的
        pose_interpolator_ = std::make_unique<PoseInterpolator>(config_.pose_interpolation);

        // 设置默认起始姿态
        BodyPose start_pose(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        pose_interpolator_->setCurrentPose(start_pose);

        ESP_LOGI(TAG, "Created PoseInterpolator with step: %.3fm, angle: %.3frad",
                 config_.pose_interpolation.position_step, config_.pose_interpolation.angle_step);
    }

    // 确保设置了起始姿态
    BodyPose current = pose_interpolator_->getCurrentPose();
    ESP_LOGI(TAG, "Current pose before interpolation: (%.3f,%.3f,%.3f)-(%.3f,%.3f,%.3f)",
             current.x, current.y, current.z, current.roll, current.pitch, current.yaw);

    pose_interpolator_->setTargetPose(pose);
    current_state_ = MotionState::INTERPOLATING;
    motion_step_count_ = 0;

    ESP_LOGI(TAG, "Target pose set, starting smooth motion");
    return true;
}

bool SmoothMotionController::setCurrentPose(const BodyPose& pose) {
    if (pose_interpolator_) {
        pose_interpolator_->setCurrentPose(pose);
        pose_interpolator_->jumpToTarget();
    }

    // 使用QuadrupedModel进行运动学计算
    Robot::Kinematics::CoordinateTransform::Vector3 position(pose.x, pose.y, pose.z);
    Robot::Kinematics::CoordinateTransform::Vector3 orientation(pose.roll, pose.pitch, pose.yaw);

    if (quadruped_model_->updateBodyPose(position, orientation)) {
        auto kinematics_angles = quadruped_model_->getAllJointAngles();

        // 转换数据类型：ThreeJointAngles → AllLegJoints
        AllLegJoints joint_angles;
        for (int leg = 0; leg < 4; leg++) {
            joint_angles[leg][0] = JointAngle(kinematics_angles[leg].hip_side, true);
            joint_angles[leg][1] = JointAngle(kinematics_angles[leg].hip_pitch, true);
            joint_angles[leg][2] = JointAngle(kinematics_angles[leg].knee_pitch, true);
        }

        if (joint_smoother_) {
            // 直接传递关节角度给JointSmoother
            return joint_smoother_->setAllJointAngles(joint_angles);
        } else {
            ESP_LOGW(TAG, "No JointSmoother available for direct pose setting");
            return false;
        }
    }

    ESP_LOGE(TAG, "Failed to calculate kinematics for pose");
    return false;
}



bool SmoothMotionController::stepMotion() {
    if (current_state_ == MotionState::IDLE) {
        return false; // 没有运动需要执行
    }

    motion_step_count_++;
    ESP_LOGD(TAG, "Step %d, State: %s", motion_step_count_,
             current_state_ == MotionState::INTERPOLATING ? "INTERPOLATING" : "SMOOTHING");

    switch (current_state_) {
        case MotionState::INTERPOLATING: {
            // 执行一步姿态插值
            bool interp_continue = pose_interpolator_ && pose_interpolator_->stepInterpolation();

            // 获取当前插值姿态并计算运动学
            BodyPose current_pose = pose_interpolator_->getCurrentPose();
            ESP_LOGD(TAG, "插值位姿: (%.4f,%.4f,%.4f)-(%.4f,%.4f,%.4f)",
                     current_pose.x, current_pose.y, current_pose.z,
                     current_pose.roll, current_pose.pitch, current_pose.yaw);

            // 立即计算运动学并设置关节目标
            Robot::Kinematics::CoordinateTransform::Vector3 position(current_pose.x, current_pose.y, current_pose.z);
            Robot::Kinematics::CoordinateTransform::Vector3 orientation(current_pose.roll, current_pose.pitch, current_pose.yaw);

            if (quadruped_model_->updateBodyPose(position, orientation)) {
                auto kinematics_angles = quadruped_model_->getAllJointAngles();

                // 转换数据类型并设置关节目标
                AllLegJoints joint_angles;
                for (int leg = 0; leg < 4; leg++) {
                    joint_angles[leg][0] = JointAngle(kinematics_angles[leg].hip_side, true);
                    joint_angles[leg][1] = JointAngle(kinematics_angles[leg].hip_pitch, true);
                    joint_angles[leg][2] = JointAngle(kinematics_angles[leg].knee_pitch, true);
                }

                if (joint_smoother_) {
                    joint_smoother_->setAllJointAngles(joint_angles);
                }
            }

            if (!interp_continue) {
                // 插值完成，但仍需要完成最后的关节平滑
                ESP_LOGI(TAG, "Pose interpolation completed, final joint smoothing");
                current_state_ = MotionState::SMOOTHING;
                return true;
            }

            // 插值还未完成，切换到关节平滑状态来执行当前步的平滑
            current_state_ = MotionState::SMOOTHING;
            return true;
        }

        case MotionState::SMOOTHING: {
            // 执行关节平滑
            if (joint_smoother_ && joint_smoother_->stepJointSmoothing()) {
                // 关节还在平滑中，继续平滑
                return true;
            } else {
                // 当前步关节平滑完成
                ESP_LOGD(TAG, "Joint smoothing step completed");

                // 检查插值是否已经完成
                if (pose_interpolator_ && pose_interpolator_->isMoving()) {
                    // 插值未完成，回到插值状态继续下一步
                    current_state_ = MotionState::INTERPOLATING;
                    return true;
                } else {
                    // 插值和平滑都完成，运动结束
                    ESP_LOGI(TAG, "Both pose interpolation and joint smoothing completed");
                    current_state_ = MotionState::IDLE;
                    return false;
                }
            }
        }

        case MotionState::ERROR:
            ESP_LOGE(TAG, "Motion controller in error state");
            return false;

        default:
            current_state_ = MotionState::IDLE;
            return false;
    }
}

bool SmoothMotionController::executeMotionToTarget() {
    ESP_LOGI(TAG, "当前运动状态: %d", static_cast<int>(current_state_));
    ESP_LOGI(TAG, "开始实现运动到目标,当前机器人位姿: (%.3f,%.3f,%.3f)-(%.3f,%.3f,%.3f)",
             getCurrentPose().x, getCurrentPose().y, getCurrentPose().z,
             getCurrentPose().roll, getCurrentPose().pitch, getCurrentPose().yaw);

    while (stepMotion()) {
        vTaskDelay(config_.motion_update_rate_ms / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "Motion execution completed");
    return current_state_ != MotionState::ERROR;
}

void SmoothMotionController::stopMotion() {
    if (pose_interpolator_) {
        pose_interpolator_->stopInterpolation();
    }
    if (joint_smoother_) {
        joint_smoother_->stopSmoothing();
    }
    current_state_ = MotionState::IDLE;
    ESP_LOGI(TAG, "Motion stopped");
}

void SmoothMotionController::emergencyStop() {
    stopMotion();
    ESP_LOGW(TAG, "EMERGENCY STOP executed");
}

BodyPose SmoothMotionController::getCurrentPose() const {
    if (pose_interpolator_) {
        return pose_interpolator_->getCurrentPose();
    }
    return BodyPose();
}

BodyPose SmoothMotionController::getTargetPose() const {
    if (pose_interpolator_) {
        return pose_interpolator_->getTargetPose();
    }
    return BodyPose();
}

bool SmoothMotionController::isMoving() const {
    ESP_LOGI(TAG, "\n current_state_ 值：%d \n", static_cast<int>(current_state_));
    return current_state_ != MotionState::IDLE;
}

float SmoothMotionController::getMotionProgress() const {
    switch (current_state_) {
        case MotionState::INTERPOLATING:
            return pose_interpolator_ ? pose_interpolator_->getProgress() * 0.7f : 0.0f;
        case MotionState::SMOOTHING:
            return 0.7f + (joint_smoother_ ? joint_smoother_->getSmoothingProgress() * 0.3f : 0.3f);
        case MotionState::IDLE:
            return 1.0f;
        default:
            return 0.0f;
    }
}

bool SmoothMotionController::executeStandAction() {
    BodyPose stand_pose(0.050, 0, -0.100, 0, 0, 0); // 前进50mm，高度100mm (单位: m)
    return setTargetPose(stand_pose);
}

bool SmoothMotionController::executeRelaxAction() {
    BodyPose relax_pose(0, 0, -0.050, 0, 0, 0); // 放松姿态：高度50mm (单位: m)
    return setTargetPose(relax_pose);
}

void SmoothMotionController::setConfig(const SmoothMotionConfig& config) {
    config_ = config;
    if (pose_interpolator_) {
        pose_interpolator_->setConfig(config_.pose_interpolation);
    }
    if (joint_smoother_) {
        joint_smoother_->setConfig(config_.servo_smoothing);
    }
    ESP_LOGI(TAG, "Configuration updated");
}

void SmoothMotionController::enablePoseSmoothing(bool enable) {
    config_.enable_pose_smoothing = enable;
    ESP_LOGI(TAG, "Pose smoothing %s", enable ? "enabled" : "disabled");
}

void SmoothMotionController::enableServoSmoothing(bool enable) {
    config_.enable_servo_smoothing = enable;
    if (joint_smoother_) {
        joint_smoother_->enableSmoothing(enable);
    }
    ESP_LOGI(TAG, "Servo smoothing %s", enable ? "enabled" : "disabled");
}
 

void SmoothMotionController::reset() {
    stopMotion();
    if (pose_interpolator_) {
        pose_interpolator_->reset();
    }
    if (joint_smoother_) {
        joint_smoother_->reset();
    }
    motion_step_count_ = 0;
    current_state_ = MotionState::IDLE;
    ESP_LOGI(TAG, "SmoothMotionController reset");
}

void SmoothMotionController::printStatus() const {
    const char* state_names[] = {"IDLE", "INTERPOLATING", "SMOOTHING", "ERROR"};
    ESP_LOGI(TAG, "=== Smooth Motion Status ===");
    ESP_LOGI(TAG, "State: %s, Steps: %d", state_names[static_cast<int>(current_state_)], motion_step_count_);
    ESP_LOGI(TAG, "Progress: %.1f%%", getMotionProgress() * 100.0f);
    ESP_LOGI(TAG, "Features: PoseSmooth=%s, ServoSmooth=%s",
             config_.enable_pose_smoothing ? "ON" : "OFF",
             config_.enable_servo_smoothing ? "ON" : "OFF");
    ESP_LOGI(TAG, "========================");
}

void SmoothMotionController::printDetailedStatus() const {
    printStatus();
    if (pose_interpolator_) {
        pose_interpolator_->printStatus();
    }
    if (joint_smoother_) {
        joint_smoother_->printStatus();
    }
}

void SmoothMotionController::printWorkspaceInfo() const {
    ESP_LOGI(TAG, "=== Robot Workspace Info ===");
    if (quadruped_model_) {
        ESP_LOGI(TAG, "Using QuadrupedModel for workspace information");
        // 可以添加QuadrupedModel的工作空间信息显示
    } else {
        ESP_LOGI(TAG, "QuadrupedModel not initialized");
    }
    ESP_LOGI(TAG, "=======================");
}

} // namespace Controllers
} // namespace Robot