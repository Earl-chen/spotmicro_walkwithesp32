#include "CoordinateTransform.hpp"
#include <esp_log.h>

namespace Robot {
namespace Kinematics {

static const char* TAG = "CoordinateTransform";

// ==================== FrameManager 实现 ====================

void FrameManager::setBodyPose(const CoordinateTransform::Pose6DOF& pose) {
    body_pose_ = pose;
}

CoordinateTransform::Pose6DOF FrameManager::getBodyPose() const {
    return body_pose_;
}

CoordinateTransform::Vector3 FrameManager::transformWorldToBody(const CoordinateTransform::Vector3& world_point) const {
    // 1. 平移: 相对于机体原点
    CoordinateTransform::Vector3 translated = world_point - body_pose_.position;

    // 2. 旋转: 应用机体姿态的逆变换
    CoordinateTransform::Matrix3x3 R_body = CoordinateTransform::rotationRPY(
        body_pose_.orientation.x,
        body_pose_.orientation.y,
        body_pose_.orientation.z
    );

    // 逆变换 = 转置
    CoordinateTransform::Matrix3x3 R_body_inv = CoordinateTransform::transpose(R_body);

    return CoordinateTransform::matrixVectorMultiply(R_body_inv, translated);
}

CoordinateTransform::Vector3 FrameManager::transformBodyToWorld(const CoordinateTransform::Vector3& body_point) const {
    // 1. 旋转: 应用机体姿态变换
    CoordinateTransform::Matrix3x3 R_body = CoordinateTransform::rotationRPY(
        body_pose_.orientation.x,
        body_pose_.orientation.y,
        body_pose_.orientation.z
    );

    CoordinateTransform::Vector3 rotated = CoordinateTransform::matrixVectorMultiply(R_body, body_point);

    // 2. 平移: 加上机体位置
    return rotated + body_pose_.position;
}

CoordinateTransform::Vector3 FrameManager::transformBodyToHip(const CoordinateTransform::Vector3& body_point,
                                                             const CoordinateTransform::Vector3& hip_offset) const {
    return body_point - hip_offset;
}

CoordinateTransform::Vector3 FrameManager::transformHipToBody(const CoordinateTransform::Vector3& hip_point,
                                                             const CoordinateTransform::Vector3& hip_offset) const {
    return hip_point + hip_offset;
}

CoordinateTransform::Vector3 FrameManager::transformWorldToHip(const CoordinateTransform::Vector3& world_point,
                                                              const CoordinateTransform::Vector3& hip_offset) const {
    CoordinateTransform::Vector3 body_point = transformWorldToBody(world_point);
    return transformBodyToHip(body_point, hip_offset);
}

CoordinateTransform::Vector3 FrameManager::transformHipToWorld(const CoordinateTransform::Vector3& hip_point,
                                                              const CoordinateTransform::Vector3& hip_offset) const {
    CoordinateTransform::Vector3 body_point = transformHipToBody(hip_point, hip_offset);
    return transformBodyToWorld(body_point);
}

void FrameManager::printTransformInfo() const {
    ESP_LOGI(TAG, "机体位姿: 位置(%.3f, %.3f, %.3f)m 姿态(%.1f°, %.1f°, %.1f°)",
            body_pose_.position.x, body_pose_.position.y, body_pose_.position.z,
            CoordinateTransform::radiansToDegrees(body_pose_.orientation.x),
            CoordinateTransform::radiansToDegrees(body_pose_.orientation.y),
            CoordinateTransform::radiansToDegrees(body_pose_.orientation.z));
}

} // namespace Kinematics
} // namespace Robot