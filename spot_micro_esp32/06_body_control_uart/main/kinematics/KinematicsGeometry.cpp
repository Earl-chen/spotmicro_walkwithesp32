#include "KinematicsGeometry.hpp"
#include <cmath>

namespace Robot {
namespace Kinematics {

SpotMicroGeometry::SpotMicroGeometry(const Robot::Config::GeometryConfig& config)
    : params_(config) {}

float SpotMicroGeometry::getL1() const {
    return params_.l1;
}

float SpotMicroGeometry::getL2() const {
    return params_.l2;
}

float SpotMicroGeometry::getL3() const {
    return params_.l3;
}

float SpotMicroGeometry::getL4() const {
    return params_.l4;
}

float SpotMicroGeometry::getBodyLength() const {
    return params_.body_length;
}

float SpotMicroGeometry::getBodyWidth() const {
    return params_.body_width;
}

SpotMicroGeometry::HipOffset SpotMicroGeometry::getHipOffset(LegIndex leg) const {
    const auto& hip_offset = params_.getHipOffset(static_cast<Robot::LegID>(leg));
    return HipOffset(hip_offset.x, hip_offset.y, hip_offset.z);
}

bool SpotMicroGeometry::validateGeometry() const {
    // 基本几何约束检查
    if (!params_.isValid()) {
        return false;
    }

    // 确保腿部可以达到合理的工作空间
    float max_reach = params_.l3 + params_.l4;  // 最大伸展长度
    float min_reach = std::abs(params_.l3 - params_.l4);  // 最小收缩长度

    if (max_reach < 0.1f || min_reach > 0.05f) {  // 基于经验的合理范围
        return false;
    }

    return true;
}

SpotMicroGeometry::WorkspaceInfo SpotMicroGeometry::getWorkspaceInfo() const {
    WorkspaceInfo info;
    info.max_reach = params_.l3 + params_.l4;
    info.min_reach = std::abs(params_.l3 - params_.l4);
    info.lateral_reach = std::sqrt(params_.l1*params_.l1 + info.max_reach*info.max_reach);
    return info;
}

const Robot::Config::GeometryConfig& SpotMicroGeometry::getConfig() const {
    return params_;
}

} // namespace Kinematics
} // namespace Robot