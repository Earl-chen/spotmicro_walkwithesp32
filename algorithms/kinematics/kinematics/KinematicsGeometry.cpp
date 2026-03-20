#include "KinematicsGeometry.hpp"

namespace Robot {
namespace Kinematics {

// 定义静态数组
const std::array<SpotMicroGeometry::HipOffset, 4> SpotMicroGeometry::HIP_OFFSETS = {{
    {BODY_LENGTH/2,  BODY_WIDTH/2, 0.0f},   // 左前腿
    {BODY_LENGTH/2, -BODY_WIDTH/2, 0.0f},   // 右前腿
    {-BODY_LENGTH/2, BODY_WIDTH/2, 0.0f},   // 左后腿
    {-BODY_LENGTH/2, -BODY_WIDTH/2, 0.0f}   // 右后腿
}};

} // namespace Kinematics
} // namespace Robot