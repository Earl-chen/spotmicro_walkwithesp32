#pragma once

#include <array>
#include <cmath>  // 添加数学函数头文件

namespace Robot {
namespace Kinematics {

/**
 * @brief SpotMicro机器人几何参数类
 *
 * 与Python版本project_robot_python/robots/spot_micro/geometry.py保持完全一致
 * 所有单位均为米(m)
 */
class SpotMicroGeometry {
public:
    // 腿部连杆长度 (米)
    static constexpr float L1 = 0.0605f;    // 60.5 mm - 髋关节延伸段长度
    static constexpr float L2 = 0.010f;     // 10 mm - 髋关节垂直段长度
    static constexpr float L3 = 0.111126f;  // 111.126 mm - 大腿长度
    static constexpr float L4 = 0.1185f;    // 118.5 mm - 小腿长度

    // 身体尺寸 (米)
    static constexpr float BODY_LENGTH = 0.2075f;   // 207.5 mm
    static constexpr float BODY_WIDTH = 0.078f;     // 78 mm

    // 髋关节在机体坐标系中的位置偏移
    struct HipOffset {
        float x, y, z;
        HipOffset(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
    };

    // 髋关节偏移表 (与Python HIP_OFFSETS保持一致)
    enum LegIndex {
        LEFT_FRONT = 0,   // 左前腿
        RIGHT_FRONT = 1,  // 右前腿
        LEFT_BACK = 2,    // 左后腿
        RIGHT_BACK = 3    // 右后腿
    };

    static const std::array<HipOffset, 4> HIP_OFFSETS;

    /**
     * @brief 获取指定腿的髋关节偏移
     */
    static HipOffset getHipOffset(LegIndex leg) {
        return HIP_OFFSETS[static_cast<int>(leg)];
    }

    /**
     * @brief 检查腿部参数的有效性
     */
    static bool validateGeometry() {
        // 基本几何约束检查
        if (L1 <= 0 || L2 < 0 || L3 <= 0 || L4 <= 0) {
            return false;
        }

        // 确保腿部可以达到合理的工作空间
        float max_reach = L3 + L4;  // 最大伸展长度
        float min_reach = std::abs(L3 - L4);  // 最小收缩长度

        if (max_reach < 0.1f || min_reach > 0.05f) {  // 基于经验的合理范围
            return false;
        }

        return true;
    }

    /**
     * @brief 获取腿部工作空间信息
     */
    struct WorkspaceInfo {
        float max_reach;      // 最大伸展半径
        float min_reach;      // 最小收缩半径
        float lateral_reach;  // 侧向最大伸展 (考虑L1)
    };

    static WorkspaceInfo getWorkspaceInfo() {
        WorkspaceInfo info;
        info.max_reach = L3 + L4;
        info.min_reach = std::abs(L3 - L4);
        info.lateral_reach = std::sqrt(L1*L1 + info.max_reach*info.max_reach);
        return info;
    }

    /**
     * @brief 腿部类型判断
     */
    static bool isLeftLeg(LegIndex leg) {
        return (leg == LEFT_FRONT || leg == LEFT_BACK);
    }

    static bool isFrontLeg(LegIndex leg) {
        return (leg == LEFT_FRONT || leg == RIGHT_FRONT);
    }

    /**
     * @brief 获取腿部名称字符串 (用于调试)
     */
    static const char* getLegName(LegIndex leg) {
        static const char* names[] = {
            "LEFT_FRONT", "RIGHT_FRONT", "LEFT_BACK", "RIGHT_BACK"
        };
        return names[static_cast<int>(leg)];
    }
};

} // namespace Kinematics
} // namespace Robot