#pragma once

#include <array>
#include <cmath>  // 添加数学函数头文件
#include "../config/RobotConfig.hpp"  // 添加配置头文件

namespace Robot {
namespace Kinematics {

/**
 * @brief SpotMicro机器人几何参数类
 *
 * 使用统一的GeometryConfig配置，消除硬编码参数
 * 所有单位均为米(m)
 */
class SpotMicroGeometry {
private:
    Robot::Config::GeometryConfig params_;  // 几何配置

public:
    /**
     * @brief 构造函数
     * @param config 几何配置参数
     */
    explicit SpotMicroGeometry(const Robot::Config::GeometryConfig& config = Robot::Config::GeometryConfig());

    // 腿部连杆长度访问器 (替换静态常量)
    float getL1() const;
    float getL2() const;
    float getL3() const;
    float getL4() const;

    // 身体尺寸访问器
    float getBodyLength() const;
    float getBodyWidth() const;

    // 髋关节在机体坐标系中的位置偏移
    struct HipOffset {
        float x, y, z;
        HipOffset(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
    };

    // 髋关节偏移表 (动态计算)
    enum LegIndex {
        LEFT_FRONT = 0,   // 左前腿
        RIGHT_FRONT = 1,  // 右前腿
        LEFT_BACK = 2,    // 左后腿
        RIGHT_BACK = 3    // 右后腿
    };

    /**
     * @brief 获取指定腿的髋关节偏移
     */
    HipOffset getHipOffset(LegIndex leg) const;

    /**
     * @brief 检查腿部参数的有效性
     */
    bool validateGeometry() const;

    /**
     * @brief 获取腿部工作空间信息
     */
    struct WorkspaceInfo {
        float max_reach;      // 最大伸展半径
        float min_reach;      // 最小收缩半径
        float lateral_reach;  // 侧向最大伸展 (考虑L1)
    };

    WorkspaceInfo getWorkspaceInfo() const;

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

    /**
     * @brief 获取当前配置 (用于传递给其他组件)
     */
    const Robot::Config::GeometryConfig& getConfig() const;
};

} // namespace Kinematics
} // namespace Robot