#pragma once

#include <string>
#include "ServoConfig.hpp"
#include "MappingConfig.hpp"
#include "../RobotTypes.hpp"

namespace Robot {
namespace Config {

/**
 * @brief 机器人几何参数配置
 */
struct GeometryConfig {
    // 腿部连杆长度 (米) - 使用实际测量数据
    float l1;  // 髋关节延伸段长度 - 60.5mm
    float l2;  // 髋关节垂直段长度 - 10mm
    float l3;  // 大腿长度 - 111.126mm
    float l4;  // 小腿长度 - 118.5mm

    // 身体尺寸 (米)
    float body_length;  // 机体长度 - 207.5mm
    float body_width;   // 机体宽度 - 78mm

    // 髋关节在机体坐标系中的位置偏移 (米)
    struct HipOffset {
        float x, y, z;
        HipOffset(float x_ = 0.0f, float y_ = 0.0f, float z_ = 0.0f) : x(x_), y(y_), z(z_) {}
    };

    std::array<HipOffset, 4> hip_offsets; // 四条腿的髋关节偏移

    // 默认构造函数 - 使用SpotMicro的实际测量尺寸
    GeometryConfig()
        : l1(0.0605f), l2(0.010f), l3(0.111126f), l4(0.1185f),
          body_length(0.2075f), body_width(0.078f) {
        // 设置髋关节偏移 (基于实际身体尺寸动态计算)
        hip_offsets[0] = HipOffset(body_length/2, body_width/2, 0.0f);   // 左前
        hip_offsets[1] = HipOffset(body_length/2, -body_width/2, 0.0f);  // 右前
        hip_offsets[2] = HipOffset(-body_length/2, body_width/2, 0.0f);  // 左后
        hip_offsets[3] = HipOffset(-body_length/2, -body_width/2, 0.0f); // 右后
    }

    // 验证几何参数有效性
    bool isValid() const {
        return l1 > 0.0f && l2 >= 0.0f && l3 > 0.0f && l4 > 0.0f &&
               body_length > 0.0f && body_width > 0.0f;
    }

    // 获取指定腿的髋关节偏移
    const HipOffset& getHipOffset(LegID leg_id) const {
        return hip_offsets[static_cast<int>(leg_id)];
    }
};

/**
 * @brief 机器人控制参数配置
 */
struct ControlConfig {
    // 角度限制 (度)
    float max_joint_velocity;      // 最大关节角速度 (度/秒)
    float max_position_error;      // 最大位置误差容忍 (米)
    float kinematics_timeout;      // 运动学计算超时 (毫秒)

    // 安全参数
    bool enable_workspace_check;   // 是否启用工作空间检查
    bool enable_collision_check;   // 是否启用碰撞检查
    bool enable_emergency_stop;    // 是否启用紧急停止

    // 默认构造函数
    ControlConfig()
        : max_joint_velocity(90.0f), max_position_error(0.01f), kinematics_timeout(10.0f),
          enable_workspace_check(true), enable_collision_check(false), enable_emergency_stop(true) {}

    // 验证控制参数有效性
    bool isValid() const {
        return max_joint_velocity > 0.0f && max_position_error > 0.0f && kinematics_timeout > 0.0f;
    }
};

/**
 * @brief 机器人完整配置
 */
struct RobotConfig {
    AllServoConfig servo_config;       // 舵机配置
    MappingConfig mapping_config;      // 映射配置
    GeometryConfig geometry_config;    // 几何配置
    ControlConfig control_config;      // 控制配置

    // 配置版本信息
    int config_version;
    std::string robot_name;
    std::string description;

    // 默认构造函数
    RobotConfig()
        : config_version(1), robot_name("SpotMicro"), description("SpotMicro quadruped robot configuration") {}

    // 验证完整配置
    bool isValid() const {
        return servo_config.isValid() &&
               mapping_config.isValid() &&
               geometry_config.isValid() &&
               control_config.isValid() &&
               config_version > 0;
    }

    // 获取配置摘要
    std::string getSummary() const {
        return robot_name + " v" + std::to_string(config_version) + ": " + description;
    }
};

} // namespace Config
} // namespace Robot