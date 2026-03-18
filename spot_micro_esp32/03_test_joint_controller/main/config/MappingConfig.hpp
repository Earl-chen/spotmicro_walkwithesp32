#pragma once

#include "../RobotTypes.hpp"
#include <array>

namespace Robot {
namespace Config {

/**
 * @brief 单条腿的关节映射配置
 */
struct LegMappingConfig {
    // 关节角度到舵机角度的映射系数
    struct JointMapping {
        float k;        // 线性映射系数 (servo_angle = k * joint_angle + b)
        float b;        // 线性映射偏移
        float joint_min; // 关节角度最小值 (度)
        float joint_max; // 关节角度最大值 (度)
        bool enabled;   // 该关节是否启用

        JointMapping() : k(1.0f), b(0.0f), joint_min(-180.0f), joint_max(180.0f), enabled(true) {}
        JointMapping(float slope, float offset, float min_angle, float max_angle, bool en = true)
            : k(slope), b(offset), joint_min(min_angle), joint_max(max_angle), enabled(en) {}

        // 验证映射有效性
        bool isValid() const {
            return joint_min < joint_max && k != 0.0f;
        }

        // 关节角度转舵机角度
        float jointToServo(float joint_angle) const {
            return k * joint_angle + b;
        }

        // 舵机角度转关节角度
        float servoToJoint(float servo_angle) const {
            return (servo_angle - b) / k;
        }
    };

    std::array<JointMapping, 3> joints; // HIP_ROLL, HIP_PITCH, KNEE_PITCH

    // 获取指定关节映射
    const JointMapping& getJoint(JointType joint_type) const {
        return joints[static_cast<int>(joint_type)];
    }

    // 设置指定关节映射
    void setJoint(JointType joint_type, const JointMapping& mapping) {
        joints[static_cast<int>(joint_type)] = mapping;
    }

    // 验证映射有效性
    bool isValid() const {
        for (const auto& joint : joints) {
            if (!joint.isValid()) return false;
        }
        return true;
    }
};

/**
 * @brief 所有腿的映射配置
 */
struct MappingConfig {
    static constexpr int LEG_COUNT = 4;
    std::array<LegMappingConfig, LEG_COUNT> legs;

    // 获取指定腿的映射配置
    const LegMappingConfig& getLeg(LegID leg_id) const {
        return legs[static_cast<int>(leg_id)];
    }

    // 设置指定腿的映射配置
    void setLeg(LegID leg_id, const LegMappingConfig& config) {
        legs[static_cast<int>(leg_id)] = config;
    }

    // 获取指定关节的映射
    const LegMappingConfig::JointMapping& getJointMapping(LegID leg_id, JointType joint_type) const {
        return legs[static_cast<int>(leg_id)].getJoint(joint_type);
    }

    // 验证所有映射
    bool isValid() const {
        for (const auto& leg : legs) {
            if (!leg.isValid()) return false;
        }
        return true;
    }
};

} // namespace Config
} // namespace Robot