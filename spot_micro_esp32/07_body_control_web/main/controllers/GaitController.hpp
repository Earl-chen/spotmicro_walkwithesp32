/**
 * @file GaitController.hpp
 * @brief ESP32 步态控制器（复用 algorithms/gait 中的代码）
 * 
 * 使用说明：
 * 1. 步态算法文件已复制到 main/gait/ 目录
 * 2. CMakeLists.txt 已添加 "gait" 到 INCLUDE_DIRS
 * 3. 直接包含头文件即可使用
 */

#pragma once

#include "gait/WalkGait.hpp"
#include "kinematics/LegKinematics.hpp"
#include <array>

class GaitController {
public:
    GaitController() 
        : walk_gait_(0.05f, 0.03f, 0.8f)  // 步长50mm, 步高30mm, 步频0.8Hz
        , dt_(0.02f)  // 20ms 时间步
    {
        // 初始化四条腿
        // legs_[0] = 左前腿
        // legs_[1] = 右前腿
        // legs_[2] = 左后腿
        // legs_[3] = 右后腿
    }
    
    /**
     * @brief 更新步态（每个控制周期调用）
     */
    void update() {
        // 1. 更新步态相位
        walk_gait_.update(dt_);
        
        // 2. 获取所有腿的轨迹
        auto trajectories = walk_gait_.get_all_foot_trajectories();
        
        // 3. 转换为关节角度并应用到腿部
        using namespace Robot::Gait;
        
        // 左前腿
        apply_trajectory(0, trajectories.at(LegIndex::LEFT_FRONT));
        
        // 右前腿
        apply_trajectory(1, trajectories.at(LegIndex::RIGHT_FRONT));
        
        // 左后腿
        apply_trajectory(2, trajectories.at(LegIndex::LEFT_BACK));
        
        // 右后腿
        apply_trajectory(3, trajectories.at(LegIndex::RIGHT_BACK));
    }
    
    /**
     * @brief 设置步态参数
     */
    void set_parameters(float stride_length, float step_height, float frequency) {
        walk_gait_.set_parameters(stride_length, step_height, frequency);
    }
    
    /**
     * @brief 重置步态
     */
    void reset() {
        walk_gait_.reset();
    }
    
    /**
     * @brief 获取步态状态
     */
    Robot::Gait::GaitParams get_state() const {
        return walk_gait_.get_state();
    }

private:
    Robot::Gait::WalkGait walk_gait_;
    std::array<LegKinematics, 4> legs_;  // 使用现有的 LegKinematics
    float dt_;  // 时间步长
    
    /**
     * @brief 应用轨迹到腿部（待实现）
     */
    void apply_trajectory(int leg_index, const Robot::Gait::TrajectoryPoint& trajectory) {
        // 获取轨迹偏移
        float x_offset = trajectory.position.x;
        float z_offset = trajectory.position.z;
        
        // TODO: 将轨迹偏移转换为腿部目标位置
        // 这需要结合逆运动学（IK）计算
        
        // 伪代码：
        // 1. 获取腿部基准位置
        // Vector3 base_pos = legs_[leg_index].get_default_position();
        
        // 2. 计算目标位置
        // Vector3 target_pos;
        // target_pos.x = base_pos.x + x_offset;
        // target_pos.z = base_pos.z + z_offset;
        // target_pos.y = base_pos.y;
        
        // 3. 使用 IK 计算关节角度
        // JointAngles angles = legs_[leg_index].inverse_kinematics(target_pos);
        
        // 4. 应用到舵机（通过 JointController）
        // joint_controller.set_joint_angles(leg_index, angles);
        
        ESP_LOGD("GaitController", "Leg %d: x=%.1fmm, z=%.1fmm, swing=%d",
                 leg_index, x_offset * 1000, z_offset * 1000, trajectory.is_swing);
    }
};
