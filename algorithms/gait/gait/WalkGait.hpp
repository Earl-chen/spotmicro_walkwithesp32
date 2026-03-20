#pragma once

#include "GaitTypes.hpp"
#include "TrajectoryGenerator.hpp"
#include <map>

namespace Robot {
namespace Gait {

/**
 * @brief Walk步态控制器
 * 
 * 实现Walk步态的相位调度和轨迹生成。
 * 
 * 相位安排：
 * - right_front: 0.0
 * - left_back: 0.25
 * - left_front: 0.5
 * - right_back: 0.75
 * 
 * Walk步态特点：
 * - 三足支撑，一足抬起
 * - 相位偏移：90°（依次抬腿）
 * - 占空比：0.75（75% 时间着地）
 * - 稳定性最高，适合入门和复杂地形
 */
class WalkGait {
public:
    /**
     * @brief 构造函数
     * 
     * @param stride_length 步长 (米)，默认 0.05 (50mm)
     * @param step_height 抬腿高度 (米)，默认 0.03 (30mm)
     * @param frequency 步频 (Hz)，默认 0.8Hz
     */
    WalkGait(
        float stride_length = 0.05f,
        float step_height = 0.03f,
        float frequency = 0.8f
    ) : stride_length_(stride_length)
      , step_height_(step_height)
      , frequency_(frequency)
      , global_phase_(0.0f)
      , trajectory_type_(TrajectoryType::CYCLOID)
    {
        // Walk 步态相位偏移（依次抬腿）
        // 相位差 90° 确保三足支撑
        phase_offsets_[LegIndex::RIGHT_FRONT] = 0.0f;
        phase_offsets_[LegIndex::LEFT_BACK] = 0.25f;
        phase_offsets_[LegIndex::LEFT_FRONT] = 0.5f;
        phase_offsets_[LegIndex::RIGHT_BACK] = 0.75f;
        
        // 初始化腿部状态
        for (int i = 0; i < 4; i++) {
            leg_states_[static_cast<LegIndex>(i)] = LegState();
            leg_states_[static_cast<LegIndex>(i)].phase_offset = phase_offsets_[static_cast<LegIndex>(i)];
        }
    }
    
    /**
     * @brief 更新全局相位
     * 
     * @param dt 时间步长 (秒)
     */
    void update(float dt) {
        global_phase_ = std::fmod(global_phase_ + frequency_ * dt, 1.0f);
        
        // 更新所有腿的相位
        for (auto& pair : leg_states_) {
            pair.second.current_phase = std::fmod(
                global_phase_ + pair.second.phase_offset, 1.0f
            );
        }
    }
    
    /**
     * @brief 获取指定腿的当前相位
     * 
     * @param leg 腿索引
     * @return 相位 (0-1)
     */
    float get_leg_phase(LegIndex leg) const {
        auto it = leg_states_.find(leg);
        if (it != leg_states_.end()) {
            return it->second.current_phase;
        }
        return 0.0f;
    }
    
    /**
     * @brief 获取指定腿的足端轨迹偏移
     * 
     * @param leg 腿索引
     * @return 轨迹点（包含位置和相位信息）
     */
    TrajectoryPoint get_foot_trajectory(LegIndex leg) const {
        float leg_phase = get_leg_phase(leg);
        
        return TrajectoryGenerator::generate(
            leg_phase,
            stride_length_,
            step_height_,
            trajectory_type_
        );
    }
    
    /**
     * @brief 获取所有腿的足端轨迹偏移
     * 
     * @return map<LegIndex, TrajectoryPoint>
     */
    std::map<LegIndex, TrajectoryPoint> get_all_foot_trajectories() const {
        std::map<LegIndex, TrajectoryPoint> trajectories;
        
        for (const auto& pair : leg_states_) {
            trajectories[pair.first] = get_foot_trajectory(pair.first);
        }
        
        return trajectories;
    }
    
    /**
     * @brief 设置步态参数
     * 
     * @param stride_length 步长 (米)
     * @param step_height 抬腿高度 (米)
     * @param frequency 步频 (Hz)
     */
    void set_parameters(
        float stride_length = -1.0f,
        float step_height = -1.0f,
        float frequency = -1.0f
    ) {
        if (stride_length > 0) stride_length_ = stride_length;
        if (step_height > 0) step_height_ = step_height;
        if (frequency > 0) frequency_ = frequency;
    }
    
    /**
     * @brief 设置轨迹类型
     * 
     * @param type 轨迹类型
     */
    void set_trajectory_type(TrajectoryType type) {
        trajectory_type_ = type;
    }
    
    /**
     * @brief 获取步态状态信息
     * 
     * @return GaitParams 结构
     */
    GaitParams get_state() const {
        GaitParams params;
        params.stride_length = stride_length_;
        params.step_height = step_height_;
        params.frequency = frequency_;
        params.duty_cycle = 0.75f;  // Walk 步态占空比
        return params;
    }
    
    /**
     * @brief 重置步态相位
     */
    void reset() {
        global_phase_ = 0.0f;
        for (auto& pair : leg_states_) {
            pair.second.current_phase = pair.second.phase_offset;
        }
    }
    
    /**
     * @brief 获取全局相位
     */
    float get_global_phase() const {
        return global_phase_;
    }
    
    /**
     * @brief 获取相位偏移
     */
    float get_phase_offset(LegIndex leg) const {
        auto it = phase_offsets_.find(leg);
        if (it != phase_offsets_.end()) {
            return it->second;
        }
        return 0.0f;
    }
    
    /**
     * @brief 获取所有相位偏移
     */
    const std::map<LegIndex, float>& get_all_phase_offsets() const {
        return phase_offsets_;
    }
    
    /**
     * @brief 检查指定腿是否处于摆动相
     */
    bool is_swing_phase(LegIndex leg) const {
        float phase = get_leg_phase(leg);
        return phase < 0.25f;
    }
    
    /**
     * @brief 统计当前支撑腿数量
     */
    int count_support_legs() const {
        int count = 0;
        for (const auto& pair : leg_states_) {
            if (!is_swing_phase(pair.first)) {
                count++;
            }
        }
        return count;
    }

private:
    float stride_length_;      // 步长 (米)
    float step_height_;        // 抬腿高度 (米)
    float frequency_;          // 步频 (Hz)
    float global_phase_;       // 全局相位 (0-1循环)
    TrajectoryType trajectory_type_;  // 轨迹类型
    
    std::map<LegIndex, float> phase_offsets_;  // 相位偏移
    std::map<LegIndex, LegState> leg_states_;  // 腿部状态
};

} // namespace Gait
} // namespace Robot
