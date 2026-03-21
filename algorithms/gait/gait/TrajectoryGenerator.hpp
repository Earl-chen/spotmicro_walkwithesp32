#pragma once

#include "GaitTypes.hpp"
#include <cmath>

namespace Robot {
namespace Gait {

/**
 * @brief 足端轨迹生成器
 * 
 * 提供多种轨迹生成方法，用于步态控制。
 * 
 * 支持的轨迹类型：
 * - 摆线轨迹（Cycloid）：推荐用于 Walk 步态
 * - 椭圆轨迹（Ellipse）：简化版
 * - 贝塞尔曲线（Bezier）：高级
 */
class TrajectoryGenerator {
public:
    /**
     * @brief 摆线轨迹（修正版 - 标准Walk步态）
     * 
     * 特点：
     * - 平滑的加速和减速
     * - 适合舵机控制
     * - 摆动相占25%，支撑相占75%
     * 
     * @param phase 相位 (0-1)
     *              0-0.25: 摆动相（抬腿向前摆动）
     *              0.25-1.0: 支撑相（向后蹬地）
     * @param stride_length 步长 (米)，默认 0.05 (50mm)
     * @param step_height 抬腿高度 (米)，默认 0.03 (30mm)
     * @return (x, z) 前后位置偏移, 上下位置偏移（米）
     */
    static TrajectoryPoint cycloid_trajectory(
        float phase, 
        float stride_length = 0.05f, 
        float step_height = 0.03f
    ) {
        TrajectoryPoint point;
        point.phase = phase;
        
        if (phase < 0.25f) {  // 摆动相（25%）
            float t = phase * 4 * M_PI;
            // X从 -stride_length/2 → +stride_length/2
            point.position.x = stride_length / 2 * (t / M_PI - 1);
            // Z抬起（摆线轨迹）
            point.position.z = step_height * (1 - std::cos(t)) / 2;
            point.is_swing = true;
        } else {  // 支撑相（75%）
            float t = (phase - 0.25f) * 4 / 3;
            // X从 +stride_length/2 → -stride_length/2（修正后连续）
            point.position.x = stride_length / 2 * (1 - 2 * t);
            // Z着地（高度为0）
            point.position.z = 0;
            point.is_swing = false;
        }
        
        point.position.y = 0;  // 侧向无偏移
        return point;
    }
    
    /**
     * @brief 椭圆轨迹（简化版）
     * 
     * 特点：
     * - 计算简单
     * - 形状规则
     * - 摆动相和支撑相对称
     * 
     * @param phase 相位 (0-1)
     * @param stride_length 步长 (米)
     * @param step_height 抬腿高度 (米)
     * @return (x, z) 前后位置偏移, 上下位置偏移（米）
     */
    static TrajectoryPoint ellipse_trajectory(
        float phase, 
        float stride_length = 0.05f, 
        float step_height = 0.03f
    ) {
        TrajectoryPoint point;
        point.phase = phase;
        
        float angle = phase * 2 * M_PI;
        point.position.x = stride_length / 2 * std::cos(angle);
        point.position.z = step_height * (1 + std::sin(angle)) / 2;  // 只取上半椭圆
        point.position.y = 0;
        point.is_swing = (point.position.z > 0.001f);
        
        return point;
    }
    
    /**
     * @brief 贝塞尔曲线轨迹（高级）
     * 
     * 特点：
     * - 可自定义曲线形状
     * - 平滑且可控
     * - 适合复杂轨迹需求
     * 
     * @param phase 相位 (0-1)
     * @param stride_length 步长 (米)
     * @param step_height 抬腿高度 (米)
     * @return (x, z) 前后位置偏移, 上下位置偏移（米）
     */
    static TrajectoryPoint bezier_trajectory(
        float phase, 
        float stride_length = 0.05f, 
        float step_height = 0.03f
    ) {
        TrajectoryPoint point;
        point.phase = phase;
        
        // 简化的二次贝塞尔曲线
        // 控制点：(0, 0), (stride_length/2, step_height), (stride_length, 0)
        float t = phase;
        point.position.x = 2 * (1 - t) * t * (stride_length / 2) + t * t * stride_length;
        point.position.z = 2 * (1 - t) * t * step_height;
        point.position.y = 0;
        point.is_swing = (point.position.z > 0.001f);
        
        return point;
    }
    
    /**
     * @brief 根据轨迹类型生成轨迹
     */
    static TrajectoryPoint generate(
        float phase,
        float stride_length,
        float step_height,
        TrajectoryType type = TrajectoryType::CYCLOID
    ) {
        switch (type) {
            case TrajectoryType::CYCLOID:
                return cycloid_trajectory(phase, stride_length, step_height);
            case TrajectoryType::ELLIPSE:
                return ellipse_trajectory(phase, stride_length, step_height);
            case TrajectoryType::BEZIER:
                return bezier_trajectory(phase, stride_length, step_height);
            default:
                return cycloid_trajectory(phase, stride_length, step_height);
        }
    }
    
    /**
     * @brief 根据轨迹类型生成带转向的轨迹
     */
    static TrajectoryPoint generate_with_steering(
        float phase,
        float stride_length,
        float step_height,
        float steering_angle,
        TrajectoryType type = TrajectoryType::CYCLOID
    ) {
        if (type == TrajectoryType::CYCLOID) {
            return cycloid_trajectory_with_steering(phase, stride_length, step_height, steering_angle);
        } else {
            // 其他轨迹类型暂不支持转向
            return generate(phase, stride_length, step_height, type);
        }
    }
    
    /**
     * @brief 摆线轨迹（带转向支持）
     * 
     * @param phase 相位 (0-1)
     * @param stride_length 步长 (米)
     * @param step_height 抬腿高度 (米)
     * @param steering_angle 转向角度 (弧度)，正=左转，负=右转
     * @return TrajectoryPoint 包含x, y, z偏移
     */
    static TrajectoryPoint cycloid_trajectory_with_steering(
        float phase, 
        float stride_length = 0.05f, 
        float step_height = 0.03f,
        float steering_angle = 0.0f
    ) {
        TrajectoryPoint point;
        point.phase = phase;
        
        // 基础轨迹（前后）
        if (phase < 0.25f) {  // 摆动相（25%）
            float t = phase * 4 * M_PI;
            point.position.x = stride_length / 2 * (t / M_PI - 1);
            point.position.z = step_height * (1 - std::cos(t)) / 2;
            point.is_swing = true;
        } else {  // 支撑相（75%）
            float t = (phase - 0.25f) * 4 / 3;
            point.position.x = stride_length / 2 * (1 - 2 * t);
            point.position.z = 0;
            point.is_swing = false;
        }
        
        // 转向轨迹（侧向）
        if (std::abs(steering_angle) > 0.001f) {
            float steering_factor = std::tan(steering_angle);
            
            if (phase < 0.25f) {  // 摆动相
                point.position.y = stride_length * steering_factor * 0.3f * 
                                   std::sin(phase * 4 * M_PI);
            } else {  // 支撑相
                float t = (phase - 0.25f) * 4 / 3;
                point.position.y = stride_length * steering_factor * 0.2f * (1 - 2 * t);
            }
        } else {
            point.position.y = 0;
        }
        
        return point;
    }
    
    /**
     * @brief 验证轨迹闭合性
     * 
     * @param stride_length 步长
     * @param step_height 步高
     * @return 是否闭合（起点=终点）
     */
    static bool verify_trajectory_closed(
        float stride_length = 0.05f,
        float step_height = 0.03f
    ) {
        auto start = cycloid_trajectory(0.0f, stride_length, step_height);
        auto end = cycloid_trajectory(1.0f, stride_length, step_height);
        
        float x_error = std::abs(start.position.x - end.position.x);
        float z_error = std::abs(start.position.z - end.position.z);
        
        return (x_error < 0.0001f && z_error < 0.0001f);
    }
};

} // namespace Gait
} // namespace Robot
