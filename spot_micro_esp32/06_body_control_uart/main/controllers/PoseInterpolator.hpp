#pragma once

#include "../RobotTypes.hpp"
#include "../utils/Logger.hpp"
#include "../config/SmoothMotionConfig.hpp"
#include <memory>

namespace Robot {
namespace Controllers {

/**
 * @brief 机体姿态结构体
 */
struct BodyPose {
    // 位置 (m)
    float x, y, z;
    // 欧拉角 (弧度)
    float roll, pitch, yaw;

    BodyPose() : x(0), y(0), z(0), roll(0), pitch(0), yaw(0) {}
    BodyPose(float x_, float y_, float z_, float roll_, float pitch_, float yaw_)
        : x(x_), y(y_), z(z_), roll(roll_), pitch(pitch_), yaw(yaw_) {}

    bool operator==(const BodyPose& other) const {
        const float tolerance = 1e-6f;
        return (std::abs(x - other.x) < tolerance &&
                std::abs(y - other.y) < tolerance &&
                std::abs(z - other.z) < tolerance &&
                std::abs(roll - other.roll) < tolerance &&
                std::abs(pitch - other.pitch) < tolerance &&
                std::abs(yaw - other.yaw) < tolerance);
    }

    bool operator!=(const BodyPose& other) const {
        return !(*this == other);
    }
};

/**
 * @brief 姿态平滑插值器
 *
 * 实现机体姿态的平滑过渡，避免运动过程中的突然变化
 * 基于新增功能.txt中的iterate_to_position函数设计
 */
class PoseInterpolator {
private:
    BodyPose current_pose_;    // 当前姿态
    BodyPose target_pose_;     // 目标姿态
    InterpolationConfig config_; // 插值配置
    bool is_moving_;           // 是否正在运动
    uint32_t current_step_;    // 当前步数

    static const char* TAG;

    /**
     * @brief 计算单个维度的插值步长
     */
    float calculateStep(float current, float target, float max_step) const;

    /**
     * @brief 检查是否达到目标
     */
    bool hasReachedTarget() const;

public:
    /**
     * @brief 构造函数
     */
    PoseInterpolator();

    /**
     * @brief 构造函数，带配置参数
     */
    explicit PoseInterpolator(const InterpolationConfig& config);

    /**
     * @brief 析构函数
     */
    ~PoseInterpolator();

    /**
     * @brief 设置当前姿态
     */
    void setCurrentPose(const BodyPose& pose);

    /**
     * @brief 设置目标姿态并开始插值
     */
    void setTargetPose(const BodyPose& pose);

    /**
     * @brief 获取当前姿态
     */
    const BodyPose& getCurrentPose() const { return current_pose_; }

    /**
     * @brief 获取目标姿态
     */
    const BodyPose& getTargetPose() const { return target_pose_; }

    /**
     * @brief 执行一步插值
     * @return true: 还需要继续插值，false: 已达到目标
     */
    bool stepInterpolation();

    /**
     * @brief 检查是否正在运动
     */
    bool isMoving() const { return is_moving_; }

    /**
     * @brief 停止插值运动
     */
    void stopInterpolation();

    /**
     * @brief 立即跳转到目标姿态
     */
    void jumpToTarget();

    /**
     * @brief 更新插值配置
     */
    void setConfig(const InterpolationConfig& config);

    /**
     * @brief 获取插值配置
     */
    const InterpolationConfig& getConfig() const { return config_; }

    /**
     * @brief 获取插值进度 (0.0 - 1.0)
     */
    float getProgress() const;

    /**
     * @brief 重置到初始状态
     */
    void reset();

    /**
     * @brief 调试信息输出
     */
    void printStatus() const;
};

} // namespace Controllers
} // namespace Robot