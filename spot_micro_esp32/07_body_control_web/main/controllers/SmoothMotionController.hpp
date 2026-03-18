#pragma once

#include "PoseInterpolator.hpp"
#include "JointSmoother.hpp"
#include "RobotController.hpp"
#include "../kinematics/QuadrupedModel.hpp"
#include "../config/SmoothMotionConfig.hpp"
#include "../RobotTypes.hpp"
#include "../utils/Logger.hpp"
#include <memory>

namespace Robot {
namespace Controllers {

/**
 * @brief 运动状态
 */
enum class MotionState {
    IDLE,           // 空闲
    INTERPOLATING,  // 正在插值
    SMOOTHING,      // 正在平滑关节
    ERROR           // 错误状态
};

/**
 * @brief 统一平滑运动控制器
 *
 * 线性流水线架构：
 * SmoothMotionController → PoseInterpolator → QuadrupedModel → JointSmoother → ServoDriver
 *
 * 提供完整的从机体姿态到硬件输出的平滑控制流程
 */
class SmoothMotionController {
private:
    // 线性流水线组件
    std::unique_ptr<PoseInterpolator> pose_interpolator_;
    std::shared_ptr<Robot::Kinematics::QuadrupedModel> quadruped_model_;
    std::unique_ptr<JointSmoother> joint_smoother_;
    std::shared_ptr<RobotController> robot_controller_;

    // 配置和状态
    SmoothMotionConfig config_;
    MotionState current_state_;
    uint32_t motion_step_count_;

    static const char* TAG;

public:
    /**
     * @brief 构造函数
     */
    SmoothMotionController(std::shared_ptr<RobotController> robot_controller);

    /**
     * @brief 构造函数，带配置
     */
    SmoothMotionController(std::shared_ptr<RobotController> robot_controller,
                          const SmoothMotionConfig& config);

    /**
     * @brief 析构函数
     */
    ~SmoothMotionController();

    /**
     * @brief 初始化控制器
     */
    bool initialize();

    /**
     * @brief 设置QuadrupedModel（注入依赖）
     */
    void setQuadrupedModel(std::shared_ptr<Robot::Kinematics::QuadrupedModel> model);

    /**
     * @brief 设置目标机体姿态 (平滑过渡)
     * @param pose 目标姿态 (位置: m, 角度: 弧度)
     */
    bool setTargetPose(const BodyPose& pose);

    /**
     * @brief 立即设置机体姿态 (不平滑)
     */
    bool setCurrentPose(const BodyPose& pose);

    /**
     * @brief 执行一步运动更新
     * @return true: 还需要继续更新，false: 运动完成
     */
    bool stepMotion();

    /**
     * @brief 执行完整运动直到到达目标
     */
    bool executeMotionToTarget();

    /**
     * @brief 停止所有运动
     */
    void stopMotion();

    /**
     * @brief 紧急停止 (立即停止)
     */
    void emergencyStop();

    /**
     * @brief 获取当前运动状态
     */
    MotionState getMotionState() const { return current_state_; }

    /**
     * @brief 获取当前机体姿态
     */
    BodyPose getCurrentPose() const;

    /**
     * @brief 获取目标机体姿态
     */
    BodyPose getTargetPose() const;

    /**
     * @brief 检查是否正在运动
     */
    bool isMoving() const;

    /**
     * @brief 获取运动进度 (0.0 - 1.0)
     */
    float getMotionProgress() const;

    /**
     * @brief 预设动作
     */
    bool executeStandAction();
    bool executeRelaxAction();

    /**
     * @brief 配置管理
     */
    void setConfig(const SmoothMotionConfig& config);
    const SmoothMotionConfig& getConfig() const { return config_; }

    /**
     * @brief 启用/禁用功能
     */
    void enablePoseSmoothing(bool enable);
    void enableServoSmoothing(bool enable);

    /**
     * @brief 重置控制器
     */
    void reset();

    /**
     * @brief 调试信息输出
     */
    void printStatus() const;
    void printDetailedStatus() const;

    /**
     * @brief 获取腿部工作空间信息
     */
    void printWorkspaceInfo() const;
};

} // namespace Controllers
} // namespace Robot