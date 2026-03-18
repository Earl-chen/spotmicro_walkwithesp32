#pragma once

#include "../drivers/PCA9685Driver.hpp"
#include "../drivers/ServoDriver.hpp"
#include "../controllers/JointController.hpp"
#include "../config/ConfigManager.hpp"
#include "../utils/Logger.hpp"
#include <memory>

namespace Robot {
namespace Controllers {

/**
 * @brief 机器人统一控制器
 *
 * 集成所有控制层，提供统一的接口
 */
class RobotController {
public:
    enum class ControlMode {
        PWM_MODE = 0,
        SERVO_ANGLE_MODE = 1,
        JOINT_ANGLE_MODE = 2
    };

    /**
     * @brief 构造函数
     */
    RobotController();

    /**
     * @brief 析构函数
     */
    ~RobotController();

    /**
     * @brief 初始化机器人控制器
     */
    bool init();

    /**
     * @brief 设置控制模式
     */
    void setControlMode(ControlMode mode) { current_mode_ = mode; }

    /**
     * @brief 获取控制模式
     */
    ControlMode getControlMode() const { return current_mode_; }

    // 访问各个控制层
    std::shared_ptr<Drivers::PCA9685Driver> getHardwareDriver() { return hw_driver_; }
    std::shared_ptr<ServoDriver> getServoDriver() { return servo_driver_; }
    std::shared_ptr<JointController> getJointController() { return joint_controller_; }
    std::shared_ptr<Config::ConfigManager> getConfigManager() { return config_manager_; }

private:
    ControlMode current_mode_;

    std::shared_ptr<Drivers::PCA9685Driver> hw_driver_;
    std::shared_ptr<ServoDriver> servo_driver_;
    std::shared_ptr<JointController> joint_controller_;
    std::shared_ptr<Config::ConfigManager> config_manager_;
    bool initialized_;
};

} // namespace Controllers
} // namespace Robot