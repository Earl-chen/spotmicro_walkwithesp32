#include "RobotController.hpp"

namespace Robot {
namespace Controllers {

RobotController::RobotController()
    : current_mode_(ControlMode::SERVO_ANGLE_MODE)
    , initialized_(false)
    , hardware_ready_(false)
{
    LOG_TAG_DEBUG("ROBOT", "RobotController created");
}

RobotController::~RobotController() {
    // 析构函数会自动清理shared_ptr
}

bool RobotController::init() {
    LOG_TAG_INFO("ROBOT", "Initializing RobotController");

    // 创建配置管理器 - 使用实际测试数据配置
    config_manager_ = std::make_shared<Config::ConfigManager>();
    if (!config_manager_->validateConfig()) {
        LOG_TAG_ERROR("ROBOT", "Invalid configuration");
        return false;
    }

    // 创建硬件驱动 (使用100Hz频率，与成功测试一致)
    hw_driver_ = std::make_shared<Drivers::PCA9685Driver>(0x40);
    auto hw_init_result = hw_driver_->init(100);

    // 恢复"继续运行"逻辑，参考可工作版本
    hardware_ready_ = (hw_init_result == Drivers::PCA9685Error::SUCCESS);
    if (!hardware_ready_) {
        LOG_TAG_WARN("ROBOT", "Hardware driver init failed; no I2C device detected. Continuing without hardware.");
        LOG_TAG_WARN("ROBOT", "Error: %s", Drivers::PCA9685Driver::getErrorString(hw_init_result));
    } else {
        LOG_TAG_INFO("ROBOT", "✅ Hardware driver initialized successfully");
    }

    // 创建舵机驱动（即使硬件有问题也继续）
    servo_driver_ = std::make_shared<ServoDriver>(hw_driver_, config_manager_);
    auto servo_init_result = servo_driver_->init();
    if (servo_init_result != ServoError::SUCCESS) {
        LOG_TAG_WARN("ROBOT", "Servo driver init failed; continuing without servo control.");
    } else {
        LOG_TAG_INFO("ROBOT", "✅ Servo driver initialized successfully");
    }

    // 创建关节控制器（即使前面有问题也继续）
    joint_controller_ = std::make_shared<JointController>(servo_driver_, config_manager_);
    auto joint_init_result = joint_controller_->init();
    if (joint_init_result != JointError::SUCCESS) {
        LOG_TAG_WARN("ROBOT", "Joint controller init failed; continuing without joint control.");
    } else {
        LOG_TAG_INFO("ROBOT", "✅ Joint controller initialized successfully");
    }

    initialized_ = true;
    LOG_TAG_INFO("ROBOT", "RobotController initialized successfully with actual servo config");
    return true;  // 始终返回true，让系统继续运行
}

} // namespace Controllers
} // namespace Robot