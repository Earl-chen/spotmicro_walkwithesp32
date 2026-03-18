idf_component_register(
    SRCS
        "main.cpp"
        "test_smooth_motion_controller.cpp"
        "config/ConfigManager.cpp"
        "controllers/JointController.cpp"
        "controllers/JointSmoother.cpp"
        "controllers/PoseInterpolator.cpp"
        "controllers/RobotController.cpp"
        "controllers/SmoothMotionController.cpp"
        "drivers/PCA9685Driver.cpp"
        "drivers/ServoDriver.cpp"
        "kinematics/CoordinateTransform.cpp"
        "kinematics/KinematicsGeometry.cpp"
        "kinematics/Leg.cpp"
        "kinematics/LegKinematics.cpp"
        "kinematics/QuadrupedModel.cpp"
        "utils/Logger.cpp"
        "utils/UartTerminal.cpp"

    INCLUDE_DIRS
        "."
        "drivers"
        "config"
        "controllers"
        "kinematics"
        "utils"

    REQUIRES driver esp_timer nvs_flash pca9685
)
