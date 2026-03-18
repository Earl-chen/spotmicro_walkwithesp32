idf_component_register(
    SRCS
        # 主程序 (v4.0.0 新架构版本)
        "pca9685Test.cpp"

        # 配置系统
        "config/ConfigManager.cpp"

        # 工具系统
        "utils/Logger.cpp"

        # 驱动层
        "drivers/PCA9685Driver.cpp"
        "drivers/ServoDriver.cpp"

        # 控制层
        "controllers/JointController.cpp"
        "controllers/RobotController.cpp"
        "controllers/SmoothMotionController.cpp"  # 新增：平滑运动控制器
        "controllers/PoseInterpolator.cpp"        # 新增：姿态插值器
        "controllers/JointSmoother.cpp"           # 新增：关节平滑器

        # 运动学层
        "kinematics/CoordinateTransform.cpp"
        "kinematics/LegKinematics.cpp"
        "kinematics/KinematicsGeometry.cpp"
        "kinematics/QuadrupedModel.cpp"           # 新增：四足机器人模型
        "kinematics/Leg.cpp"                     # 新增：腿部模块

    INCLUDE_DIRS
        "."
        "config"
        "utils"
        "drivers"
        "controllers"
        "kinematics"

    REQUIRES pca9685
)

 