idf_component_register(
    SRCS
        "main.cpp"
        "drivers/test_servo_angle_control.cpp"
        "drivers/PCA9685Driver.cpp"
        "drivers/ServoDriver.cpp"
        "config/ConfigManager.cpp"
        "utils/UartTerminal.cpp"
        "utils/Logger.cpp"

    INCLUDE_DIRS
        "."
        "drivers"
        "config"
        "utils"

    REQUIRES driver esp_timer nvs_flash pca9685
)
