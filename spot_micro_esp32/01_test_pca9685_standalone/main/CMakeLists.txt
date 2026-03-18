idf_component_register(
    SRCS
        "main.cpp"
        "drivers/test_pca9685_driver_refactored.cpp"
        "drivers/PCA9685Driver.cpp"
        "utils/UartTerminal.cpp"
        "utils/Logger.cpp"

    INCLUDE_DIRS
        "."
        "drivers"
        "utils"

    REQUIRES driver esp_timer nvs_flash pca9685
)
