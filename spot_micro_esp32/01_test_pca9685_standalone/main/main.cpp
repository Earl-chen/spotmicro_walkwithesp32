/**
 * @file main.cpp
 * @brief PCA9685独立测试项目入口
 *
 * 🎯 测试目标：验证PCA9685 PWM驱动功能
 *
 * 硬件连接：
 * - SDA: GPIO21
 * - SCL: GPIO22
 * - PCA9685地址: 0x40
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// 声明测试函数
extern void test_pca9685_driver_refactored();

static const char* TAG = "MAIN";

extern "C" void app_main() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  PCA9685 独立测试程序");
    ESP_LOGI(TAG, "  硬件: ESP32 + PCA9685");
    ESP_LOGI(TAG, "  I2C: SDA=GPIO21, SCL=GPIO22");
    ESP_LOGI(TAG, "========================================");

    // 等待串口连接稳定
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 运行测试
    test_pca9685_driver_refactored();

    ESP_LOGI(TAG, "测试程序结束");
}
