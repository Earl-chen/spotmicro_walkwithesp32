/**
 * @file i2c_diagnostic.h
 * @brief I2C诊断工具头文件
 */

#pragma once

#include <cstdint>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 扫描I2C总线上的所有设备
 */
void i2c_scan_devices(void);

/**
 * @brief 检查特定地址的PCA9685设备状态
 * @param addr PCA9685设备地址 (默认0x40)
 * @return true 如果设备正常响应
 */
bool check_pca9685_device(uint8_t addr = 0x40);

/**
 * @brief 测试PCA9685的基础PWM输出功能
 * @param addr 设备地址
 * @param channel 测试通道 (0-15)
 * @param pwm_value PWM值 (0-4095)
 * @return true 如果测试成功
 */
bool test_pca9685_pwm_output(uint8_t addr, uint8_t channel, uint16_t pwm_value);

/**
 * @brief 初始化I2C诊断系统
 * @return ESP_OK 如果成功
 */
esp_err_t init_i2c_diagnostic(void);

#ifdef __cplusplus
}
#endif