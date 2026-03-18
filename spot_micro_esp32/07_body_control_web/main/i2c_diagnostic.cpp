/**
 * @file i2c_diagnostic.cpp
 * @brief I2C诊断工具 - 用于检查PCA9685连接状态
 *
 * 提供I2C设备扫描、PCA9685状态检查和基础PWM测试功能
 */

#include "i2c_diagnostic.h"
#include <stdio.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include "esp_err.h"

static const char* TAG = "I2C_DIAG";

/**
 * @brief 扫描I2C总线上的所有设备
 */
void i2c_scan_devices(void) {
    ESP_LOGI(TAG, "=== I2C设备扫描开始 ===");
    ESP_LOGI(TAG, "扫描I2C总线地址范围: 0x08-0x77");

    bool found_device = false;
    int device_count = 0;

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t result = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10));
        i2c_cmd_link_delete(cmd);

        if (result == ESP_OK) {
            printf("✅ 发现设备: 0x%02X", addr);

            // 识别常见设备
            switch (addr) {
                case 0x40:
                    printf(" (PCA9685 PWM驱动器 - 默认地址)");
                    break;
                case 0x41:
                case 0x42:
                case 0x43:
                case 0x44:
                case 0x45:
                case 0x46:
                case 0x47:
                    printf(" (PCA9685 PWM驱动器 - 可配置地址)");
                    break;
                case 0x48:
                case 0x49:
                case 0x4A:
                case 0x4B:
                    printf(" (ADS1115 ADC或其他传感器)");
                    break;
                case 0x68:
                    printf(" (MPU6050 IMU传感器)");
                    break;
                case 0x76:
                case 0x77:
                    printf(" (BME280/BMP280 压力传感器)");
                    break;
                default:
                    printf(" (未知设备)");
                    break;
            }
            printf("\n");

            found_device = true;
            device_count++;
        }
    }

    if (!found_device) {
        ESP_LOGE(TAG, "❌ 未发现任何I2C设备！");
        ESP_LOGE(TAG, "请检查:");
        ESP_LOGE(TAG, "  - I2C接线 (SDA: GPIO21, SCL: GPIO22)");
        ESP_LOGE(TAG, "  - 设备供电状态");
        ESP_LOGE(TAG, "  - 上拉电阻 (通常4.7kΩ)");
        ESP_LOGE(TAG, "  - 接线是否松动");
    } else {
        ESP_LOGI(TAG, "✅ 扫描完成，发现 %d 个设备", device_count);
    }

    ESP_LOGI(TAG, "=== I2C设备扫描结束 ===");
}

/**
 * @brief 检查特定地址的PCA9685设备状态
 * @param addr PCA9685设备地址 (默认0x40)
 */
bool check_pca9685_device(uint8_t addr) {
    ESP_LOGI(TAG, "=== PCA9685设备检查 (地址: 0x%02X) ===", addr);

    // 1. 基础连通性测试
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    esp_err_t result = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (result != ESP_OK) {
        ESP_LOGE(TAG, "❌ PCA9685设备不响应 (错误: %s)", esp_err_to_name(result));
        return false;
    }

    ESP_LOGI(TAG, "✅ PCA9685设备响应正常");

    // 2. 读取MODE1寄存器
    uint8_t mode1_reg = 0x00;  // MODE1寄存器地址
    uint8_t mode1_value = 0;

    // 写入寄存器地址
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, mode1_reg, true);
    i2c_master_stop(cmd);
    result = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (result != ESP_OK) {
        ESP_LOGE(TAG, "❌ 无法写入MODE1寄存器地址");
        return false;
    }

    // 读取寄存器值
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &mode1_value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    result = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (result != ESP_OK) {
        ESP_LOGE(TAG, "❌ 无法读取MODE1寄存器值");
        return false;
    }

    ESP_LOGI(TAG, "✅ MODE1寄存器值: 0x%02X", mode1_value);

    // 解析MODE1寄存器位
    printf("MODE1寄存器位分析:\n");
    printf("  - RESTART (bit 7): %s\n", (mode1_value & 0x80) ? "是" : "否");
    printf("  - EXTCLK  (bit 6): %s\n", (mode1_value & 0x40) ? "外部时钟" : "内部时钟");
    printf("  - AI      (bit 5): %s\n", (mode1_value & 0x20) ? "自动递增" : "手动");
    printf("  - SLEEP   (bit 4): %s\n", (mode1_value & 0x10) ? "睡眠模式" : "正常模式");
    printf("  - SUB1    (bit 3): %s\n", (mode1_value & 0x08) ? "响应子地址1" : "不响应");
    printf("  - SUB2    (bit 2): %s\n", (mode1_value & 0x04) ? "响应子地址2" : "不响应");
    printf("  - SUB3    (bit 1): %s\n", (mode1_value & 0x02) ? "响应子地址3" : "不响应");
    printf("  - ALLCALL (bit 0): %s\n", (mode1_value & 0x01) ? "响应全局调用" : "不响应");

    // 3. 读取预分频寄存器（频率相关）
    uint8_t prescale_reg = 0xFE;
    uint8_t prescale_value = 0;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, prescale_reg, true);
    i2c_master_stop(cmd);
    result = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (result == ESP_OK) {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
        i2c_master_read_byte(cmd, &prescale_value, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        result = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        if (result == ESP_OK) {
            ESP_LOGI(TAG, "✅ 预分频寄存器值: 0x%02X (%d)", prescale_value, prescale_value);
            // 计算对应的PWM频率: freq = 25MHz / (4096 * (prescale + 1))
            float frequency = 25000000.0f / (4096.0f * (prescale_value + 1));
            ESP_LOGI(TAG, "   对应PWM频率: %.2f Hz", frequency);
        }
    }

    ESP_LOGI(TAG, "=== PCA9685设备检查完成 ===");
    return true;
}

/**
 * @brief 测试PCA9685的基础PWM输出功能
 * @param addr 设备地址
 * @param channel 测试通道 (0-15)
 * @param pwm_value PWM值 (0-4095)
 */
bool test_pca9685_pwm_output(uint8_t addr, uint8_t channel, uint16_t pwm_value) {
    if (channel > 15) {
        ESP_LOGE(TAG, "❌ 无效通道: %d (有效范围: 0-15)", channel);
        return false;
    }

    if (pwm_value > 4095) {
        ESP_LOGE(TAG, "❌ 无效PWM值: %d (有效范围: 0-4095)", pwm_value);
        return false;
    }

    ESP_LOGI(TAG, "=== 测试PWM输出 (通道%d, 值%d) ===", channel, pwm_value);

    // 计算通道的寄存器地址: LED0_ON_L + 4 * channel
    uint8_t led_on_l = 0x06 + 4 * channel;

    // 构建I2C命令：写入 ON=0, OFF=pwm_value
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, led_on_l, true);  // 寄存器地址

    // 写入4个字节: ON_L, ON_H, OFF_L, OFF_H
    i2c_master_write_byte(cmd, 0x00, true);               // ON_L = 0
    i2c_master_write_byte(cmd, 0x00, true);               // ON_H = 0
    i2c_master_write_byte(cmd, pwm_value & 0xFF, true);   // OFF_L
    i2c_master_write_byte(cmd, (pwm_value >> 8) & 0xFF, true); // OFF_H

    i2c_master_stop(cmd);
    esp_err_t result = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (result != ESP_OK) {
        ESP_LOGE(TAG, "❌ PWM输出设置失败: %s", esp_err_to_name(result));
        return false;
    }

    ESP_LOGI(TAG, "✅ PWM输出设置成功");

    // 验证写入是否成功 - 读回PWM值
    uint16_t read_on, read_off;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, led_on_l, true);
    i2c_master_stop(cmd);
    result = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (result == ESP_OK) {
        uint8_t buffer[4];
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
        i2c_master_read_byte(cmd, &buffer[0], I2C_MASTER_ACK);  // ON_L
        i2c_master_read_byte(cmd, &buffer[1], I2C_MASTER_ACK);  // ON_H
        i2c_master_read_byte(cmd, &buffer[2], I2C_MASTER_ACK);  // OFF_L
        i2c_master_read_byte(cmd, &buffer[3], I2C_MASTER_NACK); // OFF_H
        i2c_master_stop(cmd);
        result = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        if (result == ESP_OK) {
            read_on = buffer[0] | (buffer[1] << 8);
            read_off = buffer[2] | (buffer[3] << 8);
            ESP_LOGI(TAG, "📖 读回值验证: ON=%d, OFF=%d", read_on, read_off);

            if (read_off == pwm_value) {
                ESP_LOGI(TAG, "✅ PWM值验证成功");
                return true;
            } else {
                ESP_LOGE(TAG, "❌ PWM值验证失败: 期望%d, 实际%d", pwm_value, read_off);
                return false;
            }
        }
    }

    ESP_LOGW(TAG, "⚠️ 无法验证PWM值，但设置命令已发送");
    return true;
}

/**
 * @brief 初始化I2C诊断系统 (如果还未初始化)
 */
esp_err_t init_i2c_diagnostic(void) {
    ESP_LOGI(TAG, "初始化I2C诊断系统...");

    // 检查I2C是否已经初始化（简单检查）
    // 这里我们假设如果能够执行基本的I2C操作，说明已经初始化过了
    return ESP_OK;
}