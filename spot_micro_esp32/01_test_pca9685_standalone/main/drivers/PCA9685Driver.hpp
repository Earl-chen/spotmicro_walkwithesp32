#pragma once

#include <cstdint>
#include <array>
#include "../utils/Logger.hpp"

namespace Robot {
namespace Drivers {

/**
 * @brief PCA9685硬件驱动错误码
 */
enum class PCA9685Error {
    SUCCESS = 0,           // 成功
    INIT_FAILED = 1,       // 初始化失败
    I2C_ERROR = 2,         // I2C通信错误
    INVALID_CHANNEL = 3,   // 无效通道
    INVALID_VALUE = 4,     // 无效PWM值
    DEVICE_NOT_FOUND = 5,  // 设备未找到
    TIMEOUT = 6            // 通信超时
};

/**
 * @brief PCA9685驱动状态
 */
enum class PCA9685Status {
    UNINITIALIZED = 0,     // 未初始化
    INITIALIZED = 1,       // 已初始化
    ERROR = 2              // 错误状态
};

/**
 * @brief PCA9685硬件抽象驱动
 *
 * 纯硬件操作层，不包含业务逻辑
 * 负责与PCA9685芯片的直接通信
 */
class PCA9685Driver {
public:
    static constexpr int MAX_CHANNELS = 16;
    static constexpr uint16_t PWM_MIN = 0;
    static constexpr uint16_t PWM_MAX = 4095;
    static constexpr uint8_t DEFAULT_I2C_ADDRESS = 0x40;

    /**
     * @brief 构造函数
     * @param i2c_address I2C设备地址 (默认0x40)
     */
    explicit PCA9685Driver(uint8_t i2c_address = DEFAULT_I2C_ADDRESS);

    /**
     * @brief 析构函数
     */
    ~PCA9685Driver();

    /**
     * @brief 初始化PCA9685
     * @param frequency PWM频率 (默认50Hz，适合舵机)
     * @param sda_pin SDA引脚 (默认GPIO21)
     * @param scl_pin SCL引脚 (默认GPIO22)
     * @param i2c_speed I2C速度 (默认100kHz)
     * @return 错误码
     */
    PCA9685Error init(uint16_t frequency = 50, int sda_pin = 21, int scl_pin = 22, uint32_t i2c_speed = 100000);

    /**
     * @brief 反初始化，释放资源
     */
    void deinit();

    /**
     * @brief 设置单个通道PWM值
     * @param channel 通道编号 (0-15)
     * @param value PWM值 (0-4095)
     * @return 错误码
     */
    PCA9685Error setPWM(int channel, uint16_t value);

    /**
     * @brief 获取单个通道PWM值
     * @param channel 通道编号 (0-15)
     * @param value 输出PWM值
     * @return 错误码
     */
    PCA9685Error getPWM(int channel, uint16_t& value) const;

    /**
     * @brief 批量设置PWM值
     * @param values PWM值数组 (最多16个通道)
     * @param count 设置的通道数量
     * @return 错误码
     */
    PCA9685Error setAllPWM(const uint16_t* values, int count);

    /**
     * @brief 批量设置PWM值 (std::array版本)
     * @param values PWM值数组
     * @return 错误码
     */
    PCA9685Error setAllPWM(const std::array<uint16_t, MAX_CHANNELS>& values);

    /**
     * @brief 获取所有通道PWM值
     * @param values 输出PWM值数组
     * @return 错误码
     */
    PCA9685Error getAllPWM(std::array<uint16_t, MAX_CHANNELS>& values) const;

    /**
     * @brief 设置PWM频率
     * @param frequency 频率值 (24-1526 Hz)
     * @return 错误码
     */
    PCA9685Error setFrequency(uint16_t frequency);

    /**
     * @brief 获取当前PWM频率
     * @return 当前频率
     */
    uint16_t getFrequency() const { return current_frequency_; }



    /**
     * @brief 获取设备状态
     */
    PCA9685Status getStatus() const { return status_; }

    /**
     * @brief 获取I2C地址
     */
    uint8_t getI2CAddress() const { return i2c_address_; }

    /**
     * @brief 检查通道是否有效
     */
    bool isValidChannel(int channel) const {
        return channel >= 0 && channel < MAX_CHANNELS;
    }

    /**
     * @brief 检查PWM值是否有效
     */
    bool isValidPWM(uint16_t value) const {
        return value <= PWM_MAX;
    }

    /**
     * @brief 获取错误描述
     */
    static const char* getErrorString(PCA9685Error error);

private:
    uint8_t i2c_address_;                           // I2C设备地址
    uint16_t current_frequency_;                    // 当前PWM频率
    PCA9685Status status_;                          // 驱动状态
    std::array<uint16_t, MAX_CHANNELS> pwm_cache_; // PWM值缓存

    /**
     * @brief 验证设备连接
     */
    PCA9685Error verifyDevice();

    /**
     * @brief 写入寄存器
     */
    PCA9685Error writeRegister(uint8_t reg, uint8_t value);

    /**
     * @brief 读取寄存器
     */
    PCA9685Error readRegister(uint8_t reg, uint8_t& value);

    /**
     * @brief 写入PWM寄存器
     */
    PCA9685Error writePWMRegister(int channel, uint16_t on_time, uint16_t off_time);

    /**
     * @brief 读取PWM寄存器
     */
    PCA9685Error readPWMRegister(int channel, uint16_t& on_time, uint16_t& off_time);

    /**
     * @brief 计算频率预分频值
     */
    uint8_t calculatePrescale(uint16_t frequency);
};

} // namespace Drivers
} // namespace Robot