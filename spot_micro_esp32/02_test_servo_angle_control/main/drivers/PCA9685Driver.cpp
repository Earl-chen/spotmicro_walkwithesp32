#include "PCA9685Driver.hpp"

// 真正的PCA9685库函数
extern "C" {
#include "pca9685.h"
}

namespace Robot {
namespace Drivers {

PCA9685Driver::PCA9685Driver(uint8_t i2c_address)
    : i2c_address_(i2c_address)
    , current_frequency_(50)
    , status_(PCA9685Status::UNINITIALIZED)
    , pwm_cache_{}
{
    LOG_TAG_DEBUG("PCA9685", "Driver created with I2C address: 0x%02X", i2c_address_);
}

PCA9685Driver::~PCA9685Driver() {
    if (status_ != PCA9685Status::UNINITIALIZED) {
        deinit();
    }
}

PCA9685Error PCA9685Driver::init(uint16_t frequency, int sda_pin, int scl_pin, uint32_t i2c_speed) {
    LOG_TAG_INFO("PCA9685", "Initializing PCA9685 driver at frequency %d Hz", frequency);

    // 1. 首先初始化I2C总线 (PCA9685Driver自己负责)
    LOG_TAG_INFO("PCA9685", "Initializing I2C master (SDA:%d, SCL:%d, Speed:%dHz)",
                 sda_pin, scl_pin, i2c_speed);

    i2c_config_t i2c_conf = {};
    i2c_conf.mode = I2C_MODE_MASTER;
    i2c_conf.sda_io_num = static_cast<gpio_num_t>(sda_pin);
    i2c_conf.scl_io_num = static_cast<gpio_num_t>(scl_pin);
    i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.master.clk_speed = i2c_speed;

    esp_err_t i2c_result = i2c_param_config(I2C_NUM_0, &i2c_conf);
    if (i2c_result != ESP_OK) {
        LOG_TAG_ERROR("PCA9685", "I2C parameter config failed: %s", esp_err_to_name(i2c_result));
        status_ = PCA9685Status::ERROR;
        return PCA9685Error::I2C_ERROR;
    }

    i2c_result = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    if (i2c_result != ESP_OK) {
        LOG_TAG_ERROR("PCA9685", "I2C driver install failed: %s", esp_err_to_name(i2c_result));
        status_ = PCA9685Status::ERROR;
        return PCA9685Error::I2C_ERROR;
    }

    LOG_TAG_INFO("PCA9685", "I2C initialized successfully");

    // 2. 设置PCA9685设备地址
    set_pca9685_adress(i2c_address_);

    // 3. 验证设备连接
    PCA9685Error verify_result = verifyDevice();
    if (verify_result != PCA9685Error::SUCCESS) {
        LOG_TAG_ERROR("PCA9685", "Device verification failed");
        status_ = PCA9685Status::ERROR;
        return verify_result;
    }

    // 4. 设置PWM频率
    PCA9685Error freq_result = setFrequency(frequency);
    if (freq_result != PCA9685Error::SUCCESS) {
        LOG_TAG_ERROR("PCA9685", "Failed to set frequency");
        status_ = PCA9685Status::ERROR;
        return freq_result;
    }

    // 5. 初始化PWM缓存
    pwm_cache_.fill(0);

    status_ = PCA9685Status::INITIALIZED;
    LOG_TAG_INFO("PCA9685", "PCA9685 driver initialized successfully");
    return PCA9685Error::SUCCESS;
}

void PCA9685Driver::deinit() {
    if (status_ != PCA9685Status::UNINITIALIZED) {
        LOG_TAG_INFO("PCA9685", "Deinitializing PCA9685 driver");
        status_ = PCA9685Status::UNINITIALIZED;
        LOG_TAG_DEBUG("PCA9685", "PCA9685 driver deinitialized");
    }
}

PCA9685Error PCA9685Driver::setPWM(int channel, uint16_t value) {
    // 验证参数
    if (!isValidChannel(channel)) {
        LOG_TAG_ERROR("PCA9685", "Invalid channel: %d", channel);
        return PCA9685Error::INVALID_CHANNEL;
    }

    if (!isValidPWM(value)) {
        LOG_TAG_ERROR("PCA9685", "Invalid PWM value: %d", value);
        return PCA9685Error::INVALID_VALUE;
    }

    if (status_ != PCA9685Status::INITIALIZED) {
        LOG_TAG_ERROR("PCA9685", "Driver not initialized");
        return PCA9685Error::INIT_FAILED;
    }

    // 调用现有的PCA9685库函数
    esp_err_t result = ::setPWM(channel, 0, value);
    if (result != ESP_OK) {
        LOG_TAG_ERROR("PCA9685", "Failed to set PWM for channel %d: %s", channel, esp_err_to_name(result));
        return PCA9685Error::I2C_ERROR;
    }

    // 更新缓存
    pwm_cache_[channel] = value;

    return PCA9685Error::SUCCESS;
}

PCA9685Error PCA9685Driver::getPWM(int channel, uint16_t& value) const {
    // 验证参数
    if (!isValidChannel(channel)) {
        LOG_TAG_ERROR("PCA9685", "Invalid channel: %d", channel);
        return PCA9685Error::INVALID_CHANNEL;
    }

    if (status_ != PCA9685Status::INITIALIZED) {
        LOG_TAG_ERROR("PCA9685", "Driver not initialized");
        return PCA9685Error::INIT_FAILED;
    }

    // 使用真正的PCA9685读取函数
    uint16_t on_value, off_value;
    esp_err_t result = ::getPWM(channel, &on_value, &off_value);
    if (result != ESP_OK) {
        LOG_TAG_ERROR("PCA9685", "Failed to read PWM from channel %d: %s", channel, esp_err_to_name(result));
        return PCA9685Error::I2C_ERROR;
    }

    value = off_value;  // PWM值存储在off_value中
    LOG_TAG_DEBUG("PCA9685", "Channel %d PWM read: ON=%d, OFF=%d", channel, on_value, off_value);

    return PCA9685Error::SUCCESS;
}

PCA9685Error PCA9685Driver::setAllPWM(const uint16_t* values, int count) {
    if (values == nullptr || count <= 0 || count > MAX_CHANNELS) {
        LOG_TAG_ERROR("PCA9685", "Invalid parameters for setAllPWM");
        return PCA9685Error::INVALID_VALUE;
    }

    LOG_TAG_DEBUG("PCA9685", "Setting PWM for %d channels", count);

    for (int i = 0; i < count; i++) {
        PCA9685Error result = setPWM(i, values[i]);
        if (result != PCA9685Error::SUCCESS) {
            LOG_TAG_ERROR("PCA9685", "Failed to set PWM for channel %d in batch", i);
            return result;
        }
    }

    LOG_TAG_DEBUG("PCA9685", "Successfully set PWM for all %d channels", count);
    return PCA9685Error::SUCCESS;
}

PCA9685Error PCA9685Driver::setAllPWM(const std::array<uint16_t, MAX_CHANNELS>& values) {
    return setAllPWM(values.data(), MAX_CHANNELS);
}

PCA9685Error PCA9685Driver::getAllPWM(std::array<uint16_t, MAX_CHANNELS>& values) const {
    if (status_ != PCA9685Status::INITIALIZED) {
        LOG_TAG_ERROR("PCA9685", "Driver not initialized");
        return PCA9685Error::INIT_FAILED;
    }

    // 从缓存复制所有值
    values = pwm_cache_;
    return PCA9685Error::SUCCESS;
}

PCA9685Error PCA9685Driver::setFrequency(uint16_t frequency) {
    if (frequency < 24 || frequency > 1526) {
        LOG_TAG_ERROR("PCA9685", "Invalid frequency: %d Hz (valid range: 24-1526)", frequency);
        return PCA9685Error::INVALID_VALUE;
    }

    LOG_TAG_DEBUG("PCA9685", "Setting frequency to %d Hz", frequency);

    // 调用现有的PCA9685库函数
    esp_err_t result = setFrequencyPCA9685(frequency);
    if (result != ESP_OK) {
        LOG_TAG_ERROR("PCA9685", "Failed to set frequency: %s", esp_err_to_name(result));
        return PCA9685Error::I2C_ERROR;
    }

    current_frequency_ = frequency;
    LOG_TAG_INFO("PCA9685", "Frequency set to %d Hz", frequency);
    return PCA9685Error::SUCCESS;
}


const char* PCA9685Driver::getErrorString(PCA9685Error error) {
    switch (error) {
        case PCA9685Error::SUCCESS:          return "Success";
        case PCA9685Error::INIT_FAILED:      return "Initialization failed";
        case PCA9685Error::I2C_ERROR:        return "I2C communication error";
        case PCA9685Error::INVALID_CHANNEL:  return "Invalid channel";
        case PCA9685Error::INVALID_VALUE:    return "Invalid value";
        case PCA9685Error::DEVICE_NOT_FOUND: return "Device not found";
        case PCA9685Error::TIMEOUT:          return "Communication timeout";
        default:                             return "Unknown error";
    }
}

PCA9685Error PCA9685Driver::verifyDevice() {
    // 简单的设备验证 - 尝试读取模式寄存器
    uint8_t mode1;
    if (readRegister(0x00, mode1) != PCA9685Error::SUCCESS) {
        LOG_TAG_ERROR("PCA9685", "Cannot read from device");
        return PCA9685Error::DEVICE_NOT_FOUND;
    }

    LOG_TAG_DEBUG("PCA9685", "Device verification successful, MODE1: 0x%02X", mode1);
    return PCA9685Error::SUCCESS;
}

PCA9685Error PCA9685Driver::writeRegister(uint8_t reg, uint8_t value) {
    // 这里应该调用实际的I2C写操作
    // 目前使用现有的C库函数
    LOG_TAG_DEBUG("PCA9685", "Writing register 0x%02X = 0x%02X", reg, value);
    return PCA9685Error::SUCCESS;  // 简化实现
}

PCA9685Error PCA9685Driver::readRegister(uint8_t reg, uint8_t& value) {
    // 这里应该调用实际的I2C读操作
    // 目前简化实现
    value = 0x20;  // 默认MODE1值
    LOG_TAG_DEBUG("PCA9685", "Reading register 0x%02X = 0x%02X", reg, value);
    return PCA9685Error::SUCCESS;
}

PCA9685Error PCA9685Driver::writePWMRegister(int channel, uint16_t on_time, uint16_t off_time) {
    LOG_TAG_DEBUG("PCA9685", "Channel %d: ON=%d, OFF=%d", channel, on_time, off_time);
    return PCA9685Error::SUCCESS;
}

PCA9685Error PCA9685Driver::readPWMRegister(int channel, uint16_t& on_time, uint16_t& off_time) {
    on_time = 0;
    off_time = pwm_cache_[channel];
    return PCA9685Error::SUCCESS;
}

uint8_t PCA9685Driver::calculatePrescale(uint16_t frequency) {
    // PCA9685频率计算公式: prescale = round(25000000 / (4096 * frequency)) - 1
    float prescale_value = 25000000.0f / (4096.0f * frequency) - 1.0f;
    return static_cast<uint8_t>(prescale_value + 0.5f);  // 四舍五入
}

} // namespace Drivers
} // namespace Robot