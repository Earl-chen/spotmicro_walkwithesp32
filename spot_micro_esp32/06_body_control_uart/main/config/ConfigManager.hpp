#pragma once

#include "RobotConfig.hpp"
#include <string>

namespace Robot {
namespace Config {

/**
 * @brief 配置管理器类
 *
 * 负责机器人配置的加载和验证
 * 简化版本，专注于核心配置管理
 */
class ConfigManager {
public:
    /**
     * @brief 构造函数
     */
    ConfigManager();

    /**
     * @brief 析构函数
     */
    ~ConfigManager();

    /**
     * @brief 加载默认配置
     * @return 是否成功
     */
    bool loadConfigs();

    /**
     * @brief 获取当前配置
     */
    const RobotConfig& getConfig() const { return config_; }

    /**
     * @brief 设置配置
     * @param config 新配置
     * @return 是否成功设置 (验证通过)
     */
    bool setConfig(const RobotConfig& config);

    /**
     * @brief 验证配置有效性
     * @return 是否有效
     */
    bool validateConfig() const;

    /**
     * @brief 获取配置错误信息
     */
    std::string getLastError() const { return last_error_; }

    // 便捷访问接口
    const AllServoConfig& getServoConfig() const { return config_.servo_config; }
    const MappingConfig& getMappingConfig() const { return config_.mapping_config; }
    const GeometryConfig& getGeometryConfig() const { return config_.geometry_config; }
    const ControlConfig& getControlConfig() const { return config_.control_config; }

    // 设置接口
    bool setServoConfig(const AllServoConfig& servo_config);
    bool setMappingConfig(const MappingConfig& mapping_config);
    bool setGeometryConfig(const GeometryConfig& geometry_config);
    bool setControlConfig(const ControlConfig& control_config);

private:
    RobotConfig config_;        // 当前配置
    std::string last_error_;    // 最后的错误信息

    /**
     * @brief 创建基于实际测试数据的舵机配置
     */
    AllServoConfig createActualConfig();

    /**
     * @brief 设置关节映射关系 (基于README映射)
     */
    void setupJointMappings();

    /**
     * @brief 设置错误信息
     */
    void setError(const std::string& error);
};

} // namespace Config
} // namespace Robot