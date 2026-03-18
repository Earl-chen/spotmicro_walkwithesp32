/**
 * @file robot_bridge.h
 * @brief 机器人控制桥接层头文件
 * 
 * 提供C接口，连接网页控制和C++机器人控制系统
 * 支持pose6平滑姿态控制和action_cone圆锥动作
 * 
 * @author ESP32四足机器人项目组
 * @date 2025-09-26
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// 错误码定义
#define ROBOT_BRIDGE_OK           0     // 成功
#define ROBOT_BRIDGE_ERROR_PARAM  -1    // 参数错误
#define ROBOT_BRIDGE_ERROR_IO     -2    // 驱动错误
#define ROBOT_BRIDGE_ERROR_BUSY   -3    // 控制器忙
#define ROBOT_BRIDGE_ERROR_UNINIT -4    // 未初始化

/**
 * @brief 姿态状态结构体
 */
typedef struct {
    float x;        // X位置 (米)
    float y;        // Y位置 (米) 
    float z;        // Z位置 (米)
    float roll;     // Roll角度 (度)
    float pitch;    // Pitch角度 (度)
    float yaw;      // Yaw角度 (度)
} robot_pose_state_t;

/**
 * @brief 初始化机器人桥接系统
 * 
 * 调用init_robot_system()和initSmoothMotionSystem()
 * 必须在其他接口调用前执行
 * 
 * @return 0=成功，负值=错误码
 */
int robot_bridge_init(void);

/**
 * @brief 检查桥接系统是否已初始化
 * 
 * @return true=已初始化，false=未初始化
 */
bool robot_bridge_is_initialized(void);

/**
 * @brief 6DOF平滑姿态控制
 * 
 * 设置机器人目标姿态，执行平滑运动控制
 * 角度参数自动转换为弧度，位置参数进行安全限幅
 * 
 * @param x X位置 (米，范围：-0.12 ~ 0.12)
 * @param y Y位置 (米，范围：-0.12 ~ 0.12) 
 * @param z Z位置 (米，范围：-0.22 ~ -0.05)
 * @param roll_deg Roll角度 (度，范围：-180 ~ 180)
 * @param pitch_deg Pitch角度 (度，范围：-180 ~ 180)
 * @param yaw_deg Yaw角度 (度，范围：-180 ~ 180)
 * @return 0=成功，负值=错误码
 */
int robot_bridge_pose6(float x, float y, float z, 
                       float roll_deg, float pitch_deg, float yaw_deg);

/**
 * @brief 圆锥连续动作控制
 * 
 * 执行圆锥轨迹运动：起始位姿 → 36点轨迹 → 回中性位姿
 * 
 * @param beta_deg 圆锥半顶角 (度，范围：5 ~ 45)
 * @return 0=成功，负值=错误码
 */
int robot_bridge_action_cone(float beta_deg);

/**
 * @brief 获取当前姿态状态
 * 
 * @param pose 输出当前姿态状态
 * @return 0=成功，负值=错误码
 */
int robot_bridge_get_current_pose(robot_pose_state_t* pose);

/**
 * @brief 紧急停止所有运动
 * 
 * @return 0=成功，负值=错误码
 */
int robot_bridge_emergency_stop(void);

/**
 * @brief 获取最后的错误信息
 * 
 * @return 错误信息字符串
 */
const char* robot_bridge_get_last_error(void);

// ========== 扩展接口（可选实现） ==========

/**
 * @brief 设置单个关节角度
 * 
 * @param leg_id 腿部ID (0-3: 左前、右前、左后、右后)
 * @param joint_id 关节ID (0-2: 髋侧摆、髋俯仰、膝俯仰)
 * @param angle_deg 关节角度 (度)
 * @return 0=成功，负值=错误码
 */
int robot_bridge_set_joint_angle(int leg_id, int joint_id, float angle_deg);

/**
 * @brief 设置单个舵机角度
 * 
 * @param servo_id 舵机ID (0-11)
 * @param angle_deg 舵机角度 (度，0-180)
 * @return 0=成功，负值=错误码
 */
int robot_bridge_set_servo_angle(int servo_id, float angle_deg);

/**
 * @brief 设置单个舵机PWM值
 * 
 * @param channel PWM通道 (0-15)
 * @param pwm_value PWM值 (150-4095)
 * @return 0=成功，负值=错误码
 */
int robot_bridge_set_pwm(int channel, uint16_t pwm_value);

/**
 * @brief 执行预设姿势
 * 
 * @param pose_id 姿势ID (0=趴下, 1=站立)
 * @return 0=成功，负值=错误码
 */
int robot_bridge_set_preset_pose(int pose_id);

#ifdef __cplusplus
}
#endif