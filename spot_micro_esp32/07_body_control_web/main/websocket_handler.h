/**
 * @file websocket_handler.h
 * @brief WebSocket实时通信处理器头文件
 * 
 * 提供双向实时通信功能，支持：
 * - 实时状态推送
 * - 控制指令接收
 * - 连接状态管理
 * 
 * @author ESP32四足机器人项目组
 * @date 2025-09-26
 */

#ifndef WEBSOCKET_HANDLER_H
#define WEBSOCKET_HANDLER_H

#include <esp_http_server.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#ifdef __cplusplus
extern "C" {
#endif

// WebSocket消息类型
typedef enum {
    WS_MSG_TYPE_CONTROL = 1,    // 控制指令
    WS_MSG_TYPE_STATUS = 2,     // 状态信息
    WS_MSG_TYPE_ERROR = 3,      // 错误信息
    WS_MSG_TYPE_HEARTBEAT = 4   // 心跳包
} ws_msg_type_t;

// WebSocket控制指令类型
typedef enum {
    WS_CTRL_POSE6 = 1,          // 6DOF姿态控制
    WS_CTRL_ACTION = 2,         // 预定义动作
    WS_CTRL_PRESET = 3,         // 预设姿势
    WS_CTRL_EMERGENCY = 4       // 紧急停止
} ws_ctrl_type_t;

// WebSocket消息结构
typedef struct {
    ws_msg_type_t type;
    char* data;  // 改为指针，动态分配
    size_t data_len;
    size_t data_capacity;
    httpd_handle_t server;
    int fd;
} ws_message_t;

// WebSocket连接状态
typedef struct {
    httpd_handle_t server;
    int fd;
    bool connected;
    uint32_t last_heartbeat;
    uint32_t message_count;
} ws_connection_t;

// 机器人状态信息
typedef struct {
    float position[3];          // x, y, z
    float orientation[3];       // roll, pitch, yaw
    float joint_angles[12];     // 12个关节角度
    bool servo_status[12];      // 舵机状态
    float battery_voltage;      // 电池电压
    uint32_t uptime_ms;         // 运行时间
    char last_action[32];       // 最后执行的动作
} robot_status_t;

/**
 * @brief 初始化WebSocket处理器
 * @return ESP_OK 成功，其他值失败
 */
esp_err_t websocket_handler_init(void);
void websocket_set_default_beta(float beta_deg);

/**
 * @brief 注册WebSocket处理器到HTTP服务器
 * @param server HTTP服务器句柄
 * @return ESP_OK 成功，其他值失败
 */
esp_err_t websocket_register_handlers(httpd_handle_t server);

/**
 * @brief WebSocket连接处理器
 * @param req HTTP请求
 * @return ESP_OK 成功，其他值失败
 */
esp_err_t websocket_handler(httpd_req_t *req);

/**
 * @brief 广播状态信息到所有连接的客户端
 * @param status 机器人状态信息
 * @return ESP_OK 成功，其他值失败
 */
esp_err_t websocket_broadcast_status(const robot_status_t* status);

/**
 * @brief 发送错误信息到指定客户端
 * @param fd 客户端文件描述符
 * @param error_msg 错误消息
 * @return ESP_OK 成功，其他值失败
 */
esp_err_t websocket_send_error(int fd, const char* error_msg);

/**
 * @brief 发送心跳包到指定客户端
 * @param fd 客户端文件描述符
 * @return ESP_OK 成功，其他值失败
 */
esp_err_t websocket_send_heartbeat(int fd);

/**
 * @brief 获取当前连接数
 * @return 连接数
 */
int websocket_get_connection_count(void);

/**
 * @brief 清理断开的连接
 */
void websocket_cleanup_connections(void);

/**
 * @brief 启动状态推送任务
 */
void websocket_start_status_task(void);

/**
 * @brief 停止状态推送任务
 */
void websocket_stop_status_task(void);

#ifdef __cplusplus
}
#endif

#endif // WEBSOCKET_HANDLER_H