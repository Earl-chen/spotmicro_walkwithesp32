/**
 * @file websocket_handler.c
 * @brief WebSocket实时通信处理器实现
 * 
 * 实现双向实时通信功能，包括：
 * - WebSocket连接管理
 * - 实时状态推送
 * - 控制指令处理
 * - 心跳检测机制
 * 
 * @author ESP32四足机器人项目组
 * @date 2025-09-26
 */

#include "websocket_handler.h"
#include "robot_bridge.h"
#include <string.h>
#include <cJSON.h>
#include <sys/time.h>

static const char *TAG = "WEBSOCKET";
static float g_ws_default_beta_deg = 30.0f;

// 最大连接数
#define MAX_WS_CONNECTIONS 4
#define WS_HEARTBEAT_INTERVAL_MS 5000
#define WS_STATUS_PUSH_INTERVAL_MS 100  // 10Hz状态推送

// 全局变量
static ws_connection_t g_connections[MAX_WS_CONNECTIONS];
static QueueHandle_t g_ws_message_queue = NULL;
static TaskHandle_t g_status_task_handle = NULL;
static bool g_status_task_running = false;
static robot_status_t g_robot_status = {0};
// WebSocket异常日志节流（毫秒时间戳），避免日志洪泛阻塞httpd任务
static uint32_t g_last_ws_warn_ms = 0;

// 内部函数声明
static esp_err_t ws_send_frame(httpd_req_t *req, const char* data, size_t len, httpd_ws_type_t type);
static esp_err_t ws_send_frame(httpd_req_t *req, const char* data, size_t len, httpd_ws_type_t type)
{
    if (!req || !data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = type;
    ws_pkt.payload = (uint8_t*)data;
    ws_pkt.len = len;
    return httpd_ws_send_frame(req, &ws_pkt);
}
static esp_err_t ws_process_control_message(int fd, const char* data, size_t len);
static void ws_status_push_task(void *pvParameters);
static uint32_t get_timestamp_ms(void);
static int find_connection_by_fd(int fd);
static int add_connection(httpd_handle_t server, int fd);
static void remove_connection(int fd);
static esp_err_t ws_send_json_response(int fd, const char* status, const char* message);

/**
 * @brief 初始化WebSocket处理器
 */
esp_err_t websocket_handler_init(void)
{
    ESP_LOGI(TAG, "初始化WebSocket处理器...");

    // 初始化连接数组
    memset(g_connections, 0, sizeof(g_connections));
    for (int i = 0; i < MAX_WS_CONNECTIONS; i++) {
        g_connections[i].fd = -1;
    }

    // 创建消息队列
    g_ws_message_queue = xQueueCreate(10, sizeof(ws_message_t));
    if (!g_ws_message_queue) {
        ESP_LOGE(TAG, "创建WebSocket消息队列失败");
        return ESP_FAIL;
    }

    // 初始化机器人状态
    memset(&g_robot_status, 0, sizeof(g_robot_status));
    strcpy(g_robot_status.last_action, "初始化");

    ESP_LOGI(TAG, "✅ WebSocket处理器初始化成功");
    return ESP_OK;
}

/**
 * @brief 注册WebSocket处理器到HTTP服务器
 */
esp_err_t websocket_register_handlers(httpd_handle_t server)
{
    if (!server) {
        ESP_LOGE(TAG, "HTTP服务器句柄无效");
        return ESP_ERR_INVALID_ARG;
    }

    // WebSocket端点
    httpd_uri_t ws_uri = {
        .uri        = "/ws",
        .method     = HTTP_GET,
        .handler    = websocket_handler,
        .user_ctx   = NULL,
        .is_websocket = true
    };

    esp_err_t ret = httpd_register_uri_handler(server, &ws_uri);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "注册WebSocket处理器失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "✅ WebSocket处理器注册成功: /ws");
    return ESP_OK;
}

/**
 * @brief WebSocket连接处理器
 */
esp_err_t websocket_handler(httpd_req_t *req)
{
    // 握手阶段或普通GET到 /ws：不在此调用 ws_recv，交由 httpd 完成协议切换
    if (req->method == HTTP_GET) {
        return ESP_OK;
    }

    // 获取WebSocket帧
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    
    // 首先获取帧信息
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        uint32_t now_ms = get_timestamp_ms();
        if (now_ms - g_last_ws_warn_ms > 1000) {
            ESP_LOGW(TAG, "获取WebSocket帧信息失败: %s - 强制关闭会话", esp_err_to_name(ret));
            g_last_ws_warn_ms = now_ms;
        }
        // 强制关闭会话并清理，避免错误循环与日志洪泛
        int fd = httpd_req_to_sockfd(req);
        httpd_sess_trigger_close(req->handle, fd);
        remove_connection(fd);
        return ESP_OK; // 返回成功，避免httpd打印handler失败
    }

    ESP_LOGD(TAG, "WebSocket帧: type=%d, len=%d", ws_pkt.type, ws_pkt.len);

    if (ws_pkt.len) {
        // 分配缓冲区接收数据
        uint8_t *buf = calloc(1, ws_pkt.len + 1);
        if (!buf) {
            ESP_LOGE(TAG, "分配WebSocket缓冲区失败");
            return ESP_ERR_NO_MEM;
        }

        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            uint32_t now_ms = get_timestamp_ms();
            if (now_ms - g_last_ws_warn_ms > 1000) {
                ESP_LOGW(TAG, "接收WebSocket数据失败: %s - 强制关闭会话", esp_err_to_name(ret));
                g_last_ws_warn_ms = now_ms;
            }
            free(buf);
            // 强制关闭会话并清理，避免错误循环与日志洪泛
            int fd = httpd_req_to_sockfd(req);
            httpd_sess_trigger_close(req->handle, fd);
            remove_connection(fd);
            return ESP_OK; // 返回成功，避免httpd打印handler失败
        }
        
        // 验证帧的有效性
        if (ws_pkt.type == HTTPD_WS_TYPE_TEXT && ws_pkt.len > 0) {
            // 检查文本帧是否包含有效的UTF-8字符
            buf[ws_pkt.len] = '\0'; // 确保字符串终止
        }

        // 处理不同类型的帧
        switch (ws_pkt.type) {
            case HTTPD_WS_TYPE_TEXT: {
                ESP_LOGI(TAG, "📨 收到文本消息: %.*s", ws_pkt.len, (char*)ws_pkt.payload);
                
                // 添加连接到管理列表
                int fd = httpd_req_to_sockfd(req);
                add_connection(req->handle, fd);
                
                // 处理控制消息
                ret = ws_process_control_message(fd, (char*)ws_pkt.payload, ws_pkt.len);
                break;
            }
            
            case HTTPD_WS_TYPE_BINARY:
                ESP_LOGW(TAG, "收到二进制消息，暂不支持");
                break;
                
            case HTTPD_WS_TYPE_PING:
                ESP_LOGD(TAG, "收到PING，发送PONG");
                ws_pkt.type = HTTPD_WS_TYPE_PONG;
                ret = ws_send_frame(req, (const char*)ws_pkt.payload, ws_pkt.len, HTTPD_WS_TYPE_PONG);
                break;
                
            case HTTPD_WS_TYPE_PONG:
                ESP_LOGD(TAG, "收到PONG");
                break;
                
            case HTTPD_WS_TYPE_CLOSE:
                ESP_LOGI(TAG, "🔌 WebSocket连接关闭");
                remove_connection(httpd_req_to_sockfd(req));
                break;
                
            default:
                ESP_LOGW(TAG, "未知WebSocket帧类型: %d", ws_pkt.type);
                break;
        }

        free(buf);
    }

    return ret;
}

/**
 * @brief 处理控制消息
 */
static esp_err_t ws_process_control_message(int fd, const char* data, size_t len)
{
    // 解析JSON消息
    cJSON *json = cJSON_ParseWithLength(data, len);
    if (!json) {
        ESP_LOGE(TAG, "JSON解析失败");
        return websocket_send_error(fd, "Invalid JSON format");
    }

    cJSON *type_item = cJSON_GetObjectItem(json, "type");
    cJSON *data_item = cJSON_GetObjectItem(json, "data");
    
    if (!type_item || !cJSON_IsString(type_item)) {
        cJSON_Delete(json);
        return websocket_send_error(fd, "Missing or invalid 'type' field");
    }

    const char* msg_type = type_item->valuestring;
    ESP_LOGI(TAG, "🎮 处理控制消息类型: %s", msg_type);

    esp_err_t result = ESP_OK;

    if (strcmp(msg_type, "pose6") == 0) {
        // 6DOF姿态控制
        if (!data_item || !cJSON_IsObject(data_item)) {
            result = websocket_send_error(fd, "Missing pose6 data");
        } else {
            cJSON *x = cJSON_GetObjectItem(data_item, "x");
            cJSON *y = cJSON_GetObjectItem(data_item, "y");
            cJSON *z = cJSON_GetObjectItem(data_item, "z");
            cJSON *roll = cJSON_GetObjectItem(data_item, "roll");
            cJSON *pitch = cJSON_GetObjectItem(data_item, "pitch");
            cJSON *yaw = cJSON_GetObjectItem(data_item, "yaw");

            if (x && y && z && roll && pitch && yaw) {
                int bridge_result = robot_bridge_pose6(
                    (float)cJSON_GetNumberValue(x),
                    (float)cJSON_GetNumberValue(y),
                    (float)cJSON_GetNumberValue(z),
                    (float)cJSON_GetNumberValue(roll),
                    (float)cJSON_GetNumberValue(pitch),
                    (float)cJSON_GetNumberValue(yaw)
                );

                if (bridge_result == ROBOT_BRIDGE_OK) {
                    result = ws_send_json_response(fd, "ok", "Pose6 executed successfully");
                    strcpy(g_robot_status.last_action, "pose6");
                } else {
                    result = websocket_send_error(fd, "Pose6 execution failed");
                }
            } else {
                result = websocket_send_error(fd, "Invalid pose6 parameters");
            }
        }
    }
    else if (strcmp(msg_type, "action") == 0) {
        // 预定义动作默认β来自WS内部可配置值
        // 预定义动作
        if (!data_item || !cJSON_IsObject(data_item)) {
            result = websocket_send_error(fd, "Missing action data");
        } else {
            cJSON *action_id = cJSON_GetObjectItem(data_item, "id");
            cJSON *beta = cJSON_GetObjectItem(data_item, "beta");

            if (action_id && cJSON_IsNumber(action_id)) {
                int id = (int)cJSON_GetNumberValue(action_id);
                float beta_val = beta ? (float)cJSON_GetNumberValue(beta) : g_ws_default_beta_deg;

                if (id == 1) {
                    int bridge_result = robot_bridge_action_cone(beta_val);
                    if (bridge_result == ROBOT_BRIDGE_OK) {
                        result = ws_send_json_response(fd, "ok", "Cone action executed successfully");
                        strcpy(g_robot_status.last_action, "action_cone");
                    } else {
                        result = websocket_send_error(fd, "Cone action execution failed");
                    }
                } else {
                    result = websocket_send_error(fd, "Invalid action ID");
                }
            } else {
                result = websocket_send_error(fd, "Invalid action parameters");
            }
        }
    }
    else if (strcmp(msg_type, "preset") == 0) {
        // 预设姿势
        if (!data_item || !cJSON_IsObject(data_item)) {
            result = websocket_send_error(fd, "Missing preset data");
        } else {
            cJSON *preset_id = cJSON_GetObjectItem(data_item, "id");
            if (preset_id && cJSON_IsNumber(preset_id)) {
                int id = (int)cJSON_GetNumberValue(preset_id);
                int bridge_result = robot_bridge_set_preset_pose(id);
                
                if (bridge_result == ROBOT_BRIDGE_OK) {
                    result = ws_send_json_response(fd, "ok", "Preset pose executed successfully");
                    snprintf(g_robot_status.last_action, sizeof(g_robot_status.last_action), "preset_%d", id);
                } else {
                    result = websocket_send_error(fd, "Preset pose execution failed");
                }
            } else {
                result = websocket_send_error(fd, "Invalid preset parameters");
            }
        }
    }
    else if (strcmp(msg_type, "emergency") == 0) {
        // 紧急停止
        int bridge_result = robot_bridge_emergency_stop();
        if (bridge_result == ROBOT_BRIDGE_OK) {
            result = ws_send_json_response(fd, "ok", "Emergency stop executed");
            strcpy(g_robot_status.last_action, "emergency_stop");
        } else {
            result = websocket_send_error(fd, "Emergency stop failed");
        }
    }
    else if (strcmp(msg_type, "heartbeat") == 0) {
        // 心跳包：更新服务端连接的心跳时间戳，并回发确认
        int idx = find_connection_by_fd(fd);
        if (idx >= 0) {
            g_connections[idx].last_heartbeat = get_timestamp_ms();
        }
        result = websocket_send_heartbeat(fd);
    }
    else {
        ESP_LOGW(TAG, "未知消息类型: %s", msg_type);
        result = websocket_send_error(fd, "Unknown message type");
    }

    cJSON_Delete(json);
    return result;
}

/**
 * @brief 广播状态信息
 */
esp_err_t websocket_broadcast_status(const robot_status_t* status)
{
    if (!status) {
        return ESP_ERR_INVALID_ARG;
    }

    // 创建状态JSON
    cJSON *json = cJSON_CreateObject();
    cJSON *type = cJSON_CreateString("status");
    cJSON *data = cJSON_CreateObject();

    cJSON_AddItemToObject(json, "type", type);
    cJSON_AddItemToObject(json, "data", data);

    // 添加位置信息
    cJSON *position = cJSON_CreateArray();
    for (int i = 0; i < 3; i++) {
        cJSON_AddItemToArray(position, cJSON_CreateNumber(status->position[i]));
    }
    cJSON_AddItemToObject(data, "position", position);

    // 添加姿态信息
    cJSON *orientation = cJSON_CreateArray();
    for (int i = 0; i < 3; i++) {
        cJSON_AddItemToArray(orientation, cJSON_CreateNumber(status->orientation[i]));
    }
    cJSON_AddItemToObject(data, "orientation", orientation);

    // 添加其他状态信息
    cJSON_AddNumberToObject(data, "battery", status->battery_voltage);
    cJSON_AddNumberToObject(data, "uptime", status->uptime_ms);
    cJSON_AddStringToObject(data, "last_action", status->last_action);
    cJSON_AddNumberToObject(data, "timestamp", get_timestamp_ms());

    char *json_string = cJSON_Print(json);
    if (!json_string) {
        cJSON_Delete(json);
        return ESP_ERR_NO_MEM;
    }

    // 广播到所有连接的客户端
    int sent_count = 0;
    for (int i = 0; i < MAX_WS_CONNECTIONS; i++) {
        if (g_connections[i].connected && g_connections[i].fd >= 0) {
            httpd_ws_frame_t ws_pkt;
            memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
            ws_pkt.payload = (uint8_t*)json_string;
            ws_pkt.len = strlen(json_string);
            ws_pkt.type = HTTPD_WS_TYPE_TEXT;

            esp_err_t ret = httpd_ws_send_frame_async(g_connections[i].server, g_connections[i].fd, &ws_pkt);
            if (ret == ESP_OK) {
                sent_count++;
            } else {
                if (ret == ESP_ERR_INVALID_STATE) {
                    // 会话已关闭或不可写：静默标记断开，避免警告噪音
                    g_connections[i].connected = false;
                    ESP_LOGD(TAG, "客户端%d连接已关闭，停止推送", g_connections[i].fd);
                } else {
                    // 轻量限流日志，避免洪泛
                    uint32_t now_ms = get_timestamp_ms();
                    if (now_ms - g_last_ws_warn_ms > 1000) {
                        ESP_LOGW(TAG, "发送状态到客户端%d失败: %s - 触发关闭", g_connections[i].fd, esp_err_to_name(ret));
                        g_last_ws_warn_ms = now_ms;
                    }
                    // 触发会话关闭并标记断开，避免僵尸连接
                    if (g_connections[i].server) {
                        httpd_sess_trigger_close(g_connections[i].server, g_connections[i].fd);
                    }
                    g_connections[i].connected = false;
                }
            }
        }
    }

    free(json_string);
    cJSON_Delete(json);

    if (sent_count > 0) {
        ESP_LOGD(TAG, "状态广播到%d个客户端", sent_count);
    }

    return ESP_OK;
}

/**
 * @brief 发送错误信息
 */
esp_err_t websocket_send_error(int fd, const char* error_msg)
{
    return ws_send_json_response(fd, "error", error_msg);
}

/**
 * @brief 发送心跳包
 */
esp_err_t websocket_send_heartbeat(int fd)
{
    return ws_send_json_response(fd, "heartbeat", "pong");
}

/**
 * @brief 获取连接数
 */
int websocket_get_connection_count(void)
{
    int count = 0;
    for (int i = 0; i < MAX_WS_CONNECTIONS; i++) {
        if (g_connections[i].connected) {
            count++;
        }
    }
    return count;
}

/**
 * @brief 清理断开的连接
 */
void websocket_cleanup_connections(void)
{
    for (int i = 0; i < MAX_WS_CONNECTIONS; i++) {
        if (!g_connections[i].connected && g_connections[i].fd >= 0) {
            ESP_LOGI(TAG, "清理断开的连接: fd=%d", g_connections[i].fd);
            g_connections[i].fd = -1;
            g_connections[i].server = NULL;
        }
    }
}

/**
 * @brief 启动状态推送任务
 */
void websocket_start_status_task(void)
{
    if (!g_status_task_running) {
        g_status_task_running = true;
        xTaskCreate(ws_status_push_task, "ws_status_task", 4096, NULL, 5, &g_status_task_handle);
        ESP_LOGI(TAG, "✅ WebSocket状态推送任务已启动");
    }
}

/**
 * @brief 停止状态推送任务
 */
void websocket_stop_status_task(void)
{
    if (g_status_task_running) {
        g_status_task_running = false;
        if (g_status_task_handle) {
            vTaskDelete(g_status_task_handle);
            g_status_task_handle = NULL;
        }
        ESP_LOGI(TAG, "WebSocket状态推送任务已停止");
    }
}

// ==================== 内部函数实现 ====================

/**
 * @brief 状态推送任务
 */
static void ws_status_push_task(void *pvParameters)
{
    ESP_LOGI(TAG, "WebSocket状态推送任务开始运行");

    while (g_status_task_running) {
        // 更新机器人状态（这里可以从实际的机器人控制器获取状态）
        g_robot_status.uptime_ms = get_timestamp_ms();
        g_robot_status.battery_voltage = 12.5f; // 模拟电池电压

        // 如果有连接的客户端，则推送状态
        if (websocket_get_connection_count() > 0) {
            websocket_broadcast_status(&g_robot_status);
        }

        // 心跳超时检查：超过2倍心跳间隔(WS_HEARTBEAT_INTERVAL_MS*2)未收到心跳则主动关闭会话，避免僵尸连接占用资源
        uint32_t now_ms = get_timestamp_ms();
        for (int i = 0; i < MAX_WS_CONNECTIONS; i++) {
            if (g_connections[i].connected && g_connections[i].fd >= 0) {
                if (now_ms - g_connections[i].last_heartbeat > (WS_HEARTBEAT_INTERVAL_MS * 2)) {
                    ESP_LOGW(TAG, "心跳超时(> %d ms)，关闭并断开客户端: fd=%d", WS_HEARTBEAT_INTERVAL_MS * 2, g_connections[i].fd);
                    if (g_connections[i].server) {
                        httpd_sess_trigger_close(g_connections[i].server, g_connections[i].fd);
                    }
                    g_connections[i].connected = false;
                }
            }
        }

        // 清理断开的连接
        websocket_cleanup_connections();

        vTaskDelay(pdMS_TO_TICKS(WS_STATUS_PUSH_INTERVAL_MS));
    }

    ESP_LOGI(TAG, "WebSocket状态推送任务结束");
    vTaskDelete(NULL);
}

/**
 * @brief 获取时间戳
 */
static uint32_t get_timestamp_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint32_t)(tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

/**
 * @brief 根据fd查找连接
 */
static int find_connection_by_fd(int fd)
{
    for (int i = 0; i < MAX_WS_CONNECTIONS; i++) {
        if (g_connections[i].fd == fd) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief 添加连接
 */
static int add_connection(httpd_handle_t server, int fd)
{
    // 检查是否已存在
    int existing = find_connection_by_fd(fd);
    if (existing >= 0) {
        g_connections[existing].connected = true;
        g_connections[existing].last_heartbeat = get_timestamp_ms();
        return existing;
    }

    // 查找空闲位置
    for (int i = 0; i < MAX_WS_CONNECTIONS; i++) {
        if (g_connections[i].fd < 0) {
            g_connections[i].server = server;
            g_connections[i].fd = fd;
            g_connections[i].connected = true;
            g_connections[i].last_heartbeat = get_timestamp_ms();
            g_connections[i].message_count = 0;
            
            ESP_LOGI(TAG, "✅ 新连接已添加: fd=%d, slot=%d", fd, i);
            
            // 如果这是第一个连接，启动状态推送任务
            if (websocket_get_connection_count() == 1) {
                websocket_start_status_task();
            }
            
            return i;
        }
    }

    ESP_LOGW(TAG, "连接池已满，无法添加新连接: fd=%d", fd);
    return -1;
}

/**
 * @brief 移除连接
 */
static void remove_connection(int fd)
{
    int index = find_connection_by_fd(fd);
    if (index >= 0) {
        ESP_LOGI(TAG, "🔌 移除连接: fd=%d, slot=%d", fd, index);
        g_connections[index].fd = -1;
        g_connections[index].server = NULL;
        g_connections[index].connected = false;
        
        // 如果没有连接了，停止状态推送任务
        if (websocket_get_connection_count() == 0) {
            websocket_stop_status_task();
        }
    }
}

/**
 * @brief 发送JSON响应
 */
void websocket_set_default_beta(float beta_deg)
{
    if (beta_deg >= 5.0f && beta_deg <= 45.0f) {
        g_ws_default_beta_deg = beta_deg;
        ESP_LOGI(TAG, "WS默认β角更新为: %.1f°", beta_deg);
    } else {
        ESP_LOGW(TAG, "WS默认β角未更新，参数超出范围: %.1f°", beta_deg);
    }
}

static esp_err_t ws_send_json_response(int fd, const char* status, const char* message)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "status", status);
    cJSON_AddStringToObject(json, "message", message);
    cJSON_AddNumberToObject(json, "timestamp", get_timestamp_ms());

    char *json_string = cJSON_Print(json);
    if (!json_string) {
        cJSON_Delete(json);
        return ESP_ERR_NO_MEM;
    }

    // 查找连接
    int conn_index = find_connection_by_fd(fd);
    if (conn_index < 0 || !g_connections[conn_index].connected) {
        free(json_string);
        cJSON_Delete(json);
        return ESP_ERR_NOT_FOUND;
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*)json_string;
    ws_pkt.len = strlen(json_string);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    esp_err_t ret = httpd_ws_send_frame_async(g_connections[conn_index].server, fd, &ws_pkt);
    
    free(json_string);
    cJSON_Delete(json);

    if (ret == ESP_OK) {
        return ESP_OK;
    } else if (ret == ESP_ERR_INVALID_STATE) {
        // 连接已不可写或已关闭：静默处理并标记断开，避免错误噪音
        g_connections[conn_index].connected = false;
        ESP_LOGD(TAG, "WebSocket fd=%d已关闭，丢弃发送", fd);
        return ESP_OK;
    } else {
        // 轻量限流日志，避免洪泛
        uint32_t now_ms = get_timestamp_ms();
        if (now_ms - g_last_ws_warn_ms > 1000) {
            ESP_LOGW(TAG, "WebSocket发送失败: fd=%d, err=%s - 触发关闭", fd, esp_err_to_name(ret));
            g_last_ws_warn_ms = now_ms;
        }
        // 触发会话关闭并标记断开，避免僵尸连接
        httpd_sess_trigger_close(g_connections[conn_index].server, fd);
        g_connections[conn_index].connected = false;
        return ESP_OK;
    }
}