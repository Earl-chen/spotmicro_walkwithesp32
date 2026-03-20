/**
 * @file gait_web_handler.cpp
 * @brief 步态控制的 Web/WebSocket 处理函数
 * 
 * 提供步态控制的 WebSocket 命令处理
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "cJSON.h"
#include "robot_bridge.h"

static const char* TAG = "GaitWebHandler";

/**
 * @brief 处理步态控制命令（JSON 格式）
 * 
 * @param json_str JSON 命令字符串
 * @param response_buf 响应缓冲区
 * @param buf_len 缓冲区长度
 * @return int 0=成功，负值=错误
 */
extern "C" int handle_gait_command(const char* json_str, char* response_buf, size_t buf_len) {
    if (!json_str || !response_buf || buf_len == 0) {
        return -1;
    }
    
    // 解析 JSON
    cJSON* root = cJSON_Parse(json_str);
    if (!root) {
        ESP_LOGE(TAG, "JSON 解析失败");
        snprintf(response_buf, buf_len, 
                 "{\"type\":\"error\",\"message\":\"Invalid JSON\"}");
        return -1;
    }
    
    // 获取命令
    cJSON* cmd_json = cJSON_GetObjectItem(root, "command");
    if (!cmd_json || !cJSON_IsString(cmd_json)) {
        cJSON_Delete(root);
        snprintf(response_buf, buf_len, 
                 "{\"type\":\"error\",\"message\":\"Missing command\"}");
        return -1;
    }
    
    const char* command = cmd_json->valuestring;
    cJSON* params = cJSON_GetObjectItem(root, "params");
    
    int result = 0;
    
    // 处理不同命令
    if (strcmp(command, "start_walk_gait") == 0) {
        // 启动 Walk 步态
        float stride_length = 0.05f;  // 默认 50mm
        float step_height = 0.03f;    // 默认 30mm
        float frequency = 0.8f;       // 默认 0.8Hz
        
        if (params) {
            cJSON* val;
            
            val = cJSON_GetObjectItem(params, "stride_length");
            if (val && cJSON_IsNumber(val)) {
                stride_length = val->valuedouble;
            }
            
            val = cJSON_GetObjectItem(params, "step_height");
            if (val && cJSON_IsNumber(val)) {
                step_height = val->valuedouble;
            }
            
            val = cJSON_GetObjectItem(params, "frequency");
            if (val && cJSON_IsNumber(val)) {
                frequency = val->valuedouble;
            }
        }
        
        result = robot_bridge_start_walk_gait(stride_length, step_height, frequency);
        
        if (result == 0) {
            snprintf(response_buf, buf_len, 
                     "{\"type\":\"ack\",\"message\":\"Walk gait started\"}");
        } else {
            snprintf(response_buf, buf_len, 
                     "{\"type\":\"error\",\"message\":\"Failed to start gait\"}");
        }
        
    } else if (strcmp(command, "stop_gait") == 0) {
        // 停止步态
        result = robot_bridge_stop_gait();
        
        if (result == 0) {
            snprintf(response_buf, buf_len, 
                     "{\"type\":\"ack\",\"message\":\"Gait stopped\"}");
        } else {
            snprintf(response_buf, buf_len, 
                     "{\"type\":\"error\",\"message\":\"Failed to stop gait\"}");
        }
        
    } else if (strcmp(command, "stand_pose") == 0) {
        // 站立姿势
        result = robot_bridge_set_preset_pose(1);
        
        if (result == 0) {
            snprintf(response_buf, buf_len, 
                     "{\"type\":\"ack\",\"message\":\"Stand pose executed\"}");
        } else {
            snprintf(response_buf, buf_len, 
                     "{\"type\":\"error\",\"message\":\"Failed to stand\"}");
        }
        
    } else if (strcmp(command, "get_gait_state") == 0) {
        // 获取步态状态
        robot_gait_state_t state;
        result = robot_bridge_get_gait_state(&state);
        
        if (result == 0) {
            snprintf(response_buf, buf_len,
                     "{\"type\":\"gait_state\",\"is_running\":%s,\"global_phase\":%.2f,"
                     "\"support_legs\":%d,\"stride_length\":%.4f,\"step_height\":%.4f,"
                     "\"frequency\":%.2f}",
                     state.is_running ? "true" : "false",
                     state.global_phase,
                     state.support_legs,
                     state.stride_length,
                     state.step_height,
                     state.frequency);
        } else {
            snprintf(response_buf, buf_len, 
                     "{\"type\":\"error\",\"message\":\"Failed to get state\"}");
        }
        
    } else {
        snprintf(response_buf, buf_len, 
                 "{\"type\":\"error\",\"message\":\"Unknown command: %s\"}", command);
        result = -1;
    }
    
    cJSON_Delete(root);
    return result;
}

/**
 * @brief 获取步态控制 HTML 页面内容
 * 
 * @return const char* HTML 内容（静态字符串）
 */
extern "C" const char* get_gait_control_html() {
    // 返回步态控制 HTML 页面
    // 实际实现中应该从文件系统或嵌入的 HTML 读取
    return 
        "<!DOCTYPE html>"
        "<html><head><meta charset=\"UTF-8\">"
        "<title>SpotMicro Gait Control</title>"
        "<body><h1>Gait Control Panel</h1>"
        "<p>Step 1: Start Walk Gait</p>"
        "<p>Step 2: Adjust Parameters</p>"
        "</body></html>";
}
