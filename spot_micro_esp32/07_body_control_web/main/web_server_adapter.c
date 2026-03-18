/**
 * @file web_server_adapter.c
 * @brief HTTP服务器适配层实现
 * 
 * 处理网页控制请求，调用robot_bridge接口
 * 复用simple工程的HTML界面和路由设计
 * 
 * @author ESP32四足机器人项目组
 * @date 2025-09-26
 */

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <esp_log.h>
#include <sys/param.h>
#include "esp_netif.h"
#include <esp_http_server.h>
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_tls.h"
#include "esp_check.h"
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// 统一网络门控辅助函数（来自主程序）
extern bool network_is_available_cached(void);

// 机器人桥接层
#include "robot_bridge.h"
#include "websocket_handler.h"

#define EXAMPLE_HTTP_QUERY_KEY_MAX_LEN  (64)

static const char *TAG = "WEB_ADAPTER";

// 单实例保护
static httpd_handle_t g_server_instance = NULL;
static bool g_server_running = false;
// 防重入：正在启动标志
static bool g_server_starting = false;

// 网络统计结构
typedef struct {
    uint32_t total_connections;
    uint32_t active_connections;
    uint32_t send_errors;
    uint32_t recv_errors;
    uint32_t last_error_time;
} http_stats_t;

static http_stats_t g_http_stats = {0};

// 网络可用性检测：STA优先，其次AP，判断是否已获取有效IP
static bool is_network_available(void)
{
    // 统一使用缓存的网络可用性判断，避免各模块逻辑不一致
    return network_is_available_cached();
}

// ========== 幂等重试封装（最多3次，指数退避100ms→200ms→400ms） ==========

static int retry_robot_pose6(float x, float y, float z, float roll, float pitch, float yaw) {
    for (int attempt = 0; attempt < 3; ++attempt) {
        int r = robot_bridge_pose6(x, y, z, roll, pitch, yaw);
        if (r == ROBOT_BRIDGE_OK) return r;
        vTaskDelay(pdMS_TO_TICKS(100 * (1 << attempt)));
    }
    return ROBOT_BRIDGE_ERROR_IO;
}

static int retry_action_cone(float beta_deg) {
    for (int attempt = 0; attempt < 3; ++attempt) {
        int r = robot_bridge_action_cone(beta_deg);
        if (r == ROBOT_BRIDGE_OK) return r;
        vTaskDelay(pdMS_TO_TICKS(100 * (1 << attempt)));
    }
    return ROBOT_BRIDGE_ERROR_IO;
}

static int retry_preset(int preset_id) {
    for (int attempt = 0; attempt < 3; ++attempt) {
        int r = robot_bridge_set_preset_pose(preset_id);
        if (r == ROBOT_BRIDGE_OK) return r;
        vTaskDelay(pdMS_TO_TICKS(100 * (1 << attempt)));
    }
    return ROBOT_BRIDGE_ERROR_IO;
}

static int retry_emergency(void) {
    // 紧急停止优先尝试一次即可，随后快速重试两次以确保触达
    for (int attempt = 0; attempt < 3; ++attempt) {
        int r = robot_bridge_emergency_stop();
        if (r == ROBOT_BRIDGE_OK) return r;
        vTaskDelay(pdMS_TO_TICKS(50 * (1 << attempt)));
    }
    return ROBOT_BRIDGE_ERROR_IO;
}

// 姿态状态全局变量
static robot_pose_state_t g_pose_state = {0.05f, 0.0f, -0.10f, 0.0f, 0.0f, 0.0f};
static float g_cone_beta_deg = 30.0f;

// HTML页面内容（复用simple工程的界面设计）

#if 0
"<!doctype html>\n"
"<html lang=\"zh-CN\">\n"
"<head>\n"
"  <meta charset=\"utf-8\" />\n"
"  <meta name=\"viewport\" content=\"width=device-width,initial-scale=1\" />\n"
"  <title>四足机器人控制系统</title>\n"
"  <style>\n"
"    :root{--gap:16px;--card-bg:#ffffff;--border:#cfcfcf;--accent:#1565C0;--highlight:#00BCD4}\n"
"    html,body{height:100%;margin:0;font-family:Inter, -apple-system, BlinkMacSystemFont, \"Segoe UI\", Roboto, \"Helvetica Neue\", Arial;background:linear-gradient(135deg, #f5f7fa 0%, #c3cfe2 100%);color:var(--accent)}\n"
"    .container{max-width:1000px;margin:20px auto;padding:20px;box-sizing:border-box}\n"
"    header.title{text-align:center;font-size:28px;font-weight:700;margin-bottom:30px;color:var(--accent);text-shadow:0 2px 4px rgba(0,0,0,0.1)}\n"
"    .status-bar{background:var(--accent);color:white;padding:12px 20px;border-radius:8px;margin-bottom:20px;display:flex;justify-content:space-between;align-items:center;box-shadow:0 4px 12px rgba(21,101,192,0.3)}\n"
"    .status-indicator{display:flex;align-items:center;gap:8px}\n"
"    .status-dot{width:12px;height:12px;border-radius:50%;background:#4CAF50;animation:pulse 2s infinite}\n"
"    @keyframes pulse{0%,100%{opacity:1}50%{opacity:0.5}}\n"
"    .card{background:var(--card-bg);border:none;border-radius:12px;padding:20px;margin-bottom:20px;box-shadow:0 8px 32px rgba(0,0,0,0.1);transition:transform 0.2s ease}\n"
"    .card:hover{transform:translateY(-2px)}\n"
"    .card .card-head{margin-bottom:16px;border-bottom:2px solid #f0f0f0;padding-bottom:12px}\n"
"    .card .card-head h2{margin:0;font-size:18px;color:var(--accent);font-weight:600}\n"
"    .controls-grid{display:grid;grid-template-columns:1fr 1fr;gap:20px}\n"
"    .sliders{display:flex;flex-direction:column;gap:16px}\n"
"    .slider-row{display:flex;flex-direction:column;gap:6px;padding:12px;border:1px solid #e8e8e8;border-radius:8px;background:#fafafa;transition:border-color 0.2s ease}\n"
"    .slider-row:hover{border-color:var(--highlight)}\n"
"    .slider-label{display:flex;justify-content:space-between;font-size:14px;font-weight:500}\n"
"    .slider-value{color:var(--highlight);font-weight:600}\n"
"    .slider-range{display:flex;justify-content:space-between;align-items:center;width:100%;font-size:12px;color:#666;margin-top:4px}\n"
"    input[type=range]{width:100%;height:6px;border-radius:3px;background:#ddd;outline:none;-webkit-appearance:none;margin:8px 0}\n"
"    input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;appearance:none;width:20px;height:20px;border-radius:50%;background:var(--highlight);cursor:pointer;box-shadow:0 2px 6px rgba(0,188,212,0.4)}\n"
"    input[type=range]::-moz-range-thumb{width:20px;height:20px;border-radius:50%;background:var(--highlight);cursor:pointer;border:none;box-shadow:0 2px 6px rgba(0,188,212,0.4)}\n"
"    .actions-grid{display:grid;grid-template-columns:repeat(auto-fit, minmax(200px, 1fr));gap:12px}\n"
"    .action-btn{display:flex;align-items:center;justify-content:center;height:56px;border:2px solid var(--highlight);border-radius:8px;background:linear-gradient(135deg, var(--highlight) 0%, #26C6DA 100%);color:white;cursor:pointer;font-weight:600;font-size:16px;transition:all 0.2s ease;box-shadow:0 4px 12px rgba(0,188,212,0.3)}\n"
"    .action-btn:hover{transform:translateY(-2px);box-shadow:0 6px 20px rgba(0,188,212,0.4)}\n"
"    .action-btn:active{transform:scale(0.98)}\n"
"    .emergency-btn{background:linear-gradient(135deg, #f44336 0%, #e57373 100%);border-color:#f44336;box-shadow:0 4px 12px rgba(244,67,54,0.3)}\n"
"    .emergency-btn:hover{box-shadow:0 6px 20px rgba(244,67,54,0.4)}\n"
"    .note{font-size:13px;color:#777;margin-top:12px;padding:8px;background:#f9f9f9;border-radius:6px}\n"
"    .status-msg{position:fixed;top:20px;right:20px;padding:12px 20px;background:#4CAF50;color:white;border-radius:8px;font-size:14px;font-weight:500;display:none;z-index:1000;box-shadow:0 4px 12px rgba(76,175,80,0.3)}\n"
"    .status-msg.error{background:#f44336;box-shadow:0 4px 12px rgba(244,67,54,0.3)}\n"
"    @media (max-width: 768px){.controls-grid{grid-template-columns:1fr}.actions-grid{grid-template-columns:1fr}}\n"
"  </style>\n"
"</head>\n"
"<body>\n"
"  <div id=\"status-msg\" class=\"status-msg\">命令已发送</div>\n"
"  <div class=\"container\">\n"
"    <header class=\"title\">🤖 四足机器人控制系统</header>\n"
"    \n"
"    <div class=\"status-bar\">\n"
"      <div class=\"status-indicator\">\n"
"        <div class=\"status-dot\"></div>\n"
"        <span>系统在线</span>\n"
"      </div>\n"
"      <div id=\"current-time\">--:--:--</div>\n"
"    </div>\n"
"\n"
"    <div class=\"controls-grid\">\n"
"      <section class=\"card\" aria-labelledby=\"position-title\">\n"
"        <div class=\"card-head\">\n"
"          <h2 id=\"position-title\">📍 位置控制</h2>\n"
"        </div>\n"
"        <div class=\"sliders\">\n"
"          <div class=\"slider-row\">\n"
"            <div class=\"slider-label\"><label for=\"pos-x\">X轴位置</label><span class=\"slider-value\" id=\"val-x\">0</span></div>\n"
"            <input id=\"pos-x\" type=\"range\" min=\"-80\" max=\"80\" value=\"0\" />\n"
"            <div class=\"slider-range\"><span>-80mm</span><span>80mm</span></div>\n"
"          </div>\n"
"          <div class=\"slider-row\">\n"
"            <div class=\"slider-label\"><label for=\"pos-y\">Y轴位置</label><span class=\"slider-value\" id=\"val-y\">0</span></div>\n"
"            <input id=\"pos-y\" type=\"range\" min=\"-80\" max=\"80\" value=\"0\" />\n"
"            <div class=\"slider-range\"><span>-80mm</span><span>80mm</span></div>\n"
"          </div>\n"
"          <div class=\"slider-row\">\n"
"            <div class=\"slider-label\"><label for=\"pos-z\">Z轴位置</label><span class=\"slider-value\" id=\"val-z\">0</span></div>\n"
"            <input id=\"pos-z\" type=\"range\" min=\"-220\" max=\"0\" value=\"0\" />\n"
"            <div class=\"slider-range\"><span>-220mm</span><span>0mm</span></div>\n"
"          </div>\n"
"        </div>\n"
"      </section>\n"
"\n"
"      <section class=\"card\" aria-labelledby=\"attitude-title\">\n"
"        <div class=\"card-head\">\n"
"          <h2 id=\"attitude-title\">🎯 姿态控制</h2>\n"
"        </div>\n"
"        <div class=\"sliders\">\n"
"          <div class=\"slider-row\">\n"
"            <div class=\"slider-label\"><label for=\"rot-roll\">Roll翻滚</label><span class=\"slider-value\" id=\"val-roll\">0°</span></div>\n"
"            <input id=\"rot-roll\" type=\"range\" min=\"-45\" max=\"45\" value=\"0\" />\n"
"            <div class=\"slider-range\"><span>-45°</span><span>45°</span></div>\n"
"          </div>\n"
"          <div class=\"slider-row\">\n"
"            <div class=\"slider-label\"><label for=\"rot-pitch\">Pitch俯仰</label><span class=\"slider-value\" id=\"val-pitch\">0°</span></div>\n"
"            <input id=\"rot-pitch\" type=\"range\" min=\"-45\" max=\"45\" value=\"0\" />\n"
"            <div class=\"slider-range\"><span>-45°</span><span>45°</span></div>\n"
"          </div>\n"
"          <div class=\"slider-row\">\n"
"            <div class=\"slider-label\"><label for=\"rot-yaw\">Yaw偏航</label><span class=\"slider-value\" id=\"val-yaw\">0°</span></div>\n"
"            <input id=\"rot-yaw\" type=\"range\" min=\"-45\" max=\"45\" value=\"0\" />\n"
"            <div class=\"slider-range\"><span>-45°</span><span>45°</span></div>\n"
"          </div>\n"
"        </div>\n"
"      </section>\n"
"    </div>\n"
"\n"
"    <section class=\"card\" aria-labelledby=\"preset-title\">\n"
"      <div class=\"card-head\">\n"
"        <h2 id=\"preset-title\">🎮 预定义动作</h2>\n"
"      </div>\n"
"      <div class=\"actions-grid\" role=\"list\">\n"
"        <div class=\"action-btn\" role=\"button\" tabindex=\"0\" data-action=\"1\">🌀 圆锥动作</div>\n"
"        <div class=\"action-btn\" role=\"button\" tabindex=\"0\" data-preset=\"0\">😴 趴下休息</div>\n"
"        <div class=\"action-btn\" role=\"button\" tabindex=\"0\" data-preset=\"1\">🚶 站立准备</div>\n"
"        <div class=\"action-btn emergency-btn\" role=\"button\" tabindex=\"0\" data-emergency=\"1\">🚨 紧急停止</div>\n"
"      </div>\n"
"\n"
"      <div class=\"slider-row\">\n"
"        <div class=\"slider-label\"><label for=\"beta-angle\">β角度</label><span class=\"slider-value\" id=\"val-beta\">30°</span></div>\n"
"        <input id=\"beta-angle\" type=\"range\" min=\"5\" max=\"45\" value=\"30\" />\n"
"        <div class=\"slider-range\"><span>5°</span><span>45°</span></div>\n"
"      </div>\n"
"\n"
"      <div class=\"note\">\n"
"        💡 提示：滑块控制实时姿态，动作按钮执行预设动作。紧急停止可立即中断所有运动。\n"
"      </div>\n"
"    </section>\n"
"  </div>\n"
"\n"
"  <script>\n"
"    // WebSocket连接管理\n"
"    let ws = null;\n"
"    let wsConnected = false;\n"
"    let reconnectTimer = null;\n"
"    let heartbeatTimer = null;\n"
"\n"
"    // 状态显示函数\n"
"    function showStatus(msg, isError = false) {\n"
"      const status = document.getElementById('status-msg');\n"
"      status.textContent = msg || '命令已发送';\n"
"      status.className = 'status-msg' + (isError ? ' error' : '');\n"
"      status.style.display = 'block';\n"
"      setTimeout(() => status.style.display = 'none', 3000);\n"
"    }\n"
"\n"
"    // WebSocket连接函数\n"
"    function connectWebSocket() {\n"
"      const wsUrl = 'ws://' + window.location.host + '/ws';\n"
"      console.log('连接WebSocket:', wsUrl);\n"
"      \n"
"      ws = new WebSocket(wsUrl);\n"
"      \n"
"      ws.onopen = function(event) {\n"
"        console.log('✅ WebSocket连接成功');\n"
"        wsConnected = true;\n"
"        updateConnectionStatus(true);\n"
"        showStatus('🔗 实时连接已建立');\n"
"        \n"
"        // 启动心跳\n"
"        startHeartbeat();\n"
"        \n"
"        // 清除重连定时器\n"
"        if (reconnectTimer) {\n"
"          clearTimeout(reconnectTimer);\n"
"          reconnectTimer = null;\n"
"        }\n"
"      };\n"
"      \n"
"      ws.onmessage = function(event) {\n"
"        try {\n"
"          const message = JSON.parse(event.data);\n"
"          handleWebSocketMessage(message);\n"
"        } catch (e) {\n"
"          console.error('WebSocket消息解析失败:', e);\n"
"        }\n"
"      };\n"
"      \n"
"      ws.onclose = function(event) {\n"
"        console.log('🔌 WebSocket连接关闭:', event.code, event.reason);\n"
"        wsConnected = false;\n"
"        updateConnectionStatus(false);\n"
"        stopHeartbeat();\n"
"        \n"
"        // 自动重连\n"
"        if (!reconnectTimer) {\n"
"          reconnectTimer = setTimeout(() => {\n"
"            console.log('🔄 尝试重连WebSocket...');\n"
"            connectWebSocket();\n"
"          }, 3000);\n"
"        }\n"
"      };\n"
"      \n"
"      ws.onerror = function(error) {\n"
"        console.error('❌ WebSocket错误:', error);\n"
"        showStatus('🔌 连接错误，正在重试...', true);\n"
"      };\n"
"    }\n"
"\n"
"    // 更新连接状态显示\n"
"    function updateConnectionStatus(connected) {\n"
"      const statusDot = document.querySelector('.status-dot');\n"
"      const statusText = statusDot.nextElementSibling;\n"
"      \n"
"      if (connected) {\n"
"        statusDot.style.background = '#4CAF50';\n"
"        statusText.textContent = '实时连接';\n"
"      } else {\n"
"        statusDot.style.background = '#f44336';\n"
"        statusText.textContent = '连接断开';\n"
"      }\n"
"    }\n"
"\n"
"    // 启动心跳\n"
"    function startHeartbeat() {\n"
"      heartbeatTimer = setInterval(() => {\n"
"        if (wsConnected && ws.readyState === WebSocket.OPEN) {\n"
"          sendWebSocketMessage('heartbeat', {});\n"
"        }\n"
"      }, 30000); // 30秒心跳\n"
"    }\n"
"\n"
"    // 停止心跳\n"
"    function stopHeartbeat() {\n"
"      if (heartbeatTimer) {\n"
"        clearInterval(heartbeatTimer);\n"
"        heartbeatTimer = null;\n"
"      }\n"
"    }\n"
"\n"
"    // 发送WebSocket消息\n"
"    function sendWebSocketMessage(type, data) {\n"
"      if (wsConnected && ws.readyState === WebSocket.OPEN) {\n"
"        const message = {\n"
"          type: type,\n"
"          data: data,\n"
"          timestamp: Date.now()\n"
"        };\n"
"        ws.send(JSON.stringify(message));\n"
"        return true;\n"
"      }\n"
"      return false;\n"
"    }\n"
"\n"
"    // 处理WebSocket消息\n"
"    function handleWebSocketMessage(message) {\n"
"      console.log('📨 收到WebSocket消息:', message);\n"
"      \n"
"      switch (message.type) {\n"
"        case 'status':\n"
"          updateRobotStatus(message.data);\n"
"          break;\n"
"        case 'heartbeat':\n"
"          console.log('💓 心跳响应');\n"
"          break;\n"
"        default:\n"
"          if (message.status === 'ok') {\n"
"            showStatus('✅ ' + (message.message || '命令执行成功'));\n"
"          } else if (message.status === 'error') {\n"
"            showStatus('❌ ' + (message.message || '命令执行失败'), true);\n"
"          }\n"
"          break;\n"
"      }\n"
"    }\n"
"\n"
"    // 更新机器人状态显示\n"
"    function updateRobotStatus(status) {\n"
"      // 这里可以更新页面上的状态显示\n"
"      // 例如更新位置、姿态、电池电量等信息\n"
"      console.log('🤖 机器人状态更新:', status);\n"
"    }\n"
"\n"
"    // 智能发送请求（优先WebSocket，降级到HTTP）\n"
"    function sendRequest(url, params = {}) {\n"
"      // 尝试通过WebSocket发送（更快更实时）\n"
"      if (wsConnected) {\n"
"        const success = sendControlViaWebSocket(url, params);\n"
"        if (success) {\n"
"          return;\n"
"        }\n"
"      }\n"
"      \n"
"      // 降级到HTTP请求\n"
"      sendHttpRequest(url, params);\n"
"    }\n"
"\n"
"    // 通过WebSocket发送控制指令\n"
"    function sendControlViaWebSocket(url, params) {\n"
"      try {\n"
"        let type, data;\n"
"        \n"
"        if (url.includes('/api/pos/') || url.includes('/api/rot/')) {\n"
"          // 位置和姿态控制 -> pose6\n"
"          const currentPose = getCurrentPose();\n"
"          \n"
"          if (url.includes('/api/pos/x')) currentPose.x = parseFloat(params.value) * 0.001;\n"
"          else if (url.includes('/api/pos/y')) currentPose.y = parseFloat(params.value) * 0.001;\n"
"          else if (url.includes('/api/pos/z')) currentPose.z = parseFloat(params.value) * 0.001;\n"
"          else if (url.includes('/api/rot/roll')) currentPose.roll = parseFloat(params.value);\n"
"          else if (url.includes('/api/rot/pitch')) currentPose.pitch = parseFloat(params.value);\n"
"          else if (url.includes('/api/rot/yaw')) currentPose.yaw = parseFloat(params.value);\n"
"          \n"
"          type = 'pose6';\n"
"          data = currentPose;\n"
"        } else if (url.includes('/api/action/')) {\n"
"          // 预定义动作\n"
"          const actionId = parseInt(url.split('/').pop());\n"
"          type = 'action';\n"
"          data = { id: actionId, beta: params.beta || 30 };\n"
"        } else if (url.includes('/api/preset/')) {\n"
"          // 预设姿势\n"
"          const presetId = parseInt(url.split('/').pop());\n"
"          type = 'preset';\n"
"          data = { id: presetId };\n"
"        } else if (url.includes('/api/emergency/')) {\n"
"          // 紧急停止\n"
"          type = 'emergency';\n"
"          data = {};\n"
"        } else {\n"
"          return false; // 不支持的URL\n"
"        }\n"
"        \n"
"        return sendWebSocketMessage(type, data);\n"
"      } catch (e) {\n"
"        console.error('WebSocket控制指令发送失败:', e);\n"
"        return false;\n"
"      }\n"
"    }\n"
"\n"
"    // 获取当前姿态状态\n"
"    function getCurrentPose() {\n"
"      return {\n"
"        x: parseFloat(document.getElementById('pos-x').value) * 0.001,\n"
"        y: parseFloat(document.getElementById('pos-y').value) * 0.001,\n"
"        z: parseFloat(document.getElementById('pos-z').value) * 0.001,\n"
"        roll: parseFloat(document.getElementById('rot-roll').value),\n"
"        pitch: parseFloat(document.getElementById('rot-pitch').value),\n"
"        yaw: parseFloat(document.getElementById('rot-yaw').value)\n"
"      };\n"
"    }\n"
"\n"
"    // HTTP请求发送（降级方案）\n"
"    function sendHttpRequest(url, params = {}) {\n"
"      const xhr = new XMLHttpRequest();\n"
"      const queryString = Object.keys(params).map(key => key + '=' + encodeURIComponent(params[key])).join('&');\n"
"      const fullUrl = url + (queryString ? '?' + queryString : '');\n"
"      \n"
"      xhr.open('GET', fullUrl, true);\n"
"      xhr.timeout = 5000; // 5秒超时\n"
"      \n"
"      xhr.onreadystatechange = function() {\n"
"        if (xhr.readyState === 4) {\n"
"          if (xhr.status === 200) {\n"
"            try {\n"
"              const response = JSON.parse(xhr.responseText);\n"
"              if (response.status === 'ok') {\n"
"                showStatus('✅ 命令执行成功');\n"
"              } else {\n"
"                showStatus('⚠️ ' + (response.message || '命令执行异常'), true);\n"
"              }\n"
"            } catch (e) {\n"
"              showStatus('✅ 命令已发送');\n"
"            }\n"
"          } else {\n"
"            showStatus('❌ 网络错误 (状态码: ' + xhr.status + ')', true);\n"
"          }\n"
"        }\n"
"      };\n"
"      \n"
"      xhr.ontimeout = function() {\n"
"        showStatus('⏰ 请求超时，请检查网络连接', true);\n"
"      };\n"
"      \n"
"      xhr.onerror = function() {\n"
"        showStatus('🔌 网络连接失败', true);\n"
"      };\n"
"      \n"
"      xhr.send();\n"
"    }\n"
"\n"
"    // 滑块控制映射\n"
"    const sliderMap = [\n"
"      ['pos-x','val-x','/api/pos/x'],\n"
"      ['pos-y','val-y','/api/pos/y'],\n"
"      ['pos-z','val-z','/api/pos/z'],\n"
"      ['rot-roll','val-roll','/api/rot/roll'],\n"
"      ['rot-pitch','val-pitch','/api/rot/pitch'],\n"
"      ['rot-yaw','val-yaw','/api/rot/yaw']\n"
"    ];\n"
"\n"
"    // 初始化滑块控制\n"
"    sliderMap.forEach(([inputId, valId, apiUrl]) => {\n"
"      const input = document.getElementById(inputId);\n"
"      const out = document.getElementById(valId);\n"
"      if (!input || !out) return;\n"
"      \n"
"      let debounceTimer;\n"
"      const update = () => {\n"
"        let displayValue = input.value;\n"
"        \n"
"        // 根据控制类型格式化显示值\n"
"        if (inputId.startsWith('pos-')) {\n"
"          displayValue = input.value + 'mm';\n"
"        } else if (inputId.startsWith('rot-')) {\n"
"          displayValue = input.value + '°';\n"
"        }\n"
"        \n"
"        out.textContent = displayValue;\n"
"        \n"
"        // 防抖发送请求\n"
"        clearTimeout(debounceTimer);\n"
"        debounceTimer = setTimeout(() => {\n"
"          sendRequest(apiUrl, {value: input.value});\n"
"        }, 200);\n"
"      };\n"
"      \n"
"      input.addEventListener('input', update);\n"
"      update(); // 初始化显示\n"
"    });\n"
"\n"
"    // β角度滑块（设置默认β）\n"
"    (function(){\n"
"      const betaInput = document.getElementById('beta-angle');\n"
"      const betaOut = document.getElementById('val-beta');\n"
"      if (!betaInput || !betaOut) return;\n"
"      let timer;\n"
"      const updateBeta = () => {\n"
"        betaOut.textContent = betaInput.value + '°';\n"
"        clearTimeout(timer);\n"
"        timer = setTimeout(() => {\n"
"          sendRequest('/api/action/beta', { beta: betaInput.value });\n"
"        }, 200);\n"
"      };\n"
"      betaInput.addEventListener('input', updateBeta);\n"
"      updateBeta();\n"
"    })();\n"
"\n"
"    // 动作按钮控制\n"
"    document.querySelectorAll('.action-btn').forEach(btn => {\n"
"      btn.addEventListener('click', () => {\n"
"        // 视觉反馈\n"
"        btn.style.transform = 'scale(0.95)';\n"
"        setTimeout(() => {\n"
"          btn.style.transform = '';\n"
"        }, 150);\n"
"        \n"
"        // 处理不同类型的按钮\n"
"        const actionId = btn.getAttribute('data-action');\n"
"        const presetId = btn.getAttribute('data-preset');\n"
"        const emergency = btn.getAttribute('data-emergency');\n"
"        \n"
"        if (actionId) {\n"
"          // 预定义动作\n"
"          const betaEl = document.getElementById('beta-angle');\n"
"          const betaVal = betaEl ? parseInt(betaEl.value) : 30;\n"
"          sendRequest('/api/action/' + actionId, {beta: betaVal});\n"
"        } else if (presetId !== null) {\n"
"          // 预设姿势\n"
"          sendRequest('/api/preset/' + presetId);\n"
"        } else if (emergency) {\n"
"          // 紧急停止\n"
"          sendRequest('/api/emergency/stop');\n"
"        }\n"
"      });\n"
"    });\n"
"\n"
"    // 更新时间显示\n"
"    function updateTime() {\n"
"      const now = new Date();\n"
"      const timeStr = now.toLocaleTimeString('zh-CN', {hour12: false});\n"
"      document.getElementById('current-time').textContent = timeStr;\n"
"    }\n"
"    \n"
"    setInterval(updateTime, 1000);\n"
"    updateTime();\n"
"\n"
"    // 页面加载完成提示\n"
"    window.addEventListener('load', () => {\n"
"      showStatus('🚀 四足机器人控制系统已就绪');\n"
"      \n"
"      // 建立WebSocket连接\n"
"      setTimeout(() => {\n"
"        connectWebSocket();\n"
"      }, 1000);\n"
"    });\n"
"\n"
"    // 页面卸载时关闭WebSocket\n"
"    window.addEventListener('beforeunload', () => {\n"
"      if (ws) {\n"
"        ws.close();\n"
"      }\n"
"    });\n"
"  </script>\n"
"</body>\n"
"</html>";
#endif

// 安全响应发送函数
static esp_err_t safe_send_response(httpd_req_t *req, const char *response)
{
    esp_err_t ret = httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send response: %s (non-fatal)", esp_err_to_name(ret));
        g_http_stats.send_errors++;
        // 发送失败时立即关闭会话，避免僵尸连接与资源占用
        int sockfd = httpd_req_to_sockfd(req);
        httpd_sess_trigger_close(req->handle, sockfd);
        // 非致命处理，避免 httpd 打印 uri handler 失败
        return ESP_OK;
    }
    return ESP_OK;
}

// 主页处理器
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

static esp_err_t root_get_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "Serving main page (embedded)");
    httpd_resp_set_type(req, "text/html");
    size_t len = index_html_end - index_html_start;
    esp_err_t ret = httpd_resp_send(req, (const char*)index_html_start, len);
    if (ret != ESP_OK) {
        g_http_stats.send_errors++;
        int sockfd = httpd_req_to_sockfd(req);
        httpd_sess_trigger_close(req->handle, sockfd);
        return ESP_OK;
    }
    return ESP_OK;
}

// 位置和姿态控制统一处理器
static esp_err_t unified_api_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "API request: %s", req->uri);
    g_http_stats.total_connections++;

    // 网络门控：网络不可用时直接返回503，避免占用资源
    if (!is_network_available()) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Connection", "close");
        esp_err_t err = safe_send_response(req, "{\"status\":\"error\",\"message\":\"network unavailable\"}");
        if (err != ESP_OK) {
            int sockfd = httpd_req_to_sockfd(req);
            httpd_sess_trigger_close(req->handle, sockfd);
        }
        return err;
    }
    
    const char* uri = req->uri;
    
    // 检查请求有效性
    if (!req || !uri) {
        ESP_LOGE(TAG, "Invalid request parameters");
        return ESP_FAIL;
    }
    
    // 使用堆分配替代栈数组，避免栈溢出
    char* uri_path = malloc(256);
    char* query = malloc(256);
    if (!uri_path || !query) {
        free(uri_path);
        free(query);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "application/json");
        return safe_send_response(req, "{\"status\":\"error\",\"message\":\"Memory allocation failed\"}");
    }
    
    // 提取URI路径（不含查询参数）
    const char* query_start = strchr(uri, '?');
    if (query_start) {
        size_t path_len = query_start - uri;
        if (path_len >= 255) path_len = 255;
        strncpy(uri_path, uri, path_len);
        uri_path[path_len] = '\0';
    } else {
        strncpy(uri_path, uri, 255);
        uri_path[255] = '\0';
    }
    
    // 解析查询参数
    if (httpd_req_get_url_query_str(req, query, 256) != ESP_OK) {
        ESP_LOGW(TAG, "Missing query parameters for %s", uri);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        esp_err_t result = safe_send_response(req, "{\"status\":\"error\",\"message\":\"missing query parameters\"}");
        free(uri_path);
        free(query);
        return result;
    }
    
    char param[32];
    if (httpd_query_key_value(query, "value", param, sizeof(param)) != ESP_OK) {
        ESP_LOGW(TAG, "Missing value parameter for %s", uri);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        esp_err_t result = safe_send_response(req, "{\"status\":\"error\",\"message\":\"missing value parameter\"}");
        free(uri_path);
        free(query);
        return result;
    }
    
    int value = atoi(param);
    ESP_LOGD(TAG, "Processing %s with value: %d", uri_path, value);
    
    // 根据URI路径更新姿态状态
    bool param_updated = false;
    
    if (strcmp(uri_path, "/api/pos/x") == 0) {
        if (value >= -80 && value <= 80) {
            g_pose_state.x = value * 0.001f; // mm转m
            param_updated = true;
        }
    } else if (strcmp(uri_path, "/api/pos/y") == 0) {
        if (value >= -80 && value <= 80) {
            g_pose_state.y = value * 0.001f;
            param_updated = true;
        }
    } else if (strcmp(uri_path, "/api/pos/z") == 0) {
        if (value >= -220 && value <= 0) {
            g_pose_state.z = value * 0.001f; // mm转m
            param_updated = true;
        }
    } else if (strcmp(uri_path, "/api/rot/roll") == 0) {
        if (value >= -45 && value <= 45) {
            g_pose_state.roll = (float)value;
            param_updated = true;
        }
    } else if (strcmp(uri_path, "/api/rot/pitch") == 0) {
        if (value >= -45 && value <= 45) {
            g_pose_state.pitch = (float)value;
            param_updated = true;
        }
    } else if (strcmp(uri_path, "/api/rot/yaw") == 0) {
        if (value >= -45 && value <= 45) {
            g_pose_state.yaw = (float)value;
            param_updated = true;
        }
    }
    
    if (!param_updated) {
        ESP_LOGW(TAG, "Invalid endpoint or value out of range: %s = %d", uri_path, value);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        esp_err_t result = safe_send_response(req, "{\"status\":\"error\",\"message\":\"invalid endpoint or value out of range\"}");
        free(uri_path);
        free(query);
        return result;
    }
    
    // 调用桥接层执行pose6控制
    int result = retry_robot_pose6(g_pose_state.x, g_pose_state.y, g_pose_state.z,
                                   g_pose_state.roll, g_pose_state.pitch, g_pose_state.yaw);
    
    httpd_resp_set_type(req, "application/json");
    
    esp_err_t response_result;
    if (result == ROBOT_BRIDGE_OK) {
        ESP_LOGD(TAG, "Pose updated: pos(%.3f,%.3f,%.3f) rot(%.1f°,%.1f°,%.1f°)",
                 g_pose_state.x, g_pose_state.y, g_pose_state.z,
                 g_pose_state.roll, g_pose_state.pitch, g_pose_state.yaw);
        response_result = safe_send_response(req, "{\"status\":\"ok\"}");
    } else {
        const char* error_msg = robot_bridge_get_last_error();
        ESP_LOGE(TAG, "❌ Pose control failed: %s", error_msg);
        httpd_resp_set_status(req, "500 Internal Server Error");
        response_result = safe_send_response(req, "{\"status\":\"error\",\"message\":\"robot control failed\"}");
    }
    
    // 释放堆内存
    free(uri_path);
    free(query);
    return response_result;
}

// 动作控制处理器
static esp_err_t action_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "Action request: %s", req->uri);
    g_http_stats.total_connections++;

    if (!is_network_available()) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Connection", "close");
        esp_err_t err = safe_send_response(req, "{\"status\":\"error\",\"message\":\"network unavailable\"}");
        if (err != ESP_OK) {
            int sockfd = httpd_req_to_sockfd(req);
            httpd_sess_trigger_close(req->handle, sockfd);
        }
        return err;
    }
    
    // 提取动作ID：/api/action/1 -> 1
    const char* uri = req->uri;
    const char* action_str = strrchr(uri, '/');
    if (!action_str) {
        ESP_LOGW(TAG, "Malformed action URI: %s", uri);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        return safe_send_response(req, "{\"status\":\"error\",\"message\":\"malformed action URI\"}");
    }
    
    int action_id = atoi(action_str + 1);
    ESP_LOGD(TAG, "Processing action ID: %d", action_id);
    
    // 解析beta参数（可选）
    // 使用统一beta默认值，无需重复定义局部变量 // 默认值
    float beta = g_cone_beta_deg;
    char* query = malloc(256);
    if (!query) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "application/json");
        return safe_send_response(req, "{\"status\":\"error\",\"message\":\"Memory allocation failed\"}");
    }
    
    if (httpd_req_get_url_query_str(req, query, 256) == ESP_OK) {
        char beta_param[32];
        if (httpd_query_key_value(query, "beta", beta_param, sizeof(beta_param)) == ESP_OK) {
            float user_beta = strtof(beta_param, NULL);
            if (user_beta >= 5.0f && user_beta <= 45.0f) {
                beta = user_beta;
            }
        }
    }
    
    int result = ROBOT_BRIDGE_ERROR_PARAM;
    
    if (action_id == 1) {
        // 圆锥动作
        ESP_LOGI(TAG, "🌀 执行圆锥动作，beta=%.1f°", beta);
        result = retry_action_cone(beta);
    } else {
        ESP_LOGW(TAG, "Invalid action ID: %d", action_id);
    }
    
    httpd_resp_set_type(req, "application/json");
    
    esp_err_t response_result;
    if (result == ROBOT_BRIDGE_OK) {
        ESP_LOGI(TAG, "✅ Action %d executed successfully", action_id);
        response_result = safe_send_response(req, "{\"status\":\"ok\"}");
    } else {
        const char* error_msg = robot_bridge_get_last_error();
        ESP_LOGE(TAG, "❌ Action %d failed: %s", action_id, error_msg);
        httpd_resp_set_status(req, "500 Internal Server Error");
        response_result = safe_send_response(req, "{\"status\":\"error\",\"message\":\"action execution failed\"}");
    }
    
    free(query);
    return response_result;
}

static esp_err_t action_beta_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "Action beta request: %s", req->uri);
    g_http_stats.total_connections++;

    if (!is_network_available()) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Connection", "close");
        esp_err_t err = safe_send_response(req, "{\"status\":\"error\",\"message\":\"network unavailable\"}");
        if (err != ESP_OK) {
            int sockfd = httpd_req_to_sockfd(req);
            httpd_sess_trigger_close(req->handle, sockfd);
        }
        return err;
    }

    char* query = malloc(256);
    if (!query) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "application/json");
        return safe_send_response(req, "{\"status\":\"error\",\"message\":\"Memory allocation failed\"}");
    }


    if (httpd_req_get_url_query_str(req, query, 256) == ESP_OK) {
        char beta_param[32];
        if (httpd_query_key_value(query, "beta", beta_param, sizeof(beta_param)) == ESP_OK) {
            float user_beta = strtof(beta_param, NULL);
            if (user_beta >= 5.0f && user_beta <= 45.0f) {
                g_cone_beta_deg = user_beta; // 更新默认β
                websocket_set_default_beta(g_cone_beta_deg); // 同步到WS默认β
            }
        }
    }

    free(query);
    httpd_resp_set_type(req, "application/json");
    return safe_send_response(req, "{\"status\":\"ok\"}");
}

// 预设姿势处理器
static esp_err_t preset_handler(httpd_req_t *req)
{
    ESP_LOGD(TAG, "Preset request: %s", req->uri);
    g_http_stats.total_connections++;

    if (!is_network_available()) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Connection", "close");
        esp_err_t err = safe_send_response(req, "{\"status\":\"error\",\"message\":\"network unavailable\"}");
        if (err != ESP_OK) {
            int sockfd = httpd_req_to_sockfd(req);
            httpd_sess_trigger_close(req->handle, sockfd);
        }
        return err;
    }
    
    // 提取预设ID：/api/preset/0 -> 0
    const char* uri = req->uri;
    const char* preset_str = strrchr(uri, '/');
    if (!preset_str) {
        ESP_LOGW(TAG, "Malformed preset URI: %s", uri);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        return safe_send_response(req, "{\"status\":\"error\",\"message\":\"malformed preset URI\"}");
    }
    
    int preset_id = atoi(preset_str + 1);
    ESP_LOGD(TAG, "Processing preset ID: %d", preset_id);
    
    if (preset_id < 0 || preset_id > 1) {
        ESP_LOGW(TAG, "Invalid preset ID: %d", preset_id);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        return safe_send_response(req, "{\"status\":\"error\",\"message\":\"invalid preset ID\"}");
    }
    
    ESP_LOGI(TAG, "🎮 执行预设姿势: %s", preset_id == 0 ? "趴下" : "站立");
    int result = retry_preset(preset_id);
    
    httpd_resp_set_type(req, "application/json");
    
    if (result == ROBOT_BRIDGE_OK) {
        ESP_LOGI(TAG, "✅ Preset %d executed successfully", preset_id);
        return safe_send_response(req, "{\"status\":\"ok\"}");
    } else {
        const char* error_msg = robot_bridge_get_last_error();
        ESP_LOGE(TAG, "❌ Preset %d failed: %s", preset_id, error_msg);
        httpd_resp_set_status(req, "500 Internal Server Error");
        return safe_send_response(req, "{\"status\":\"error\",\"message\":\"preset execution failed\"}");
    }
}

// 紧急停止处理器
static esp_err_t emergency_handler(httpd_req_t *req)
{
    ESP_LOGW(TAG, "🚨 Emergency stop requested");
    g_http_stats.total_connections++;

    if (!is_network_available()) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Connection", "close");
        esp_err_t err = safe_send_response(req, "{\"status\":\"error\",\"message\":\"network unavailable\"}");
        if (err != ESP_OK) {
            int sockfd = httpd_req_to_sockfd(req);
            httpd_sess_trigger_close(req->handle, sockfd);
        }
        return err;
    }
    
    int result = retry_emergency();
    
    httpd_resp_set_type(req, "application/json");
    
    if (result == ROBOT_BRIDGE_OK) {
        ESP_LOGI(TAG, "✅ Emergency stop executed");
        return safe_send_response(req, "{\"status\":\"ok\"}");
    } else {
        ESP_LOGE(TAG, "❌ Emergency stop failed");
        httpd_resp_set_status(req, "500 Internal Server Error");
        return safe_send_response(req, "{\"status\":\"error\",\"message\":\"emergency stop failed\"}");
    }
}

static esp_err_t favicon_handler(httpd_req_t *req)
{
    // 返回 204，避免浏览器默认请求造成错误日志
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_set_type(req, "image/x-icon");
    return httpd_resp_send(req, NULL, 0);
}
// URI处理器定义
static const httpd_uri_t root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};

// 精确匹配的API处理器
static const httpd_uri_t api_pos_x   = { .uri = "/api/pos/x",   .method = HTTP_GET, .handler = unified_api_handler, .user_ctx = NULL };
static const httpd_uri_t api_pos_y   = { .uri = "/api/pos/y",   .method = HTTP_GET, .handler = unified_api_handler, .user_ctx = NULL };
static const httpd_uri_t api_pos_z   = { .uri = "/api/pos/z",   .method = HTTP_GET, .handler = unified_api_handler, .user_ctx = NULL };
static const httpd_uri_t api_rot_roll  = { .uri = "/api/rot/roll",  .method = HTTP_GET, .handler = unified_api_handler, .user_ctx = NULL };
static const httpd_uri_t api_rot_pitch = { .uri = "/api/rot/pitch", .method = HTTP_GET, .handler = unified_api_handler, .user_ctx = NULL };
static const httpd_uri_t api_rot_yaw   = { .uri = "/api/rot/yaw",   .method = HTTP_GET, .handler = unified_api_handler, .user_ctx = NULL };

static const httpd_uri_t api_action_beta = { .uri = "/api/action/beta", .method = HTTP_GET, .handler = action_beta_handler, .user_ctx = NULL };
static const httpd_uri_t api_action_1 = { .uri = "/api/action/1", .method = HTTP_GET, .handler = action_handler, .user_ctx = NULL };

static const httpd_uri_t api_preset_0 = { .uri = "/api/preset/0", .method = HTTP_GET, .handler = preset_handler, .user_ctx = NULL };
static const httpd_uri_t api_preset_1 = { .uri = "/api/preset/1", .method = HTTP_GET, .handler = preset_handler, .user_ctx = NULL };

static const httpd_uri_t api_emergency = { .uri = "/api/emergency/stop", .method = HTTP_GET, .handler = emergency_handler, .user_ctx = NULL };
static const httpd_uri_t favicon = { .uri = "/favicon.ico", .method = HTTP_GET, .handler = favicon_handler, .user_ctx = NULL };

// 启动Web服务器
httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    // 提升 httpd 任务栈，避免复杂 URI/WS 处理导致溢出
    config.stack_size = 10240;

    // 优化服务器配置
    config.max_uri_handlers = 20;  // 增加处理器数量以支持WebSocket
    config.server_port = 12580;
    config.send_wait_timeout = 3;
    config.recv_wait_timeout = 3;
    config.keep_alive_enable = false;
    config.max_open_sockets = 4;   // 与LWIP_MAX_SOCKETS(7)匹配，HTTPD内部占3，剩余4可用
    config.backlog_conn = 3;       // 降低连接排队，避免超出可用socket
    config.lru_purge_enable = true;

    // 单实例防重启与防重入：已运行或正在启动则直接返回现有句柄
    if ((g_server_running || g_server_starting) && g_server_instance) {
        ESP_LOGW(TAG, "Web server running/starting, return existing handle");
        return g_server_instance;
    }
    g_server_starting = true;

    ESP_LOGI(TAG, "Starting web server on port: %d", config.server_port);
    
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");

        // 注册所有URI处理器
        const httpd_uri_t* handlers[] = {
            &root,
            &api_pos_x, &api_pos_y, &api_pos_z,
            &api_rot_roll, &api_rot_pitch, &api_rot_yaw,
            &api_action_beta, &api_action_1,
            &api_preset_0, &api_preset_1,
            &api_emergency,
            &favicon
        };
        
        int registered_count = 0;
        for (size_t i = 0; i < sizeof(handlers)/sizeof(handlers[0]); ++i) {
            esp_err_t ret = httpd_register_uri_handler(server, handlers[i]);
            if (ret == ESP_OK) {
                registered_count++;
            } else {
                ESP_LOGE(TAG, "Failed to register handler %zu: %s", i, esp_err_to_name(ret));
            }
        }
        
        // 初始化并注册WebSocket处理器
        if (websocket_handler_init() == ESP_OK) {
            if (websocket_register_handlers(server) == ESP_OK) {
                registered_count++;
                ESP_LOGI(TAG, "✅ WebSocket处理器注册成功");
            } else {
                ESP_LOGE(TAG, "❌ WebSocket处理器注册失败");
            }
        } else {
            ESP_LOGE(TAG, "❌ WebSocket处理器初始化失败");
        }
        
        ESP_LOGD(TAG, "Successfully registered %d URI handlers", registered_count);

        // 动态获取本机 IP（优先 STA，回退 AP）
        esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (!netif) netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");

        esp_netif_ip_info_t ip_info;
        if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0) {
            char ip_str[16];
            esp_ip4addr_ntoa(&ip_info.ip, ip_str, sizeof(ip_str));
            ESP_LOGI(TAG, "🚀 Web server ready at http://%s:%d", ip_str, config.server_port);
            ESP_LOGI(TAG, "🔗 WebSocket endpoint: ws://%s:%d/ws", ip_str, config.server_port);
        } else {
            ESP_LOGW(TAG, "🌐 Web server listening on port %d, waiting for IP...", config.server_port);
            ESP_LOGI(TAG, "🔗 WebSocket endpoint will be: ws://<device-ip>:%d/ws", config.server_port);
        }
        
        // 记录全局实例与运行状态
        g_server_instance = server;
        g_server_running = true;
        g_server_starting = false;
        return server;
    }

    ESP_LOGE(TAG, "Error starting server!");
    g_server_starting = false;
    return NULL;
}

// 停止Web服务器
esp_err_t stop_webserver(httpd_handle_t server)
{
    if (server) {
        ESP_LOGI(TAG, "Stopping web server");
        esp_err_t r = httpd_stop(server);
        // 清理全局实例与运行状态
        if (r == ESP_OK) {
            if (server == g_server_instance) {
                g_server_instance = NULL;
                g_server_running = false;
            }
        }
        // 无论结果如何，复位启动中标志，避免卡死
        g_server_starting = false;
        return r;
    }
    return ESP_OK;
}