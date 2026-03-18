# ESP32四足机器人网页实时控制系统

## 🤖 项目概述

基于ESP32的四足机器人实时网页控制系统，整合现有的HTTP网页界面和四足机器人控制系统，实现通过网页端对机器人的实时位置、姿态和预定义动作控制。

### ✨ 核心特性

- **🌐 实时网页控制界面**：现代化Material Design风格，响应式布局
- **🔗 WebSocket双向通信**：优先WebSocket实时通信，HTTP降级备用
- **🎮 多层次控制模式**：支持PWM直接控制、舵机角度控制、关节角度控制、运动学控制
- **📡 运动指令编解码**：网页控制指令到机器人控制参数的精确转换和映射
- **🎯 平滑运动控制**：集成pose6姿态控制和action_cy圆锥动作等预定义运动模式
- **⚡ 预定义动作执行**：圆锥动作、趴下/站立姿势、紧急停止等快捷操作

### 🏗️ 系统架构

```
网页界面 → WebSocket/HTTP → web_server_adapter → robot_bridge → SmoothMotionController → 硬件驱动 → 舵机执行
```

## 📋 技术栈

- **核心框架**：ESP-IDF + C++17 + FreeRTOS
- **通信协议**：HTTP服务器 + WebSocket实时通信
- **硬件驱动**：I2C通信 + PCA9685 PWM驱动器
- **前端技术**：HTML + JavaScript + CSS (Material Design)

## 🚀 快速开始

### 环境要求

- ESP-IDF v4.4+
- ESP32开发板
- PCA9685 PWM驱动器
- 12个舵机（四足机器人）

### 编译和烧录

```bash
# 进入项目目录
cd spot_micro_esp32/body_control_web

# 配置项目
idf.py menuconfig

# 编译项目
idf.py build

# 烧录到ESP32
idf.py -p /dev/ttyUSB0 flash monitor
```

### WiFi配置

在 `pca9685Test.cpp` 中修改WiFi配置：

```cpp
#define WIFI_SSID "your_wifi_name"
#define WIFI_PASS "your_wifi_password"
```

### 访问控制界面

1. 启动ESP32，等待WiFi连接成功
2. 在浏览器中访问：`http://192.168.1.7:12580`
3. 使用滑块控制机器人位置和姿态
4. 点击预定义动作按钮执行特定动作

## 📖 使用指南

### 网页控制界面

#### 位置控制
- **X轴位置**：-100mm 到 100mm
- **Y轴位置**：-100mm 到 100mm  
- **Z轴位置**：-100mm 到 100mm（映射到实际-0.20m到0.0m）

#### 姿态控制
- **Roll翻滚**：-180° 到 180°
- **Pitch俯仰**：-180° 到 180°
- **Yaw偏航**：-180° 到 180°

#### 预定义动作
- **🌀 圆锥动作**：执行圆锥形运动轨迹
- **😴 趴下休息**：切换到趴下姿势
- **🚶 站立准备**：切换到站立姿势
- **🚨 紧急停止**：立即停止所有运动

### WebSocket实时通信

系统支持WebSocket实时通信，提供以下特性：

- **智能协议选择**：优先使用WebSocket，连接失败时降级到HTTP
- **心跳机制**：30秒心跳保持连接稳定
- **自动重连**：连接断开后3秒自动重连
- **防抖控制**：200ms防抖避免频繁请求

#### WebSocket消息格式

```json
{
  "type": "pose6|action|preset|emergency|heartbeat",
  "data": {
    // 具体数据内容
  },
  "timestamp": 1234567890
}
```

## 🔧 API文档

### HTTP API

#### 位置控制
- `GET /api/pos/x?value={-100到100}` - X轴位置控制
- `GET /api/pos/y?value={-100到100}` - Y轴位置控制
- `GET /api/pos/z?value={-100到100}` - Z轴位置控制

#### 姿态控制
- `GET /api/rot/roll?value={-180到180}` - Roll角度控制
- `GET /api/rot/pitch?value={-180到180}` - Pitch角度控制
- `GET /api/rot/yaw?value={-180到180}` - Yaw角度控制

#### 动作控制
- `GET /api/action/1?beta={5到45}` - 圆锥动作（可选beta参数）
- `GET /api/preset/0` - 趴下姿势
- `GET /api/preset/1` - 站立姿势
- `GET /api/emergency/stop` - 紧急停止

#### 响应格式

```json
{
  "status": "ok|error",
  "message": "描述信息（可选）"
}
```

### WebSocket API

#### 连接端点
```
ws://192.168.1.7:12580/ws
```

#### 消息类型

**姿态控制 (pose6)**
```json
{
  "type": "pose6",
  "data": {
    "x": 0.05,
    "y": 0.0,
    "z": -0.10,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0
  }
}
```

**动作控制 (action)**
```json
{
  "type": "action",
  "data": {
    "id": 1,
    "beta": 30
  }
}
```

**预设姿势 (preset)**
```json
{
  "type": "preset",
  "data": {
    "id": 0
  }
}
```

**紧急停止 (emergency)**
```json
{
  "type": "emergency",
  "data": {}
}
```

## 🏗️ 项目结构

```
spot_micro_esp32/body_control_web/
├── main/
│   ├── config/                 # 配置管理模块
│   │   ├── ConfigManager.cpp
│   │   ├── ConfigManager.hpp
│   │   ├── RobotConfig.hpp
│   │   ├── ServoConfig.hpp
│   │   └── ...
│   ├── drivers/                # 硬件驱动层
│   │   ├── PCA9685Driver.cpp
│   │   ├── PCA9685Driver.hpp
│   │   ├── ServoDriver.cpp
│   │   └── ServoDriver.hpp
│   ├── controllers/            # 控制器层
│   │   ├── JointController.cpp
│   │   ├── RobotController.cpp
│   │   ├── SmoothMotionController.cpp
│   │   └── ...
│   ├── kinematics/            # 运动学模块
│   │   ├── LegKinematics.cpp
│   │   ├── QuadrupedModel.cpp
│   │   └── ...
│   ├── robot_bridge.h         # 桥接层头文件
│   ├── robot_bridge.cpp       # 桥接层实现
│   ├── web_server_adapter.c   # HTTP服务器适配层
│   ├── websocket_handler.h    # WebSocket处理器头文件
│   ├── websocket_handler.c    # WebSocket处理器实现
│   ├── pca9685Test.cpp        # 主程序入口
│   └── CMakeLists.txt         # 构建配置
├── components/                 # 组件库
│   └── pca9685/
└── README.md                  # 项目文档
```

## 🔧 核心模块说明

### 桥接层 (robot_bridge)

桥接层是连接网页控制和机器人系统的核心组件，提供以下接口：

- `robot_bridge_pose6()` - 六自由度姿态控制
- `robot_bridge_action_cone()` - 圆锥动作执行
- `robot_bridge_set_preset_pose()` - 预设姿势切换
- `robot_bridge_emergency_stop()` - 紧急停止
- `robot_bridge_get_last_error()` - 获取最后错误信息

### HTTP适配层 (web_server_adapter)

HTTP适配层处理网页请求并调用桥接层接口：

- 路由处理和参数解析
- 请求验证和错误处理
- 响应格式化和状态管理
- WebSocket处理器集成

### WebSocket处理器 (websocket_handler)

WebSocket处理器提供实时双向通信：

- 连接管理和心跳机制
- 消息解析和指令分发
- 状态广播和错误处理
- 自动重连和连接恢复

## ⚙️ 配置说明

### WiFi配置

```cpp
// 在 pca9685Test.cpp 中修改
#define WIFI_SSID "your_wifi_name"
#define WIFI_PASS "your_wifi_password"
```

### 服务器配置

```cpp
// HTTP服务器端口
config.server_port = 12580;

// 最大连接数
config.max_open_sockets = 8;

// 超时设置
config.send_wait_timeout = 5;
config.recv_wait_timeout = 5;
```

### 机器人参数配置

机器人的物理参数和运动限制在配置文件中定义：

- `RobotConfig.hpp` - 机器人基本参数
- `ServoConfig.hpp` - 舵机配置参数
- `MappingConfig.hpp` - 映射关系配置

## 🧪 测试和调试

### 编译测试

```bash
# 清理构建
idf.py fullclean

# 重新编译
idf.py build
```

### 运行时调试

```bash
# 监控串口输出
idf.py monitor

# 查看WiFi连接状态
# 查看HTTP服务器启动日志
# 查看WebSocket连接日志
```

### 网页测试

1. 打开浏览器开发者工具
2. 访问控制界面
3. 检查WebSocket连接状态
4. 测试各项控制功能
5. 观察控制台日志输出

## 🚨 故障排除

### 常见问题

**WiFi连接失败**
- 检查SSID和密码是否正确
- 确认WiFi信号强度
- 检查路由器设置

**网页无法访问**
- 确认ESP32已连接WiFi
- 检查IP地址是否正确
- 确认防火墙设置

**WebSocket连接失败**
- 检查浏览器WebSocket支持
- 确认服务器正常运行
- 查看控制台错误信息

**机器人控制无响应**
- 检查硬件连接
- 确认PCA9685驱动器工作正常
- 查看串口调试信息

### 调试日志

系统提供详细的调试日志：

```
[WEB_ADAPTER] 🚀 Web server ready at http://192.168.1.7:12580
[WEB_ADAPTER] 🔗 WebSocket endpoint: ws://192.168.1.7:12580/ws
[ROBOT_BRIDGE] ✅ Pose updated: pos(0.050,0.000,-0.100) rot(0.0°,0.0°,0.0°)
[WEBSOCKET] 📨 收到WebSocket消息: {"type":"pose6","data":{...}}
```

## 📝 开发说明

### 添加新的控制功能

1. 在 `robot_bridge.h` 中声明新接口
2. 在 `robot_bridge.cpp` 中实现功能
3. 在 `web_server_adapter.c` 中添加HTTP处理器
4. 在 `websocket_handler.c` 中添加WebSocket消息处理
5. 更新网页界面添加控制元素

### 扩展WebSocket消息类型

1. 在 `websocket_handler.h` 中定义新消息类型
2. 在 `websocket_handler.c` 中实现消息处理逻辑
3. 更新网页JavaScript代码支持新消息
4. 添加相应的错误处理和状态反馈

## 📄 许可证

本项目采用MIT许可证，详见LICENSE文件。

## 🤝 贡献

欢迎提交Issue和Pull Request来改进项目。

## 📞 联系方式

如有问题或建议，请联系项目维护者。

---

**ESP32四足机器人项目组**  
*让机器人控制变得简单而强大* 🤖✨