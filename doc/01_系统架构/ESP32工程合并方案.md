# ESP32 工程合并方案 (修订版 v3.0)

> **文档状态**: 评审通过，待执行
> **修订日期**: 2025-03-04
> **基于**: 代码详细分析 + 评审报告 + 差异验证

---

## 一、现状分析

### 1. 两个工程对比

| 项目 | body_control_uart | body_control_web |
|------|-------------------|------------------|
| **主程序** | pca9685Test.cpp (1174行) | pca9685Test.cpp (1715行) |
| **源文件数** | 36 个 | 43 个 |
| **工程大小** | 207MB | 164MB |
| **控制方式** | UART 串口 | WiFi 网页 + UART |
| **网络功能** | ❌ 无 | ✅ 有 |

### 2. 文件差异分析

#### ✅ 完全相同的文件 (可直接共用)

```
config/
├── ConfigManager.cpp         ✅ 183行
├── ConfigManager.hpp         ✅ 88行
├── MappingConfig.hpp         ✅ 93行
├── RobotConfig.hpp           ✅ 114行
├── ServoConfig.hpp           ✅ 84行
└── SmoothMotionConfig.hpp    ✅ 356行

kinematics/
├── CoordinateTransform.cpp   ✅ 80行
├── CoordinateTransform.hpp   ✅ 248行
├── KinematicsGeometry.cpp    ✅ 68行
├── KinematicsGeometry.hpp    ✅ 99行
├── Leg.cpp                   ✅ 293行
├── Leg.hpp                   ✅ 210行
├── LegKinematics.cpp         ✅ 185行
├── LegKinematics.hpp         ✅ 109行
├── QuadrupedModel.cpp        ✅ 560行
└── QuadrupedModel.hpp        ✅ 287行

controllers/
├── JointController.hpp       ✅ 282行
├── JointSmoother.cpp         ✅ 423行
├── JointSmoother.hpp         ✅ 231行
├── PoseInterpolator.cpp      ✅ 192行
├── PoseInterpolator.hpp      ✅ 148行
├── SmoothMotionController.cpp ✅ 405行
└── SmoothMotionController.hpp ✅ 167行

drivers/
├── PCA9685Driver.cpp         ✅ 相同
├── PCA9685Driver.hpp         ✅ 相同
├── ServoDriver.cpp           ✅ 相同
└── ServoDriver.hpp           ✅ 相同

utils/
├── Logger.cpp                ✅ 173行
├── Logger.hpp                ✅ 154行
├── UartTerminal.cpp          ✅ 294行
└── UartTerminal.hpp          ✅ 205行
```

#### ⚠️ 有差异的文件 (使用 web 版)

```
controllers/
├── JointController.cpp       ⚠️ uart:493 vs web:515 (web版错误处理更好)
├── RobotController.cpp       ⚠️ uart:53 vs web:64
└── RobotController.hpp       ⚠️ uart:67 vs web:73

main/
└── pca9685Test.cpp           ⚠️ uart:1174 vs web:1715 (需合并)
```

#### 📝 差异详细分析

**1. JointController.cpp 差异 (uart:493 vs web:515)**

| 位置 | uart版 | web版 | 说明 |
|------|--------|-------|------|
| setJointAngle() | 直接调用 servo_driver_->setAngle() | 先检查 isHardwareReady() | web版支持无硬件运行 |
| 错误处理 | LOG_TAG_ERROR + 返回错误 | LOG_TAG_WARN + 继续运行 | web版更宽容 |

```cpp
// web版新增代码 (line 112-129)
if (!servo_driver_->isHardwareReady()) {
    // 更新缓存但不发送到硬件
    current_angles_[...] = angle;
    return JointError::SUCCESS;  // 视为成功
}
```

**2. RobotController.hpp 差异 (uart:67 vs web:73)**

| 项目 | uart版 | web版 |
|------|--------|-------|
| 成员变量 | 无 hardware_ready_ | 有 hardware_ready_ |
| 方法 | 无 isHardwareReady() | 有 isHardwareReady() |

**3. RobotController.cpp 差异 (uart:53 vs web:64)**

| 行为 | uart版 | web版 |
|------|--------|-------|
| 硬件初始化失败 | return false (系统停止) | 继续运行 (仿真模式) |
| 日志级别 | ERROR | WARN |
| 返回值 | 可能返回 false | 始终返回 true |

```cpp
// web版关键代码 (line 30-39)
hardware_ready_ = (hw_init_result == Drivers::PCA9685Error::SUCCESS);
if (!hardware_ready_) {
    LOG_TAG_WARN("ROBOT", "Hardware driver init failed; no I2C device detected. Continuing without hardware.");
} else {
    LOG_TAG_INFO("ROBOT", "✅ Hardware driver initialized successfully");
}
```

**4. pca9685Test.cpp 差异 (uart:1174 vs web:1715)**

| 模块 | uart版 | web版 |
|------|--------|-------|
| WiFi 初始化 | ❌ 无 | ✅ 有 (约200行) |
| HTTP 服务器 | ❌ 无 | ✅ 有 (约150行) |
| WebSocket | ❌ 无 | ✅ 有 (约100行) |
| robot_bridge | ❌ 无 | ✅ 有 (约100行) |
| UART 命令 | ✅ 有 | ✅ 有 (相同) |
| 运动学控制 | ✅ 有 | ✅ 有 (相同) |

#### 🌐 web 独有文件 (网络功能)

```
main/
├── robot_bridge.h            # C++ ↔ C 桥接层头文件
├── robot_bridge.cpp          # C++ ↔ C 桥接层实现
├── web_server_adapter.c      # HTTP 服务器适配层
├── websocket_handler.h       # WebSocket 头文件
├── websocket_handler.c       # WebSocket 实现
├── i2c_diagnostic.h          # I2C 诊断头文件
├── i2c_diagnostic.cpp        # I2C 诊断实现
└── web/
    └── index.html            # 网页界面
```

---

## 二、合并方案设计

### 设计原则

1. **单一工程**: 合并为一个 `body_control` 工程
2. **最小改动**: 保持原有目录结构，不增加 `core/` 嵌套
3. **条件编译**: 通过 Kconfig 选择功能
4. **共用核心**: 核心代码只保留一份

### 新目录结构

```
spot_micro_esp32/
└── body_control/                      # 合并后的工程
    ├── CMakeLists.txt                 # 主工程配置
    ├── Kconfig.projbuild              # 配置菜单 (新增)
    ├── sdkconfig.defaults             # 默认配置 (新增)
    │
    ├── components/
    │   └── pca9685/
    │       ├── CMakeLists.txt
    │       ├── pca9685.c
    │       └── pca9685.h
    │
    └── main/
        ├── CMakeLists.txt             # 条件编译 (新增)
        ├── main.cpp                   # 合并后的主程序 (新增)
        ├── RobotTypes.hpp
        │
        ├── config/                    # 保持原结构
        ├── controllers/               # 保持原结构
        ├── drivers/                   # 保持原结构
        ├── kinematics/                # 保持原结构
        ├── utils/                     # 保持原结构
        │
        └── network/                   # 网络功能 (条件编译)
            ├── robot_bridge.h
            ├── robot_bridge.cpp
            ├── web_server_adapter.c
            ├── websocket_handler.h
            ├── websocket_handler.c
            ├── i2c_diagnostic.h
            ├── i2c_diagnostic.cpp
            └── web/
                └── index.html
```

---

## 三、配置文件设计

### Kconfig.projbuild

```kconfig
menu "Robot Control Configuration"

    config ROBOT_ENABLE_WIFI
        bool "Enable WiFi and Web Control"
        default y
        help
            Enable WiFi connectivity and web-based control interface.
            If disabled, only UART serial control will be available.

    config ROBOT_WIFI_SSID
        string "WiFi SSID"
        default "your_wifi_ssid"
        depends on ROBOT_ENABLE_WIFI
        help
            WiFi network name (SSID) to connect to.

    config ROBOT_WIFI_PASSWORD
        string "WiFi Password"
        default "your_wifi_password"
        depends on ROBOT_ENABLE_WIFI
        help
            WiFi network password.

    config ROBOT_HTTP_SERVER_PORT
        int "HTTP Server Port"
        default 12580
        depends on ROBOT_ENABLE_WIFI
        help
            HTTP server port number.

    config ROBOT_ENABLE_UART_CONTROL
        bool "Enable UART Serial Control"
        default y
        help
            Enable UART serial command interface.
            Can be used alongside WiFi control.

    config ROBOT_DEBUG_LOGGING
        bool "Enable Debug Logging"
        default n
        help
            Enable verbose debug logging for development.

endmenu
```

### sdkconfig.defaults

```
CONFIG_HTTPD_WS_SUPPORT=y
```

---

## 四、CMakeLists.txt 设计

### 主工程 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(body_control)
```

### main/CMakeLists.txt (关键修正版)

```cmake
# ============================================================
# ESP32 四足机器人控制系统 - 条件编译 CMakeLists.txt
#
# 重要: idf_component_register() 只能调用一次！
# ============================================================

# ---------- 1. 设置核心源文件 (始终编译) ----------
set(COMPONENT_SRCS
    # 主程序
    "main.cpp"

    # 配置系统
    "config/ConfigManager.cpp"

    # 工具系统
    "utils/Logger.cpp"
    "utils/UartTerminal.cpp"

    # 驱动层
    "drivers/PCA9685Driver.cpp"
    "drivers/ServoDriver.cpp"

    # 控制层
    "controllers/JointController.cpp"
    "controllers/JointSmoother.cpp"
    "controllers/PoseInterpolator.cpp"
    "controllers/RobotController.cpp"
    "controllers/SmoothMotionController.cpp"

    # 运动学层
    "kinematics/CoordinateTransform.cpp"
    "kinematics/KinematicsGeometry.cpp"
    "kinematics/Leg.cpp"
    "kinematics/LegKinematics.cpp"
    "kinematics/QuadrupedModel.cpp"
)

# ---------- 2. 设置包含目录 ----------
set(COMPONENT_INCLUDE_DIRS
    "."
    "config"
    "controllers"
    "drivers"
    "kinematics"
    "utils"
)

# ---------- 3. 设置核心依赖 ----------
set(COMPONENT_REQUIRES
    pca9685
    driver
    esp_timer
    nvs_flash
)

# ---------- 4. 条件添加 WiFi 网络功能 ----------
if(CONFIG_ROBOT_ENABLE_WIFI)
    # 添加网络源文件
    list(APPEND COMPONENT_SRCS
        "network/robot_bridge.cpp"
        "network/web_server_adapter.c"
        "network/websocket_handler.c"
        "network/i2c_diagnostic.cpp"
    )

    # 添加网络包含目录
    list(APPEND COMPONENT_INCLUDE_DIRS
        "network"
    )

    # 添加网络依赖
    list(APPEND COMPONENT_REQUIRES
        esp_wifi
        esp_http_server
        esp_netif
        esp_event
        json
    )

    message(STATUS "WiFi and Web Control: ENABLED")
else()
    message(STATUS "WiFi and Web Control: DISABLED (UART only)")
endif()

# ---------- 5. 注册组件 (只能调用一次!) ----------
# 注意: EMBED_TXTFILES 需要在 idf_component_register 中指定
# 但由于条件编译，需要分两种情况处理

if(CONFIG_ROBOT_ENABLE_WIFI)
    # WiFi 开启: 包含网页文件
    idf_component_register(
        SRCS ${COMPONENT_SRCS}
        INCLUDE_DIRS ${COMPONENT_INCLUDE_DIRS}
        REQUIRES ${COMPONENT_REQUIRES}
        PRIV_REQUIRES esp_timer
        EMBED_TXTFILES "network/web/index.html"
    )
else()
    # WiFi 关闭: 不包含网页文件
    idf_component_register(
        SRCS ${COMPONENT_SRCS}
        INCLUDE_DIRS ${COMPONENT_INCLUDE_DIRS}
        REQUIRES ${COMPONENT_REQUIRES}
        PRIV_REQUIRES esp_timer
    )
endif()
```

**⚠️ 注意事项**:
1. `idf_component_register()` 虽然出现两次，但只会执行其中一次（互斥）
2. `EMBED_TXTFILES` 只在 WiFi 开启时生效
3. 嵌入的文件通过 `extern const char index_html_start[]` 等符号访问

---

## 五、主程序设计 (main.cpp)

### 1. 代码框架

```cpp
/**
 * @file main.cpp
 * @brief ESP32 四足机器人控制系统 v4.0.0 (合并版)
 */

#include <esp_log.h>
#include "config/ConfigManager.hpp"
#include "controllers/RobotController.hpp"
#include "controllers/SmoothMotionController.hpp"
#include "kinematics/QuadrupedModel.hpp"
#include "utils/Logger.hpp"

#ifdef CONFIG_ROBOT_ENABLE_WIFI
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "network/robot_bridge.h"
#include "network/web_server_adapter.h"
#endif

#ifdef CONFIG_ROBOT_ENABLE_UART_CONTROL
#include "utils/UartTerminal.hpp"
#endif

// ==================== 全局变量 ====================

static std::shared_ptr<RobotController> g_robot_controller_;
static std::shared_ptr<SmoothMotionController> g_smooth_controller_;
static std::shared_ptr<QuadrupedModel> g_quadruped_model_;

#ifdef CONFIG_ROBOT_ENABLE_WIFI
static httpd_handle_t g_web_server_ = nullptr;
static bool g_wifi_connected_ = false;
#endif

// ==================== 初始化函数 ====================

void init_robot_system() {
    LOG_INFO("初始化机器人系统...");

    g_robot_controller_ = std::make_shared<RobotController>();
    g_robot_controller_->init();

    g_quadruped_model_ = std::make_shared<QuadrupedModel>();

    g_smooth_controller_ = std::make_shared<SmoothMotionController>(g_robot_controller_);
    g_smooth_controller_->setQuadrupedModel(g_quadruped_model_);
    g_smooth_controller_->initialize();

    LOG_INFO("机器人系统初始化完成");
}

#ifdef CONFIG_ROBOT_ENABLE_WIFI
void init_wifi_system() {
    // WiFi 初始化代码
    // ...
}

void init_web_control_system() {
    // 启动 HTTP 服务器
    g_web_server_ = start_webserver();

    // 初始化机器人桥接层
    robot_bridge_init(g_robot_controller_);
}
#endif

// ==================== 主程序入口 ====================

extern "C" void app_main() {
    LOG_INFO("=== ESP32 四足机器人控制系统 v4.0.0 (合并版) ===");

    // 1. 初始化机器人系统 (必须)
    init_robot_system();

#ifdef CONFIG_ROBOT_ENABLE_WIFI
    // 2. 初始化 WiFi 和网页控制 (可选)
    init_wifi_system();
    init_web_control_system();
    LOG_INFO("🌐 网页控制已启用");
#else
    LOG_INFO("🌐 网页控制未启用 (仅串口控制)");
#endif

#ifdef CONFIG_ROBOT_ENABLE_UART_CONTROL
    // 3. 初始化 UART 串口控制 (可选)
    LOG_INFO("💻 串口控制已启用");
#endif

    LOG_INFO("系统初始化完成！");
}
```

### 2. 主程序合并详细指南

**源文件**:
- `body_control_web/main/pca9685Test.cpp` (1715行) - 主要来源
- `body_control_uart/main/pca9685Test.cpp` (1174行) - 参考

**合并策略**: 以 web 版为基础，添加条件编译宏

#### 步骤 A: 提取 WiFi 初始化代码

从 web 版 pca9685Test.cpp 提取以下函数，用 `#ifdef CONFIG_ROBOT_ENABLE_WIFI` 包裹：

```cpp
#ifdef CONFIG_ROBOT_ENABLE_WIFI
// WiFi 配置
#define WIFI_SSID "your_wifi_ssid"
#define WIFI_PASS "your_wifi_password"
#define SERVER_PORT 12580

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data);
static void init_wifi();
static httpd_handle_t start_webserver(void);
static void stop_webserver(httpd_handle_t server);
#endif
```

#### 步骤 B: 提取 Web 服务器代码

```cpp
#ifdef CONFIG_ROBOT_ENABLE_WIFI
#include "network/robot_bridge.h"
#include "network/web_server_adapter.h"

// HTTP 服务器句柄
static httpd_handle_t g_web_server_ = nullptr;

// WebSocket 连接状态
static bool g_wifi_connected_ = false;
#endif
```

#### 步骤 C: 整合 UART 命令处理

UART 命令处理代码在两个版本中相同，直接使用：

```cpp
#ifdef CONFIG_ROBOT_ENABLE_UART_CONTROL
#include "utils/UartTerminal.hpp"

static std::shared_ptr<UartTerminal> g_uart_terminal_;

void init_uart_control() {
    g_uart_terminal_ = std::make_shared<UartTerminal>(
        g_robot_controller_,
        g_smooth_controller_,
        g_quadruped_model_
    );
    g_uart_terminal_->start();
}
#endif
```

#### 步骤 D: 主函数整合

```cpp
extern "C" void app_main() {
    // 1. 初始化机器人系统 (必须)
    init_robot_system();

#ifdef CONFIG_ROBOT_ENABLE_WIFI
    // 2. 初始化 WiFi
    init_wifi();

    // 3. 启动 Web 服务器
    g_web_server_ = start_webserver();

    // 4. 初始化机器人桥接层
    robot_bridge_init(g_robot_controller_, g_smooth_controller_, g_quadruped_model_);

    LOG_INFO("🌐 网页控制已启用: http://%s:%d", wifi_get_ip(), SERVER_PORT);
#endif

#ifdef CONFIG_ROBOT_ENABLE_UART_CONTROL
    // 5. 初始化 UART 串口控制
    init_uart_control();
    LOG_INFO("💻 串口控制已启用");
#endif

    LOG_INFO("系统初始化完成！");
}
```

### 3. 关键代码位置对照表

| 功能模块 | web版位置 | 行数 | 合并后位置 |
|----------|-----------|------|------------|
| WiFi 初始化 | pca9685Test.cpp | 100-200 | main.cpp `#ifdef WIFI` |
| HTTP 服务器 | pca9685Test.cpp | 200-350 | main.cpp `#ifdef WIFI` |
| WebSocket | websocket_handler.c | 全文 | network/ |
| robot_bridge | robot_bridge.cpp | 全文 | network/ |
| UART 命令 | pca9685Test.cpp | 400-600 | main.cpp `#ifdef UART` |
| 运动学控制 | pca9685Test.cpp | 600-900 | main.cpp (通用) |
| 主循环 | pca9685Test.cpp | 900-1100 | main.cpp (通用) |

---

## 六、合并执行步骤

### 步骤 1: 创建新工程目录

```bash
cd /home/cm/1_work/2_Robot/2_Quadruped_robot/0_SpotMicroAI/Ai_Code/spot_micro_esp32

mkdir -p body_control/main/network/web
mkdir -p body_control/components/pca9685
```

### 步骤 2: 复制核心代码 (从 web 版)

```bash
# 复制核心目录 (保持原有结构)
cp -r body_control_web/main/config body_control/main/
cp -r body_control_web/main/controllers body_control/main/
cp -r body_control_web/main/drivers body_control/main/
cp -r body_control_web/main/kinematics body_control/main/
cp -r body_control_web/main/utils body_control/main/

# 复制类型定义
cp body_control_web/main/RobotTypes.hpp body_control/main/
```

### 步骤 3: 复制网络功能代码

```bash
mkdir -p body_control/main/network/web

cp body_control_web/main/robot_bridge.* body_control/main/network/
cp body_control_web/main/web_server_adapter.c body_control/main/network/
cp body_control_web/main/websocket_handler.* body_control/main/network/
cp body_control_web/main/i2c_diagnostic.* body_control/main/network/
cp body_control_web/main/web/index.html body_control/main/network/web/
```

### 步骤 4: 复制组件

```bash
cp -r body_control_web/components/* body_control/components/
```

### 步骤 5: 创建配置文件

```bash
# sdkconfig.defaults
cat > body_control/sdkconfig.defaults << 'EOF'
CONFIG_HTTPD_WS_SUPPORT=y
EOF

# Kconfig.projbuild (使用上面的内容)
# CMakeLists.txt (使用上面的内容)
```

### 步骤 6: 创建主程序

```bash
# 创建 main.cpp (使用上面的代码框架)
# 创建 main/CMakeLists.txt (使用上面的条件编译版本)
```

### 步骤 7: 编译测试

```bash
cd body_control

# 配置项目
idf.py menuconfig
# → Component config → Robot Control Configuration

# 编译测试 (WiFi 开启)
idf.py build

# 编译测试 (WiFi 关闭)
# menuconfig 中关闭 ROBOT_ENABLE_WIFI
idf.py build
```

### 步骤 8: 删除旧工程 (确认无误后)

```bash
cd ..
rm -rf body_control_uart
rm -rf body_control_web
```

---

## 七、验证清单

| 验证项 | 命令 | 预期结果 |
|--------|------|----------|
| ✅ WiFi 开启编译 | `idf.py build` | 编译成功 |
| ✅ WiFi 关闭编译 | `idf.py build` | 编译成功 |
| ✅ UART 控制功能 | 串口发送命令 | 正常响应 |
| ✅ 网页控制功能 | 浏览器访问 | 界面正常 |
| ✅ WebSocket 通信 | 实时控制 | 响应正常 |
| ✅ 运动学计算 | pose6 命令 | 计算正确 |
| ✅ 舵机控制 | s 命令 | 动作正确 |

---

## 八、预期效果

| 指标 | 合并前 | 合并后 |
|------|--------|--------|
| **工程数量** | 2 个 | 1 个 |
| **代码重复** | ~90% | 0% |
| **存储空间** | ~370MB | ~170MB |
| **维护难度** | 高 (需同步) | 低 (单一工程) |
| **配置灵活性** | 固定 | 可配置 (menuconfig) |

---

## 九、注意事项

### ⚠️ CMakeLists.txt 关键点

1. **idf_component_register() 只能调用一次** - 但在互斥的 if/else 中可以各调用一次
2. 使用 `list(APPEND ...)` 条件添加源文件
3. 使用 `set()` 预定义变量，最后统一注册
4. **EMBED_TXTFILES 必须在 idf_component_register 中指定**，不能用 target_add_binary_data

### ⚠️ include 路径

保持原有目录结构，不需要修改 `#include` 路径：
- 核心模块: `#include "config/ConfigManager.hpp"`
- 网络模块: `#include "network/robot_bridge.h"`

### ⚠️ 配置宏名称

- 使用 `CONFIG_ROBOT_ENABLE_WIFI` (不是 `CONFIG_ENABLE_WIFI`)
- 使用 `CONFIG_ROBOT_ENABLE_UART_CONTROL`
- 检查宏: `#ifdef CONFIG_ROBOT_ENABLE_WIFI` (不是 `#if`)

### ⚠️ 硬件兼容性

合并后的代码支持两种运行模式：

| 模式 | 条件 | 行为 |
|------|------|------|
| **硬件模式** | PCA9685 就绪 | 正常控制舵机 |
| **仿真模式** | PCA9685 未检测到 | 更新缓存但不发送 PWM |

判断方式: `robot_controller_->isHardwareReady()`

### ⚠️ 网页文件访问

使用 EMBED_TXTFILES 嵌入后，通过以下方式访问：

```c
// 声明外部符号 (由链接器生成)
extern const uint8_t index_html_start[] asm("_binary_network_web_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_network_web_index_html_end");

// 在 HTTP 处理函数中使用
httpd_resp_send(req, (const char*)index_html_start, index_html_end - index_html_start);
```

---

## 十、评审结论

### 评审结果: ✅ 通过

| 评估项 | 评分 | 说明 |
|--------|------|------|
| **整体方案可行性** | ⭐⭐⭐⭐⭐ (5/5) | 方案设计合理，可执行 |
| **文件分析准确性** | ⭐⭐⭐⭐⭐ (5/5) | 文件差异分析完全正确 |
| **CMakeLists.txt 设计** | ⭐⭐⭐⭐⭐ (5/5) | 条件编译正确 |
| **主程序合并细节** | ⭐⭐⭐⭐⭐ (5/5) | 提供详细合并指南 |

### 可执行性评估

1. **技术可行性**: ✅ 条件编译机制成熟，ESP-IDF 原生支持
2. **风险等级**: 🟢 低风险 - 保留原有代码，只做整合
3. **回滚方案**: 保留原工程目录，确认无误后再删除

---

## 十一、执行进度记录

### 执行日期: 2025-03-05

### 步骤完成状态

| 步骤 | 描述 | 状态 | 备注 |
|------|------|------|------|
| 1 | 创建新工程目录 | ✅ 完成 | `body_control/` 目录已创建 |
| 2 | 复制核心代码 | ✅ 完成 | config, controllers, drivers, kinematics, utils |
| 3 | 复制网络功能代码 | ✅ 完成 | network/ 目录 (robot_bridge, web_server, websocket) |
| 4 | 复制 PCA9685 组件 | ✅ 完成 | components/pca9685/ |
| 5 | 创建配置文件 | ✅ 完成 | Kconfig.projbuild, sdkconfig.defaults, CMakeLists.txt |
| 6 | 创建主程序 | ✅ 完成 | main.cpp 已添加条件编译 |
| 7 | 编译测试 | ✅ 完成 | 编译成功，生成 body_control.bin |
| 8 | 删除旧工程 | ⏸️ 待执行 | 确认编译通过后执行 |

### 遇到的编译问题 (已解决)

#### 问题 1: C++ 标准不兼容 ✅ 已解决
- **现象**: `std::make_unique` 在 C++11 下不可用
- **原因**: ESP-IDF v4.4 默认使用 gnu++11
- **解决**: 在 CMakeLists.txt 中添加 `set(CMAKE_CXX_STANDARD 17)`

#### 问题 2: WiFi 代码未条件编译 ✅ 已解决
- **现象**: WiFi 关闭时，WiFi 相关代码仍然编译
- **原因**: main.cpp 中 WiFi 代码没有 `#ifdef CONFIG_ROBOT_ENABLE_WIFI` 包裹
- **解决**: 在 main.cpp 中添加条件编译宏包裹 WiFi 相关代码

#### 问题 3: ESP-IDF API 版本差异 ✅ 已解决
- **现象**: `UART_SCLK_DEFAULT` 未定义
- **原因**: ESP-IDF v4.4 不支持此常量
- **解决**: 替换为 `UART_SCLK_APB` (main.cpp 和 UartTerminal.cpp)

### 当前工作目录结构

```
spot_micro_esp32/body_control/
├── CMakeLists.txt              ✅ (已添加 C++17 支持)
├── Kconfig.projbuild           ✅
├── sdkconfig.defaults          ✅
├── components/pca9685/         ✅
└── main/
    ├── CMakeLists.txt          ✅ (已更新源文件列表)
    ├── main.cpp                ⚠️ (需添加条件编译)
    ├── RobotTypes.hpp          ✅
    ├── config/                 ✅ (6个文件)
    ├── controllers/            ✅ (10个文件)
    ├── drivers/                ✅ (8个文件)
    ├── kinematics/             ✅ (10个文件)
    ├── utils/                  ✅ (7个文件)
    └── network/                ✅ (8个文件)
```

### 下一步行动

1. ✅ 修改 `main.cpp` 添加 `#ifdef CONFIG_ROBOT_ENABLE_WIFI` 条件编译
2. ✅ 修复 `UartTerminal.cpp` 的 ESP-IDF API 兼容性
3. ✅ 修正 include 路径
4. ✅ 重新编译测试 - 成功生成 body_control.bin

### 最终目录结构

```
spot_micro_esp32/body_control/
├── CMakeLists.txt              ✅
├── Kconfig.projbuild           ✅
├── sdkconfig.defaults          ✅
├── components/pca9685/         ✅
└── main/
    ├── CMakeLists.txt          ✅
    ├── main.cpp                ✅ (已添加条件编译)
    ├── RobotTypes.hpp          ✅
    ├── config/                 ✅ (6个文件)
    ├── controllers/            ✅ (10个文件)
    ├── drivers/                ✅ (8个文件)
    ├── kinematics/             ✅ (10个文件)
    ├── utils/                  ✅ (7个文件)
    └── network/                ✅ (8个文件)
```

### 编译结果

```
Project build complete. To flash, run this command:
idf.py -p (PORT) flash
```

**固件大小**: 0x460800 bytes (约 4.4 MB)

---

**文档版本**: v3.2 (执行完成版)
**创建日期**: 2025-03-04
**最后更新**: 2025-03-05
**状态**: ✅ 编译测试通过
