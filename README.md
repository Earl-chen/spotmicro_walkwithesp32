# SpotMicroAI - 四足机器人控制系统

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.5-blue.svg)](https://github.com/espressif/esp-idf)
[![Platform](https://img.shields.io/badge/Platform-ESP32-orange.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Status](https://img.shields.io/badge/Status-开发中-yellow.svg)]()

一个完整的四足机器人控制系统，包含 **Python 快速原型**、**C++ 核心算法** 和 **ESP32 嵌入式固件**。

---

## 🔄 开发流程

```
1️⃣ Python 快速原型
   spot_micro_body_control_sim/
   ↓ 算法验证通过
   
2️⃣ C++ 实现（PC端验证）
   algorithms/
   ├── kinematics/    运动学
   └── gait/          步态
   ↓ C++ 验证通过
   
3️⃣ ESP32 部署（复用C++代码）
   spot_micro_esp32/
```

---

## 📁 项目结构

```
spotmicro/
│
├── algorithms/                    # 🧮 C++ 核心算法（PC验证+ESP32复用）
│   │
│   ├── kinematics/                # 运动学（FK/IK）
│   │   ├── kinematics/            # 算法实现（6个文件）
│   │   ├── tests/                 # 测试框架（2个文件）
│   │   └── kinematics_test        # 命令行工具
│   │
│   └── gait/                      # 步态控制
│       ├── gait/                  # 算法实现（3个文件）
│       ├── tests/                 # 测试框架（1个文件）
│       └── gait_test              # 命令行工具
│
│   # 💡 用途：Python验证 → C++实现 → ESP32部署
│   # 与 spot_micro_esp32 共享相同代码
│
├── spot_micro_body_control_sim/   # 🐍 Python 快速原型（53个.py）
│   ├── 01_spot_micro_leg_kinematics_lab/   # 单腿运动学实验
│   ├── 02_spot_micro_simulator_standalone/ # 独立仿真器
│   ├── 03_spot_micro_simulator_framework/  # 完整仿真框架（37个.py）
│   ├── 04_spot_micro_gait_control/         # Walk步态控制（10个.py）
│   └── fonts/                              # 共享字体
│
├── spot_micro_esp32/              # 🔧 ESP32 固件（7个项目，19个.c/.h）
│   ├── 01_test_pca9685_standalone/         # ⭐ PCA9685驱动测试
│   ├── 02_test_servo_angle_control/        # ⭐⭐ 舵机角度控制
│   ├── 03_test_joint_controller/           # ⭐⭐⭐ 关节控制器
│   ├── 04_test_kinematics_controller/      # ⭐⭐⭐⭐ 运动学控制器
│   ├── 05_test_smooth_motion_controller/   # ⭐⭐⭐⭐ 平滑运动控制器
│   ├── 06_body_control_uart/               # ⭐⭐⭐⭐⭐ UART串口控制
│   └── 07_body_control_web/                # ⭐⭐⭐⭐⭐⭐ WiFi网页控制
│
├── doc/                           # 📚 文档（4个.md）
│   ├── 01_系统架构/                         # 系统架构设计
│   ├── 02_运动学公式/                       # 运动学公式推导
│   └── 03_硬件相关/                         # 硬件连接、舵机校准
│
├── CLAUDE.md                      # Claude Code 项目配置
├── .gitignore                     # Git 忽略规则
└── README.md                      # 本文档
```

---

## 📊 项目统计

| 模块 | 类型 | 文件数 | 用途 |
|------|------|--------|------|
| **algorithms/** | C++ | 14个 | 核心算法（PC验证+ESP32复用） |
| **spot_micro_body_control_sim/** | Python | 53个 | 快速原型验证 |
| **spot_micro_esp32/** | C | 19个 | 实机部署 |
| **doc/** | Markdown | 4个 | 文档 |

---

## 🚀 核心功能

### 🦾 运动控制
| 功能 | 说明 |
|------|------|
| **12关节控制** | 每条腿3个关节，支持独立控制 |
| **正向运动学 (FK)** | 关节角度 → 脚部坐标 |
| **逆向运动学 (IK)** | 脚部坐标 → 关节角度 |
| **平滑运动** | 姿态插值、关节平滑 |
| **预设姿势** | 站立/趴下/休息等 |

### 🎮 控制接口
| 方式 | 项目 | 特点 |
|------|------|------|
| **UART** | 06_body_control_uart | 串口命令行，调试方便 |
| **Web** | 07_body_control_web | WiFi + WebSocket，远程控制 |

### ⚠️ 安全特性
- 每个舵机独立校准 (中位PWM: **615-685**, 不是350!)
- 关节角度限制保护
- 紧急停止功能

---

## 🔧 硬件要求

### ESP32 固件
| 组件 | 规格 |
|------|------|
| **MCU** | ESP32-D0WD-V3 |
| **PWM驱动** | PCA9685 (I2C地址: 0x40) |
| **舵机** | 12个标准舵机 (0-180°) |
| **I2C引脚** | SDA=GPIO21, SCL=GPIO22 |
| **供电** | 舵机独立5V电源 (推荐3A以上) |

### 舵机分配
```
通道 0-2  → 左前腿 (髋侧摆, 髋俯仰, 膝俯仰)
通道 3-5  → 右前腿
通道 6-8  → 左后腿
通道 9-11 → 右后腿
```

---

## 🏃 快速开始

### ESP32 固件开发

```bash
# 1. 加载 ESP-IDF 环境
source ~/esp/v5.5/esp-idf/export.sh

# 2. 进入项目 (推荐从01开始学习)
cd spot_micro_esp32/01_test_pca9685_standalone

# 3. 编译
idf.py build

# 4. 烧录并监控
idf.py -p /dev/ttyUSB0 flash monitor
```

### Python 仿真

```bash
# 1. 安装依赖
pip install numpy matplotlib

# 2. 运行仿真
cd spot_micro_body_control_sim/03_spot_micro_simulator_framework
python3 app/spot_micro_app.py
```

### 核心算法测试（algorithms/）

```bash
# 运动学测试
cd algorithms/kinematics
make -f Makefile.kinematics_test
./kinematics_test --quick         # 快速验证
./kinematics_test --roundtrip     # FK-IK往返测试

# 步态测试
cd algorithms/gait
make -f Makefile.gait_test
./gait_test --quick               # 快速验证
./gait_test --batch               # 批量测试
```

> 💡 **开发流程**：
> 1. **Python原型**：在 `spot_micro_body_control_sim/` 中快速验证算法思路
> 2. **C++实现**：在 `algorithms/` 中实现算法，PC端验证通过
> 3. **ESP32部署**：复用 `algorithms/` 的C++代码，部署到实机

---

## 📚 学习路径

推荐按以下顺序学习：

```
Step 1: 01_test_pca9685_standalone
        ↓ 学习 I2C通信、PWM输出、舵机基础

Step 2: 02_test_servo_angle_control
        ↓ 学习 角度→PWM转换、舵机校准

Step 3: 03_test_joint_controller
        ↓ 学习 关节映射、预设姿势

Step 4: 04_test_kinematics_controller
        ↓ 学习 正向/逆向运动学、坐标变换

Step 5: 05_test_smooth_motion_controller
        ↓ 学习 姿态插值、平滑运动

Step 6: 06_body_control_uart
        ↓ 掌握 UART串口完整控制系统

Step 7: 07_body_control_web
        ↓ 掌握 WiFi + HTTP + WebSocket 网页控制
```

---

## 📖 文档索引

| 文档 | 说明 |
|------|------|
| [舵机校准指南](doc/03_硬件相关/舵机校准指南.md) | 使用01项目校准舵机 |
| [逆运动学公式推导](doc/02_运动学公式/逆运动学公式推导.md) | IK数学公式 |
| [ESP32工程合并方案](doc/01_系统架构/ESP32工程合并方案.md) | 系统架构设计 |

---

## 🛠️ 开发环境

| 工具 | 版本 |
|------|------|
| **ESP-IDF** | v5.5 |
| **Python** | 3.7+ |
| **构建工具** | CMake + Ninja |
| **编译器** | xtensa-esp32-elf-gcc |

---

## 📄 许可证

MIT License

---

## 🙏 致谢

基于 [SpotMicroAI](https://github.com/OpenQuadruped/spot_micro_ai) 开源项目开发。
