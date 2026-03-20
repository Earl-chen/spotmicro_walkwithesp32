# SpotMicroAI - 四足机器人控制系统

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.5-blue.svg)](https://github.com/espressif/esp-idf)
[![Platform](https://img.shields.io/badge/Platform-ESP32-orange.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Status](https://img.shields.io/badge/Status-开发中-yellow.svg)]()

一个完整的四足机器人控制系统，包含 **ESP32 嵌入式固件**、**Python 3D 仿真** 和 **PC 端运动学验证工具**。

---

## 📁 项目结构

```
Ai_Code/
├── spot_micro_esp32/              # 🔧 ESP32 固件项目 (1.2GB, 243个源文件)
│   ├── 01_test_pca9685_standalone/         # ⭐ PCA9685驱动测试 (1,765行)
│   ├── 02_test_servo_angle_control/        # ⭐⭐ 舵机角度控制 (3,347行)
│   ├── 03_test_joint_controller/           # ⭐⭐⭐ 关节控制器 (7,954行)
│   ├── 04_test_kinematics_controller/      # ⭐⭐⭐⭐ 运动学控制器 (8,180行)
│   ├── 05_test_smooth_motion_controller/   # ⭐⭐⭐⭐ 平滑运动控制器 (7,836行)
│   ├── 06_body_control_uart/               # ⭐⭐⭐⭐⭐ UART串口控制 (8,615行)
│   └── 07_body_control_web/                # ⭐⭐⭐⭐⭐⭐ WiFi网页控制 (12,712行)
│
├── spot_micro_body_control_sim/   # 🖥️ Python 3D 仿真 (656KB)
│   ├── 01_spot_micro_leg_kinematics_lab/   # 单腿运动学实验室
│   ├── 02_spot_micro_simulator_standalone/ # 独立仿真器
│   └── 03_spot_micro_simulator_framework/   # 完整仿真框架
│
├── kinematics_test_standalone/   # 🧪 PC端运动学测试 (5.3MB)
│   │                                       # 与 spot_micro_esp32 共享相同运动学代码
│   │                                       # 用于在PC上验证算法后再部署到ESP32
│   ├── kinematics/                          # 运动学库（与ESP32固件同源）
│   ├── tests/                               # 测试框架
│   └── kinematics_test_main.cpp             # 命令行测试程序
│
├── doc/                          # 📚 文档中心 (84KB)
│   ├── 01_系统架构/                         # 系统架构设计
│   ├── 02_运动学公式/                       # 运动学公式推导
│   ├── 03_硬件相关/                         # 硬件连接、舵机校准
│   ├── 04_使用指南/                         # 使用指南
│   └── 05_历史归档/                         # 历史文档归档
│
├── CLAUDE.md                     # Claude Code 项目配置
├── .gitignore                    # Git 忽略规则
└── README.md                     # 本文档
```

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

### PC 端运动学测试

```bash
# 编译运行
cd kinematics_test_standalone
make -f Makefile.kinematics_test
./kinematics_test

# 快速验证
./kinematics_test --quick

# FK-IK往返测试（验证算法精度）
./kinematics_test --roundtrip
```

> 💡 **用途**：本模块与 `spot_micro_esp32/` 共享相同的运动学算法代码。在修改运动学算法前，先在此验证 FK-IK 往返误差 < 0.001°，确保算法正确后再部署到 ESP32。

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
