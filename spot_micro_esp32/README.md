# ESP32 四足机器人控制系统

## 📁 项目结构

```
spot_micro_esp32/
├── 01_test_pca9685_standalone/       # ⭐ PCA9685硬件驱动
├── 02_test_servo_angle_control/      # ⭐⭐ 舵机角度控制
├── 03_test_joint_controller/         # ⭐⭐⭐ 关节控制器
├── 04_test_kinematics_controller/    # ⭐⭐⭐⭐ 运动学控制器
├── 05_test_smooth_motion_controller/ # ⭐⭐⭐⭐ 平滑运动控制器
├── 06_body_control_uart/             # ⭐⭐⭐⭐⭐ UART串口控制
└── 07_body_control_web/              # ⭐⭐⭐⭐⭐⭐ WiFi网页控制
```

## 🎯 学习路径

```
Step 1: 01_test_pca9685_standalone (1,765行)
        ↓ I2C通信、PWM输出、舵机基础

Step 2: 02_test_servo_angle_control (3,347行)
        ↓ 角度→PWM转换、舵机校准

Step 3: 03_test_joint_controller (7,954行)
        ↓ 关节映射、预设姿势

Step 4: 04_test_kinematics_controller (8,180行)
        ↓ 正向/逆向运动学、坐标变换

Step 5: 05_test_smooth_motion_controller (7,836行)
        ↓ 姿态插值、平滑运动

Step 6: 06_body_control_uart (8,615行)
        ↓ UART串口命令行控制

Step 7: 07_body_control_web (12,712行)
        ↓ WiFi + HTTP + WebSocket 网页控制
```

## 🔧 硬件要求

- ESP32 开发板
- PCA9685 PWM 驱动器 (I2C地址: 0x40)
- 12个舵机 (四足机器人)
- I2C接线: SDA=GPIO21, SCL=GPIO22

## ⚠️ 重要安全提示

**舵机中位PWM值范围: 615-685** (不是350!)

每个舵机有独立的校准范围，请勿使用统一的PWM值！

## 🚀 快速开始

```bash
# 进入第一步
cd 01_test_pca9685_standalone

# 编译
idf.py build

# 烧录
idf.py -p /dev/ttyUSB0 flash monitor
```

## 📊 代码复杂度

| 项目 | 代码行数 | 难度 |
|------|---------|------|
| 01_test_pca9685_standalone | 1,765 | ⭐ |
| 02_test_servo_angle_control | 3,347 | ⭐⭐ |
| 03_test_joint_controller | 7,954 | ⭐⭐⭐ |
| 04_test_kinematics_controller | 8,180 | ⭐⭐⭐⭐ |
| 05_test_smooth_motion_controller | 7,836 | ⭐⭐⭐⭐ |
| 06_body_control_uart | 8,615 | ⭐⭐⭐⭐⭐ |
| 07_body_control_web | 12,712 | ⭐⭐⭐⭐⭐⭐ |
