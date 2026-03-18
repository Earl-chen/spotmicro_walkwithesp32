# Spot Micro Python - 四足机器人控制系统

本仓库包含三套 Spot Micro 四足机器人的 Python 实现，按复杂度从低到高排序。

## 目录结构

```
spot_micro_python/
│
├── 01_spot_micro_leg_kinematics_lab/      # 基础：单腿运动学实验室
│   # - 纯数学计算（IK/FK）
│   # - 批量计算支持
│   # - 工作空间分析
│   # - 交互式研究工具
│
├── 02_spot_micro_simulator_standalone/    # 中级：独立模拟器
│   # - 单文件完整实现
│   # - 18参数控制（6位姿 + 12关节）
│   # - 脚步锁定功能
│   # - 开箱即用
│
└── 03_spot_micro_simulator_framework/     # 高级：模块化框架
    # - 三层架构（core/robots/app）
    # - 双模式控制（IK/FK）
    # - 坐标系树管理
    # - 可扩展设计
```

## 快速开始

### 1. 入门 - 单腿运动学实验室

```bash
cd 01_spot_micro_leg_kinematics_lab
python run_lab.py
```

功能菜单：
- 基础功能测试（IK/FK精度验证）
- 单腿交互式可视化
- 双腿对比显示
- 批量计算演示

### 2. 进阶 - 独立模拟器

```bash
cd 02_spot_micro_simulator_standalone
python complete_quadruped_world_control.py
```

特点：
- 单文件，无需配置
- 18个滑块控制
- 实时3D可视化

### 3. 高级 - 模块化框架

```bash
cd 03_spot_micro_simulator_framework
python run_spot_micro.py
```

特点：
- 生产级架构
- 支持扩展新机器人
- 完整文档和测试

## 机器人参数

| 参数 | 值 | 说明 |
|-----|-----|-----|
| L1 | 60.5 mm | 髋关节延伸段 |
| L2 | 10.0 mm | 髋关节垂直段 |
| L3 | 111.126 mm | 大腿长度 |
| L4 | 118.5 mm | 小腿长度 |
| 机体长度 | 207.5 mm | - |
| 机体宽度 | 78 mm | - |

## 依赖

```bash
pip install numpy matplotlib
```

## 学习路径

1. **01_leg_kinematics_lab** - 理解单腿运动学原理
2. **02_simulator_standalone** - 体验完整四足控制
3. **03_simulator_framework** - 学习模块化架构设计

## 许可证

MIT License
