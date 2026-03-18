# Spot Micro 运动学实验室

单腿运动学分析工具包，专注于运动学算法研究和轨迹规划开发。

## 功能特点

- **批量计算** - 支持轨迹点的批量 IK/FK 计算
- **工作空间分析** - 计算腿部可达范围边界
- **交互式工具** - 角度与坐标双向实时转换
- **双腿对比** - 左右腿姿态并排对比显示
- **模块化设计** - 核心计算与应用逻辑分离

## 目录结构

```
01_spot_micro_leg_kinematics_lab/
├── __init__.py              # 模块入口，导出公共API
├── kinematics_core.py       # 核心数学计算（纯函数）
├── kinematics_app.py        # 应用层封装（验证+日志）
├── kinematics_visualizer.py # 3D可视化（交互式+静态）
├── run_lab.py               # 主程序入口（菜单系统）
└── README.md
```

## 快速开始

### 安装依赖

```bash
pip install numpy matplotlib
```

### 运行主程序

```bash
cd 01_spot_micro_leg_kinematics_lab
python run_lab.py
```

### 快速演示模式

```bash
python run_lab.py --demo
```

## 功能菜单

```
1. 基础功能测试         - IK/FK 精度验证
2. 左腿交互式可视化     - 角度↔坐标双向转换
3. 右腿交互式可视化     - 角度↔坐标双向转换
4. 双腿对比可视化       - 左右腿并排显示
5. 四足机器人控制器测试 - 站立姿态/步态测试
6. 批量计算演示         - 轨迹规划演示
7. 退出
```

## 架构设计

### 三层架构

```
┌─────────────────────────────────────┐
│           run_lab.py                │  入口层：菜单系统
└─────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────┐
│      kinematics_visualizer.py       │  可视化层：3D渲染
└─────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────┐
│        kinematics_app.py            │  应用层：验证+日志
└─────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────┐
│        kinematics_core.py           │  核心层：纯数学计算
└─────────────────────────────────────┘
```

### 模块职责

| 模块 | 职责 |
|------|------|
| `kinematics_core.py` | 纯数学计算，无副作用，支持批量计算 |
| `kinematics_app.py` | 工作空间验证、角度范围检查、错误处理 |
| `kinematics_visualizer.py` | 3D可视化、交互式界面 |
| `run_lab.py` | 主程序入口、功能菜单 |

## 编程接口

### 基础使用

```python
from kinematics_app import create_leg_controller

# 创建腿部控制器
leg = create_leg_controller('spot_micro', verbose=True)

# 逆运动学：位置 → 角度
angles = leg.solve_position(50, 60.5, -180, 'left')
# 返回: (hip_side_deg, hip_pitch_deg, knee_pitch_deg)

# 正运动学：角度 → 位置
position = leg.calculate_position(10, -20, -45, 'left')
# 返回: (x, y, z)
```

### 批量计算

```python
from kinematics_core import LegKinematics

kinematics = LegKinematics()

# 定义轨迹点
trajectory = [
    (0, 60.5, -200),
    (50, 60.5, -180),
    (100, 60.5, -160),
]

# 批量逆运动学
angles_list = kinematics.batch_inverse_kinematics(trajectory, is_left=True)

# 批量正运动学
positions = kinematics.batch_forward_kinematics(angles_list, is_left=True)
```

### 四足机器人控制

```python
from kinematics_app import QuadrupedController

controller = QuadrupedController(verbose=True)

# 设置四条腿的目标位置
positions = {
    'front_left':  (30, 60.5, -180),
    'front_right': (-30, -60.5, -200),
    'rear_left':   (-30, 60.5, -200),
    'rear_right':  (30, -60.5, -180),
}
results = controller.set_leg_positions(positions)

# 获取默认站立姿态
stance = controller.get_default_stance()
```

### 可视化

```python
from kinematics_app import create_leg_controller
from kinematics_visualizer import create_visualizer

leg = create_leg_controller('spot_micro', verbose=False)

# 交互式可视化
visualizer = create_visualizer(leg, mode='interactive', leg_side='left')
visualizer.create_interactive_window()

# 双腿对比
dual_viz = create_visualizer(leg, mode='dual')
fig = dual_viz.create_comparison_plot(
    left_angles=(15, -20, -60),
    right_angles=(-15, -20, -60)
)
```

## 机器人参数

### Spot Micro 默认配置

```python
# 腿部几何参数 (mm)
l1 = 60.5    # 髋关节延伸段长度
l2 = 10.0    # 髋关节垂直段长度
l3 = 111.126 # 大腿长度
l4 = 118.5   # 小腿长度

# 工作空间
min_reach = 50.0   # 最小距离
max_reach = 295.0  # 最大距离
```

### 角度范围

| 关节 | 范围 |
|------|------|
| 髋侧摆 (hip_side) | -90° ~ 90° |
| 髋俯仰 (hip_pitch) | -90° ~ 90° |
| 膝俯仰 (knee_pitch) | -180° ~ 0° |

### 默认站立姿态

```python
default_stance = {
    'front_left':  (0, 60.5, -200),
    'front_right': (0, -60.5, -200),
    'rear_left':   (0, 60.5, -200),
    'rear_right':  (0, -60.5, -200),
}
```

## 核心算法

### 逆运动学

```
输入: 脚部位置 (x, y, z)
输出: 关节角度 (hip_side, hip_pitch, knee_pitch)

算法步骤:
1. H = sqrt(y² + z²)
2. G = sqrt(H² - l1²)
3. F = G - l2
4. S = sqrt(F² + x²)
5. hip_side = atan2(z, y) ± acos(l1/H)
6. hip_pitch = acos((S²+l3²-l4²)/(2*S*l3)) - atan2(x, F)
7. knee_pitch = -acos((S²-l3²-l4²)/(2*l3*l4))
```

### 正运动学

```
输入: 关节角度 (hip_side, hip_pitch, knee_pitch)
输出: 脚部位置 (x, y, z)

算法: 旋转矩阵链式变换
1. R_hip_side = rot_x(hip_side)
2. R_hip_pitch = rot_y(hip_pitch)
3. R_knee = rot_y(knee_pitch)
4. foot_pos = hip + R_hip_side @ (l1 + l2 + R_hip_pitch @ (l3 + R_knee @ l4))
```

## API 参考

### LegKinematics (核心层)

| 方法 | 说明 |
|------|------|
| `inverse_kinematics(x, y, z, is_left)` | 逆运动学，返回弧度 |
| `forward_kinematics(hip_side_rad, hip_pitch_rad, knee_pitch_rad, is_left)` | 正运动学，输入弧度 |
| `batch_inverse_kinematics(positions, is_left)` | 批量逆运动学 |
| `batch_forward_kinematics(angles_list, is_left)` | 批量正运动学 |
| `get_joint_positions(...)` | 获取所有关节位置 |
| `get_workspace_bounds()` | 获取工作空间边界 |

### LegController (应用层)

| 方法 | 说明 |
|------|------|
| `solve_position(x, y, z, leg_side)` | 逆运动学，返回度数 |
| `calculate_position(hip_side_deg, hip_pitch_deg, knee_pitch_deg, leg_side)` | 正运动学，输入度数 |
| `get_joint_positions(...)` | 获取关节位置（用于可视化） |
| `test_basic_positions()` | 运行基本位置测试 |

### QuadrupedController

| 方法 | 说明 |
|------|------|
| `set_leg_positions(positions_dict)` | 设置多条腿的目标位置 |
| `get_default_stance()` | 获取默认站立姿态 |
| `test_symmetric_gait()` | 测试对称步态 |

### 工厂函数

```python
# 创建腿部控制器
leg = create_leg_controller('spot_micro', verbose=True)

# 创建可视化器
viz = create_visualizer(leg, mode='interactive', leg_side='left')
# mode: 'interactive' | 'static' | 'dual'
```

## 适用场景

- 运动学算法研究和验证
- 轨迹规划开发
- 工作空间分析
- 步态设计
- 教学演示

## 许可证

MIT License
