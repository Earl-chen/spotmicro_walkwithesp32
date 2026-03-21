# 03_gait_control - SpotMicro 步态控制模块

SpotMicro 四足机器人步态控制算法和可视化工具集

---

## 📋 目录结构

```
03_gait_control/
├── README.md                           # 本文件
├── gait_algo_core/                     # 核心算法（独立库）
│   ├── __init__.py                     # 模块初始化
│   ├── trajectory.py                   # 轨迹生成器
│   └── walk_gait.py                    # Walk 步态控制器（含差速转向）
├── tests/                              # 测试和验证
│   ├── test_walk_gait.py               # Walk 步态单元测试
│   ├── test_steering.py                # 差速转向测试
│   ├── test_chinese_font.py            # 中文字体测试
│   ├── chinese_font_config.py          # 字体配置工具
│   ├── verify_duty_cycle.py            # 占空比验证（75%）
│   ├── verify_support_legs.py          # 支撑腿验证（3条）
│   ├── verify_trajectory.py            # 轨迹生成验证
│   └── visual/                         # 可视化工具
│       ├── visualize_walk_gait.py      # 3D 步态动画
│       ├── visualize_steering.py       # 转向步态可视化
│       ├── plot_gait_trajectory_v2.py  # 步态轨迹图（2×2 子图）
│       ├── plot_leg_phases.py          # 腿部相位图
│       ├── generate_walking_gif.py     # 行走 GIF（高质量）
│       └── generate_lightweight_gif.py # 行走 GIF（轻量版）
├── docs/                               # 文档
│   ├── 运行指南.md
│   ├── 步态修正计划.md
│   ├── Walk步态占空比修正计划.md
│   └── 工作总结.md
└── TEST_FIX_GUIDE.md                   # 测试修复指南

共享资源（平级）:
../fonts/BabelStoneHan.ttf              # 中文字体（Git LFS）
```

---

## 🚀 快速开始

### 安装依赖

```bash
pip install numpy matplotlib
```

### 运行测试

**前提**：先进入 `03_gait_control` 目录

```bash
cd /path/to/spot_micro_body_control_sim/03_gait_control

# 运行所有测试
python3 tests/test_walk_gait.py

# 验证占空比（75%）
python3 tests/verify_duty_cycle.py

# 验证支撑腿数量（3条）
python3 tests/verify_support_legs.py

# 验证轨迹生成
python3 tests/verify_trajectory.py

# 测试差速转向
python3 tests/test_steering.py

# 测试中文字体
python3 tests/test_chinese_font.py
```

### 生成可视化

**前提**：先进入 `03_gait_control` 目录

```bash
cd /path/to/spot_micro_body_control_sim/03_gait_control

# 3D 步态动画
python3 tests/visual/visualize_walk_gait.py

# 转向可视化
python3 tests/visual/visualize_steering.py

# 步态轨迹图（2×2 子图）
python3 tests/visual/plot_gait_trajectory_v2.py

# 腿部相位图
python3 tests/visual/plot_leg_phases.py

# 行走 GIF（高质量，3.9MB）
python3 tests/visual/generate_walking_gif.py

# 行走 GIF（轻量版，250KB）
python3 tests/visual/generate_lightweight_gif.py
```

---

## 📚 核心算法（gait_algo_core/）

### 1. trajectory.py - 轨迹生成器

**功能**：生成摆线、椭圆、贝塞尔三种轨迹

**API**：
```python
from gait_algo_core.trajectory import TrajectoryGenerator

# 摆线轨迹（推荐）
x, y, z = TrajectoryGenerator.cycloid_trajectory(
    phase=0.5,           # 相位（0-1）
    stride_length=0.04,  # 步长（米）
    step_height=0.025    # 步高（米）
)

# 椭圆轨迹
x, y, z = TrajectoryGenerator.ellipse_trajectory(phase, stride_length, step_height)

# 贝塞尔轨迹
x, y, z = TrajectoryGenerator.bezier_trajectory(phase, stride_length, step_height)
```

**返回值**：
- `x`：前后偏移（米）
- `y`：侧向偏移（米，基础步态为 0）
- `z`：上下偏移（米）

---

### 2. walk_gait.py - Walk 步态控制器

**功能**：控制四条腿的协调运动，支持差速转向

**API**：
```python
from gait_algo_core.walk_gait import WalkGait

# 创建步态控制器
gait = WalkGait(
    stride_length=0.05,  # 步长 5cm
    step_height=0.03,    # 步高 3cm
    frequency=0.8        # 频率 0.8Hz
)

# 更新步态（每帧调用）
gait.update(dt=0.02)  # 20ms 时间步

# 获取腿的相位
phase = gait.get_leg_phase('right_front')  # 0-1

# 获取足端轨迹偏移
x, z = gait.get_foot_trajectory('right_front')

# 获取所有腿的轨迹
trajectories = gait.get_all_foot_trajectories()
# 返回：{'right_front': (x, z), 'left_back': (x, z), ...}

# 设置转向（差速转向）
gait.set_steering(0.5)   # 左转
gait.set_steering(-0.5)  # 右转
gait.set_steering(0.0)   # 直行

# 设置转向强度
gait.set_steering_factor(0.3)  # 转向系数（0.1-0.5）

# 设置轨迹类型
gait.set_trajectory_type('cycloid')  # 'cycloid', 'ellipse', 'bezier'

# 获取状态
state = gait.get_state()
# 返回：{'global_phase': ..., 'stride_length': ..., 'steering_angle': ..., ...}

# 重置
gait.reset()
```

**差速转向原理**：
- **左转**：左侧腿步长减小，右侧腿步长增大
- **右转**：左侧腿步长增大，右侧腿步长减小
- 通过调整左右腿的步长差异实现转向

**腿名称**：
- `right_front` - 右前腿
- `left_back` - 左后腿
- `left_front` - 左前腿
- `right_back` - 右后腿

---

## 🧪 测试脚本（tests/）

### 1. test_walk_gait.py - Walk 步态单元测试

**功能**：综合测试步态控制器的所有功能

**运行**：
```bash
# 在 03_gait_control 目录下运行
python3 tests/test_walk_gait.py
```

**测试内容**：
- ✅ 轨迹生成器测试
- ✅ 占空比测试（25%/75%）
- ✅ 支撑腿数量测试（每时刻 3 条）
- ✅ 相位差测试（90°）
- ✅ 状态管理测试

**预期输出**：
```
🎉 所有测试通过！Walk步态修正成功！
```

---

### 2. test_steering.py - 差速转向测试

**功能**：测试差速转向功能

**运行**：
```bash
# 在 03_gait_control 目录下运行
python3 tests/test_steering.py
```

**测试内容**：
- ✅ 直行模式测试
- ✅ 左转模式测试
- ✅ 右转模式测试
- ✅ 转向系数测试

---

### 3. verify_duty_cycle.py - 占空比验证

**功能**：验证每条腿的占空比（摆动 25%，支撑 75%）

**运行**：
```bash
# 在 03_gait_control 目录下运行
python3 tests/verify_duty_cycle.py
```

**输出**：
- 占空比统计图（`duty_cycle_verification.png`）
- 控制台输出详细数据

**预期结果**：
```
✅ 右前腿: 摆动25帧 (25%), 支撑75帧 (75%)
✅ 左后腿: 摆动25帧 (25%), 支撑75帧 (75%)
✅ 左前腿: 摆动25帧 (25%), 支撑75帧 (75%)
✅ 右后腿: 摆动25帧 (25%), 支撑75帧 (75%)
✅ 3条腿支撑: 100帧 (100%)
```

---

### 4. verify_support_legs.py - 支撑腿验证

**功能**：验证每时刻支撑腿数量（恒定 3 条）

**运行**：
```bash
# 在 03_gait_control 目录下运行
python3 tests/verify_support_legs.py
```

**预期结果**：
```
全局相位 0.000:
  摆动相（抬腿）: 1条 - 右前腿
  支撑相（着地）: 3条 - 左后腿, 左前腿, 右后腿
  ✅ 符合Walk步态（3足支撑）

支撑腿数量分布：
  3条腿支撑: 100帧 (100.0%) ✅
```

---

### 5. verify_trajectory.py - 轨迹验证

**功能**：验证轨迹生成的正确性

**运行**：
```bash
# 在 03_gait_control 目录下运行
python3 tests/verify_trajectory.py
```

**输出**：
- 轨迹验证图（`trajectory_verification.png`）
- 3 个子图：X 偏移、Z 偏移、2D 轨迹

**验证内容**：
- ✅ X 范围对称（-2cm ~ +2cm）
- ✅ Z 范围正确（0 ~ 2.5cm）
- ✅ 轨迹连续性
- ✅ 起点终点一致性

---

### 6. test_chinese_font.py - 中文字体测试

**功能**：测试中文字体配置是否正确

**运行**：
```bash
# 在 03_gait_control 目录下运行
python3 tests/test_chinese_font.py
```

**输出**：
- 测试图片（`test_chinese_font.png`）

---

### 7. chinese_font_config.py - 字体配置工具

**功能**：提供中文字体配置工具函数

**API**：
```python
from chinese_font_config import setup_chinese_font

# 配置中文字体
font_prop = setup_chinese_font()

# 在图表中使用
plt.title("中文标题", fontproperties=font_prop)
```

---

## 🎨 可视化工具（tests/visual/）

### 1. visualize_walk_gait.py - 3D 步态动画

**功能**：生成高质量的 3D 步态动画（使用真实运动学）

**运行**：
```bash
# 在 03_gait_control 目录下运行
python3 tests/visual/visualize_walk_gait.py
```

**输出**：
- `walk_gait_animation.gif` - 3D 动画（100 帧，30fps）

**特点**：
- 使用 SpotLegKinematics 进行真实运动学计算
- 绘制完整的机器人形态（身体 + 关节连接）
- 3D 视图展示真实的腿部运动
- IK 成功率 100%

---

### 2. visualize_steering.py - 转向步态可视化

**功能**：生成转向轨迹的对比图和动画

**运行**：
```bash
# 在 03_gait_control 目录下运行
python3 tests/visual/visualize_steering.py
```

**输出**：
- `steering_comparison.png` - 转向对比图（3×3 子图）
- `steering_animation.gif` - 转向动画（左转 30°）

**对比内容**：
- **第 1 行**：直行（0°）
- **第 2 行**：左转（+30°）
- **第 3 行**：右转（-30°）

**子图布局**：
- **第 1 列**：XY 平面（俯视图）
- **第 2 列**：XZ 平面（侧视图）
- **第 3 列**：YZ 平面（正视图）

---

### 3. plot_gait_trajectory_v2.py - 步态轨迹图

**功能**：生成详细的步态轨迹分析图（2×2 子图）

**运行**：
```bash
# 在 03_gait_control 目录下运行
python3 tests/visual/plot_gait_trajectory_v2.py
```

**输出**：
- `gait_trajectory_v2.png` - 轨迹图（291KB）

**子图内容**：
- **左上**：X 轨迹（四条腿的 X 偏移随全局相位变化）
- **右上**：Z 轨迹（四条腿的 Z 偏移随全局相位变化）
- **左下**：2D 轨迹（四条腿的足端在 X-Z 平面的运动轨迹）
- **右下**：单腿详细轨迹（右前腿，颜色区分摆动相/支撑相）

**右下角图特点**：
- 不同颜色和形状的关键点标记
- 箭头显示运动方向
- 右上角完整图例
- 相位标注（0.00, 0.12, 0.25, 0.50, 0.75, 1.00）

---

### 4. plot_leg_phases.py - 腿部相位图

**功能**：生成腿部相位关系图

**运行**：
```bash
# 在 03_gait_control 目录下运行
python3 tests/visual/plot_leg_phases.py
```

**输出**：
- 腿部相位图

---

### 5. generate_walking_gif.py - 行走 GIF（高质量）

**功能**：生成高质量的行走动画 GIF

**运行**：
```bash
# 在 03_gait_control 目录下运行
python3 tests/visual/generate_walking_gif.py
```

**输出**：
- `walk_hq.gif` - 高质量 GIF（3.9MB）

**特点**：
- 双视图（俯视图 + Z 轴高度曲线）
- 轨迹线显示（最近 50 帧）
- 高帧率（30 fps）
- 高分辨率（100 DPI）
- 中文字体支持

---

### 6. generate_lightweight_gif.py - 行走 GIF（轻量版）

**功能**：生成轻量级的行走动画 GIF

**运行**：
```bash
# 在 03_gait_control 目录下运行
python3 tests/visual/generate_lightweight_gif.py
```

**输出**：
- `walk_straight.gif` - 轻量版 GIF（250KB）

**特点**：
- 单视图（仅俯视图）
- 小尺寸（8×6 英寸）
- 低帧率（12 fps）
- 少帧数（50 帧）
- 低 DPI（80）

---

## 🎯 Walk 步态标准

| 参数 | 数值 | 说明 |
|------|------|------|
| **占空比** | 25%/75% | 摆动 25%，支撑 75% |
| **相位差** | 90° | 四条腿依次抬腿 |
| **支撑腿** | 3 条 | 每时刻恒定 3 条支撑腿 |
| **稳定性** | 极高 | 三角形支撑面 |
| **IK 成功率** | 100% | 所有测试通过 |

---

## 📦 依赖

| 依赖 | 版本 | 说明 |
|------|------|------|
| Python | 3.10+ | 运行环境 |
| numpy | - | 数值计算 |
| matplotlib | 3.0+ | 可视化（3.2+ 最佳） |
| Git LFS | - | 字体文件管理 |

---

## 🔧 中文字体配置

### 字体文件

**路径**：`../fonts/BabelStoneHan.ttf`

**大小**：50MB（支持 CJK 统一汉字）

### 使用方法

```python
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
import os

# 获取字体路径
font_path = os.path.join(os.path.dirname(__file__), '..', 'fonts', 'BabelStoneHan.ttf')

# 显式添加字体到 matplotlib 缓存
fm.fontManager.addfont(font_path)

# 创建 FontProperties
font_prop = fm.FontProperties(fname=font_path)

# 设置全局字体
plt.rcParams['font.family'] = font_prop.get_name()
plt.rcParams['axes.unicode_minus'] = False

# 在图表中使用
plt.title("中文标题", fontproperties=font_prop)
plt.xlabel("X 轴", fontproperties=font_prop)
plt.legend(prop=font_prop)
```

---

## 📊 测试结果（2026-03-21）

### ✅ 所有核心测试通过

```
✅ 轨迹生成器测试通过
✅ 占空比测试通过（25%/75%）
✅ 支撑腿数量测试通过（每时刻 3 条）
✅ 相位差测试通过（90°）
✅ 差速转向测试通过
✅ 中文字体测试通过
✅ IK 成功率 100%
```

---

## 📝 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-03-19 | Walk 步态修正完成，占空比 25%/75% |
| v1.1 | 2026-03-21 | 添加差速转向功能，改进步态可视化 |

---

## 🔗 相关模块

- **01_leg_kinematics** - 腿部运动学
- **02_simulator_standalone** - 独立仿真器
- **04_spot_micro_gait_control** - 步态控制（旧版）

---

## 👤 作者

- 太子（OpenClaw Agent）

---

## 📄 许可证

继承自 SpotMicro 主项目许可证
