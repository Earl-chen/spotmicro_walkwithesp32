# 四足机器人双模式控制系统

[![Python 3.6+](https://img.shields.io/badge/python-3.6+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

一个完全模块化的四足机器人控制系统，支持**逆运动学**和**正向运动学**双模式控制，具备世界坐标系6DOF控制和脚步锁定功能。

## 🎯 核心特性

### 🔄 双模式控制
- **逆运动学模式**: 脚步世界坐标固定，通过机体位姿控制
- **正向运动学模式**: 直接控制12个关节角度，实现各种机器人动作
- **一键切换**: 界面内无缝切换控制模式

### 🏗️ 模块化架构
- **core/**: 核心通用库（坐标变换、坐标系管理）
- **robots/**: 机器人特定实现（几何参数、运动学）
- **app/**: 应用层（控制器、界面、模型）

### 🎮 交互式界面
- **3D实时可视化**: 世界坐标轴（红）+ 机体坐标轴（蓝）
- **智能滑块控制**: 根据模式自动显示相应控制元素
- **实时状态监控**: 左右分栏显示机体位姿、关节角度、脚部坐标
- **中文界面支持**: 完美的中文字体显示

## 📖 背景知识

### 什么是逆运动学？
**逆运动学**（Inverse Kinematics, IK）是指根据机器人末端执行器（脚）的**目标位置**，计算机器人各关节应有的**角度**。

**本系统的逆运动学模式特点**：
- 你拖动机体位姿滑块（X/Y/Z位置 + Roll/Pitch/Yaw姿态）
- 系统自动计算保持脚步固定所需的关节角度
- 类似于：你想让机器人身体移动，但脚不能滑动，系统自动计算各腿怎么调整

### 什么是正向运动学？
**正向运动学**（Forward Kinematics, FK）是指根据机器人各关节的**角度**，计算机器人末端执行器的**位置**。

**本系统的正向运动学模式特点**：
- 你直接拖动12个关节滑块（髋侧摆、髋俯仰、膝俯仰）
- 系统计算脚部的新位置
- 类似于：你手动控制每个关节，机器人动作完全由你决定

### 两种模式的区别
| 特性 | 逆运动学模式 | 正向运动学模式 |
|------|-------------|---------------|
| 控制对象 | 机体位姿（6个自由度） | 12个关节角度 |
| 脚步位置 | 世界坐标固定 | 随关节变化 |
| 使用场景 | 行走、姿态调整 | 特定动作演示 |

## 🚀 快速开始

### 安装依赖
```bash
pip install numpy matplotlib
```

### 运行主程序
```bash
cd 03_spot_micro_simulator_framework
python3 run_spot_micro.py
```

### 运行环境要求
- **Python**: 3.6+
- **必需库**: numpy, matplotlib
- **操作系统**: Linux, macOS, Windows（需支持图形界面）
- **内存**: 建议512MB以上
- **图形界面**: 需要支持matplotlib的图形显示环境
  - **本地运行**: 确保系统有图形桌面环境
  - **远程服务器**: 需配置 X11 转发或使用 VNC
    ```bash
    # X11 转发示例
    ssh -X user@server
    # 或使用 VNC/noVNC 等工具
    ```

### 控制说明
1. **默认逆运动学模式**: 拖动机体位姿滑块（位置+姿态），脚步世界坐标自动锁定
2. **切换正向运动学**: 点击左下角"切换到正向运动学"按钮
3. **关节控制模式**: 拖动12个关节滑块直接控制机器人姿态
4. **返回逆运动学**: 再次点击按钮即可切换回来

## 📚 详细文档

### 文档阅读顺序建议

1. **本文档（README.md）** - 了解项目概况和快速上手
2. **[01-架构详细分析](docs/01-架构详细分析.md)** - 理解三层架构和设计模式
3. **[02-API完整参考](docs/02-API完整参考.md)** - 查阅具体的类和方法
4. **[03-算法实现细节](docs/03-算法实现细节.md)** - 深入了解数学原理（高级）

### 技术文档链接

- **[01-架构详细分析](docs/01-架构详细分析.md)** - 三层架构、设计模式、数据流向
- **[02-API完整参考](docs/02-API完整参考.md)** - 所有类和方法的完整API文档
- **[03-算法实现细节](docs/03-算法实现细节.md)** - 坐标变换、运动学算法的数学原理

## 📁 项目架构

```
03_spot_micro_simulator_framework/
├── core/                       # 🔧 核心通用库
│   ├── config.py              # 全局配置参数
│   ├── types.py               # 数据类型定义（LegJoints, IKResult等）
│   ├── transform.py           # 坐标变换（WorldTransform类）
│   ├── frame_manager.py       # 坐标系管理（FrameManager类）
│   ├── utils/
│   │   └── rotations.py       # 旋转矩阵工具函数
│   └── kinematics/
│       └── ik_base.py         # 运动学求解器抽象基类
├── robots/                     # 🤖 机器人特定实现
│   └── spot_micro/
│       ├── geometry.py        # SpotMicro几何参数（L1,L2,L3,L4,髋关节偏移）
│       └── leg_kinematics.py  # 腿部运动学求解器（正向+逆向）
├── app/                        # 🎮 应用层
│   ├── robot_model.py         # 机器人模型（集成坐标系+运动学）
│   ├── controller.py          # 控制器（双模式控制逻辑）
│   └── ui/
│       └── ui_matplotlib.py   # matplotlib交互界面
├── run_spot_micro.py          # 🎯 主程序入口
├── module_tests/               # 🧪 模块测试
│   ├── test_transform.py      # 坐标变换测试
│   ├── test_frame_manager.py  # 坐标系管理测试
│   └── test_kinematics_roundtrip.py # 运动学往返精度测试
├── docs/                       # 📖 详细技术文档
│   ├── 01-架构详细分析.md     # 三层架构、设计模式
│   ├── 02-API完整参考.md      # 完整API文档
│   └── 03-算法实现细节.md     # 数学原理和算法
└── README.md                   # 📖 本文档
```

## 🎮 详细使用说明

### ⚡ 30秒上手

1. **安装并运行**
   ```bash
   pip install numpy matplotlib
   cd 03_spot_micro_simulator_framework
   python3 run_spot_micro.py
   ```

2. **开始控制**
   - 默认是逆运动学模式（脚步固定）→ 拖动机体位姿滑块
   - 点击左下角按钮 → 切换到正向运动学模式
   - 正向模式 → 拖动12个关节滑块直接控制

3. **观察效果**
   - 红线 = 世界坐标轴（固定）
   - 蓝线 = 机体坐标轴（跟随机器人）
   - 右下角状态栏显示实时数据

---

### 逆运动学模式（脚步锁定）
```python
from run_spot_micro import build_system

# 构建系统
model, controller = build_system()

# 启用逆运动学模式
controller.enable_inverse_kinematics_mode()

# 改变机体位姿（脚部世界坐标自动保持不变）
controller.set_body_pose(0.1, 0.0, 0.05, 10.0, 0.0, 15.0, radians=False)

# 获取状态
pose = controller.get_body_pose()
foot_positions = controller.get_foot_positions_world()
```

### 正向运动学模式（关节控制）
```python
# 启用正向运动学模式
controller.enable_forward_kinematics_mode()

# 设置所有关节角度（度）
joint_angles = {
    'left_front': [10, -20, -45],   # 髋侧摆, 髋俯仰, 膝俯仰
    'left_back': [-5, 15, -30],
    'right_front': [0, -10, -60],
    'right_back': [5, 20, -40]
}
controller.set_all_joint_angles(joint_angles, radians=False)

# 获取当前关节角度
current_angles = controller.get_all_joint_angles(radians=False)
```

### 单腿控制
```python
# 设置单条腿的关节角度
controller.set_joint_angles('left_front', 10, -20, -45, radians=False)

# 检查当前模式
if controller.is_inverse_kinematics_mode():
    print("当前为逆运动学模式")
else:
    print("当前为正向运动学模式")
```

## 🧪 测试验证

### 运行所有测试
```bash
# 模块测试
python3 module_tests/test_transform.py          # 坐标变换测试
python3 module_tests/test_frame_manager.py      # 坐标系管理测试
python3 module_tests/test_kinematics_roundtrip.py # 运动学精度测试
```

### 测试结果
- ✅ **坐标变换精度**: 往返计算误差 < 1e-10
- ✅ **运动学精度**: FK-IK往返误差 < 1e-6米
- ✅ **脚步锁定**: 位姿变化时脚部位移 < 0.001米

## 🔧 技术特点

### 坐标系架构
```
世界坐标系 (world)
    └── 机体坐标系 (body)
        ├── 左前髋坐标系 (hip_left_front)
        ├── 左后髋坐标系 (hip_left_back)  
        ├── 右前髋坐标系 (hip_right_front)
        └── 右后髋坐标系 (hip_right_back)
```

### 运动学求解
- **正向运动学**: `joints → foot_position`
- **逆向运动学**: `foot_position → joints`
- **脚步锁定**: `body_pose_change → auto_joint_update`

### 数据流架构
```
用户输入 → Controller → RobotModel → SpotLegKinematics → 关节更新
    ↓
UI显示 ← 状态查询 ← FrameManager ← 坐标变换 ← WorldTransform
```

## 🎨 界面特性

### 双模式界面
- **逆运动学模式**: 显示机体位姿滑块（6个）+ 脚步锁定状态
- **正向运动学模式**: 显示关节控制滑块（12个）+ 关节角度状态
- **智能切换**: 自动显示/隐藏相应控制元素和文字

### 状态监控
- **左栏**: 控制模式、机体位姿、关节角度
- **右栏**: 脚部世界坐标、功能说明、坐标轴说明
- **实时更新**: 所有数据随控制同步更新

### 可视化效果
- **红色线条**: 世界坐标轴（固定参考系）
- **蓝色线条**: 机体坐标轴（跟随机器人运动）
- **彩色关节**: 不同颜色表示不同关节类型
- **实时追踪**: 3D视图自动调整显示范围

## 🔧 配置参数

### 机器人几何参数（robots/spot_micro/geometry.py）
```python
L1 = 0.0605   # 髋关节延伸段长度 (60.5mm)
L2 = 0.010    # 髋关节垂直段长度 (10mm)
L3 = 0.111126 # 大腿长度 (111.126mm)
L4 = 0.1185   # 小腿长度 (118.5mm)

# 机体尺寸
BODY_LENGTH = 0.2075  # 207.5mm
BODY_WIDTH = 0.078    # 78mm

# 髋关节偏移（基于机体尺寸计算）
HIP_OFFSETS = {
    'left_front':  ( BODY_LENGTH/2,  BODY_WIDTH/2, 0.0),
    'left_back':   (-BODY_LENGTH/2,  BODY_WIDTH/2, 0.0),
    'right_front': ( BODY_LENGTH/2, -BODY_WIDTH/2, 0.0),
    'right_back':  (-BODY_LENGTH/2, -BODY_WIDTH/2, 0.0)
}
```

### 控制范围（app/ui/ui_matplotlib.py）
```python
# 机体位置控制范围
X, Y: ±500mm
Z: -300mm ~ +100mm

# 机体姿态控制范围  
Roll, Pitch: ±45°
Yaw: ±180°

# 关节角度控制范围
髋侧摆, 髋俯仰: ±90°
膝俯仰: -180° ~ 0°
```

## 🔍 调试模式

代码包含详细调试输出，运行时会显示：
- 坐标系变化日志（`[Controller]` 前缀）
- 逆运动学计算过程
- 关节角度更新信息
- UI初始化状态（`[UI]` 前缀）

**查看调试输出**：
```bash
# 查看完整调试信息
python3 run_spot_micro.py

# 重定向输出到日志文件
python3 run_spot_micro.py 2>&1 | tee debug.log
```

**静默运行**（仅显示界面，不输出日志）：
```bash
python3 run_spot_micro.py > /dev/null 2>&1
```

---

## 🐛 故障排除

### 常见问题

**1. ImportError: No module named 'xxx'**
```bash
# 确保在 03_spot_micro_simulator_framework 目录下运行
cd 03_spot_micro_simulator_framework
python3 run_spot_micro.py
```

**2. matplotlib后端问题**
- 程序会自动尝试TkAgg、Qt5Agg、Agg后端
- 确保系统支持图形界面显示

**3. 字体显示问题**
- 已完全修复，不再使用emoji和特殊字符
- 支持WenQuanYi字体和通用字体

**4. 切换模式时的错误**
- 已修复TypeError问题
- 界面元素正确显示/隐藏

**5. 远程服务器显示问题**
- 确保已配置 X11 转发：`ssh -X user@server`
- 或使用 VNC/noVNC 等远程桌面工具

## 📊 性能指标

### 计算性能
- **运动学求解**: < 0.1ms/腿
- **坐标变换**: < 0.01ms/次
- **界面刷新**: 30+ FPS
- **模式切换**: < 100ms

### 精度指标
- **坐标变换精度**: 1e-10米
- **运动学精度**: 1e-6米  
- **脚步锁定精度**: < 0.001米
- **角度控制精度**: 0.1°

## 🔄 与原代码对比

### 架构改进
| 原代码 | 重构后 | 改进 |
|--------|--------|------|
| 1010行单文件 | 15个模块 | ✅ 模块化 |
| 硬编码参数 | 配置文件 | ✅ 可配置 |
| 无类型注解 | 完整类型 | ✅ 类型安全 |
| 无单元测试 | 完整测试 | ✅ 可测试 |

### 功能扩展
| 功能 | 原代码 | 重构后 | 状态 |
|------|--------|--------|------|
| 逆运动学控制 | ✅ | ✅ | 保持兼容 |
| 正向运动学控制 | ❌ | ✅ | 新增功能 |
| 模式切换 | ❌ | ✅ | 新增功能 |
| 关节直接控制 | ❌ | ✅ | 新增功能 |
| 双栏状态显示 | ❌ | ✅ | 界面优化 |

### 兼容性保证
- ✅ **核心算法**: 与原代码完全一致
- ✅ **功能行为**: 脚步锁定效果相同
- ✅ **初始状态**: 位置差异为0
- ✅ **控制效果**: 机体控制响应一致

## 🚀 扩展指南

### 添加新机器人类型
1. 在`robots/`下创建新目录
2. 实现`geometry.py`（几何参数）
3. 实现`leg_kinematics.py`（继承`IKSolverBase`）
4. 在主程序中切换机器人类型

### 添加新控制模式
1. 在`Controller`中添加新模式方法
2. 在`MatplotlibUI`中添加对应界面元素
3. 实现模式切换逻辑
4. 更新状态显示

### 添加新的运动学求解器
1. 继承`IKSolverBase`抽象基类
2. 实现`forward()`和`inverse()`方法
3. 在机器人配置中指定求解器
4. 编写单元测试验证精度

## 📄 许可证

MIT License - 详见 [LICENSE](LICENSE) 文件

## 🤝 贡献

欢迎提交Issue和Pull Request！

### 开发流程
1. Fork项目
2. 创建特性分支 (`git checkout -b feature/amazing-feature`)
3. 提交修改 (`git commit -m 'Add amazing feature'`)
4. 推送分支 (`git push origin feature/amazing-feature`)
5. 提交Pull Request

### 代码规范
- 遵循PEP 8编码规范
- 添加类型注解和文档字符串
- 编写相应的单元测试
- 保持向后兼容性

## 📞 联系方式

如有问题或建议，请通过以下方式联系：
- 创建GitHub Issue
- 发送邮件至维护者

---

**🎉 完整的模块化四足机器人双模式控制系统，功能强大，易于扩展！**