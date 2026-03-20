# 四足机器人双模式控制系统 - API完整参考

## 目录

- [核心层 (core/)](#核心层-core)
- [机器人层 (robots/)](#机器人层-robots)
- [应用层 (app/)](#应用层-app)
- [主程序 (run_spot_micro.py)](#主程序-run_spot_micro)

---

## 核心层 (core/)

### types.py - 数据类型定义

#### Pose
```python
@dataclass
class Pose:
    x: float        # X 位置 (米)
    y: float        # Y 位置 (米)
    z: float        # Z 位置 (米)
    roll: float     # 横滚角 (弧度)
    pitch: float    # 俯仰角 (弧度)
    yaw: float      # 偏航角 (弧度)
```

#### LegJoints
```python
@dataclass
class LegJoints:
    hip_side: float    # 髋侧摆角度 (弧度)
    hip_pitch: float   # 髋俯仰角度 (弧度)
    knee_pitch: float  # 膝俯仰角度 (弧度)
```

#### IKResult
```python
@dataclass
class IKResult:
    success: bool                    # IK 是否成功
    joints: Optional[LegJoints]      # 计算出的关节角度
    reason: Optional[str]            # 失败原因
```

---

### transform.py - 坐标变换类

#### WorldTransform

**初始化**
```python
WorldTransform(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0)
```

**方法**

| 方法 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `set_pose_euler(x, y, z, roll, pitch, yaw, radians=True)` | 位置和姿态 | None | 设置位姿 |
| `get_pose_euler(radians=True)` | - | (位置, roll, pitch, yaw) | 获取位姿 |
| `matrix()` | - | np.ndarray (4x4) | 获取变换矩阵 |
| `inverse_matrix()` | - | np.ndarray (4x4) | 获取逆变换矩阵 |
| `transform_point_body_to_world(p)` | 点坐标 (3,) | np.ndarray (3,) | 点变换到世界坐标系 |
| `transform_vector_body_to_world(v)` | 向量 (3,) | np.ndarray (3,) | 向量变换到世界坐标系 |
| `transform_point_world_to_body(p)` | 点坐标 (3,) | np.ndarray (3,) | 点变换到机体坐标系 |
| `set_on_change(cb)` | 回调函数 | None | 设置变化回调 |

**使用示例**
```python
# 创建变换
wt = WorldTransform(0.1, 0.2, 0.3, 0.1, 0.2, 0.3)  # 弧度

# 设置位姿（度数）
wt.set_pose_euler(1.0, 2.0, 3.0, 10.0, 20.0, 30.0, radians=False)

# 点坐标变换
p_world = wt.transform_point_body_to_world([0, 0, -0.24])

# 获取变换矩阵
T = wt.matrix()
```

---

### frame_manager.py - 坐标系管理器

#### FrameManager

**方法**

| 方法 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `set_frame(name, parent, rel_transform)` | 坐标系名、父坐标系、变换 | None | 添加坐标系 |
| `get_transform_matrix(from_frame, to_frame)` | 源坐标系、目标坐标系 | np.ndarray (4x4) | 获取变换矩阵 |
| `transform_point(p, from_frame, to_frame)` | 点、源坐标系、目标坐标系 | np.ndarray (3,) | 点坐标变换 |

**异常**

| 异常类型 | 触发条件 |
|---------|---------|
| `KeyError` | 指定的坐标系不存在 |
| `ValueError` | 检测到循环坐标系依赖 |

**使用示例**
```python
fm = FrameManager()

# 创建坐标系层次
fm.set_frame("world", None, WorldTransform(0, 0, 0, 0, 0, 0))
fm.set_frame("body", "world", WorldTransform(0, 0, 0.1, 0, 0, 0))
fm.set_frame("hip_left", "body", WorldTransform(0.1, 0.05, 0, 0, 0, 0))

# 点坐标变换
p_world = fm.transform_point([0, 0, -0.24], "hip_left", "world")
```

---

### kinematics/ik_base.py - 运动学求解器基类

#### IKSolverBase

**抽象方法**

| 方法 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `forward(hip_pos_body, joints)` | 髋位置、关节角度 | List[np.ndarray] | 正运动学 |
| `inverse(foot_pos_in_hip_frame)` | 脚部位置 | IKResult | 逆运动学 |

**使用示例**
```python
class MyLegKinematics(IKSolverBase):
    def forward(self, hip_pos_body, joints):
        # 实现正运动学
        return [hip_pos_body, ..., foot_pos]

    def inverse(self, foot_pos_in_hip_frame):
        # 实现逆运动学
        return IKResult(True, LegJoints(...), None)
```

---

### utils/rotations.py - 旋转矩阵工具

**函数**

| 函数 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `rot_x(roll)` | 横滚角 (弧度) | np.ndarray (3x3) | 绕 X 轴旋转矩阵 |
| `rot_y(pitch)` | 俯仰角 (弧度) | np.ndarray (3x3) | 绕 Y 轴旋转矩阵 |
| `rot_z(yaw)` | 偏航角 (弧度) | np.ndarray (3x3) | 绕 Z 轴旋转矩阵 |
| `euler_to_matrix(roll, pitch, yaw)` | 欧拉角 (弧度) | np.ndarray (3x3) | 欧拉角转旋转矩阵 |
| `matrix_to_euler(R)` | 旋转矩阵 (3x3) | Tuple[float, float, float] | 旋转矩阵转欧拉角 |

**旋转约定**
```
R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
```

**使用示例**
```python
# 创建旋转矩阵
R = euler_to_matrix(0.1, 0.2, 0.3)

# 提取欧拉角
roll, pitch, yaw = matrix_to_euler(R)
```

---

## 机器人层 (robots/)

### spot_micro/geometry.py - 几何参数

**常量**

| 常量 | 值 | 说明 |
|------|-----|------|
| `L1` | 0.0605 m | 髋侧摆延伸段长度 |
| `L2` | 0.010 m | 髋俯仰段长度 |
| `L3` | 0.111126 m | 大腿长度 |
| `L4` | 0.1185 m | 小腿长度 |
| `BODY_LENGTH` | 0.2075 m | 机体长度 |
| `BODY_WIDTH` | 0.078 m | 机体宽度 |

**HIP_OFFSETS** - 髋关节位置
```python
HIP_OFFSETS = {
    "left_front":  (BODY_LENGTH/2,  BODY_WIDTH/2, 0.0),
    "left_back":   (-BODY_LENGTH/2, BODY_WIDTH/2, 0.0),
    "right_front": (BODY_LENGTH/2, -BODY_WIDTH/2, 0.0),
    "right_back":  (-BODY_LENGTH/2, -BODY_WIDTH/2, 0.0),
}
```

---

### spot_micro/leg_kinematics.py - 腿部运动学求解器

#### SpotLegKinematics

**初始化**
```python
SpotLegKinematics(l1=L1, l2=L2, l3=L3, l4=L4, is_left=True)
```

**方法**

| 方法 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `forward(hip_pos_body, joints)` | 髋位置、关节角度 | List[np.ndarray] | 正运动学 |
| `inverse(foot_pos_in_hip_frame)` | 脚部位置 | IKResult | 逆运动学 |

**正运动学返回值**
```python
[hip_pos, extension_pos, hip_pitch_pos, knee_pos, foot_pos]
```

**逆运动学约束**
- 最小距离: `H >= l1` (脚部到髋关节距离)
- 最小距离: `F > 0` (脚部到髋俯仰关节距离，即 `G > l2`)
- 最大距离: `S <= l3 + l4` (脚部到膝关节距离不超过腿长)
- 角度范围:
  - `hip_side ∈ [-π/2, π/2]`
  - `hip_pitch ∈ [-π/2, π/2]`
  - `knee_pitch ∈ [-π, 0]`

**IK 失败原因**
| 失败原因 | 说明 |
|---------|------|
| `Target too close to hip` | H < l1，脚部太靠近髋关节 |
| `Target too close to hip pitch joint` | F <= 0，脚部太靠近髋俯仰关节 |
| `Target too far` | S > l3+l4，脚部超出工作空间 |
| `Math error` | 数值计算错误 |

**使用示例**
```python
# 创建求解器
legkin = SpotLegKinematics(is_left=True)

# 正运动学
joints = LegJoints(0.0, -0.5, -1.0)
positions = legkin.forward(hip_pos, joints)
foot_pos = positions[-1]

# 逆运动学
ik_result = legkin.inverse(foot_pos_in_hip)
if ik_result.success:
    joints = ik_result.joints
```

---

## 应用层 (app/)

### robot_model.py - 机器人模型

#### RobotModel

**初始化**
```python
RobotModel(frame_manager: FrameManager, body_frame: str = "body")
```

**方法**

| 方法 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `add_leg(leg_name, leg_kin, hip_frame_name, initial_joints)` | 腿名、求解器、髋坐标系、初始角度 | None | 添加腿 |
| `update_joint_angles(leg_name, joints)` | 腿名、关节角度 | None | 更新关节角度 |
| `get_leg_joints_body(leg_name)` | 腿名 | List[np.ndarray] | 获取机体坐标系关节位置 |
| `get_leg_joints_world(leg_name)` | 腿名 | List[np.ndarray] | 获取世界坐标系关节位置 |
| `get_body_outline_world()` | - | List[np.ndarray] | 获取机体轮廓 |

**使用示例**
```python
# 创建模型
model = RobotModel(fm, body_frame="body")

# 添加腿
legkin = SpotLegKinematics(is_left=True)
model.add_leg("left_front", legkin, "hip_left_front", LegJoints(0, 0, 0))

# 更新关节角度
model.update_joint_angles("left_front", LegJoints(0.1, -0.5, -1.0))

# 获取脚部位置
foot_pos = model.get_leg_joints_world("left_front")[-1]
```

---

### controller.py - 控制器

#### Controller

**初始化**
```python
Controller(model: RobotModel)
```

**方法**

| 方法 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `set_body_pose(x, y, z, roll, pitch, yaw, radians=True)` | 机体位姿 | None | 设置机体位姿（原子操作） |
| `set_joint_angles(leg_name, hip_side, hip_pitch, knee_pitch, radians=True)` | 腿名、关节角度 | None | 设置单腿关节角度 |
| `set_all_joint_angles(joint_angles_dict, radians=True)` | 关节角度字典 | None | 设置所有关节角度 |
| `get_body_pose()` | - | Dict | 获取机体位姿 |
| `get_foot_positions_world()` | - | Dict | 获取所有脚部世界坐标 |
| `get_all_joint_angles(radians=True)` | - | Dict | 获取所有关节角度 |
| `capture_fixed_feet()` | - | None | 捕获脚步位置（启用锁定） |
| `enable_forward_kinematics_mode()` | - | None | 切换到正向运动学模式 |
| `enable_inverse_kinematics_mode()` | - | None | 切换到逆运动学模式 |
| `is_inverse_kinematics_mode()` | - | bool | 检查当前模式 |

**set_body_pose() 原子操作说明**

当启用脚步锁定模式时，`set_body_pose()` 使用原子操作确保状态一致性：
1. 先保存当前机体位姿
2. 临时设置新的机体位姿
3. 计算所有腿的逆运动学
4. 如果**所有**IK 都成功，则更新关节角度
5. 如果**任何**IK 失败，则**回滚**机体位姿到原状态

这确保了机体位姿和关节角度始终保持一致，不会出现部分更新的情况。

**set_joint_angles() 行为说明**

⚠️ **重要**: 调用 `set_joint_angles()` 或 `set_all_joint_angles()` 会**自动禁用脚步锁定模式**。

这是因为在正向运动学模式下，用户直接控制关节角度，脚步位置不再固定。如果需要重新启用脚步锁定，请调用 `enable_inverse_kinematics_mode()`。

```python
# 当前处于逆运动学模式（脚步锁定）
controller.set_joint_angles('left_front', 10, -20, -45, radians=False)
# 此时脚步锁定已自动禁用！

# 如需重新启用脚步锁定
controller.enable_inverse_kinematics_mode()
```

**get_body_pose() 返回格式**
```python
{
    'position': [x, y, z],              # 米
    'orientation_rad': [roll, pitch, yaw],  # 弧度
    'orientation_deg': [roll, pitch, yaw]   # 度数
}
```

**使用示例**
```python
# 创建控制器
controller = Controller(model)

# 逆运动学模式
controller.enable_inverse_kinematics_mode()
controller.set_body_pose(0.1, 0.0, -0.1, 5.0, 0.0, 10.0, radians=False)

# 正向运动学模式
controller.enable_forward_kinematics_mode()
controller.set_joint_angles('left_front', 10, -20, -45, radians=False)

# 批量设置关节角度
joint_angles = {
    'left_front': [10, -20, -45],
    'left_back': [-5, 15, -30],
}
controller.set_all_joint_angles(joint_angles, radians=False)

# 获取状态
pose = controller.get_body_pose()
foot_positions = controller.get_foot_positions_world()
joint_angles = controller.get_all_joint_angles(radians=False)
```

---

### ui/ui_matplotlib.py - Matplotlib UI

#### MatplotlibUI

**初始化**
```python
MatplotlibUI(controller: Controller, model: RobotModel)
```

**方法**

| 方法 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `start()` | - | None | 启动界面 |

**控制模式**
- `"inverse"` - 逆运动学模式（脚步锁定）
- `"forward"` - 正向运动学模式（关节控制）

**使用示例**
```python
# 创建并启动 UI
ui = MatplotlibUI(controller, model)
ui.start()
```

---

## 主程序 (run_spot_micro.py)

### run_spot_micro.py - 主程序入口

**函数**

| 函数 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `build_system()` | - | (model, controller) | 构建系统 |
| `run()` | - | None | 运行主程序 |

**坐标系层次**
```
world (根)
  └── body (机体)
       ├── hip_left_front  (左前髋)
       ├── hip_left_back   (左后髋)
       ├── hip_right_front (右前髋)
       └── hip_right_back  (右后髋)
```

**使用示例**
```python
from run_spot_micro import build_system

# 构建系统
model, controller = build_system()

# 控制机器人
controller.set_body_pose(0, 0, -0.1, 0, 0, 0)
```

---

## 常用代码片段

### 完整的控制流程

```python
# 1. 导入模块
from run_spot_micro import build_system

# 2. 构建系统
model, controller = build_system()

# 3. 逆运动学模式控制
controller.enable_inverse_kinematics_mode()
controller.set_body_pose(0.1, 0.0, -0.1, 5.0, 0.0, 10.0, radians=False)

# 4. 获取状态
pose = controller.get_body_pose()
print(f"机体位置: {pose['position']}")
print(f"机体姿态: {pose['orientation_deg']}")

# 5. 切换到正向运动学模式
controller.enable_forward_kinematics_mode()
controller.set_joint_angles('left_front', 10, -20, -45, radians=False)

# 6. 获取关节角度
angles = controller.get_all_joint_angles(radians=False)
print(f"左前腿角度: {angles['left_front']}")
```

### 自定义机器人

```python
from core.frame_manager import FrameManager
from core.transform import WorldTransform
from core.types import LegJoints
from app.robot_model import RobotModel
from app.controller import Controller
from robots.spot_micro.leg_kinematics import SpotLegKinematics
from robots.spot_micro import geometry

# 1. 创建坐标系
fm = FrameManager()
fm.set_frame("world", None, WorldTransform(0, 0, 0, 0, 0, 0))
fm.set_frame("body", "world", WorldTransform(0, 0, 0, 0, 0, 0))

# 2. 创建模型
model = RobotModel(fm, body_frame="body")

# 3. 添加腿（自定义参数）
legkin = SpotLegKinematics(l1=0.070, l2=0.015, l3=0.120, l4=0.135)
model.add_leg("left_front", legkin, "hip_left_front", LegJoints(0, 0, 0))

# 4. 创建控制器
controller = Controller(model)
```
