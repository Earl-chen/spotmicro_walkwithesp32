# core/types.py
"""
核心数据类型定义

定义机器人系统中使用的基本数据结构：
- Pose: 位姿（位置 + 姿态）
- LegJoints: 腿部关节角度
- IKResult: 逆运动学结果
"""
from dataclasses import dataclass
from typing import Optional


@dataclass
class Pose:
    """
    位姿数据类

    表示一个物体在三维空间中的位置和姿态。

    属性：
        x: X 轴位置 (米)
        y: Y 轴位置 (米)
        z: Z 轴位置 (米)
        roll: 横滚角 (弧度) - 绕 X 轴旋转
        pitch: 俯仰角 (弧度) - 绕 Y 轴旋转
        yaw: 偏航角 (弧度) - 绕 Z 轴旋转
    """
    x: float
    y: float
    z: float
    roll: float   # 弧度
    pitch: float  # 弧度
    yaw: float    # 弧度


@dataclass
class LegJoints:
    """
    腿部关节角度数据类

    表示单条腿的三个关节角度。

    属性：
        hip_side: 髋侧摆角度 (弧度) - 控制腿部左右摆动
        hip_pitch: 髋俯仰角度 (弧度) - 控制大腿前后摆动
        knee_pitch: 膝俯仰角度 (弧度) - 控制小腿弯曲
    """
    hip_side: float    # 弧度
    hip_pitch: float   # 弧度
    knee_pitch: float  # 弧度


@dataclass
class IKResult:
    """
    逆运动学计算结果

    表示逆运动学求解的结果。

    属性：
        success: 是否求解成功
        joints: 计算出的关节角度 (成功时)
        reason: 失败原因 (失败时)
    """
    success: bool
    joints: Optional[LegJoints] = None
    reason: Optional[str] = None
