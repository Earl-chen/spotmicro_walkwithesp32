# core/kinematics/ik_base.py
"""
运动学求解器抽象基类

定义运动学求解器的标准接口，支持：
- 正运动学 (FK): 关节角度 → 末端位置
- 逆运动学 (IK): 末端位置 → 关节角度
"""
from abc import ABC, abstractmethod
from typing import List
import numpy as np
from core.types import LegJoints, IKResult


class IKSolverBase(ABC):
    """
    运动学求解器抽象基类

    所有腿部运动学求解器必须继承此类并实现以下方法：
    - forward(): 正运动学
    - inverse(): 逆运动学
    """

    @abstractmethod
    def forward(self, hip_pos_body: np.ndarray, joints: LegJoints) -> List[np.ndarray]:
        """
        正运动学计算

        根据髋关节位置和关节角度，计算腿部各关节的 3D 坐标。

        参数：
            hip_pos_body: 髋关节在机体坐标系中的位置 (米)
            joints: 关节角度 (弧度)

        返回：
            list: 各关节坐标列表 [hip, extension, hip_pitch, knee, foot]
        """
        raise NotImplementedError

    @abstractmethod
    def inverse(self, foot_pos_in_hip_frame: np.ndarray) -> IKResult:
        """
        逆运动学计算

        根据脚部在髋坐标系中的位置，计算所需的关节角度。

        参数：
            foot_pos_in_hip_frame: 脚部在髋坐标系中的位置 (米)

        返回：
            IKResult: 包含成功状态、关节角度和失败原因
        """
        raise NotImplementedError
