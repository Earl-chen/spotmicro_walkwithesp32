# app/robot_model.py
"""
机器人模型模块

管理四足机器人的状态和运动学计算，包括：
- 四条腿的运动学求解器
- 关节角度状态
- 坐标变换缓存
"""
import numpy as np
from typing import Dict, List, Tuple
from core.frame_manager import FrameManager
from core.types import LegJoints
from core.kinematics.ik_base import IKSolverBase


class RobotModel:
    """
    四足机器人模型

    管理机器人的状态和运动学计算。

    属性：
        frame_manager: 坐标系管理器
        body_frame: 机体坐标系名称
        legs: 腿部运动学求解器字典
        joints: 腿部关节角度字典
    """

    def __init__(self, frame_manager: FrameManager, body_frame: str = "body"):
        """
        初始化机器人模型

        参数：
            frame_manager: 坐标系管理器实例
            body_frame: 机体坐标系名称，默认 "body"
        """
        self.frame_manager = frame_manager
        self.body_frame = body_frame
        self.legs: Dict[str, Tuple[IKSolverBase, str]] = {}
        self.joints: Dict[str, LegJoints] = {}
        self._cache_body_joints: Dict[str, List[np.ndarray]] = {}
        self._cache_world_joints: Dict[str, List[np.ndarray]] = {}

        # 设置缓存失效回调
        self._setup_frame_callbacks()

    def _setup_frame_callbacks(self):
        """设置坐标系变换时的缓存失效回调"""
        for frame_name, node in self.frame_manager.nodes.items():
            if node.rel:
                node.rel.set_on_change(self.invalidate_cache)

    def add_leg(self, leg_name: str, leg_kin: IKSolverBase, hip_frame_name: str, initial_joints: LegJoints = None):
        """
        添加一条腿

        参数：
            leg_name: 腿名称 (如 "left_front")
            leg_kin: 运动学求解器实例
            hip_frame_name: 髋坐标系名称
            initial_joints: 初始关节角度，默认为零位
        """
        self.legs[leg_name] = (leg_kin, hip_frame_name)
        self.joints[leg_name] = initial_joints or LegJoints(0.0, 0.0, 0.0)

        # 为髋坐标系设置缓存失效回调（如果尚未设置）
        if hip_frame_name in self.frame_manager.nodes:
            node = self.frame_manager.nodes[hip_frame_name]
            if node.rel and node.rel._on_change is None:
                node.rel.set_on_change(self.invalidate_cache)

        self.invalidate_cache(leg_name)

    def update_joint_angles(self, leg_name: str, joints: LegJoints):
        """
        更新指定腿的关节角度

        参数：
            leg_name: 腿名称
            joints: 新的关节角度
        """
        self.joints[leg_name] = joints
        self.invalidate_cache(leg_name)

    def invalidate_cache(self, leg_name: str = None):
        """
        使缓存失效

        参数：
            leg_name: 指定腿名称，None 表示使所有缓存失效
        """
        if leg_name is None:
            self._cache_body_joints.clear()
            self._cache_world_joints.clear()
        else:
            self._cache_body_joints.pop(leg_name, None)
            self._cache_world_joints.pop(leg_name, None)

    def get_leg_joints_body(self, leg_name: str) -> List[np.ndarray]:
        """
        获取腿部各关节在机体坐标系中的位置

        参数：
            leg_name: 腿名称

        返回：
            list: 各关节坐标列表 [hip, extension, hip_pitch, knee, foot]
        """
        if leg_name in self._cache_body_joints:
            return self._cache_body_joints[leg_name]

        legkin, hip_frame = self.legs[leg_name]
        hip_pos_body = self.frame_manager.transform_point([0, 0, 0], hip_frame, self.body_frame)
        body_joints = legkin.forward(hip_pos_body, self.joints[leg_name])
        self._cache_body_joints[leg_name] = body_joints

        return body_joints

    def get_leg_joints_world(self, leg_name: str) -> List[np.ndarray]:
        """
        获取腿部各关节在世界坐标系中的位置

        参数：
            leg_name: 腿名称

        返回：
            list: 各关节坐标列表 [hip, extension, hip_pitch, knee, foot]
        """
        if leg_name in self._cache_world_joints:
            return self._cache_world_joints[leg_name]

        body_joints = self.get_leg_joints_body(leg_name)
        world_joints = [self.frame_manager.transform_point(p, self.body_frame, "world") for p in body_joints]
        self._cache_world_joints[leg_name] = world_joints

        return world_joints

    def get_body_outline_world(self) -> List[np.ndarray]:
        """
        获取机体轮廓在世界坐标系中的位置

        返回：
            list: 机体框架顶点坐标列表
        """
        from robots.spot_micro.geometry import BODY_LENGTH, BODY_WIDTH

        x_half = BODY_LENGTH / 2
        y_half = BODY_WIDTH / 2

        # 机体框架坐标（机体坐标系）
        body_points_body = [
            [x_half,  y_half, 0],   # 左前
            [x_half, -y_half, 0],   # 右前
            [-x_half, -y_half, 0],  # 右后
            [-x_half,  y_half, 0],  # 左后
            [x_half,  y_half, 0]    # 闭合
        ]

        # 转换到世界坐标系
        return [self.frame_manager.transform_point(p, self.body_frame, "world") for p in body_points_body]
