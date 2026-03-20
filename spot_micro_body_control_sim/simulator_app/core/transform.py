# core/transform.py
"""
坐标变换模块

提供机体坐标系与世界坐标系之间的变换功能。
"""
import numpy as np
import math
from typing import Callable, Optional, Sequence
from core.utils.rotations import euler_to_matrix


class WorldTransform:
    """
    世界坐标系变换类

    存储和管理位姿 (x, y, z, roll, pitch, yaw)，默认使用弧度和米。
    旋转顺序：R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

    属性：
        _pos: 位置 [x, y, z] (米)
        _roll: 横滚角 (弧度)
        _pitch: 俯仰角 (弧度)
        _yaw: 偏航角 (弧度)
    """

    def __init__(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        """
        初始化变换

        参数：
            x, y, z: 位置 (米)
            roll, pitch, yaw: 姿态角 (弧度)
        """
        self._pos = np.array([x, y, z], dtype=float)
        self._roll = float(roll)
        self._pitch = float(pitch)
        self._yaw = float(yaw)
        self._T = None
        self._on_change: Optional[Callable[[], None]] = None

    def set_on_change(self, cb: Callable[[], None]):
        """
        设置变换改变时的回调函数

        参数：
            cb: 无参数回调函数
        """
        self._on_change = cb

    def set_pose_euler(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float, *, radians=True):
        """
        使用欧拉角设置位姿

        参数：
            x, y, z: 位置 (米)
            roll, pitch, yaw: 姿态角
            radians: True 表示弧度，False 表示度
        """
        if not radians:
            roll = math.radians(roll)
            pitch = math.radians(pitch)
            yaw = math.radians(yaw)
        self._pos[:] = [x, y, z]
        self._roll, self._pitch, self._yaw = float(roll), float(pitch), float(yaw)
        self._invalidate()

    def get_pose_euler(self, *, radians=True):
        """
        获取当前位姿

        参数：
            radians: True 返回弧度，False 返回度

        返回：
            tuple: (位置数组, roll, pitch, yaw)
        """
        if radians:
            return (self._pos.copy(), self._roll, self._pitch, self._yaw)
        else:
            return (self._pos.copy(), math.degrees(self._roll), math.degrees(self._pitch), math.degrees(self._yaw))

    def _invalidate(self):
        """使缓存失效，触发回调"""
        self._T = None
        if self._on_change:
            try:
                self._on_change()
            except Exception as e:
                import warnings
                warnings.warn(f"变换回调失败: {e}", RuntimeWarning, stacklevel=2)

    def _build_matrix(self):
        """构建 4x4 齐次变换矩阵"""
        if self._T is not None:
            return
        R = euler_to_matrix(self._roll, self._pitch, self._yaw)
        T = np.eye(4, dtype=float)
        T[:3, :3] = R
        T[:3, 3] = self._pos
        self._T = T

    def matrix(self):
        """
        获取 4x4 齐次变换矩阵

        返回：
            np.ndarray: 4x4 变换矩阵
        """
        self._build_matrix()
        return self._T.copy()

    def inverse_matrix(self):
        """
        获取逆变换矩阵

        返回：
            np.ndarray: 4x4 逆变换矩阵
        """
        T = self.matrix()
        R = T[:3, :3]
        t = T[:3, 3]
        inv = np.eye(4, dtype=float)
        inv[:3, :3] = R.T
        inv[:3, 3] = -R.T @ t
        return inv

    def transform_point_body_to_world(self, p: Sequence[float]):
        """
        将点从机体坐标系变换到世界坐标系

        参数：
            p: 机体坐标系中的点 [x, y, z]

        返回：
            np.ndarray: 世界坐标系中的点
        """
        p4 = np.array([p[0], p[1], p[2], 1.0], dtype=float)
        w = self.matrix() @ p4
        return w[:3]

    def transform_vector_body_to_world(self, v: Sequence[float]):
        """
        将向量从机体坐标系变换到世界坐标系

        参数：
            v: 机体坐标系中的向量 [x, y, z]

        返回：
            np.ndarray: 世界坐标系中的向量
        """
        R = self.matrix()[:3, :3]
        return R @ np.array(v, dtype=float)

    def transform_point_world_to_body(self, p_world: Sequence[float]):
        """
        将点从世界坐标系变换到机体坐标系

        参数：
            p_world: 世界坐标系中的点 [x, y, z]

        返回：
            np.ndarray: 机体坐标系中的点
        """
        p4 = np.array([p_world[0], p_world[1], p_world[2], 1.0], dtype=float)
        b = self.inverse_matrix() @ p4
        return b[:3]
