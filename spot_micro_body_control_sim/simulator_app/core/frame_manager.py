# core/frame_manager.py
"""
坐标系管理模块

提供类似 ROS TF 的坐标系树管理功能，支持：
- 坐标系层级管理
- 任意坐标系之间的点变换
- 循环依赖检测
"""
from typing import Dict, Optional
import numpy as np
from core.transform import WorldTransform


class FrameNode:
    """
    坐标系节点

    表示坐标系树中的一个节点。

    属性：
        name: 坐标系名称
        parent: 父坐标系名称 (None 表示根坐标系)
        rel: 相对于父坐标系的变换
    """

    def __init__(self, name: str, parent: Optional[str], rel_transform: Optional[WorldTransform] = None):
        """
        初始化坐标系节点

        参数：
            name: 坐标系名称
            parent: 父坐标系名称，None 表示根坐标系
            rel_transform: 相对于父坐标系的变换，默认为单位变换
        """
        self.name = name
        self.parent = parent
        self.rel = rel_transform or WorldTransform()


class FrameManager:
    """
    坐标系管理器

    类似 ROS TF 的坐标系树管理器，每个坐标系存储到父坐标系的变换。
    支持任意两个坐标系之间的点变换。

    示例：
        fm = FrameManager()
        fm.set_frame("world", None, WorldTransform())
        fm.set_frame("body", "world", WorldTransform(0, 0, 0.2))
        p_world = fm.transform_point([0, 0, 0], "body", "world")
    """

    def __init__(self):
        """初始化空的坐标系管理器"""
        self.nodes: Dict[str, FrameNode] = {}

    def set_frame(self, name: str, parent: Optional[str], rel_transform: Optional[WorldTransform] = None):
        """
        添加或更新坐标系

        参数：
            name: 坐标系名称
            parent: 父坐标系名称，None 表示根坐标系
            rel_transform: 相对于父坐标系的变换
        """
        self.nodes[name] = FrameNode(name, parent, rel_transform)

    def _accumulate_to_root(self, frame: str):
        """
        累积从指定坐标系到根坐标系的变换矩阵

        参数：
            frame: 起始坐标系名称

        返回：
            list: 变换矩阵列表，从 frame 到 root

        异常：
            ValueError: 检测到循环依赖
            KeyError: 坐标系不存在
        """
        mats = []
        cur = frame
        visited = set()  # 记录已访问的坐标系，用于检测循环依赖

        while cur is not None:
            # 循环依赖检测
            if cur in visited:
                raise ValueError(f"检测到循环依赖: '{cur}' 在 '{frame}' 的链中出现两次")
            visited.add(cur)

            node = self.nodes.get(cur)
            if node is None:
                raise KeyError(f"坐标系 '{cur}' 不存在。可用坐标系: {list(self.nodes.keys())}")
            mats.append(node.rel.matrix())
            cur = node.parent

        return mats

    def get_transform_matrix(self, from_frame: str, to_frame: str):
        """
        获取从源坐标系到目标坐标系的变换矩阵

        参数：
            from_frame: 源坐标系名称
            to_frame: 目标坐标系名称

        返回：
            np.ndarray: 4x4 齐次变换矩阵
        """
        if from_frame == to_frame:
            return np.eye(4)

        # 累积到根坐标系的变换
        mats_from = self._accumulate_to_root(from_frame)
        mats_to = self._accumulate_to_root(to_frame)

        # 计算 from_frame 到世界坐标系的变换
        T_from_world = np.eye(4)
        for m in mats_from:
            T_from_world = m @ T_from_world

        # 计算 to_frame 到世界坐标系的变换
        T_to_world = np.eye(4)
        for m in mats_to:
            T_to_world = m @ T_to_world

        # 计算 from_frame 到 to_frame 的变换
        T_from_to = np.linalg.inv(T_to_world) @ T_from_world
        return T_from_to

    def transform_point(self, p, from_frame: str, to_frame: str):
        """
        将点从一个坐标系变换到另一个坐标系

        参数：
            p: 点坐标 [x, y, z]
            from_frame: 源坐标系名称
            to_frame: 目标坐标系名称

        返回：
            np.ndarray: 变换后的点坐标 [x, y, z]
        """
        T = self.get_transform_matrix(from_frame, to_frame)
        p4 = np.array([p[0], p[1], p[2], 1.0], dtype=float)
        res = T @ p4
        return res[:3]
