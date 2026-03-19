# app/gait/__init__.py
"""
步态控制模块

提供四足机器人的步态生成和控制功能。

模块：
- trajectory: 足端轨迹生成器（摆线、椭圆等）
- walk_gait: Walk 步态实现
- gait_controller: 步态控制器（集成到系统）
"""

from .trajectory import TrajectoryGenerator
from .walk_gait import WalkGait
from .gait_controller import GaitController

__all__ = ['TrajectoryGenerator', 'WalkGait', 'GaitController']
