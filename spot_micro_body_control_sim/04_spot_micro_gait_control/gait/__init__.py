# gait/__init__.py
"""
步态控制模块

提供四足机器人的步态生成和控制功能。

模块：
- trajectory: 足端轨迹生成器（摆线、椭圆等）
- walk_gait: Walk 步态实现

注意：
- GaitController 已移至 03_spot_micro_simulator_framework/app/gait/
"""

from .trajectory import TrajectoryGenerator
from .walk_gait import WalkGait

__all__ = ['TrajectoryGenerator', 'WalkGait']
