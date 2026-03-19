# gait/__init__.py
"""
步态控制模块

提供四足机器人的步态生成和控制功能。

模块：
- trajectory: 足端轨迹生成器（摆线、椭圆等）
- walk_gait: Walk 步态实现
- gait_controller: 步态控制器（集成到系统，在03模块中）
"""

from .trajectory import TrajectoryGenerator
from .walk_gait import WalkGait
# GaitController在03_spot_micro_simulator_framework中，因为它依赖仿真框架

__all__ = ['TrajectoryGenerator', 'WalkGait']
