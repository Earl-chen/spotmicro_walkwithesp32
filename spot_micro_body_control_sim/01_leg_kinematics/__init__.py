#!/usr/bin/env python3
"""
Spot Micro 腿部运动学实验室
==========================

单腿运动学分析工具包，包含：
- 核心计算（IK/FK/批量计算/工作空间分析）
- 应用封装（验证、错误处理、控制器）
- 3D 可视化（交互式界面）

使用方法：
    cd 01_spot_micro_leg_kinematics_lab
    python run_lab.py

    # 或快速演示模式
    python run_lab.py --demo
"""

from .kinematics_core import LegKinematics, QuadrupedKinematics
from .kinematics_app import LegController, QuadrupedController, create_leg_controller
from .kinematics_visualizer import create_visualizer

__all__ = [
    'LegKinematics',
    'QuadrupedKinematics',
    'LegController',
    'QuadrupedController',
    'create_leg_controller',
    'create_visualizer',
]

__version__ = '2.0.0'
