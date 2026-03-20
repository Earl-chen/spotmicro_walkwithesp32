# app/gait/walk_gait.py
"""
Walk 步态实现

Walk 步态特点：
- 三足支撑，一足抬起
- 相位偏移：90°（依次抬腿）
- 占空比：0.75（75% 时间着地）
- 稳定性最高，适合入门和复杂地形
"""

import numpy as np
from typing import Dict, Tuple
from .trajectory import TrajectoryGenerator


class WalkGait:
    """
    Walk 步态控制器
    
    实现 Walk 步态的相位调度和轨迹生成。
    
    相位安排：
    - right_front: 0.0
    - left_back: 0.25
    - left_front: 0.5
    - right_back: 0.75
    
    使用示例：
        >>> gait = WalkGait(stride_length=0.05, step_height=0.03, frequency=0.8)
        >>> 
        >>> # 在主循环中
        >>> gait.update(dt=0.02)
        >>> 
        >>> # 获取所有腿的轨迹
        >>> trajectories = gait.get_all_foot_trajectories()
        >>> for leg_name, (x, z) in trajectories.items():
        ...     print(f"{leg_name}: x={x:.3f}, z={z:.3f}")
    """
    
    def __init__(self, 
                 stride_length: float = 0.05,
                 step_height: float = 0.03,
                 frequency: float = 0.8):
        """
        初始化 Walk 步态控制器
        
        参数：
            stride_length: 步长 (米)，默认 0.05 (50mm)
            step_height: 抬腿高度 (米)，默认 0.03 (30mm)
            frequency: 步频 (Hz)，默认 0.8Hz
        
        推荐参数范围：
            - 步长：40-60 mm (0.04-0.06 m)
            - 步高：20-40 mm (0.02-0.04 m)
            - 步频：0.5-1.0 Hz（适合舵机）
        """
        self.stride_length = stride_length
        self.step_height = step_height
        self.frequency = frequency
        
        # Walk 步态相位偏移（依次抬腿）
        # 相位差 90° 确保三足支撑
        self.phase_offsets = {
            'right_front': 0.0,    # 右前腿
            'left_back': 0.25,     # 左后腿
            'left_front': 0.5,     # 左前腿
            'right_back': 0.75     # 右后腿
        }
        
        # 全局相位（0-1循环）
        self.global_phase = 0.0
        
        # 轨迹类型（'cycloid', 'ellipse', 'bezier'）
        self.trajectory_type = 'cycloid'
        
    def update(self, dt: float):
        """
        更新全局相位
        
        参数：
            dt: 时间步长 (秒)
        
        示例：
            >>> gait = WalkGait(frequency=0.8)
            >>> gait.update(0.02)  # 20ms 时间步
        """
        self.global_phase = (self.global_phase + self.frequency * dt) % 1.0
    
    def get_leg_phase(self, leg_name: str) -> float:
        """
        获取指定腿的当前相位
        
        参数：
            leg_name: 腿名称 ('right_front', 'left_back', 'left_front', 'right_back')
        
        返回：
            phase: 相位 (0-1)
        
        异常：
            ValueError: 未知的腿名称
        
        示例：
            >>> gait = WalkGait()
            >>> phase = gait.get_leg_phase('right_front')
        """
        if leg_name not in self.phase_offsets:
            raise ValueError(f"Unknown leg: {leg_name}. Valid legs: {list(self.phase_offsets.keys())}")
        
        leg_phase = (self.global_phase + self.phase_offsets[leg_name]) % 1.0
        return leg_phase
    
    def get_foot_trajectory(self, leg_name: str) -> Tuple[float, float]:
        """
        获取指定腿的足端轨迹偏移
        
        参数：
            leg_name: 腿名称
        
        返回：
            (x, z): 前后位置偏移, 上下位置偏移（米）
        
        示例：
            >>> x, z = gait.get_foot_trajectory('right_front')
            >>> # x: 前后偏移，z: 上下偏移
        """
        leg_phase = self.get_leg_phase(leg_name)
        
        # 根据轨迹类型生成轨迹
        if self.trajectory_type == 'cycloid':
            x, y, z = TrajectoryGenerator.cycloid_trajectory(
                leg_phase, 
                self.stride_length, 
                self.step_height
            )
        elif self.trajectory_type == 'ellipse':
            x, y, z = TrajectoryGenerator.ellipse_trajectory(
                leg_phase, 
                self.stride_length, 
                self.step_height
            )
        elif self.trajectory_type == 'bezier':
            x, y, z = TrajectoryGenerator.bezier_trajectory(
                leg_phase, 
                self.stride_length, 
                self.step_height
            )
        else:
            raise ValueError(f"Unknown trajectory type: {self.trajectory_type}")
        
        # 注意：y 轴偏移用于转向功能，基础步态中为0
        # 返回 (x, z) 保持向后兼容
        return x, z
    
    def get_all_foot_trajectories(self) -> Dict[str, Tuple[float, float]]:
        """
        获取所有腿的足端轨迹偏移
        
        返回：
            dict: {'leg_name': (x, z), ...}
        
        示例：
            >>> trajectories = gait.get_all_foot_trajectories()
            >>> for leg_name, (x, z) in trajectories.items():
            ...     print(f"{leg_name}: x={x*1000:.1f}mm, z={z*1000:.1f}mm")
        """
        trajectories = {}
        for leg_name in self.phase_offsets.keys():
            trajectories[leg_name] = self.get_foot_trajectory(leg_name)
        return trajectories
    
    def set_parameters(self, 
                       stride_length: float = None,
                       step_height: float = None,
                       frequency: float = None):
        """
        设置步态参数
        
        参数：
            stride_length: 步长 (米)
            step_height: 抬腿高度 (米)
            frequency: 步频 (Hz)
        
        示例：
            >>> gait.set_parameters(stride_length=0.06, frequency=1.0)
        """
        if stride_length is not None:
            self.stride_length = stride_length
        if step_height is not None:
            self.step_height = step_height
        if frequency is not None:
            self.frequency = frequency
    
    def set_trajectory_type(self, trajectory_type: str):
        """
        设置轨迹类型
        
        参数：
            trajectory_type: 'cycloid', 'ellipse', 'bezier'
        
        示例：
            >>> gait.set_trajectory_type('ellipse')
        """
        if trajectory_type not in ['cycloid', 'ellipse', 'bezier']:
            raise ValueError(f"Invalid trajectory type: {trajectory_type}")
        self.trajectory_type = trajectory_type
    
    def get_state(self) -> Dict:
        """
        获取步态状态信息
        
        返回：
            dict: 步态状态信息
        
        示例：
            >>> state = gait.get_state()
            >>> print(f"全局相位: {state['global_phase']:.2f}")
        """
        return {
            'global_phase': self.global_phase,
            'stride_length': self.stride_length,
            'step_height': self.step_height,
            'frequency': self.frequency,
            'trajectory_type': self.trajectory_type,
            'duty_cycle': 0.75  # Walk 步态占空比
        }
    
    def reset(self):
        """重置步态相位"""
        self.global_phase = 0.0


# 单元测试
if __name__ == '__main__':
    print("测试 Walk 步态...")
    
    # 创建 Walk 步态实例
    gait = WalkGait(stride_length=0.05, step_height=0.03, frequency=0.8)
    
    # 测试相位偏移
    print("\n相位偏移测试:")
    gait.global_phase = 0.0
    for leg_name in gait.phase_offsets.keys():
        phase = gait.get_leg_phase(leg_name)
        print(f"  {leg_name}: {phase:.2f}")
    
    # 测试轨迹生成
    print("\n轨迹生成测试 (一个完整周期):")
    dt = 0.02  # 20ms
    for i in range(10):
        gait.update(dt)
        trajectories = gait.get_all_foot_trajectories()
        print(f"\n步骤 {i+1}, 全局相位: {gait.global_phase:.2f}")
        for leg_name, (x, z) in trajectories.items():
            print(f"  {leg_name}: x={x*1000:.1f}mm, z={z*1000:.1f}mm")
    
    print("\n✅ Walk 步态测试完成")
