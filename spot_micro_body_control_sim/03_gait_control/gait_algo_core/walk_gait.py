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
        
        # 转向角度（差速转向）
        # 正值 = 左转，负值 = 右转，0 = 直行
        self.steering_angle = 0.0
        
        # 转向系数（控制转向强度）
        self.steering_factor = 0.3
        
        # 速度控制参数
        # forward_speed: 前进速度 (m/s)，正值前进，负值后退，0停止
        self.forward_speed = 0.0
        # lateral_speed: 侧向速度 (m/s)，正值左移，负值右移，0不侧移
        self.lateral_speed = 0.0
        # yaw_rate: 偏航角速度 (rad/s)，正值左转，负值右转，0不转
        self.yaw_rate = 0.0
        
        # 速度限制
        self.max_forward_speed = 0.1  # 最大前进速度 (m/s)
        self.max_lateral_speed = 0.05  # 最大侧向速度 (m/s)
        self.max_yaw_rate = 1.0  # 最大偏航角速度 (rad/s)
        
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
        获取指定腿的足端轨迹偏移（支持差速转向）
        
        参数：
            leg_name: 腿名称
        
        返回：
            (x, z): 前后位置偏移, 上下位置偏移（米）
        
        示例：
            >>> x, z = gait.get_foot_trajectory('right_front')
            >>> # x: 前后偏移，z: 上下偏移
        """
        leg_phase = self.get_leg_phase(leg_name)
        
        # 根据速度控制调整步长
        # forward_speed 影响步长的大小和方向
        effective_stride = self.stride_length
        
        # 如果有前进速度，调整有效步长
        if self.forward_speed != 0:
            # 速度越快，步长越大（线性映射）
            speed_ratio = abs(self.forward_speed) / self.max_forward_speed
            speed_ratio = min(1.0, speed_ratio)  # 限制在 0-1 之间
            effective_stride = self.stride_length * speed_ratio
            
            # 负速度 = 后退，反转步长方向
            if self.forward_speed < 0:
                effective_stride = -effective_stride
        
        # 根据轨迹类型生成轨迹
        if self.trajectory_type == 'cycloid':
            x, y, z = TrajectoryGenerator.cycloid_trajectory(
                leg_phase, 
                abs(effective_stride),  # 使用绝对值生成轨迹
                self.step_height
            )
        elif self.trajectory_type == 'ellipse':
            x, y, z = TrajectoryGenerator.ellipse_trajectory(
                leg_phase, 
                abs(effective_stride),
                self.step_height
            )
        elif self.trajectory_type == 'bezier':
            x, y, z = TrajectoryGenerator.bezier_trajectory(
                leg_phase, 
                abs(effective_stride),
                self.step_height
            )
        else:
            raise ValueError(f"Unknown trajectory type: {self.trajectory_type}")
        
        # 如果后退，反转 x 轴
        if effective_stride < 0:
            x = -x
        
        # 差速转向：调整左右腿的步长
        if self.steering_angle != 0:
            if leg_name in ['left_front', 'left_back']:
                # 左侧腿：左转时步长减小，右转时步长增大
                x *= (1 - self.steering_angle * self.steering_factor)
            else:  # 'right_front', 'right_back'
                # 右侧腿：左转时步长增大，右转时步长减小
                x *= (1 + self.steering_angle * self.steering_factor)
        
        # 侧向速度：影响 y 轴偏移（简化实现，通过相位偏移实现）
        # 注意：y 轴偏移用于侧向步态，基础步态中为0
        # 这里可以扩展为侧向移动功能
        
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
    
    def set_steering(self, steering_angle: float):
        """
        设置转向角度（差速转向）
        
        参数：
            steering_angle: 转向角度（弧度）
                           正值 = 左转
                           负值 = 右转
                           0 = 直行
        
        推荐范围：
            -1.0 ~ +1.0 弧度（约 -60° ~ +60°）
        
        示例：
            >>> gait.set_steering(0.5)   # 左转
            >>> gait.set_steering(-0.5)  # 右转
            >>> gait.set_steering(0.0)   # 直行
        """
        # 限制转向角度范围
        steering_angle = max(-1.0, min(1.0, steering_angle))
        self.steering_angle = steering_angle
    
    def set_steering_factor(self, factor: float):
        """
        设置转向系数（控制转向强度）
        
        参数：
            factor: 转向系数（0.1 ~ 0.5）
                   越大转向越明显
        
        示例：
            >>> gait.set_steering_factor(0.3)  # 中等转向强度
        """
        self.steering_factor = max(0.1, min(0.5, factor))
    
    def set_velocity(self, forward: float = 0.0, lateral: float = 0.0, yaw_rate: float = 0.0):
        """
        设置速度控制参数（统一的速度控制接口）
        
        参数：
            forward: 前进速度 (m/s)
                    正值 = 前进
                    负值 = 后退
                    0 = 停止
            lateral: 侧向速度 (m/s)
                    正值 = 左移
                    负值 = 右移
                    0 = 不侧移
            yaw_rate: 偏航角速度 (rad/s)
                     正值 = 左转
                     负值 = 右转
                     0 = 直行
        
        推荐范围：
            - forward: -0.1 ~ +0.1 m/s
            - lateral: -0.05 ~ +0.05 m/s
            - yaw_rate: -1.0 ~ +1.0 rad/s
        
        示例：
            >>> gait.set_velocity(forward=0.05)  # 前进 5cm/s
            >>> gait.set_velocity(forward=-0.05)  # 后退 5cm/s
            >>> gait.set_velocity(forward=0.05, yaw_rate=0.3)  # 前进+左转
            >>> gait.set_velocity(0, 0, 0)  # 停止
        """
        # 限制速度范围
        self.forward_speed = max(-self.max_forward_speed, min(self.max_forward_speed, forward))
        self.lateral_speed = max(-self.max_lateral_speed, min(self.max_lateral_speed, lateral))
        self.yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_rate))
        
        # yaw_rate 映射到 steering_angle（转向）
        # 线性映射：yaw_rate/max_yaw_rate -> steering_angle
        if yaw_rate != 0:
            self.steering_angle = yaw_rate / self.max_yaw_rate
        else:
            self.steering_angle = 0.0
    
    def set_direction(self, forward: float):
        """
        设置运动方向（简化的前进/后退控制）
        
        参数：
            forward: 方向控制
                    1.0 = 全速前进
                    -1.0 = 全速后退
                    0.0 = 停止
                    0.5 = 半速前进
                    -0.5 = 半速后退
        
        示例：
            >>> gait.set_direction(1.0)   # 全速前进
            >>> gait.set_direction(-1.0)  # 全速后退
            >>> gait.set_direction(0.0)   # 停止
            >>> gait.set_direction(0.5)   # 半速前进
        """
        # 限制在 -1.0 ~ 1.0 之间
        forward = max(-1.0, min(1.0, forward))
        
        # 映射到 forward_speed
        self.forward_speed = forward * self.max_forward_speed
        
        # 重置其他速度参数
        self.lateral_speed = 0.0
        self.yaw_rate = 0.0
        self.steering_angle = 0.0
    
    def get_velocity(self) -> Dict[str, float]:
        """
        获取当前速度参数
        
        返回：
            dict: {
                'forward': 前进速度 (m/s),
                'lateral': 侧向速度 (m/s),
                'yaw_rate': 偏航角速度 (rad/s)
            }
        
        示例：
            >>> vel = gait.get_velocity()
            >>> print(f"前进速度: {vel['forward']:.3f} m/s")
        """
        return {
            'forward': self.forward_speed,
            'lateral': self.lateral_speed,
            'yaw_rate': self.yaw_rate
        }
    
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
            'duty_cycle': 0.75,  # Walk 步态占空比
            'steering_angle': self.steering_angle,
            'steering_factor': self.steering_factor,
            'forward_speed': self.forward_speed,
            'lateral_speed': self.lateral_speed,
            'yaw_rate': self.yaw_rate,
            'max_forward_speed': self.max_forward_speed,
            'max_lateral_speed': self.max_lateral_speed,
            'max_yaw_rate': self.max_yaw_rate
        }
    
    def cmd_vel(self, linear_x: float = 0.0, linear_y: float = 0.0, angular_z: float = 0.0):
        """
        ROS 风格的速度控制接口
        
        参数：
            linear_x: 前进速度 (m/s)
                     正值 = 前进
                     负值 = 后退
                     0 = 停止
            linear_y: 侧向速度 (m/s)
                     正值 = 左移
                     负值 = 右移
                     0 = 不侧移
            angular_z: 偏航角速度 (rad/s)
                      正值 = 左转
                      负值 = 右转
                      0 = 直行
        
        说明：
            这是 ROS 风格的 cmd_vel 接口封装，底层调用 set_velocity 方法。
            参数命名遵循 ROS 标准：
            - linear_x: 线速度 X 分量（前进方向）
            - linear_y: 线速度 Y 分量（侧向）
            - angular_z: 角速度 Z 分量（偏航）
        
        示例：
            >>> gait.cmd_vel(linear_x=0.05)  # 前进 5cm/s
            >>> gait.cmd_vel(linear_x=-0.05)  # 后退 5cm/s
            >>> gait.cmd_vel(linear_x=0.05, angular_z=0.3)  # 前进+左转
            >>> gait.cmd_vel(0, 0, 0)  # 停止
        """
        self.set_velocity(forward=linear_x, lateral=linear_y, yaw_rate=angular_z)
    
    def zero_radius_turn(self, yaw_rate: float):
        """
        零半径转向（原地转向）
        
        参数：
            yaw_rate: 偏航角速度 (rad/s)
                     正值 = 左转
                     负值 = 右转
                     0 = 停止
        
        说明：
            零半径转向是一种特殊的转向方式，机器人原地旋转而不产生线性位移。
            通过设置 forward=0 和 lateral=0，只有 yaw_rate 来实现。
            适合在狭窄空间或需要精确调整朝向的场景。
        
        示例：
            >>> gait.zero_radius_turn(0.5)   # 原地左转
            >>> gait.zero_radius_turn(-0.5)  # 原地右转
            >>> gait.zero_radius_turn(0.0)   # 停止
        """
        # 零半径转向：只有角速度，无前进和侧向速度
        self.set_velocity(forward=0.0, lateral=0.0, yaw_rate=yaw_rate)
    
    def cmd_vel_adaptive(self, linear_x: float = 0.0, linear_y: float = 0.0, angular_z: float = 0.0):
        """
        自适应速度控制接口（根据角速度和前进速度自动选择转向方案）
        
        参数：
            linear_x: 前进速度 (m/s)
                     正值 = 前进
                     负值 = 后退
                     0 = 停止
            linear_y: 侧向速度 (m/s)
                     正值 = 左移
                     负值 = 右移
                     0 = 不侧移
            angular_z: 偏航角速度 (rad/s)
                      正值 = 左转
                      负值 = 右转
                      0 = 直行
        
        说明：
            自适应转向策略：
            - 当 angular_z > 0.5 且 linear_x < 0.02 时，使用零半径转向
              （大角度转向且几乎没有前进速度 → 原地转向更高效）
            - 其他情况使用差速转向
              （小角度转向或有前进速度 → 差速转向更稳定）
        
        示例：
            >>> # 差速转向（前进+小角度转向）
            >>> gait.cmd_vel_adaptive(linear_x=0.05, angular_z=0.3)
            >>> 
            >>> # 零半径转向（大角度转向+几乎无前进）
            >>> gait.cmd_vel_adaptive(linear_x=0.01, angular_z=0.8)
            >>> 
            >>> # 停止
            >>> gait.cmd_vel_adaptive(0, 0, 0)
        """
        # 自适应转向逻辑
        use_zero_radius = (abs(angular_z) > 0.5) and (abs(linear_x) < 0.02)
        
        if use_zero_radius:
            # 零半径转向：只有角速度
            self.zero_radius_turn(yaw_rate=angular_z)
        else:
            # 差速转向：正常调用 cmd_vel
            self.cmd_vel(linear_x=linear_x, linear_y=linear_y, angular_z=angular_z)
    
    def reset(self):
        """重置步态相位、转向角度和速度参数"""
        self.global_phase = 0.0
        self.steering_angle = 0.0
        self.forward_speed = 0.0
        self.lateral_speed = 0.0
        self.yaw_rate = 0.0


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
