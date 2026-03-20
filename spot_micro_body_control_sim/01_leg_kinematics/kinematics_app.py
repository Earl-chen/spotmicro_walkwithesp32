#!/usr/bin/env python3
"""
四足机器人应用层模块
==================

本模块是对核心运动学模块 (kinematics_core) 的应用层封装，提供：
- 工作空间约束验证
- 关节角度范围检查
- 错误处理和日志输出
- 单位转换（弧度 ↔ 度）

设计模式
--------
采用"包装器模式"(Wrapper Pattern)，在核心计算层之上添加应用逻辑：
- 核心层 (kinematics_core): 纯数学计算，输入输出均为弧度
- 应用层 (本模块): 添加验证、日志、单位转换，输入输出均为度

主要类
------
- LegController: 单腿控制器，提供 IK/FK 的应用层接口
- QuadrupedController: 四足机器人控制器，管理四条腿的协调运动

使用示例
--------
>>> from kinematics_app import create_leg_controller
>>> leg = create_leg_controller('spot_micro')
>>> angles = leg.solve_position(50, 60.5, -180, 'left')  # 返回度数
>>> position = leg.calculate_position(*angles, 'left')   # 输入度数

依赖
----
- kinematics_core: 核心运动学计算模块
- numpy: 数值计算
- math: 数学函数
"""

import math
import numpy as np
from typing import Tuple, Optional, Dict, List
from kinematics_core import LegKinematics, QuadrupedKinematics


# ============================================================================
# 单腿控制器
# ============================================================================

class LegController:
    """
    单腿控制器 - 应用层封装

    职责：
    - 工作空间验证：检查目标位置是否在可达范围内
    - 角度范围检查：确保关节角度不超出机械限制
    - 错误处理：捕获核心层异常，返回友好结果
    - 单位转换：核心层用弧度，本层用度
    - 日志输出：可选的详细日志信息

    属性
    ----
    kinematics : LegKinematics
        核心运动学计算器
    min_reach : float
        最小可达距离 (mm)
    max_reach : float
        最大可达距离 (mm)
    angle_limits : dict
        各关节的角度限制 (度)
    verbose : bool
        是否输出详细日志
    """

    def __init__(self, l1=60.5, l2=10.0, l3=111.126, l4=118.5,
                 workspace_limits=(50.0, 295.0), verbose=True):
        """
        初始化腿部控制器

        参数
        ----
        l1 : float
            髋关节延伸段长度 (mm)，默认 60.5
        l2 : float
            髋关节垂直段长度 (mm)，默认 10.0
        l3 : float
            大腿长度 (mm)，默认 111.126
        l4 : float
            小腿长度 (mm)，默认 118.5
        workspace_limits : tuple
            (min_reach, max_reach) 工作空间范围 (mm)
        verbose : bool
            是否输出详细日志，默认 True
        """
        # 创建核心运动学计算器
        self.kinematics = LegKinematics(l1, l2, l3, l4)

        # 工作空间限制
        self.min_reach, self.max_reach = workspace_limits

        # 日志开关
        self.verbose = verbose

        # 关节角度限制 (度)
        # 这些限制基于舵机的物理转动范围
        self.angle_limits = {
            'hip_side': (-90, 90),      # 髋侧摆：左右摆动范围
            'hip_pitch': (-90, 90),     # 髋俯仰：前后摆动范围
            'knee_pitch': (-180, 0)     # 膝俯仰：只能向后弯曲
        }

        # 输出初始化信息
        if self.verbose:
            print(f"腿部控制器初始化完成")
            print(f"几何参数: l1={l1}, l2={l2}, l3={l3}, l4={l4}")
            print(f"工作范围: {self.min_reach}-{self.max_reach}mm")

    # ------------------------------------------------------------------------
    # 主要接口方法
    # ------------------------------------------------------------------------

    def solve_position(self, x: float, y: float, z: float, leg_side: str = 'left') -> Optional[Tuple[float, float, float]]:
        """
        逆运动学求解 - 从脚部位置计算关节角度

        这是应用层的主要接口，封装了：
        1. 工作空间验证
        2. 核心层 IK 计算
        3. 角度范围检查
        4. 单位转换（弧度→度）

        参数
        ----
        x : float
            脚部 X 坐标 (mm)，前进方向为正
        y : float
            脚部 Y 坐标 (mm)，左侧为正（左腿）或右侧为负（右腿）
        z : float
            脚部 Z 坐标 (mm)，向下为负
        leg_side : str
            腿侧别，'left' 或 'right'，默认 'left'

        返回
        ----
        tuple or None
            成功: (hip_side_deg, hip_pitch_deg, knee_pitch_deg) 角度（度）
            失败: None（目标位置不可达或角度超限）

        示例
        ----
        >>> angles = leg.solve_position(50, 60.5, -180, 'left')
        >>> if angles:
        ...     print(f"髋侧摆={angles[0]}°, 髋俯仰={angles[1]}°, 膝俯仰={angles[2]}°")
        """
        # 确定腿的左右侧
        is_left = (leg_side == 'left')

        # 输出日志
        if self.verbose:
            print(f"\n=== 位置解算 ===")
            print(f"目标: ({x:.1f}, {y:.1f}, {z:.1f}) - {leg_side}腿")

        # 步骤1: 工作空间检查
        # 确保目标位置在腿的可达范围内
        if not self._check_workspace(x, y, z):
            return None

        try:
            # 步骤2: 调用核心层逆运动学计算
            # 核心层返回弧度值
            angles_rad = self.kinematics.inverse_kinematics(x, y, z, is_left)

            # 步骤3: 单位转换 - 弧度转度
            angles_deg = [math.degrees(angle) for angle in angles_rad]

            # 步骤4: 角度范围检查
            # 确保计算出的角度在舵机允许范围内
            if not self._check_angle_limits(angles_deg):
                return None

            # 输出成功日志
            if self.verbose:
                print(f"解算成功: 髋侧摆={angles_deg[0]:.1f}°, 髋俯仰={angles_deg[1]:.1f}°, 膝俯仰={angles_deg[2]:.1f}°")

            return tuple(angles_deg)

        except ValueError as e:
            # 捕获核心层异常（如目标位置数学上不可达）
            if self.verbose:
                print(f"解算失败: {e}")
            return None

    def calculate_position(self, hip_side_deg: float, hip_pitch_deg: float,
                         knee_pitch_deg: float, leg_side: str = 'left') -> Tuple[float, float, float]:
        """
        正运动学计算 - 从关节角度计算脚部位置

        参数
        ----
        hip_side_deg : float
            髋侧摆角度 (度)
        hip_pitch_deg : float
            髋俯仰角度 (度)
        knee_pitch_deg : float
            膝俯仰角度 (度)
        leg_side : str
            腿侧别，'left' 或 'right'，默认 'left'

        返回
        ----
        tuple
            (x, y, z) 脚部位置 (mm)

        示例
        ----
        >>> pos = leg.calculate_position(10, -20, -45, 'left')
        >>> print(f"脚部位置: x={pos[0]}, y={pos[1]}, z={pos[2]}")
        """
        is_left = (leg_side == 'left')

        # 输出日志
        if self.verbose:
            print(f"\n=== 位置计算 ===")
            print(f"角度: 髋侧摆={hip_side_deg:.1f}°, 髋俯仰={hip_pitch_deg:.1f}°, 膝俯仰={knee_pitch_deg:.1f}° - {leg_side}腿")

        # 单位转换 - 度转弧度
        # 核心层需要弧度输入
        angles_rad = [math.radians(angle) for angle in [hip_side_deg, hip_pitch_deg, knee_pitch_deg]]

        # 调用核心层正运动学计算
        position = self.kinematics.forward_kinematics(*angles_rad, is_left)

        # 输出结果日志
        if self.verbose:
            print(f"计算结果: ({position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f})")

        return position

    def get_joint_positions(self, hip_side_deg: float, hip_pitch_deg: float,
                          knee_pitch_deg: float, leg_side: str = 'left') -> List[Tuple[float, float, float]]:
        """
        获取所有关节的3D位置 - 用于可视化

        返回从髋关节到脚部所有关键点的坐标，用于绘制腿部结构。

        参数
        ----
        hip_side_deg : float
            髋侧摆角度 (度)
        hip_pitch_deg : float
            髋俯仰角度 (度)
        knee_pitch_deg : float
            膝俯仰角度 (度)
        leg_side : str
            腿侧别，默认 'left'

        返回
        ----
        list
            5个关节位置的列表: [髋关节, 延伸点, 髋俯仰关节, 膝关节, 脚部]
            每个位置为 (x, y, z) 元组
        """
        is_left = (leg_side == 'left')

        # 度转弧度
        angles_rad = [math.radians(angle) for angle in [hip_side_deg, hip_pitch_deg, knee_pitch_deg]]

        # 调用核心层获取关节位置
        joints = self.kinematics.get_joint_positions(*angles_rad, is_left)

        # 转换为纯 Python 元组列表（便于序列化）
        return [(float(j[0]), float(j[1]), float(j[2])) for j in joints]

    # ------------------------------------------------------------------------
    # 内部验证方法
    # ------------------------------------------------------------------------

    def _check_workspace(self, x: float, y: float, z: float) -> bool:
        """
        检查目标位置是否在工作空间内

        工作空间是一个以髋关节为圆心的球形区域：
        - 内边界：min_reach（避免腿部折叠到自身）
        - 外边界：max_reach（受腿部长度限制）

        参数
        ----
        x, y, z : float
            目标位置坐标 (mm)

        返回
        ----
        bool
            True: 位置在可达范围内
            False: 位置过近或过远
        """
        # 计算目标位置到髋关节的距离
        distance = math.sqrt(x**2 + y**2 + z**2)

        # 检查是否过近
        if distance < self.min_reach:
            if self.verbose:
                print(f"位置过近: 距离={distance:.1f}mm < {self.min_reach}mm")
            return False

        # 检查是否过远
        if distance > self.max_reach:
            if self.verbose:
                print(f"位置过远: 距离={distance:.1f}mm > {self.max_reach}mm")
            return False

        return True

    def _check_angle_limits(self, angles_deg: List[float]) -> bool:
        """
        检查关节角度是否在允许范围内

        角度限制基于舵机的物理转动范围，超出范围可能导致：
        - 舵机损坏
        - 机械碰撞
        - 控制不稳定

        参数
        ----
        angles_deg : list
            [hip_side, hip_pitch, knee_pitch] 角度（度）

        返回
        ----
        bool
            True: 所有角度都在允许范围内
            False: 有角度超限
        """
        hip_side, hip_pitch, knee_pitch = angles_deg

        # 定义要检查的角度及其限制
        limits = [
            (hip_side, self.angle_limits['hip_side'], '髋侧摆'),
            (hip_pitch, self.angle_limits['hip_pitch'], '髋俯仰'),
            (knee_pitch, self.angle_limits['knee_pitch'], '膝俯仰')
        ]

        # 逐个检查
        for angle, (min_limit, max_limit), name in limits:
            if not (min_limit <= angle <= max_limit):
                if self.verbose:
                    print(f"{name}角度超限: {angle:.1f}° 不在 [{min_limit}, {max_limit}] 范围内")
                return False

        return True

    # ------------------------------------------------------------------------
    # 测试方法
    # ------------------------------------------------------------------------

    def test_basic_positions(self):
        """
        测试基本位置的逆运动学解算

        运行一组预设的测试用例，验证：
        1. IK 能正确求解
        2. FK 能正确还原
        3. IK-FK 往返误差在可接受范围内

        测试用例包括：
        - 腿部垂直下方
        - 向前伸展
        - 向后收缩
        - 向外侧摆
        """
        if self.verbose:
            print(f"\n{'='*50}")
            print("基本位置测试")
            print("="*50)

        # 定义测试用例：(x, y, z, 描述)
        test_cases = [
            (0, 60.5, -200, "腿部垂直下方"),
            (50, 60.5, -180, "向前伸展"),
            (-30, 60.5, -220, "向后收缩"),
            (0, 80, -150, "向外侧摆"),
        ]

        # 逐个测试
        for i, (x, y, z, desc) in enumerate(test_cases, 1):
            print(f"\n测试 {i}: {desc}")
            result = self.solve_position(x, y, z, 'left')

            if result:
                # 验证正运动学：IK 结果 → FK → 应该还原原始位置
                calc_pos = self.calculate_position(*result, 'left')

                # 计算往返误差
                error = math.sqrt(sum((a-b)**2 for a, b in zip((x, y, z), calc_pos)))

                # 误差小于 1mm 视为通过
                print(f"验证误差: {error:.3f}mm ({'✓' if error < 1.0 else '✗'})")
            else:
                print("❌ 解算失败")


# ============================================================================
# 四足机器人控制器
# ============================================================================

class QuadrupedController:
    """
    四足机器人控制器 - 应用层

    管理四条腿的协调运动和状态，提供：
    - 统一的四腿位置设置接口
    - 默认站立姿态计算
    - 步态测试功能

    腿部命名
    --------
    - front_left: 左前腿
    - front_right: 右前腿
    - rear_left: 左后腿
    - rear_right: 右后腿

    属性
    ----
    kinematics : QuadrupedKinematics
        四足运动学核心计算器
    leg_controllers : dict
        四个 LegController 实例
    default_stance : dict
        默认站立姿态（各腿的脚部位置）
    """

    def __init__(self, l1=60.5, l2=10.0, l3=111.126, l4=118.5, verbose=True):
        """
        初始化四足机器人控制器

        参数
        ----
        l1-l4 : float
            腿部几何参数 (mm)
        verbose : bool
            是否输出详细日志
        """
        # 创建四足运动学核心
        self.kinematics = QuadrupedKinematics(l1, l2, l3, l4)

        # 为每条腿创建独立的控制器
        # 所有腿使用相同的几何参数
        self.leg_controllers = {
            'front_left': LegController(l1, l2, l3, l4, verbose=verbose),
            'front_right': LegController(l1, l2, l3, l4, verbose=verbose),
            'rear_left': LegController(l1, l2, l3, l4, verbose=verbose),
            'rear_right': LegController(l1, l2, l3, l4, verbose=verbose)
        }

        self.verbose = verbose

        # 默认站立姿态
        # 各腿坐标系下的脚部位置 (x, y, z)
        # y 值：左腿为正，右腿为负
        self.default_stance = {
            'front_left': (0, 60.5, -200),
            'front_right': (0, -60.5, -200),
            'rear_left': (0, 60.5, -200),
            'rear_right': (0, -60.5, -200)
        }

    def set_leg_positions(self, positions_dict: Dict[str, Tuple[float, float, float]]) -> Dict[str, Optional[Tuple[float, float, float]]]:
        """
        设置多条腿的目标位置

        批量设置四条腿的目标位置，返回各腿的关节角度。

        参数
        ----
        positions_dict : dict
            {'leg_name': (x, y, z), ...}
            leg_name: 'front_left', 'front_right', 'rear_left', 'rear_right'
            (x, y, z): 脚部在各腿坐标系下的位置 (mm)

        返回
        ----
        dict
            {'leg_name': (hip_side, hip_pitch, knee_pitch) 或 None, ...}
            成功返回角度元组，失败返回 None

        示例
        ----
        >>> positions = {
        ...     'front_left': (30, 60.5, -180),
        ...     'front_right': (-30, -60.5, -200),
        ... }
        >>> results = controller.set_leg_positions(positions)
        """
        results = {}

        for leg_name, position in positions_dict.items():
            if leg_name in self.leg_controllers:
                # 根据腿名确定左右侧
                leg_side = 'left' if 'left' in leg_name else 'right'

                # 获取对应的腿部控制器
                controller = self.leg_controllers[leg_name]

                # 执行 IK 求解
                results[leg_name] = controller.solve_position(*position, leg_side)
            else:
                if self.verbose:
                    print(f"未知腿部: {leg_name}")
                results[leg_name] = None

        return results

    def get_default_stance(self) -> Dict[str, Tuple[float, float, float]]:
        """
        计算默认站立姿态的关节角度

        默认站立姿态是机器人静止站立时各腿的位置和角度。

        返回
        ----
        dict
            {'leg_name': (hip_side_deg, hip_pitch_deg, knee_pitch_deg), ...}
        """
        if self.verbose:
            print(f"\n{'='*50}")
            print("计算默认站立姿态")
            print("="*50)

        # 使用 default_stance 位置计算各腿角度
        stance_angles = self.set_leg_positions(self.default_stance)

        # 输出结果
        if self.verbose:
            for leg_name, angles in stance_angles.items():
                if angles:
                    print(f"{leg_name}: 髋侧摆={angles[0]:.1f}°, 髋俯仰={angles[1]:.1f}°, 膝俯仰={angles[2]:.1f}°")
                else:
                    print(f"{leg_name}: 解算失败")

        return stance_angles

    def test_symmetric_gait(self):
        """
        测试对称步态（对角支撑）

        对角步态是最常见的四足步态：
        - 对角线上的两条腿同步运动
        - 前左 + 后右 为一组
        - 前右 + 后左 为一组

        此测试验证对角支撑相位的 IK 求解。
        """
        if self.verbose:
            print(f"\n{'='*50}")
            print("对称步态测试")
            print("="*50)

        # 对角支撑相位
        # 前左腿和后右腿向前，前右腿和后左腿向后
        diagonal_positions = {
            'front_left': (30, 60.5, -180),    # 前左腿向前
            'front_right': (-30, -60.5, -200), # 前右腿向后
            'rear_left': (-30, 60.5, -200),    # 后左腿向后
            'rear_right': (30, -60.5, -180)    # 后右腿向前
        }

        print("对角支撑相位:")
        results = self.set_leg_positions(diagonal_positions)

        # 统计成功率
        success_count = sum(1 for result in results.values() if result is not None)
        print(f"解算成功率: {success_count}/4")

        return results


# ============================================================================
# 工厂函数
# ============================================================================

def create_leg_controller(robot_type='spot_micro', verbose=True) -> LegController:
    """
    工厂函数：创建指定类型机器人的腿部控制器

    通过配置字典支持多种机器人类型，便于扩展。

    参数
    ----
    robot_type : str
        机器人类型标识符：
        - 'spot_micro': Spot Micro 机器人（默认）
        - 'custom': 自定义配置示例
    verbose : bool
        是否输出详细日志，默认 True

    返回
    ----
    LegController
        配置好的腿部控制器实例

    异常
    ----
    ValueError
        不支持的机器人类型

    示例
    ----
    >>> # 创建 Spot Micro 控制器
    >>> leg = create_leg_controller('spot_micro')

    >>> # 创建自定义控制器
    >>> leg = create_leg_controller('custom', verbose=False)
    """
    # 机器人配置字典
    # 每种机器人有自己的几何参数和工作空间限制
    configs = {
        'spot_micro': {
            'l1': 60.5,                    # 髋关节延伸段 (mm)
            'l2': 10.0,                    # 髋关节垂直段 (mm)
            'l3': 111.126,                 # 大腿长度 (mm)
            'l4': 118.5,                   # 小腿长度 (mm)
            'workspace_limits': (50.0, 295.0)  # 工作空间 (mm)
        },
        # 自定义机器人配置示例
        # 可以根据实际机器人参数添加更多配置
        'custom': {
            'l1': 50.0,
            'l2': 15.0,
            'l3': 100.0,
            'l4': 120.0,
            'workspace_limits': (40.0, 280.0)
        }
    }

    # 检查机器人类型是否支持
    if robot_type not in configs:
        raise ValueError(f"不支持的机器人类型: {robot_type}")

    # 获取配置并创建控制器
    config = configs[robot_type]
    return LegController(**config, verbose=verbose)
