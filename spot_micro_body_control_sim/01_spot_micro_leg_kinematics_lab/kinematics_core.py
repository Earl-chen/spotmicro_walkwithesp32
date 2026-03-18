#!/usr/bin/env python3
"""
四足机器人运动学核心计算模块
===========================

本模块提供四足机器人腿部运动学的纯数学计算，是整个运动学系统的核心层。

设计原则
--------
1. 纯函数设计：无副作用，相同输入产生相同输出
2. 无应用逻辑：不包含日志、验证、单位转换等应用层功能
3. 高性能：支持批量计算，适合轨迹规划
4. 数值稳定：处理边界情况，避免数值溢出

腿部结构
--------
Spot Micro 机器人的腿部采用 4 段 3 自由度设计：

    髋关节 (hip_joint)
        │
        │ L1 (髋关节延伸段，水平方向)
        ▼
    延伸点 (fixed_point)
        │
        │ L2 (髋关节垂直段，垂直方向)
        ▼
    髋俯仰关节 (hip_pitch_joint)
        │
        │ L3 (大腿)
        ▼
    膝关节 (knee_joint)
        │
        │ L4 (小腿)
        ▼
    脚部 (foot)

    坐标系定义（右手法则）：
    - X 轴：前进方向（前方为正）
    - Y 轴：左右方向（左侧为正）
    - Z 轴：上下方向（上方为正）

关节定义
--------
- hip_side (髋侧摆)：绕 X 轴旋转，控制腿部左右摆动
- hip_pitch (髋俯仰)：绕 Y 轴旋转，控制大腿前后摆动
- knee_pitch (膝俯仰)：绕 Y 轴旋转，控制小腿弯曲

主要类
------
- LegKinematics: 单腿运动学计算
- QuadrupedKinematics: 四足机器人运动学管理

单位约定
--------
- 长度：毫米 (mm)
- 角度：弧度 (rad)
  - 注意：核心层使用弧度，应用层 (kinematics_app) 负责度数转换

依赖
----
- numpy: 矩阵运算
- math: 基础数学函数

使用示例
--------
>>> from kinematics_core import LegKinematics
>>>
>>> # 创建腿部运动学计算器
>>> leg = LegKinematics(l1=60.5, l2=10.0, l3=111.126, l4=118.5)
>>>
>>> # 逆运动学：位置 → 角度（返回弧度）
>>> angles = leg.inverse_kinematics(50, 60.5, -180, is_left=True)
>>> print(f"角度: {[math.degrees(a) for a in angles]}")  # 转换为度
>>>
>>> # 正运动学：角度 → 位置（输入弧度）
>>> pos = leg.forward_kinematics(*angles, is_left=True)
>>> print(f"位置: {pos}")
>>>
>>> # 批量计算（轨迹规划）
>>> trajectory = [(0, 60.5, -200), (50, 60.5, -180), (100, 60.5, -160)]
>>> results = leg.batch_inverse_kinematics(trajectory)
"""

import numpy as np
import math


# ============================================================================
# 旋转矩阵工具函数
# ============================================================================

def rot_x(angle):
    """
    绕 X 轴的旋转矩阵

    用于髋侧摆关节（hip_side）的坐标变换。

    参数
    ----
    angle : float
        旋转角度（弧度），正值表示从 +Y 向 +Z 旋转

    返回
    ----
    numpy.ndarray
        3x3 旋转矩阵

    数学公式
    --------
    Rx(θ) = | 1    0       0    |
            | 0  cos(θ) -sin(θ) |
            | 0  sin(θ)  cos(θ) |
    """
    c, s = math.cos(angle), math.sin(angle)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c]
    ])


def rot_y(angle):
    """
    绕 Y 轴的旋转矩阵

    用于髋俯仰关节（hip_pitch）和膝关节（knee_pitch）的坐标变换。

    参数
    ----
    angle : float
        旋转角度（弧度），正值表示从 +Z 向 +X 旋转

    返回
    ----
    numpy.ndarray
        3x3 旋转矩阵

    数学公式
    --------
    Ry(θ) = | cos(θ)  0  sin(θ) |
            |   0     1    0    |
            |-sin(θ)  0  cos(θ) |
    """
    c, s = math.cos(angle), math.sin(angle)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ])


# ============================================================================
# 单腿运动学计算
# ============================================================================

class LegKinematics:
    """
    单腿运动学核心计算类

    提供正运动学（FK）和逆运动学（IK）的纯数学计算。

    设计特点
    --------
    - 纯函数：无副作用，不修改内部状态
    - 高性能：支持批量计算
    - 数值稳定：处理 acos/asin 的边界情况

    属性
    ----
    l1 : float
        髋关节延伸段长度 (mm)，从髋关节到延伸点的水平距离
    l2 : float
        髋关节垂直段长度 (mm)，从延伸点到髋俯仰关节的垂直距离
    l3 : float
        大腿长度 (mm)，从髋俯仰关节到膝关节
    l4 : float
        小腿长度 (mm)，从膝关节到脚部

    工作空间
    --------
    理论最大伸展距离 = l3 + l4
    理论最小收缩距离 = |l3 - l4|
    实际可用范围还需要考虑 l1, l2 的影响
    """

    def __init__(self, l1=60.5, l2=10.0, l3=111.126, l4=118.5):
        """
        初始化腿部几何参数

        参数
        ----
        l1 : float
            髋关节延伸段长度 (mm)，默认 60.5
            这是髋关节中心到腿部侧面固定点的水平距离
        l2 : float
            髋关节垂直段长度 (mm)，默认 10.0
            这是延伸点到髋俯仰关节的垂直距离
        l3 : float
            大腿长度 (mm)，默认 111.126
            从髋俯仰关节中心到膝关节中心
        l4 : float
            小腿长度 (mm)，默认 118.5
            从膝关节中心到脚部末端
        """
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4

    # ------------------------------------------------------------------------
    # 逆运动学
    # ------------------------------------------------------------------------

    def inverse_kinematics(self, x, y, z, is_left=True):
        """
        逆运动学计算 - 从脚部位置求解关节角度

        给定脚部在腿部坐标系中的位置，计算三个关节的角度。

        算法原理
        --------
        采用几何法求解，分两步：

        第一步：求解髋侧摆角度（hip_side）
        - 将 3D 问题投影到 YZ 平面
        - 使用几何关系求解髋侧摆角

        第二步：求解髋俯仰和膝关节角度
        - 将问题简化为 2D 平面内的双连杆问题
        - 使用余弦定理求解

        参数
        ----
        x : float
            脚部 X 坐标 (mm)，前进方向为正
        y : float
            脚部 Y 坐标 (mm)
            - 左腿：正值表示向外侧
            - 右腿：负值表示向外侧
        z : float
            脚部 Z 坐标 (mm)，向下为负
        is_left : bool
            是否为左腿，默认 True
            - 左腿：l1 方向为 +Y
            - 右腿：l1 方向为 -Y

        返回
        ----
        tuple
            (hip_side, hip_pitch, knee_pitch) 三个关节角度（弧度）
            - hip_side: 髋侧摆角度，正值表示腿向外侧摆
            - hip_pitch: 髋俯仰角度，正值表示大腿向前摆
            - knee_pitch: 膝俯仰角度，始终为负（膝盖只能向后弯曲）

        异常
        ----
        ValueError
            目标位置不可达（几何约束不满足）

        示例
        ----
        >>> angles = leg.inverse_kinematics(50, 60.5, -180, is_left=True)
        >>> hip_side, hip_pitch, knee_pitch = angles
        """
        # ====================================================================
        # 第一步：投影到 YZ 平面，求解髋侧摆角度
        # ====================================================================

        # H: 原点到脚部在 YZ 平面的投影距离
        # 这是求解髋侧摆角的关键几何量
        H = math.sqrt(y**2 + z**2)

        # 几何约束检查
        # H 必须大于 l1，否则脚部太靠近髋关节轴线，无法到达
        if H < self.l1:
            raise ValueError(
                f"目标位置太近: H={H:.3f}mm < l1={self.l1}mm，"
                f"脚部必须距离髋关节轴线至少 {self.l1}mm"
            )

        # G: 从髋关节轴线到脚部投影点的有效距离
        # 使用勾股定理：H² = l1² + G²
        G = math.sqrt(H**2 - self.l1**2)

        # F: 考虑 l2 偏移后的有效臂长
        # l2 是髋俯仰关节相对于延伸点的垂直偏移
        F = G - self.l2

        # S: 从髋俯仰关节到脚部的直线距离
        # 这是 2D 双连杆问题的斜边长度
        S = math.sqrt(F**2 + x**2)

        # 计算髋侧摆角度
        # 左右腿的公式不同，因为 l1 的方向相反
        if is_left:
            # 左腿：l1 指向 +Y 方向
            # hip_side = atan2(z, y) + acos(l1/H)
            # 第一项是脚部在 YZ 平面的角度
            # 第二项是补偿 l1 偏移的角度
            hip_side_angle = math.atan2(z, y) + math.acos(self.l1 / H)
        else:
            # 右腿：l1 指向 -Y 方向
            # hip_side = π + atan2(z, y) - acos(l1/H)
            hip_side_angle = math.pi + math.atan2(z, y) - math.acos(self.l1 / H)

        # ====================================================================
        # 第二步：在髋俯仰关节坐标系中求解双连杆问题
        # ====================================================================

        # 使用余弦定理求解髋俯仰角度
        # 考虑三角形：髋俯仰关节 - 膝关节 - 脚部
        # cos(hip_pitch_relative) = (S² + l3² - l4²) / (2 * S * l3)
        cos_hip = (S**2 + self.l3**2 - self.l4**2) / (2 * S * self.l3)

        # 数值稳定性处理
        # 由于浮点误差，cos_hip 可能略微超出 [-1, 1] 范围
        # 这会导致 acos 产生 NaN
        cos_hip = np.clip(cos_hip, -1.0, 1.0)

        # 髋俯仰角度 = 相对角度 - 偏移角度
        # atan2(x, F) 是脚部相对于髋俯仰关节的方向角
        hip_pitch_angle = math.acos(cos_hip) - math.atan2(x, F)

        # 使用余弦定理求解膝关节角度
        # 考虑三角形：髋俯仰关节 - 膝关节 - 脚部
        # cos(knee) = (S² - l3² - l4²) / (2 * l3 * l4)
        cos_knee = (S**2 - self.l3**2 - self.l4**2) / (2 * self.l3 * self.l4)

        # 数值稳定性处理
        cos_knee = np.clip(cos_knee, -1.0, 1.0)

        # 膝关节角度始终为负
        # 负号表示膝盖只能向后弯曲（不能向前弯曲）
        knee_pitch_angle = -math.acos(cos_knee)

        return hip_side_angle, hip_pitch_angle, knee_pitch_angle

    # ------------------------------------------------------------------------
    # 正运动学
    # ------------------------------------------------------------------------

    def forward_kinematics(self, hip_side_rad, hip_pitch_rad, knee_pitch_rad, is_left=True):
        """
        正运动学计算 - 从关节角度计算脚部位置

        通过旋转矩阵链式变换，从髋关节开始依次计算各关节位置，
        最终得到脚部在腿部坐标系中的位置。

        算法原理
        --------
        使用 DH 变换的思想，通过旋转矩阵链式相乘：

        脚部位置 = hip_joint
                 + R_hip_side @ l1_vec              # 到延伸点
                 + R_hip_side @ l2_vec              # 到髋俯仰关节
                 + R_hip_side @ R_hip_pitch @ l3_vec    # 到膝关节
                 + R_hip_side @ R_hip_pitch @ R_knee @ l4_vec  # 到脚部

        参数
        ----
        hip_side_rad : float
            髋侧摆角度（弧度）
        hip_pitch_rad : float
            髋俯仰角度（弧度）
        knee_pitch_rad : float
            膝俯仰角度（弧度）
        is_left : bool
            是否为左腿，默认 True

        返回
        ----
        tuple
            (x, y, z) 脚部位置 (mm)

        示例
        ----
        >>> pos = leg.forward_kinematics(0.1, -0.3, -1.0, is_left=True)
        >>> x, y, z = pos
        """
        # 髋关节原点
        hip_joint = np.array([0, 0, 0])

        # ====================================================================
        # L1 段：髋关节 → 延伸点
        # ====================================================================
        # L1 方向取决于左右腿
        # 左腿：l1 指向 +Y
        # 右腿：l1 指向 -Y
        l1_vec = np.array([0, self.l1 if is_left else -self.l1, 0])

        # 髋侧摆旋转矩阵
        R_hip_side = rot_x(hip_side_rad)

        # 延伸点位置 = 髋关节 + 旋转后的 l1 向量
        fixed_point = hip_joint + R_hip_side @ l1_vec

        # ====================================================================
        # L2 段：延伸点 → 髋俯仰关节
        # ====================================================================
        # L2 始终指向 -Z 方向（向下）
        l2_vec = np.array([0, 0, -self.l2])

        # 髋俯仰关节位置
        hip_pitch_joint = fixed_point + R_hip_side @ l2_vec

        # ====================================================================
        # L3 段：髋俯仰关节 → 膝关节
        # ====================================================================
        # L3 始终指向 -Z 方向（向下）
        l3_vec = np.array([0, 0, -self.l3])

        # 髋俯仰旋转矩阵
        R_hip_pitch = rot_y(hip_pitch_rad)

        # 膝关节位置
        # 注意：R_hip_pitch 的效果是在髋俯仰关节坐标系中
        # 需要先应用 R_hip_side 变换到腿坐标系
        knee_joint = hip_pitch_joint + R_hip_side @ R_hip_pitch @ l3_vec

        # ====================================================================
        # L4 段：膝关节 → 脚部
        # ====================================================================
        # L4 始终指向 -Z 方向（向下）
        l4_vec = np.array([0, 0, -self.l4])

        # 膝关节旋转矩阵
        R_knee = rot_y(knee_pitch_rad)

        # 脚部位置
        foot_pos = knee_joint + R_hip_side @ R_hip_pitch @ R_knee @ l4_vec

        # 返回纯 Python 元组（便于序列化）
        return float(foot_pos[0]), float(foot_pos[1]), float(foot_pos[2])

    # ------------------------------------------------------------------------
    # 关节位置计算（用于可视化）
    # ------------------------------------------------------------------------

    def get_joint_positions(self, hip_side_rad, hip_pitch_rad, knee_pitch_rad, is_left=True):
        """
        计算所有关节的 3D 位置 - 用于可视化

        返回从髋关节到脚部所有关键点的坐标，用于绘制腿部结构图。

        参数
        ----
        hip_side_rad : float
            髋侧摆角度（弧度）
        hip_pitch_rad : float
            髋俯仰角度（弧度）
        knee_pitch_rad : float
            膝俯仰角度（弧度）
        is_left : bool
            是否为左腿，默认 True

        返回
        ----
        list
            5 个关节位置的列表（numpy 数组）：
            [髋关节, 延伸点, 髋俯仰关节, 膝关节, 脚部]

            位置索引：
            - 0: hip_joint (髋关节，原点)
            - 1: fixed_point (延伸点，L1 段末端)
            - 2: hip_pitch_joint (髋俯仰关节，L2 段末端)
            - 3: knee_joint (膝关节，L3 段末端)
            - 4: foot_pos (脚部，L4 段末端)
        """
        # 髋关节原点
        hip_joint = np.array([0, 0, 0])

        # L1 段
        l1_vec = np.array([0, self.l1 if is_left else -self.l1, 0])
        R_hip_side = rot_x(hip_side_rad)
        fixed_point = hip_joint + R_hip_side @ l1_vec

        # L2 段
        l2_vec = np.array([0, 0, -self.l2])
        hip_pitch_joint = fixed_point + R_hip_side @ l2_vec

        # L3 段
        l3_vec = np.array([0, 0, -self.l3])
        R_hip_pitch = rot_y(hip_pitch_rad)
        knee_joint = hip_pitch_joint + R_hip_side @ R_hip_pitch @ l3_vec

        # L4 段
        l4_vec = np.array([0, 0, -self.l4])
        R_knee = rot_y(knee_pitch_rad)
        foot_pos = knee_joint + R_hip_side @ R_hip_pitch @ R_knee @ l4_vec

        return [hip_joint, fixed_point, hip_pitch_joint, knee_joint, foot_pos]

    # ------------------------------------------------------------------------
    # 批量计算（用于轨迹规划）
    # ------------------------------------------------------------------------

    def batch_inverse_kinematics(self, positions, is_left=True):
        """
        批量逆运动学计算 - 用于轨迹规划

        一次性计算多个目标位置的关节角度，比循环调用单点计算更高效。

        参数
        ----
        positions : list
            位置列表，每个元素为 (x, y, z) 元组
        is_left : bool
            是否为左腿，默认 True

        返回
        ----
        list
            角度结果列表，每个元素为：
            - 成功: (hip_side, hip_pitch, knee_pitch) 角度元组（弧度）
            - 失败: None（位置不可达）

        示例
        ----
        >>> # 定义圆形轨迹
        >>> trajectory = [
        ...     (0, 60.5, -200),
        ...     (30, 60.5, -180),
        ...     (60, 60.5, -160),
        ... ]
        >>> results = leg.batch_inverse_kinematics(trajectory)
        >>> for i, result in enumerate(results):
        ...     if result:
        ...         print(f"点 {i}: 角度 = {[math.degrees(a) for a in result]}")
        ...     else:
        ...         print(f"点 {i}: 不可达")
        """
        results = []

        for x, y, z in positions:
            try:
                angles = self.inverse_kinematics(x, y, z, is_left)
                results.append(angles)
            except ValueError:
                # 位置不可达，记录 None
                results.append(None)

        return results

    def batch_forward_kinematics(self, angles_list, is_left=True):
        """
        批量正运动学计算 - 用于轨迹验证

        一次性计算多组关节角度对应的脚部位置。

        参数
        ----
        angles_list : list
            角度列表，每个元素为 (hip_side, hip_pitch, knee_pitch) 元组（弧度）
        is_left : bool
            是否为左腿，默认 True

        返回
        ----
        list
            位置列表，每个元素为 (x, y, z) 元组

        示例
        ----
        >>> # 验证 IK-FK 往返精度
        >>> positions = [(0, 60.5, -200), (50, 60.5, -180)]
        >>> angles = leg.batch_inverse_kinematics(positions)
        >>> valid_angles = [a for a in angles if a is not None]
        >>> verify = leg.batch_forward_kinematics(valid_angles)
        """
        results = []

        for hip_side, hip_pitch, knee_pitch in angles_list:
            pos = self.forward_kinematics(hip_side, hip_pitch, knee_pitch, is_left)
            results.append(pos)

        return results

    # ------------------------------------------------------------------------
    # 工作空间分析
    # ------------------------------------------------------------------------

    def get_workspace_bounds(self):
        """
        计算理论工作空间边界

        工作空间是脚部能够到达的所有位置的集合。
        理论边界基于腿部长度的几何限制。

        返回
        ----
        dict
            包含以下键：
            - 'min_reach': 最小伸展距离 (mm)，当腿完全收缩时
            - 'max_reach': 最大伸展距离 (mm)，当腿完全伸直时
            - 'segments': (l1, l2, l3, l4) 各段长度

        注意
        ----
        这是简化的理论边界，实际可用空间还受以下因素限制：
        - 关节角度范围
        - 机械碰撞
        - l1, l2 的偏移影响

        示例
        ----
        >>> bounds = leg.get_workspace_bounds()
        >>> print(f"工作范围: {bounds['min_reach']} - {bounds['max_reach']} mm")
        """
        # 理论最大距离：大腿和小腿完全伸直
        max_reach = self.l3 + self.l4

        # 理论最小距离：大腿和小腿完全重叠
        min_reach = abs(self.l3 - self.l4)

        return {
            'min_reach': min_reach,
            'max_reach': max_reach,
            'segments': (self.l1, self.l2, self.l3, self.l4)
        }


# ============================================================================
# 四足机器人运动学
# ============================================================================

class QuadrupedKinematics:
    """
    四足机器人完整运动学管理器

    管理四条腿的运动学计算，提供统一的四足运动学接口。

    腿部命名
    --------
    - front_left: 左前腿
    - front_right: 右前腿
    - rear_left: 左后腿
    - rear_right: 右后腿

    属性
    ----
    leg_kinematics : LegKinematics
        单腿运动学计算器（四条腿使用相同参数）
    legs : list
        腿部名称列表
    """

    def __init__(self, l1=60.5, l2=10.0, l3=111.126, l4=118.5):
        """
        初始化四足机器人运动学

        参数
        ----
        l1-l4 : float
            腿部几何参数 (mm)，四条腿使用相同参数
        """
        # 创建单腿运动学计算器
        # 假设四条腿使用相同的几何参数
        self.leg_kinematics = LegKinematics(l1, l2, l3, l4)

        # 腿部标识
        self.legs = ['front_left', 'front_right', 'rear_left', 'rear_right']

    def inverse_kinematics_all(self, positions_dict):
        """
        计算所有腿的逆运动学

        批量计算四条腿的关节角度。

        参数
        ----
        positions_dict : dict
            各腿的目标位置：
            {
                'front_left': (x, y, z),
                'front_right': (x, y, z),
                'rear_left': (x, y, z),
                'rear_right': (x, y, z)
            }
            注意：位置是在各腿的局部坐标系中

        返回
        ----
        dict
            各腿的关节角度：
            {
                'front_left': (hip_side, hip_pitch, knee_pitch) 或 None,
                ...
            }
            成功返回角度元组（弧度），失败返回 None

        示例
        ----
        >>> quad = QuadrupedKinematics()
        >>> positions = {
        ...     'front_left': (0, 60.5, -200),
        ...     'front_right': (0, -60.5, -200),
        ...     'rear_left': (0, 60.5, -200),
        ...     'rear_right': (0, -60.5, -200),
        ... }
        >>> results = quad.inverse_kinematics_all(positions)
        """
        results = {}

        for leg_name, position in positions_dict.items():
            # 根据腿名判断左右
            is_left = 'left' in leg_name

            # 解包位置
            x, y, z = position

            try:
                # 调用单腿 IK
                angles = self.leg_kinematics.inverse_kinematics(x, y, z, is_left)
                results[leg_name] = angles
            except ValueError as e:
                # 位置不可达
                results[leg_name] = None

        return results

    def forward_kinematics_all(self, angles_dict):
        """
        计算所有腿的正运动学

        从关节角度计算四条腿的脚部位置。

        参数
        ----
        angles_dict : dict
            各腿的关节角度（弧度）：
            {
                'front_left': (hip_side, hip_pitch, knee_pitch),
                'front_right': (hip_side, hip_pitch, knee_pitch),
                ...
            }

        返回
        ----
        dict
            各腿的脚部位置：
            {
                'front_left': (x, y, z),
                ...
            }

        示例
        ----
        >>> angles = {
        ...     'front_left': (0.1, -0.3, -1.0),
        ...     'front_right': (-0.1, -0.3, -1.0),
        ... }
        >>> positions = quad.forward_kinematics_all(angles)
        """
        results = {}

        for leg_name, angles in angles_dict.items():
            # 根据腿名判断左右
            is_left = 'left' in leg_name

            # 解包角度
            hip_side, hip_pitch, knee_pitch = angles

            # 调用单腿 FK
            position = self.leg_kinematics.forward_kinematics(
                hip_side, hip_pitch, knee_pitch, is_left
            )
            results[leg_name] = position

        return results
