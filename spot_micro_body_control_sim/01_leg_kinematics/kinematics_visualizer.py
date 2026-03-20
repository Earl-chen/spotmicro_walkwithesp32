#!/usr/bin/env python3
"""
四足机器人可视化模块
==================

本模块提供 3D 可视化功能，与运动学计算完全解耦。

设计原则
--------
1. 解耦设计：可视化层不包含运动学计算逻辑
2. 独立可复用：可视化器可独立使用，不依赖具体实现
3. 多种模式：支持静态图、交互式工具、双腿对比

功能概述
--------
- LegVisualizer: 静态 3D 渲染，用于生成图片
- InteractiveVisualizer: 交互式工具，支持角度↔坐标双向转换
- DualLegVisualizer: 双腿对比显示

坐标系说明
----------
3D 视图使用右手坐标系：
- X 轴：前进方向（前方为正）
- Y 轴：左右方向（左侧为正）
- Z 轴：上下方向（上方为正）

依赖
----
- matplotlib: 3D 绘图和交互界面
- numpy: 数值计算
- kinematics_app: LegController 控制器

使用示例
--------
>>> from kinematics_app import create_leg_controller
>>> from kinematics_visualizer import create_visualizer
>>>
>>> # 创建控制器
>>> leg = create_leg_controller('spot_micro')
>>>
>>> # 静态图
>>> viz = create_visualizer(leg, mode='static')
>>> fig = viz.create_static_plot(10, -20, -45, 'left')
>>>
>>> # 交互式工具
>>> interactive = create_visualizer(leg, mode='interactive', leg_side='left')
>>> interactive.create_interactive_window()
>>>
>>> # 双腿对比
>>> dual = create_visualizer(leg, mode='dual')
>>> fig = dual.create_comparison_plot((15, -20, -60), (-15, -20, -60))
"""

# ============================================================================
# 后端配置
# ============================================================================

# matplotlib 需要在导入 pyplot 之前配置交互式后端
# 优先级：TkAgg > Qt5Agg > Agg（无头模式）

import matplotlib
try:
    matplotlib.use('TkAgg')      # Linux 下最稳定的后端
except:
    try:
        matplotlib.use('Qt5Agg')  # Windows 下更好的选择
    except:
        matplotlib.use('Agg')     # 无头环境，仅保存图片

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import TextBox, Button
import math
from typing import List, Tuple, Optional
from kinematics_app import LegController


# ============================================================================
# 静态可视化器
# ============================================================================

class LegVisualizer:
    """
    单腿 3D 静态可视化器

    用于生成高质量的 3D 腿部结构图像，支持：
    - 关节标注
    - 坐标轴显示
    - 颜色区分左右腿
    - 图片保存

    属性
    ----
    controller : LegController
        腿部控制器，用于获取关节位置和执行运动学计算
    """

    def __init__(self, leg_controller: LegController):
        """
        初始化可视化器

        参数
        ----
        leg_controller : LegController
            腿部控制器实例
        """
        self.controller = leg_controller

        # 配置中文字体（支持中文标签显示）
        plt.rcParams['font.sans-serif'] = [
            'WenQuanYi Zen Hei',  # Linux 中文字体
            'DejaVu Sans',        # 通用后备字体
            'SimHei',             # Windows 中文字体
            'Arial Unicode MS'    # macOS 中文字体
        ]
        plt.rcParams['axes.unicode_minus'] = False  # 修复负号显示问题

    def draw_leg(self, ax, hip_side_deg: float, hip_pitch_deg: float,
                knee_pitch_deg: float, leg_side: str = 'left', title: str = ""):
        """
        在指定轴上绘制腿部结构

        绘制完整的腿部结构，包括：
        - 骨架连接线（5 个关节点）
        - 关节名称标注
        - 坐标轴标签
        - 脚部位置显示

        参数
        ----
        ax : matplotlib.axes.Axes
            3D 坐标轴对象
        hip_side_deg : float
            髋侧摆角度（度）
        hip_pitch_deg : float
            髋俯仰角度（度）
        knee_pitch_deg : float
            膝俯仰角度（度）
        leg_side : str
            腿侧别，'left' 或 'right'，默认 'left'
        title : str
            图标题

        返回
        ----
        tuple
            脚部位置 (x, y, z) (mm)

        关节索引
        --------
        0: 髋关节 (hip_joint) - 原点
        1: 延伸点 (fixed_point) - L1 段末端
        2: 髋俯仰关节 (hip_pitch_joint) - L2 段末端
        3: 膝关节 (knee_joint) - L3 段末端
        4: 脚部 (foot) - L4 段末端
        """
        # 清空坐标轴（防止重复绘制）
        ax.clear()

        # ====================================================================
        # 获取关节位置
        # ====================================================================
        joints = self.controller.get_joint_positions(
            hip_side_deg, hip_pitch_deg, knee_pitch_deg, leg_side
        )
        # joints 是一个列表，包含 5 个 numpy 数组，每个代表一个关节的 (x, y, z) 坐标

        # ====================================================================
        # 绘制腿部骨架
        # ====================================================================
        # 将关节列表转换为 numpy 数组，便于切片操作
        joints_array = np.array(joints)

        # 根据腿侧选择颜色
        # 左腿用红色，右腿用蓝色，便于区分
        color = 'red' if leg_side == 'left' else 'blue'

        # 绘制骨架线：o- 表示圆点+连线
        # linewidth=4: 线条粗细
        # markersize=8: 关节点大小
        # alpha=0.8: 透明度（1.0 为不透明）
        ax.plot(joints_array[:, 0], joints_array[:, 1], joints_array[:, 2],
               'o-', linewidth=4, markersize=8, color=color, alpha=0.8)

        # ====================================================================
        # 标注关节名称
        # ====================================================================
        joint_names = ['髋关节', '延伸点', '髋俯仰', '膝关节', '脚部']
        for joint, name in zip(joints, joint_names):
            # 在每个关节位置添加文本标注
            ax.text(joint[0], joint[1], joint[2], name, fontsize=10, weight='bold')

        # ====================================================================
        # 设置坐标轴标签
        # ====================================================================
        ax.set_xlabel('X (前进)', fontsize=12)
        ax.set_ylabel('Y (左右)', fontsize=12)
        ax.set_zlabel('Z (上下)', fontsize=12)

        # ====================================================================
        # 设置标题（显示脚部位置）
        # ====================================================================
        foot_pos = joints[-1]  # 脚部是最后一个关节
        full_title = f'{title}\n脚部: ({foot_pos[0]:.1f}, {foot_pos[1]:.1f}, {foot_pos[2]:.1f})'
        ax.set_title(full_title, fontsize=12)

        # ====================================================================
        # 设置显示范围
        # ====================================================================
        # 固定范围确保不同角度下的视图一致
        max_range = 150  # XY 平面范围
        ax.set_xlim([-max_range, max_range])
        ax.set_ylim([-max_range, max_range])
        ax.set_zlim([-300, 50])  # Z 轴范围（腿部主要向下延伸）

        return foot_pos

    def create_static_plot(self, hip_side_deg: float, hip_pitch_deg: float,
                         knee_pitch_deg: float, leg_side: str = 'left',
                         save_path: Optional[str] = None) -> plt.Figure:
        """
        创建静态图像

        生成指定姿态的腿部 3D 图像，可选择保存到文件。

        参数
        ----
        hip_side_deg : float
            髋侧摆角度（度）
        hip_pitch_deg : float
            髋俯仰角度（度）
        knee_pitch_deg : float
            膝俯仰角度（度）
        leg_side : str
            腿侧别，'left' 或 'right'，默认 'left'
        save_path : str, optional
            图片保存路径，如 'leg_pose.png'
            如果为 None，则不保存

        返回
        ----
        matplotlib.figure.Figure
            Figure 对象，可用于进一步操作

        示例
        ----
        >>> viz = LegVisualizer(controller)
        >>> fig = viz.create_static_plot(10, -20, -45, 'left', 'left_leg.png')
        """
        # 创建图形（10x8 英寸）
        fig = plt.figure(figsize=(10, 8))

        # 创建 3D 坐标轴（占据整个图形）
        ax = fig.add_subplot(111, projection='3d')

        # 绘制腿部
        leg_name = '左腿' if leg_side == 'left' else '右腿'
        foot_pos = self.draw_leg(ax, hip_side_deg, hip_pitch_deg, knee_pitch_deg, leg_side, leg_name)

        # 如果指定了保存路径，保存图片
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"图像已保存: {save_path}")

        return fig


# ============================================================================
# 交互式可视化器
# ============================================================================

class InteractiveVisualizer:
    """
    交互式运动学可视化工具

    提供实时角度与坐标双向转换的交互界面。

    界面布局
    --------
    ┌─────────────────────────────────────────────────────┐
    │              3D 腿部视图                             │
    │                                                       │
    └─────────────────────────────────────────────────────┘
    ┌──────────┬────────────────────────────────────────┐
    │ 控制面板 │          使用说明                       │
    │──────────│                                        │
    │ 角度输入 │  1. 输入角度后点击"角度→位置"          │
    │ [髋侧摆] │  2. 输入坐标后点击"位置→角度"          │
    │ [髋俯仰] │  3. 点击"复位"回到默认姿态             │
    │ [膝俯仰] │                                        │
    │──────────│  角度范围:                             │
    │ 坐标输入 │  • 髋侧摆: ±90°                        │
    │ [X Y Z]  │  • 髋俯仰: ±90°                        │
    │──────────│  • 膝俯仰: -180° ~ 0°                  │
    │ [角度→位]│                                        │
    │ [位置→角]│  工作范围: 50-295mm                    │
    │ [复位]   │                                        │
    └──────────┴────────────────────────────────────────┘

    功能
    ----
    - 正向计算：输入关节角度 → 显示脚部位置
    - 逆向计算：输入脚部位置 → 显示关节角度
    - 实时 3D 渲染：更新时自动重绘
    """

    def __init__(self, leg_controller: LegController, leg_side: str = 'left'):
        """
        初始化交互式可视化工具

        参数
        ----
        leg_controller : LegController
            腿部控制器实例
        leg_side : str
            默认操作的腿侧，'left' 或 'right'，默认 'left'
        """
        self.controller = leg_controller
        self.leg_side = leg_side
        self.visualizer = LegVisualizer(leg_controller)

        # 当前关节角度状态（度）
        # 初始状态：所有角度为 0
        self.current_angles = [0.0, 0.0, 0.0]  # [髋侧摆, 髋俯仰, 膝俯仰]

        # 配置中文字体
        plt.rcParams['font.sans-serif'] = [
            'WenQuanYi Zen Hei', 'DejaVu Sans', 'SimHei', 'Arial Unicode MS'
        ]
        plt.rcParams['axes.unicode_minus'] = False

    def create_interactive_window(self):
        """
        创建交互式窗口

        创建完整的交互界面，包括 3D 视图和控制面板。

        返回
        ----
        matplotlib.figure.Figure
            Figure 对象

        示例
        ----
        >>> interactive = InteractiveVisualizer(controller, 'left')
        >>> fig = interactive.create_interactive_window()
        >>> plt.show()  # 显示窗口
        """
        # ====================================================================
        # 创建主窗口
        # ====================================================================
        # 14x10 英寸，留足空间给控制面板
        self.fig = plt.figure(figsize=(14, 10))
        leg_name = '左腿' if self.leg_side == 'left' else '右腿'
        self.fig.suptitle(f'交互式运动学工具 - {leg_name}', fontsize=16)

        # 创建 3D 坐标轴（占据整个图形）
        self.ax = self.fig.add_subplot(111, projection='3d')

        # 初始绘制腿部
        foot_pos = self.visualizer.draw_leg(
            self.ax, *self.current_angles, self.leg_side, leg_name
        )

        # 创建控制面板
        self._create_controls(foot_pos)

        # 绑定按钮事件
        self._bind_events()

        # 显示窗口
        plt.show()

        return self.fig

    def _create_controls(self, initial_foot_pos):
        """
        创建控制面板

        布局参数：[left, bottom, width, height]
        - left: 距离图形左侧的比例（0-1）
        - bottom: 距离图形底部的比例（0-1）
        - width: 控件宽度（比例）
        - height: 控件高度（比例）

        参数
        ----
        initial_foot_pos : tuple
            初始脚部位置，用于初始化坐标输入框
        """
        # 调整子图位置，为控制面板留出空间
        # bottom=0.4 表示 3D 图只占据上半部分的 60%
        plt.subplots_adjust(bottom=0.4)

        # ====================================================================
        # 标题
        # ====================================================================
        self.fig.text(0.05, 0.35, '控制面板', fontsize=14, weight='bold')

        # ====================================================================
        # 角度输入区域
        # ====================================================================
        self.fig.text(0.05, 0.31, '关节角度 (度):', fontsize=12, weight='bold')

        # 创建三个文本输入框（垂直排列）
        ax_hip_side = plt.axes([0.05, 0.27, 0.15, 0.03])   # [位置x, 位置y, 宽度, 高度]
        ax_hip_pitch = plt.axes([0.05, 0.23, 0.15, 0.03])
        ax_knee = plt.axes([0.05, 0.19, 0.15, 0.03])

        # TextBox 初始化参数：(坐标轴对象, 标签, 初始值)
        self.text_hip_side = TextBox(ax_hip_side, '髋侧摆: ', initial='0.0')
        self.text_hip_pitch = TextBox(ax_hip_pitch, '髋俯仰: ', initial='0.0')
        self.text_knee = TextBox(ax_knee, '膝俯仰: ', initial='0.0')

        # ====================================================================
        # 坐标输入区域
        # ====================================================================
        self.fig.text(0.05, 0.14, '脚部坐标 (mm):', fontsize=12, weight='bold')

        ax_x = plt.axes([0.05, 0.10, 0.15, 0.03])
        ax_y = plt.axes([0.05, 0.06, 0.15, 0.03])
        ax_z = plt.axes([0.05, 0.02, 0.15, 0.03])

        # 使用初始脚部位置初始化
        self.text_x = TextBox(ax_x, 'X: ', initial=f'{initial_foot_pos[0]:.1f}')
        self.text_y = TextBox(ax_y, 'Y: ', initial=f'{initial_foot_pos[1]:.1f}')
        self.text_z = TextBox(ax_z, 'Z: ', initial=f'{initial_foot_pos[2]:.1f}')

        # ====================================================================
        # 控制按钮
        # ====================================================================
        ax_btn_angles = plt.axes([0.25, 0.25, 0.08, 0.04])
        ax_btn_coords = plt.axes([0.25, 0.12, 0.08, 0.04])
        ax_btn_reset = plt.axes([0.25, 0.06, 0.08, 0.04])

        self.btn_angles = Button(ax_btn_angles, '角度→位置')
        self.btn_coords = Button(ax_btn_coords, '位置→角度')
        self.btn_reset = Button(ax_btn_reset, '复位')

        # ====================================================================
        # 使用说明
        # ====================================================================
        instructions = [
            '使用说明:',
            '1. 输入角度后点击"角度→位置"',
            '2. 输入坐标后点击"位置→角度"',
            '3. 点击"复位"回到默认姿态',
            '',
            '角度范围:',
            '• 髋侧摆: ±90°',
            '• 髋俯仰: ±90°',
            '• 膝俯仰: -180° ~ 0°',
            '',
            '工作范围: 50-295mm'
        ]

        # 垂直排列说明文本
        for i, instruction in enumerate(instructions):
            weight = 'bold' if i == 0 or i == 5 else 'normal'  # 标题加粗
            self.fig.text(0.4, 0.35 - i * 0.02, instruction,
                         fontsize=10, weight=weight)

    def _bind_events(self):
        """
        绑定按钮事件处理

        将按钮点击事件连接到对应的处理函数。
        """
        self.btn_angles.on_clicked(self._update_from_angles)  # 角度→位置
        self.btn_coords.on_clicked(self._update_from_coords)  # 位置→角度
        self.btn_reset.on_clicked(self._reset_position)       # 复位

    def _update_from_angles(self, event):
        """
        从角度更新位置（正运动学）

        读取角度输入框的值，计算脚部位置，更新显示。

        参数
        ----
        event : matplotlib.backend_bases.Event
            按钮点击事件（未使用）
        """
        try:
            # 获取角度输入
            angles = [
                float(self.text_hip_side.text),
                float(self.text_hip_pitch.text),
                float(self.text_knee.text)
            ]

            # 正运动学计算：角度 → 位置
            position = self.controller.calculate_position(*angles, self.leg_side)

            # 更新坐标显示
            self.text_x.set_val(f'{position[0]:.1f}')
            self.text_y.set_val(f'{position[1]:.1f}')
            self.text_z.set_val(f'{position[2]:.1f}')

            # 更新当前状态
            self.current_angles = angles

            # 重绘 3D 视图
            self._redraw()

        except ValueError as e:
            print(f"角度输入错误: {e}")

    def _update_from_coords(self, event):
        """
        从坐标更新角度（逆运动学）

        读取坐标输入框的值，计算关节角度，更新显示。

        参数
        ----
        event : matplotlib.backend_bases.Event
            按钮点击事件（未使用）
        """
        try:
            # 获取坐标输入
            x = float(self.text_x.text)
            y = float(self.text_y.text)
            z = float(self.text_z.text)

            # 逆运动学计算：位置 → 角度
            result = self.controller.solve_position(x, y, z, self.leg_side)

            if result is None:
                print("目标位置不可达")
                return

            # 更新角度显示
            self.text_hip_side.set_val(f'{result[0]:.1f}')
            self.text_hip_pitch.set_val(f'{result[1]:.1f}')
            self.text_knee.set_val(f'{result[2]:.1f}')

            # 更新当前状态
            self.current_angles = list(result)

            # 重绘 3D 视图
            self._redraw()

        except ValueError as e:
            print(f"坐标输入错误: {e}")

    def _reset_position(self, event):
        """
        复位到默认姿态

        将所有关节角度重置为 0，更新显示。

        参数
        ----
        event : matplotlib.backend_bases.Event
            按钮点击事件（未使用）
        """
        # 重置为默认角度（全零）
        self.current_angles = [0.0, 0.0, 0.0]

        # 更新角度显示
        self.text_hip_side.set_val('0.0')
        self.text_hip_pitch.set_val('0.0')
        self.text_knee.set_val('0.0')

        # 计算对应的脚部位置
        position = self.controller.calculate_position(*self.current_angles, self.leg_side)
        self.text_x.set_val(f'{position[0]:.1f}')
        self.text_y.set_val(f'{position[1]:.1f}')
        self.text_z.set_val(f'{position[2]:.1f}')

        # 重绘 3D 视图
        self._redraw()

    def _redraw(self):
        """
        重绘 3D 图像

        根据当前角度重新绘制腿部结构。
        """
        leg_name = '左腿' if self.leg_side == 'left' else '右腿'
        self.visualizer.draw_leg(
            self.ax, *self.current_angles, self.leg_side, leg_name
        )
        # 刷新显示
        plt.draw()


# ============================================================================
# 双腿对比可视化器
# ============================================================================

class DualLegVisualizer:
    """
    双腿对比可视化器

    并排显示左右腿的姿态，便于对比对称性或差异。

    布局
    ----
    ┌─────────────────────┬─────────────────────┐
    │      左腿视图       │      右腿视图       │
    │                     │                     │
    └─────────────────────┴─────────────────────┘

    属性
    ----
    controller : LegController
        腿部控制器（两条腿共享）
    left_visualizer : LegVisualizer
        左腿可视化器
    right_visualizer : LegVisualizer
        右腿可视化器
    """

    def __init__(self, leg_controller: LegController):
        """
        初始化双腿可视化器

        参数
        ----
        leg_controller : LegController
            腿部控制器实例（左右腿共享同一控制器）
        """
        self.controller = leg_controller
        self.left_visualizer = LegVisualizer(leg_controller)
        self.right_visualizer = LegVisualizer(leg_controller)

        # 配置中文字体
        plt.rcParams['font.sans-serif'] = [
            'WenQuanYi Zen Hei', 'DejaVu Sans', 'SimHei', 'Arial Unicode MS'
        ]
        plt.rcParams['axes.unicode_minus'] = False

    def create_comparison_plot(self, left_angles: Tuple[float, float, float],
                             right_angles: Tuple[float, float, float],
                             save_path: Optional[str] = None) -> plt.Figure:
        """
        创建双腿对比图

        并排显示左右腿的 3D 姿态，便于对比。

        参数
        ----
        left_angles : tuple
            左腿角度 (髋侧摆, 髋俯仰, 膝俯仰)（度）
        right_angles : tuple
            右腿角度 (髋侧摆, 髋俯仰, 膝俯仰)（度）
        save_path : str, optional
            图片保存路径，如 'dual_leg_comparison.png'

        返回
        ----
        matplotlib.figure.Figure
            Figure 对象

        示例
        ----
        >>> dual = DualLegVisualizer(controller)
        >>> # 对称姿态对比
        >>> fig = dual.create_comparison_plot(
        ...     (15, -20, -60),   # 左腿向前
        ...     (-15, -20, -60),  # 右腿向前
        ...     'comparison.png'
        ... )
        """
        # 创建图形（16x8 英寸，两个 8x8 的子图）
        fig = plt.figure(figsize=(16, 8))
        fig.suptitle('左右腿对比', fontsize=16)

        # ====================================================================
        # 左腿视图（位置 1 = 左侧）
        # ====================================================================
        ax_left = fig.add_subplot(121, projection='3d')
        # 121 表示 1 行 2 列，第 1 个子图
        self.left_visualizer.draw_leg(ax_left, *left_angles, 'left', '左腿')

        # ====================================================================
        # 右腿视图（位置 2 = 右侧）
        # ====================================================================
        ax_right = fig.add_subplot(122, projection='3d')
        # 122 表示 1 行 2 列，第 2 个子图
        self.right_visualizer.draw_leg(ax_right, *right_angles, 'right', '右腿')

        # 如果指定了保存路径，保存图片
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"对比图已保存: {save_path}")

        return fig


# ============================================================================
# 工厂函数
# ============================================================================

def create_visualizer(leg_controller: LegController, mode: str = 'interactive',
                     leg_side: str = 'left') -> object:
    """
    工厂函数：创建指定类型的可视化器

    根据模式参数返回相应的可视化器实例。

    参数
    ----
    leg_controller : LegController
        腿部控制器实例
    mode : str
        可视化模式：
        - 'interactive': 交互式工具，支持双向转换
        - 'static': 静态图生成器
        - 'dual': 双腿对比显示器
    leg_side : str
        腿侧别，'left' 或 'right'（仅 interactive/static 模式需要）

    返回
    ----
    object
        对应模式的可视化器实例：
        - 'interactive': InteractiveVisualizer
        - 'static': LegVisualizer
        - 'dual': DualLegVisualizer

    异常
    ----
    ValueError
        不支持的可视化模式

    示例
    ----
    >>> # 交互式工具
    >>> viz = create_visualizer(controller, mode='interactive', leg_side='left')
    >>> viz.create_interactive_window()
    >>>
    >>> # 静态图
    >>> static = create_visualizer(controller, mode='static')
    >>> static.create_static_plot(10, -20, -45, 'left')
    >>>
    >>> # 双腿对比
    >>> dual = create_visualizer(controller, mode='dual')
    >>> dual.create_comparison_plot((15, -20, -60), (-15, -20, -60))
    """
    if mode == 'interactive':
        return InteractiveVisualizer(leg_controller, leg_side)
    elif mode == 'static':
        return LegVisualizer(leg_controller)
    elif mode == 'dual':
        return DualLegVisualizer(leg_controller)
    else:
        raise ValueError(
            f"不支持的可视化模式: {mode}\n"
            f"支持的模式: 'interactive', 'static', 'dual'"
        )
