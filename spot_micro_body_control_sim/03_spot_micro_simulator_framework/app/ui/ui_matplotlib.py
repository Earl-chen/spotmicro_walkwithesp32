# app/ui/ui_matplotlib.py
import os
import matplotlib
import sys

# 尝试加载交互式后端
_backend_loaded = False
for backend in ['TkAgg', 'Qt5Agg']:
    try:
        matplotlib.use(backend, force=True)
        _backend_loaded = True
        break
    except ImportError:
        continue

if not _backend_loaded:
    import warnings
    warnings.warn("交互式后端不可用，使用 Agg 非交互模式", RuntimeWarning)

import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button
import numpy as np
import math
from typing import Dict, Any

from app.controller import Controller
from app.robot_model import RobotModel

class MatplotlibUI:
    def __init__(self, controller: Controller, model: RobotModel):
        self.controller = controller
        self.model = model
        
        # 配置中文字体 - 参考 plot_utils.py 的方法
        self.chinese_font = None
        font_candidates = [
            '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
            '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf',
            '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
            '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc',
            '/System/Library/Fonts/PingFang.ttc',  # macOS
            'C:\\Windows\\Fonts\\msyh.ttc',  # Windows
        ]
        
        # 使用 font_manager.fontManager.addfont() 显式添加字体（关键！）
        for font_path in font_candidates:
            if os.path.exists(font_path):
                try:
                    fm.fontManager.addfont(font_path)
                    self.chinese_font = fm.FontProperties(fname=font_path)
                    # 设置全局字体
                    plt.rcParams['font.family'] = self.chinese_font.get_name()
                    plt.rcParams['axes.unicode_minus'] = False
                    break
                except:
                    continue
        
        if self.chinese_font is None:
            self.chinese_font = fm.FontProperties()  # 使用系统默认

        # 颜色配置
        self.leg_colors = {
            'left_front': 'red', 
            'left_back': 'green', 
            'right_front': 'orange', 
            'right_back': 'purple'
        }
        
        self.joint_type_colors = {
            'hip_joint': '#FF1744',
            'hip_extension': '#2196F3', 
            'hip_pitch': '#4CAF50',
            'knee_joint': '#FF9800',
            'foot': '#9C27B0'
        }
        
        # UI 组件
        self.fig = None
        self.ax = None
        self.sliders = {}
        self.joint_sliders = {}  # 关节控制滑块
        self.coord_text = None
        self.control_mode = "inverse"  # "inverse" 或 "forward"
        
        # 文字元素引用，用于模式切换时的显示/隐藏
        self.body_pose_texts = []  # 机体位姿相关文字
        self.joint_control_texts = []  # 关节控制相关文字
        
    def _set_chinese_text(self, ax_or_fig, x, y, text, **kwargs):
        """使用中文字体设置文本的辅助方法"""
        if hasattr(self, 'chinese_font') and self.chinese_font:
            kwargs['fontproperties'] = self.chinese_font
        return ax_or_fig.text(x, y, text, **kwargs)
    
    def _set_chinese_label(self, ax, xlabel=None, ylabel=None, zlabel=None, title=None, **kwargs):
        """使用中文字体设置标签的辅助方法"""
        if hasattr(self, 'chinese_font') and self.chinese_font:
            kwargs['fontproperties'] = self.chinese_font
        if xlabel:
            ax.set_xlabel(xlabel, **kwargs)
        if ylabel:
            ax.set_ylabel(ylabel, **kwargs)
        if zlabel and hasattr(ax, 'set_zlabel'):
            ax.set_zlabel(zlabel, **kwargs)
        if title:
            ax.set_title(title, **kwargs)

    def _set_axes_equal(self, ax):
        """设置3D坐标轴等比例显示"""
        # 收集所有点
        all_points = []
        
        # 添加机体轮廓点
        body_points = self.model.get_body_outline_world()
        all_points.extend(body_points)
        
        # 添加腿部关节点
        for leg_name in self.model.legs.keys():
            joints = self.model.get_leg_joints_world(leg_name)
            all_points.extend(joints)
        
        if not all_points:
            return
            
        pts = np.array(all_points)
        x_limits = (pts[:,0].min(), pts[:,0].max())
        y_limits = (pts[:,1].min(), pts[:,1].max())
        z_limits = (pts[:,2].min(), pts[:,2].max())

        x_range = x_limits[1] - x_limits[0]
        y_range = y_limits[1] - y_limits[0]
        z_range = z_limits[1] - z_limits[0]
        max_range = max(x_range, y_range, z_range)
        
        # 扩展范围以容纳坐标轴
        max_range = max(max_range, 0.2)  # 至少200mm范围
        
        x_middle = np.mean(x_limits)
        y_middle = np.mean(y_limits)
        z_middle = np.mean(z_limits)

        ax.set_xlim(x_middle - max_range/2, x_middle + max_range/2)
        ax.set_ylim(y_middle - max_range/2, y_middle + max_range/2)
        ax.set_zlim(z_middle - max_range/2, z_middle + max_range/2)
        
    def _draw_robot_in_world(self, ax):
        """在世界坐标系中绘制机器人"""
        # 绘制世界坐标轴 (红色)
        world_origin = [0, 0, 0]
        world_x = [0.15, 0, 0]
        world_y = [0, 0.15, 0]  
        world_z = [0, 0, 0.15]
        
        ax.plot([world_origin[0], world_x[0]], [world_origin[1], world_x[1]], 
                [world_origin[2], world_x[2]], 'r-', linewidth=3, label='世界X轴')
        ax.plot([world_origin[0], world_y[0]], [world_origin[1], world_y[1]], 
                [world_origin[2], world_y[2]], 'r-', linewidth=3, label='世界Y轴')
        ax.plot([world_origin[0], world_z[0]], [world_origin[1], world_z[1]], 
                [world_origin[2], world_z[2]], 'r-', linewidth=3, label='世界Z轴')
        
        # 绘制机体坐标轴 (蓝色)
        body_node = self.model.frame_manager.nodes[self.model.body_frame]
        body_origin = body_node.rel.transform_point_body_to_world([0, 0, 0])
        body_x = body_node.rel.transform_point_body_to_world([0.1, 0, 0])
        body_y = body_node.rel.transform_point_body_to_world([0, 0.1, 0])
        body_z = body_node.rel.transform_point_body_to_world([0, 0, 0.1])
        
        ax.plot([body_origin[0], body_x[0]], [body_origin[1], body_x[1]], 
                [body_origin[2], body_x[2]], 'b-', linewidth=2, label='机体X轴')
        ax.plot([body_origin[0], body_y[0]], [body_origin[1], body_y[1]], 
                [body_origin[2], body_y[2]], 'b-', linewidth=2, label='机体Y轴')
        ax.plot([body_origin[0], body_z[0]], [body_origin[1], body_z[1]], 
                [body_origin[2], body_z[2]], 'b-', linewidth=2, label='机体Z轴')
        
        # 绘制身体框架
        body_points = self.model.get_body_outline_world()
        if body_points:
            body_array = np.array(body_points)
            ax.plot(body_array[:,0], body_array[:,1], body_array[:,2], 
                   color='black', linewidth=4, label='身体框架')
        
        # 绘制腿部
        joint_type_names = ['hip_joint', 'hip_extension', 'hip_pitch', 'knee_joint', 'foot']
        
        for leg_name in self.model.legs.keys():
            joints = self.model.get_leg_joints_world(leg_name)
            if joints:
                ja = np.array(joints)
                for i in range(len(joints)-1):
                    joint_type = joint_type_names[i]
                    ax.plot([ja[i,0], ja[i+1,0]], 
                           [ja[i,1], ja[i+1,1]], 
                           [ja[i,2], ja[i+1,2]], 
                           marker='o', linewidth=3, markersize=6,
                           color=self.joint_type_colors[joint_type])
        
        # 添加图例
        ax.legend(loc='upper right', bbox_to_anchor=(0.98, 0.98), fontsize=10)

    def create_world_control_interface(self):
        """
        创建完整的四足机器人世界坐标系控制界面 - 大气中文版本
        """
        # 创建更大窗口
        self.fig = plt.figure(figsize=(20, 12))
        self.fig.canvas.manager.set_window_title('四足机器人世界坐标系控制 - 模块化重构版')
        
        # 3D视图位置 - 占据更大区域
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_position([0.45, 0.3, 0.52, 0.65])  # 更大的3D视图
        
        # 初始绘制
        self._draw_robot_in_world(self.ax)
        
        self._set_chinese_label(self.ax, 
                                xlabel='X (世界坐标)', 
                                ylabel='Y (世界坐标)', 
                                zlabel='Z (世界坐标)', 
                                title='四足机器人世界坐标系控制\n红色:世界坐标轴  蓝色:机体坐标轴',
                                fontsize=14)
        # 单独设置title的字体大小
        self._set_chinese_label(self.ax, title='四足机器人世界坐标系控制\n红色:世界坐标轴  蓝色:机体坐标轴', fontsize=16, pad=20)
        
        # 设置初始视角
        self._set_axes_equal(self.ax)
        self.ax.view_init(elev=20, azim=45)
        
        # 创建滑块 - 中文版本
        self._create_chinese_sliders()
        
        # 创建关节控制滑块（默认隐藏）
        self._create_joint_control_sliders()
        self._hide_joint_sliders()  # 默认隐藏关节滑块
        
        # 创建模式切换按钮
        self._create_mode_switch_button()
        
        # 创建状态显示 - 中文版本
        self._create_chinese_status_display()
        
        print("四足机器人世界坐标系控制界面已打开！")
        print("功能特点：")
        print("  * 脚步世界坐标始终固定不变")
        print("  * 模块化重构架构")
        print("  * 机体位姿变化时自动逆运动学计算")
        
    def _create_chinese_sliders(self):
        """创建大气的中文控制滑块布局"""
        self.sliders = {}
        
        # === 机体位姿控制区域 - 左侧大区域 ===
        # 标题背景
        title_bg = plt.Rectangle((0.02, 0.85), 0.4, 0.12, 
                               facecolor='lightblue', alpha=0.4, 
                               transform=self.fig.transFigure)
        self.fig.patches.append(title_bg)
        
        # 保存机体位姿相关文字引用
        main_title = self._set_chinese_text(self.fig, 0.22, 0.91, '机体位姿控制', fontsize=18, weight='bold', 
                ha='center', va='center')
        self.body_pose_texts.append(main_title)
        
        # 位置控制组
        pos_title = self._set_chinese_text(self.fig, 0.04, 0.82, '位置控制 (m):', fontsize=14, weight='bold')
        self.body_pose_texts.append(pos_title)
        position_controls = [
            ('body_x', 'X位置', -0.5, 0.5, 0),
            ('body_y', 'Y位置', -0.5, 0.5, 0),
            ('body_z', 'Z位置', -0.3, 0.1, -0.1)  # 初始值 -0.1，站立姿态
        ]
        
        slider_width = 0.35
        slider_height = 0.025
        
        for i, (key, label, min_val, max_val, init_val) in enumerate(position_controls):
            x_pos = 0.04
            y_pos = 0.76 - i * 0.05
            
            ax_slider = plt.axes([x_pos, y_pos, slider_width, slider_height])
            slider = Slider(ax_slider, f'{label}', min_val, max_val,
                          valinit=init_val, valfmt='%.3f m')
            slider.label.set_fontsize(12)
            self.sliders[key] = (slider, key)
        
        # 姿态控制组
        orient_title = self._set_chinese_text(self.fig, 0.04, 0.58, '姿态控制 (°):', fontsize=14, weight='bold')
        self.body_pose_texts.append(orient_title)
        orientation_controls = [
            ('body_roll', 'Roll角', -45, 45, 0),
            ('body_pitch', 'Pitch角', -45, 45, 0),
            ('body_yaw', 'Yaw角', -180, 180, 0)
        ]
        
        for i, (key, label, min_val, max_val, init_val) in enumerate(orientation_controls):
            x_pos = 0.04
            y_pos = 0.52 - i * 0.05
            
            ax_slider = plt.axes([x_pos, y_pos, slider_width, slider_height])
            slider = Slider(ax_slider, f'{label}', min_val, max_val,
                          valinit=init_val, valfmt='%.1f°')
            slider.label.set_fontsize(12)
            self.sliders[key] = (slider, key)
        
        # 控制说明
        instructions = [
            '控制说明:',
            '* 拖动滑块控制机体在世界坐标系中的位姿',
            '* 脚部位置在世界坐标系中保持锁定', 
            '* 逆运动学自动计算关节角度',
            '* 实时可视化显示坐标系变换',
            '',
            '控制范围:',
            '* 位置: X/Y ±500mm, Z -300~100mm',
            '* 姿态: Roll/Pitch ±45°, Yaw ±180°'
        ]
        
        for i, instruction in enumerate(instructions):
            style = 'bold' if i == 0 or i == 6 else 'normal'
            size = 12 if i == 0 or i == 6 else 10
            self._set_chinese_text(self.fig, 0.04, 0.32 - i * 0.025, instruction,
                        fontsize=size, weight=style)
        
        # 绑定更新函数
        def update_robot(val):
            x = self.sliders['body_x'][0].val
            y = self.sliders['body_y'][0].val
            z = self.sliders['body_z'][0].val
            roll = self.sliders['body_roll'][0].val
            pitch = self.sliders['body_pitch'][0].val
            yaw = self.sliders['body_yaw'][0].val
            
            # 设置机体位姿（度数转弧度）
            self.controller.set_body_pose(x, y, z, roll, pitch, yaw, radians=False)
            
            # 重绘机器人
            self.ax.clear()
            self._draw_robot_in_world(self.ax)
            
            self._set_chinese_label(self.ax, 
                                    xlabel='X (世界坐标)', 
                                    ylabel='Y (世界坐标)', 
                                    zlabel='Z (世界坐标)', 
                                    title='四足机器人世界坐标系控制\n红色:世界坐标轴  蓝色:机体坐标轴',
                                    fontsize=14)
            # 单独设置title的字体大小  
            self._set_chinese_label(self.ax, title='四足机器人世界坐标系控制\n红色:世界坐标轴  蓝色:机体坐标轴', fontsize=16, pad=20)
            
            # 更新状态显示
            self._update_status_display()
            
            # 重新设置坐标轴
            self._set_axes_equal(self.ax)
            self.fig.canvas.draw()
        
        # 绑定所有滑块
        for slider_data in self.sliders.values():
            slider_data[0].on_changed(update_robot)

        # 初始化：用滑块的初始值触发一次更新，让机器人状态与滑块一致
        print(f"\n[UI] 滑块初始值:")
        print(f"  body_x={self.sliders['body_x'][0].val:.3f}, "
              f"body_y={self.sliders['body_y'][0].val:.3f}, "
              f"body_z={self.sliders['body_z'][0].val:.3f}")
        print(f"  roll={self.sliders['body_roll'][0].val:.1f}°, "
              f"pitch={self.sliders['body_pitch'][0].val:.1f}°, "
              f"yaw={self.sliders['body_yaw'][0].val:.1f}°")
        print(f"\n[UI] 触发 update_robot(None) 进行初始化...")
        update_robot(None)
    
    def _create_joint_control_sliders(self):
        """创建关节角度控制滑块（正向运动学模式）"""
        self.joint_sliders = {}
        
        # === 关节控制区域 ===
        # 关节控制标题
        joint_main_title = self._set_chinese_text(self.fig, 0.04, 0.65, '关节角度控制 (正向运动学)', 
                              fontsize=16, weight='bold',
                              bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.7))
        self.joint_control_texts.append(joint_main_title)
        
        # 关节滑块布局参数
        leg_names = ['left_front', 'left_back', 'right_front', 'right_back'] 
        leg_display_names = ['左前腿', '左后腿', '右前腿', '右后腿']
        joint_names = ['髋侧摆', '髋俯仰', '膝俯仰']
        
        slider_width = 0.08
        slider_height = 0.015
        
        # 绘制关节滑块
        for i, (leg_name, leg_display) in enumerate(zip(leg_names, leg_display_names)):
            # 腿部标题
            base_x = 0.04 + (i % 2) * 0.2  # 左右两列
            base_y = 0.58 - (i // 2) * 0.15 # 上下两行
            
            leg_title = self._set_chinese_text(self.fig, base_x, base_y, leg_display, 
                                  fontsize=11, weight='bold')
            self.joint_control_texts.append(leg_title)
            
            # 每条腿的3个关节滑块
            for j, joint_name in enumerate(joint_names):
                x_pos = base_x
                y_pos = base_y - (j + 1) * 0.03
                
                ax_slider = plt.axes([x_pos, y_pos, slider_width, slider_height])
                
                # 设置角度范围
                if j == 0 or j == 1:  # 髋侧摆、髋俯仰
                    min_angle, max_angle = -90, 90
                else:  # 膝俯仰  
                    min_angle, max_angle = -180, 0
                
                slider = Slider(ax_slider, joint_name, min_angle, max_angle,
                              valinit=0, valfmt='%d°')
                slider.label.set_fontsize(9)
                
                key = f"{leg_name}_{j}"
                self.joint_sliders[key] = (slider, leg_name, j)
        
        # 绑定关节滑块更新函数
        def update_joints(val):
            if self.control_mode == "forward":
                # 收集所有关节角度
                joint_angles = {}
                for leg_name in leg_names:
                    angles = []
                    for j in range(3):  # 3个关节
                        key = f"{leg_name}_{j}"
                        if key in self.joint_sliders:
                            angle = self.joint_sliders[key][0].val
                            angles.append(angle)
                    if len(angles) == 3:
                        joint_angles[leg_name] = angles
                
                # 设置关节角度
                self.controller.set_all_joint_angles(joint_angles, radians=False)
                
                # 重绘机器人
                self.ax.clear()
                self._draw_robot_in_world(self.ax)
                
                self._set_chinese_label(self.ax, 
                                        xlabel='X (世界坐标)', 
                                        ylabel='Y (世界坐标)', 
                                        zlabel='Z (世界坐标)', 
                                        title='四足机器人正向运动学控制\n红色:世界坐标轴  蓝色:机体坐标轴',
                                        fontsize=14)
                self._set_chinese_label(self.ax, title='四足机器人正向运动学控制\n红色:世界坐标轴  蓝色:机体坐标轴', fontsize=16, pad=20)
                
                # 更新状态显示
                self._update_status_display()
                
                # 重新设置坐标轴
                self._set_axes_equal(self.ax)
                self.fig.canvas.draw()
        
        # 绑定所有关节滑块
        for slider_data in self.joint_sliders.values():
            slider_data[0].on_changed(update_joints)
    
    def _create_mode_switch_button(self):
        """创建模式切换按钮"""
        # 按钮位置
        ax_button = plt.axes([0.02, 0.02, 0.15, 0.04])
        self.mode_button = Button(ax_button, '切换到正向运动学')
        
        def switch_mode(event):
            if self.control_mode == "inverse":
                self._switch_to_forward_mode()
            else:
                self._switch_to_inverse_mode()
        
        self.mode_button.on_clicked(switch_mode)
    
    def _switch_to_forward_mode(self):
        """切换到正向运动学模式"""
        self.control_mode = "forward"
        self.controller.enable_forward_kinematics_mode()
        self.mode_button.label.set_text('切换到逆运动学')
        
        # 显示关节滑块
        self._show_joint_sliders()
        # 隐藏机体位姿滑块
        self._hide_body_pose_sliders()
        
        # 同步当前关节角度到滑块
        self._sync_joint_sliders_from_model()
        
        # 更新界面标题
        self._set_chinese_label(self.ax, title='四足机器人正向运动学控制\n红色:世界坐标轴  蓝色:机体坐标轴', fontsize=16, pad=20)
        self.fig.canvas.draw()
        print("已切换到正向运动学模式")
    
    def _switch_to_inverse_mode(self):
        """切换到逆运动学模式"""
        self.control_mode = "inverse"
        self.controller.enable_inverse_kinematics_mode()
        self.mode_button.label.set_text('切换到正向运动学')
        
        # 隐藏关节滑块
        self._hide_joint_sliders()
        # 显示机体位姿滑块
        self._show_body_pose_sliders()
        
        # 更新界面标题
        self._set_chinese_label(self.ax, title='四足机器人世界坐标系控制\n红色:世界坐标轴  蓝色:机体坐标轴', fontsize=16, pad=20)
        self.fig.canvas.draw()
        print("已切换到逆运动学模式")
    
    def _show_joint_sliders(self):
        """显示关节滑块和相关文字"""
        for slider_data in self.joint_sliders.values():
            slider_data[0].ax.set_visible(True)
        for text in self.joint_control_texts:
            text.set_visible(True)
    
    def _hide_joint_sliders(self):
        """隐藏关节滑块和相关文字"""
        for slider_data in self.joint_sliders.values():
            slider_data[0].ax.set_visible(False)
        for text in self.joint_control_texts:
            text.set_visible(False)
    
    def _show_body_pose_sliders(self):
        """显示机体位姿滑块和相关文字"""
        for slider_data in self.sliders.values():
            slider_data[0].ax.set_visible(True)
        for text in self.body_pose_texts:
            text.set_visible(True)
    
    def _hide_body_pose_sliders(self):
        """隐藏机体位姿滑块和相关文字"""
        for slider_data in self.sliders.values():
            slider_data[0].ax.set_visible(False)
        for text in self.body_pose_texts:
            text.set_visible(False)
    
    def _sync_joint_sliders_from_model(self):
        """从模型同步关节角度到滑块"""
        joint_angles = self.controller.get_all_joint_angles(radians=False)
        
        for leg_name, angles in joint_angles.items():
            for j, angle in enumerate(angles):
                key = f"{leg_name}_{j}"
                if key in self.joint_sliders:
                    self.joint_sliders[key][0].set_val(angle)
    
    def _create_chinese_status_display(self):
        """创建大气的中文状态显示区域 - 分成左右两栏，调整高度"""
        # 调整背景区域高度和位置
        status_bg = plt.Rectangle((0.45, 0.05), 0.53, 0.22, 
                                 facecolor='lightgray', alpha=0.3, 
                                 transform=self.fig.transFigure)
        self.fig.patches.append(status_bg)
        
        # 标题位置调整
        self._set_chinese_text(self.fig, 0.715, 0.26, '机器人实时状态监控', fontsize=15, weight='bold', 
                ha='center', va='center', bbox=dict(boxstyle="round,pad=0.3", facecolor="orange", alpha=0.8))
        
        # 左栏：机体状态和关节角度 - 调整位置和字体
        self.status_text_left = self._set_chinese_text(self.fig, 0.47, 0.23, self._get_left_status_text(), 
                             fontsize=9, ha='left', va='top', family='monospace',
                             bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.9))
        
        # 右栏：脚部坐标和说明 - 调整位置和字体
        self.status_text_right = self._set_chinese_text(self.fig, 0.73, 0.23, self._get_right_status_text(), 
                             fontsize=9, ha='left', va='top', family='monospace',
                             bbox=dict(boxstyle="round,pad=0.3", facecolor="lightyellow", alpha=0.9))
    
    def _update_status_display(self):
        """更新状态显示 - 分别更新左右两栏"""
        # 更新左栏
        if hasattr(self, 'status_text_left') and self.status_text_left:
            new_left_text = self._get_left_status_text()
            if hasattr(self, 'chinese_font') and self.chinese_font:
                self.status_text_left.set_fontproperties(self.chinese_font)
            self.status_text_left.set_text(new_left_text)
        
        # 更新右栏  
        if hasattr(self, 'status_text_right') and self.status_text_right:
            new_right_text = self._get_right_status_text()
            if hasattr(self, 'chinese_font') and self.chinese_font:
                self.status_text_right.set_fontproperties(self.chinese_font)
            self.status_text_right.set_text(new_right_text)
    
    def _get_left_status_text(self):
        """获取左栏状态文本：机体位姿和关节角度 - 压缩版本"""
        pose = self.controller.get_body_pose()
        
        # 显示当前控制模式
        if self.control_mode == "inverse":
            text = "控制模式: [逆运动学] (脚步固定)\n\n"
        else:
            text = "控制模式: [正向运动学] (关节控制)\n\n"
        
        text += "机体位姿状态:\n"
        text += f"位置: X={pose['position'][0]*1000:+6.1f} Y={pose['position'][1]*1000:+6.1f} Z={pose['position'][2]*1000:+6.1f}mm\n"
        text += f"姿态: R={pose['orientation_deg'][0]:+5.1f}° P={pose['orientation_deg'][1]:+5.1f}° Y={pose['orientation_deg'][2]:+5.1f}°\n\n"
        
        # 简化腿部关节角度显示 - 每腿一行
        text += "腿部关节角度 (度):\n"
        leg_names_chinese = {
            'left_front': '左前', 'left_back': '左后',
            'right_front': '右前', 'right_back': '右后'
        }
        
        for leg_name, joints in self.model.joints.items():
            chinese_name = leg_names_chinese[leg_name]
            text += f"{chinese_name}: 髋{math.degrees(joints.hip_side):+4.1f}° {math.degrees(joints.hip_pitch):+4.1f}° 膝{math.degrees(joints.knee_pitch):+4.1f}°\n"
        
        return text
    
    def _get_right_status_text(self):
        """获取右栏状态文本：脚部坐标和说明 - 压缩版本"""
        foot_pos = self.controller.get_foot_positions_world()
        
        text = "脚部世界坐标 (mm) [LOCKED]:\n"
        leg_names_chinese = {
            'left_front': '左前', 'left_back': '左后',
            'right_front': '右前', 'right_back': '右后'
        }
        
        # 简化脚部坐标显示 - 每腿一行
        for leg_name, pos in foot_pos.items():
            chinese_name = leg_names_chinese[leg_name]
            text += f"{chinese_name}: X={pos[0]*1000:+5.1f} Y={pos[1]*1000:+5.1f} Z={pos[2]*1000:+5.1f}\n"
        
        text += "\n功能说明:\n"
        text += "• 脚部坐标锁定 [LOCKED]\n"
        text += "• 逆运动学自动计算\n"
        text += "• 实时关节角度调整\n"
        
        text += "\n坐标轴说明:\n"
        text += "• 红线: 世界坐标轴\n"
        text += "• 蓝线: 机体坐标轴"
        
        return text

    def start(self):
        """启动界面"""
        import math

        # 打印初始状态（关节角度 = 0）
        print("\n=== 界面启动前的状态 ===")
        for leg_name, joints in self.model.joints.items():
            print(f"{leg_name}: hip_side={math.degrees(joints.hip_side):.1f}°, "
                  f"hip_pitch={math.degrees(joints.hip_pitch):.1f}°, "
                  f"knee={math.degrees(joints.knee_pitch):.1f}°")

        # 先启用脚步锁定（捕获关节=0时的脚部位置）
        print("\n=== 启用脚步锁定（捕获初始脚部位置）===")
        self.controller.capture_fixed_feet()

        # 打印脚步锁定后的状态（应该还是0度）
        print("\n=== 脚步锁定后的状态 ===")
        for leg_name, joints in self.model.joints.items():
            print(f"{leg_name}: hip_side={math.degrees(joints.hip_side):.1f}°, "
                  f"hip_pitch={math.degrees(joints.hip_pitch):.1f}°, "
                  f"knee={math.degrees(joints.knee_pitch):.1f}°")

        # 然后创建界面并触发初始化更新（此时脚步锁定已启用，会触发IK）
        print("\n=== 创建界面并初始化滑块 ===")
        self.create_world_control_interface()

        # 打印滑块初始化后的状态（应该通过IK计算出新的关节角度）
        print("\n=== 滑块初始化后的状态 ===")
        for leg_name, joints in self.model.joints.items():
            print(f"{leg_name}: hip_side={math.degrees(joints.hip_side):.1f}°, "
                  f"hip_pitch={math.degrees(joints.hip_pitch):.1f}°, "
                  f"knee={math.degrees(joints.knee_pitch):.1f}°")

        # 显示
        plt.show()

    def close(self):
        """关闭界面并释放资源"""
        if self.fig is not None:
            plt.close(self.fig)
            self.fig = None
            self.ax = None
            self.sliders.clear()
            self.joint_sliders.clear()
            self.body_pose_texts.clear()
            self.joint_control_texts.clear()
            print("UI 资源已释放")