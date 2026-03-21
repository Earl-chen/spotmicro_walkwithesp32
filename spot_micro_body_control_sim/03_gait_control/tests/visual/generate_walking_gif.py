#!/usr/bin/env python3
"""
四足机器人行走动画生成器（高质量版）

生成双视图 GIF 动画：
- 左图：俯视图（身体 + 四条腿 + 轨迹线）
- 右图：Z 轴高度随时间变化曲线

适配当前 WalkGait API（2026-03-21 更新）
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle
from matplotlib import font_manager as fm

# 导入通用中文字体配置模块
module_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, module_root)
from gait_algo_core.walk_gait import WalkGait
sys.path.insert(0, os.path.join(module_root, 'tests'))
from chinese_font_config import setup_chinese_font

# 配置中文字体
chinese_font = setup_chinese_font()


def count_support_legs(gait, leg_names):
    """
    统计当前支撑腿数量
    
    Args:
        gait: WalkGait 实例
        leg_names: 腿名称列表
    
    Returns:
        int: 支撑腿数量
    """
    count = 0
    for leg_name in leg_names:
        phase = gait.get_leg_phase(leg_name)
        if phase >= 0.25:  # 支撑相
            count += 1
    return count


def create_walking_animation(output_file=None):
    """
    创建行走动画
    
    Args:
        output_file: 输出文件名
    """
    # 默认输出到脚本所在目录
    if output_file is None:
        output_file = os.path.join(os.getcwd(), 'walking_animation.gif')
    
    print("生成行走动画...")
    
    # 创建步态控制器
    gait = WalkGait(stride_length=0.05, step_height=0.03, frequency=1.0)
    
    dt = 0.02
    frames = 100  # 2秒动画
    
    # 腿名称
    leg_names = ['right_front', 'left_back', 'left_front', 'right_back']
    
    # 创建图形
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    
    # === 左图：3D 俯视图 ===
    ax1.set_xlim(-200, 200)
    ax1.set_ylim(-150, 150)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3, linestyle='--')
    ax1.set_xlabel('X 轴 (mm)', fontsize=12, fontproperties=chinese_font)
    ax1.set_ylabel('Y 轴 (mm)', fontsize=12, fontproperties=chinese_font)
    ax1.set_title('Walk 步态 - 俯视图', fontsize=14, fontweight='bold', fontproperties=chinese_font)
    
    # 机器人身体
    body_width = 78
    body_length = 207.5
    body = Rectangle((-body_length/2, -body_width/2), 
                    body_length, body_width,
                    fill=True, facecolor='lightblue',
                    edgecolor='navy', linewidth=3, alpha=0.7)
    ax1.add_patch(body)
    
    # 身体中心标记
    center = Circle((0, 0), 5, color='red', zorder=10)
    ax1.add_patch(center)
    
    # 腿部基础位置（X, Y）
    leg_base_positions = {
        'right_front': (body_length/2, -body_width/2),   # 右前
        'left_back': (-body_length/2, body_width/2),     # 左后
        'left_front': (body_length/2, body_width/2),     # 左前
        'right_back': (-body_length/2, -body_width/2),   # 右后
    }
    
    leg_colors = {
        'right_front': '#4ECDC4',   # 青色
        'left_back': '#45B7D1',     # 蓝色
        'left_front': '#FF6B6B',    # 红色
        'right_back': '#FFA07A'     # 橙色
    }
    
    leg_labels = {
        'right_front': '右前腿',
        'left_back': '左后腿',
        'left_front': '左前腿',
        'right_back': '右后腿'
    }
    
    # 初始化腿部绘图元素
    leg_points = {}
    leg_trails = {}
    
    for leg_name in leg_names:
        base_x, base_y = leg_base_positions[leg_name]
        # 当前位置点
        point, = ax1.plot(base_x, base_y, 'o', color=leg_colors[leg_name],
                         markersize=18, label=leg_labels[leg_name],
                         markeredgecolor='black', markeredgewidth=2)
        leg_points[leg_name] = point
        
        # 轨迹线
        trail, = ax1.plot([], [], '-', color=leg_colors[leg_name],
                         alpha=0.3, linewidth=2)
        leg_trails[leg_name] = trail
    
    ax1.legend(loc='upper left', fontsize=10, framealpha=0.9, prop=chinese_font)
    
    # === 右图：Z 轴高度曲线 ===
    ax2.set_xlim(0, frames * dt)
    ax2.set_ylim(-5, 35)
    ax2.grid(True, alpha=0.3, linestyle='--')
    ax2.set_xlabel('时间 (s)', fontsize=12, fontproperties=chinese_font)
    ax2.set_ylabel('Z 轴高度 (mm)', fontsize=12, fontproperties=chinese_font)
    ax2.set_title('垂直轨迹（Z 轴）', fontsize=14, fontweight='bold', fontproperties=chinese_font)
    
    # 初始化轨迹曲线
    trajectory_lines = {}
    time_data = []
    z_data = {leg: [] for leg in leg_names}
    
    for leg_name in leg_names:
        line, = ax2.plot([], [], '-', color=leg_colors[leg_name],
                        linewidth=2, label=leg_labels[leg_name])
        trajectory_lines[leg_name] = line
    
    ax2.legend(loc='upper left', fontsize=10, framealpha=0.9, prop=chinese_font)
    
    # 文本显示
    info_text = ax1.text(0.02, 0.98, '', transform=ax1.transAxes,
                        fontsize=11, verticalalignment='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    def init():
        """初始化函数"""
        for leg_name in leg_names:
            leg_points[leg_name].set_data([], [])
            leg_trails[leg_name].set_data([], [])
            trajectory_lines[leg_name].set_data([], [])
        info_text.set_text('')
        return list(leg_points.values()) + list(leg_trails.values()) + \
               list(trajectory_lines.values()) + [info_text]
    
    # 存储历史轨迹
    trail_history = {leg: {'x': [], 'y': []} for leg in leg_names}
    
    def update(frame):
        """更新函数"""
        nonlocal time_data, z_data, trail_history
        
        # 更新步态
        gait.update(dt)
        
        current_time = frame * dt
        time_data.append(current_time)
        
        # 更新每条腿
        for leg_name in leg_names:
            base_x, base_y = leg_base_positions[leg_name]
            
            # 获取轨迹点（返回 x, z tuple）
            x_offset, z_offset = gait.get_foot_trajectory(leg_name)
            
            # 计算新位置（基准 + 偏移）
            new_x = base_x + x_offset * 1000  # X 轴偏移
            new_y = base_y  # Y 轴不变（无侧向偏移）
            
            # 更新位置点
            leg_points[leg_name].set_data([new_x], [new_y])
            
            # 更新轨迹历史
            trail_history[leg_name]['x'].append(new_x)
            trail_history[leg_name]['y'].append(new_y)
            
            # 保留最近50帧的轨迹
            if len(trail_history[leg_name]['x']) > 50:
                trail_history[leg_name]['x'] = trail_history[leg_name]['x'][-50:]
                trail_history[leg_name]['y'] = trail_history[leg_name]['y'][-50:]
            
            # 更新轨迹线
            leg_trails[leg_name].set_data(trail_history[leg_name]['x'],
                                         trail_history[leg_name]['y'])
            
            # 更新Z轴轨迹曲线
            z_data[leg_name].append(z_offset * 1000)
            trajectory_lines[leg_name].set_data(time_data, z_data[leg_name])
        
        # 更新信息文本
        phase = gait.global_phase  # 使用属性
        support_legs = count_support_legs(gait, leg_names)
        
        info = f'相位: {phase:.2f}\n'
        info += f'支撑腿: {support_legs}/4'
        
        info_text.set_text(info)
        
        # 动态调整X轴范围
        if current_time > 1.5:
            ax2.set_xlim(current_time - 1.5, current_time + 0.5)
        
        return list(leg_points.values()) + list(leg_trails.values()) + \
               list(trajectory_lines.values()) + [info_text]
    
    # 创建动画
    anim = FuncAnimation(fig, update, init_func=init,
                        frames=frames, interval=33, blit=True)
    
    # 保存GIF
    print(f"正在生成 {output_file}...")
    anim.save(output_file, writer='pillow', fps=30, dpi=100)
    print(f"✅ 已保存: {output_file}")
    
    plt.close()
    
    return output_file


def main():
    """主函数"""
    print("=" * 60)
    print("四足机器人行走动画生成器")
    print("=" * 60)
    
    # 保存到当前工作目录
    output_file = os.path.join(os.getcwd(), 'walk_hq.gif')
    
    # 生成动画
    create_walking_animation(output_file)
    
    print("\n" + "=" * 60)
    print("✅ 动画生成完成！")
    print("=" * 60)
    
    # 显示文件大小
    if os.path.exists(output_file):
        size = os.path.getsize(output_file) / 1024 / 1024  # MB
        print(f"  walk_hq.gif: {size:.2f} MB")


if __name__ == '__main__':
    main()
