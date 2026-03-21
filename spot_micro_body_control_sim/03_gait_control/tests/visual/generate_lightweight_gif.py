#!/usr/bin/env python3
"""
轻量版行走动画生成器

生成小体积 GIF，适合飞书发送

适配当前 WalkGait API（2026-03-21 更新）
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle

module_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, module_root)
from gait_algo_core.walk_gait import WalkGait

# 使用英文标签避免字体问题
plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


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


def create_lightweight_walking_gif(output_file=None):
    """
    创建轻量级行走动画
    
    Args:
        output_file: 输出文件名
    """
    # 默认输出到脚本所在目录
    if output_file is None:
        output_file = os.path.join(os.getcwd(), 'walk.gif')
    
    print("生成行走 GIF...", end=' ')
    
    # 创建步态控制器
    gait = WalkGait(stride_length=0.05, step_height=0.03, frequency=1.0)
    
    dt = 0.04  # 降低帧率
    frames = 50  # 减少帧数
    
    # 腿名称映射
    leg_names = ['right_front', 'left_back', 'left_front', 'right_back']
    
    # 创建图形（更小尺寸）
    fig, ax = plt.subplots(figsize=(8, 6))
    
    ax.set_xlim(-180, 180)
    ax.set_ylim(-120, 120)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3, linestyle='--')
    
    ax.set_title('Walk Gait - Straight', fontsize=14, fontweight='bold')
    ax.set_xlabel('X (mm)', fontsize=11)
    ax.set_ylabel('Z (mm)', fontsize=11)
    
    # 机器人身体
    body_width = 78
    body_length = 207.5
    body = Rectangle((-body_length/2, -body_width/2), 
                    body_length, body_width, 
                    fill=False, edgecolor='black', linewidth=2)
    ax.add_patch(body)
    
    # 腿部基础位置（X, Z）
    leg_base_positions = {
        'right_front': (body_length/2, 0),      # 右前
        'left_back': (-body_length/2, 0),       # 左后
        'left_front': (body_length/2, 0),       # 左前
        'right_back': (-body_length/2, 0),      # 右后
    }
    
    leg_labels = {
        'right_front': 'Right Front',
        'left_back': 'Left Back',
        'left_front': 'Left Front',
        'right_back': 'Right Back'
    }
    
    leg_colors = {
        'right_front': '#4ECDC4',   # 青色
        'left_back': '#45B7D1',     # 蓝色
        'left_front': '#FF6B6B',    # 红色
        'right_back': '#FFA07A'     # 橙色
    }
    
    # 初始化腿部点
    leg_points = {}
    for leg_name in leg_names:
        x, z = leg_base_positions[leg_name]
        point, = ax.plot(x, z, 'o', color=leg_colors[leg_name], 
                        markersize=12, label=leg_labels[leg_name])
        leg_points[leg_name] = point
    
    ax.legend(loc='upper left', fontsize=9, framealpha=0.9)
    
    # 信息文本
    info_text = ax.text(0.02, 0.02, '', transform=ax.transAxes,
                        fontsize=10, verticalalignment='bottom',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    def init():
        return list(leg_points.values()) + [info_text]
    
    def update(frame):
        gait.update(dt)
        phase = gait.global_phase  # 使用属性，不是方法
        support_legs = count_support_legs(gait, leg_names)
        
        for leg_name in leg_names:
            base_x, base_z = leg_base_positions[leg_name]
            x_offset, z_offset = gait.get_foot_trajectory(leg_name)  # 返回 (x, z) tuple
            new_x = base_x + x_offset * 1000  # 转换为 mm
            new_z = base_z + z_offset * 1000  # 转换为 mm
            leg_points[leg_name].set_data([new_x], [new_z])
        
        info_text.set_text(f'Phase: {phase:.2f} | Support: {support_legs}/4')
        
        return list(leg_points.values()) + [info_text]
    
    # 创建动画
    anim = FuncAnimation(fig, update, init_func=init,
                        frames=frames, interval=80, blit=True)
    
    # 保存 GIF（优化参数）
    anim.save(output_file, writer='pillow', fps=12, dpi=80)
    plt.close()
    
    print(f"✅ {os.path.basename(output_file)}")
    
    return output_file


def main():
    print("=" * 60)
    print("生成轻量级行走 GIF")
    print("=" * 60)
    
    # 保存到当前工作目录
    output_file = os.path.join(os.getcwd(), 'walk_straight.gif')
    
    print()
    create_lightweight_walking_gif(output_file)
    
    print("\n" + "=" * 60)
    print("✅ 完成！")
    print("=" * 60)
    
    # 显示文件大小
    if os.path.exists(output_file):
        size = os.path.getsize(output_file) / 1024  # KB
        print(f"  walk_straight.gif: {size:.1f} KB")


if __name__ == '__main__':
    main()
