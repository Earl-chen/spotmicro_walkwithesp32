#!/usr/bin/env python3
"""
轻量版行走动画生成器

生成小体积 GIF，适合飞书发送
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gait.walk_gait import WalkGait

# 使用英文标签避免字体问题
plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


def create_lightweight_walking_gif(steering_angle=0.0, output_file='walk.gif'):
    """
    创建轻量级行走动画
    
    Args:
        steering_angle: 转向角度（弧度）
        output_file: 输出文件名
    """
    angle_deg = steering_angle * 180 / np.pi
    print(f"生成行走 GIF（{angle_deg:.0f}°）...", end=' ')
    
    # 创建步态控制器
    gait = WalkGait(stride_length=0.05, step_height=0.03, frequency=0.8)
    gait.set_direction(steering_angle)
    
    dt = 0.04  # 降低帧率
    frames = 50  # 减少帧数
    
    # 创建图形（更小尺寸）
    fig, ax = plt.subplots(figsize=(8, 6))
    
    ax.set_xlim(-180, 180)
    ax.set_ylim(-120, 120)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3, linestyle='--')
    
    # 标题
    if abs(steering_angle) < 0.01:
        title = 'Walk Gait - Straight'
    elif steering_angle > 0:
        title = f'Walk Gait - Left Turn ({angle_deg:.0f}°)'
    else:
        title = f'Walk Gait - Right Turn ({abs(angle_deg):.0f}°)'
    
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.set_xlabel('X (mm)', fontsize=11)
    ax.set_ylabel('Y (mm)', fontsize=11)
    
    # 机器人身体
    body_width = 78
    body_length = 207.5
    body = Rectangle((-body_length/2, -body_width/2), 
                    body_length, body_width, 
                    fill=False, edgecolor='black', linewidth=2)
    ax.add_patch(body)
    
    # 腿部初始位置
    leg_base_positions = {
        'LF': (body_length/2, body_width/2),
        'RF': (body_length/2, -body_width/2),
        'LB': (-body_length/2, body_width/2),
        'RB': (-body_length/2, -body_width/2)
    }
    
    leg_labels = {
        'LF': 'Left Front',
        'RF': 'Right Front',
        'LB': 'Left Back',
        'RB': 'Right Back'
    }
    
    leg_colors = {
        'LF': '#FF6B6B',
        'RF': '#4ECDC4',
        'LB': '#45B7D1',
        'RB': '#FFA07A'
    }
    
    # 初始化腿部点
    leg_points = {}
    for leg_name, (x, y) in leg_base_positions.items():
        point, = ax.plot(x, y, 'o', color=leg_colors[leg_name], 
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
        phase = gait.get_global_phase()
        support_legs = gait.count_support_legs()
        
        for leg_name, (base_x, base_y) in leg_base_positions.items():
            point = gait.get_foot_trajectory(leg_name)
            new_x = base_x + point.x * 1000
            new_y = base_y + point.y * 1000
            leg_points[leg_name].set_data([new_x], [new_y])
        
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
    print("="*60)
    print("生成轻量级行走 GIF")
    print("="*60)
    
    # 确保输出目录存在
    output_dir = 'tests/visual'
    os.makedirs(output_dir, exist_ok=True)
    
    # 生成三种模式
    print()
    create_lightweight_walking_gif(0.0, f'{output_dir}/walk_straight.gif')
    create_lightweight_walking_gif(np.pi/6, f'{output_dir}/walk_left_turn.gif')
    create_lightweight_walking_gif(-np.pi/6, f'{output_dir}/walk_right_turn.gif')
    
    print("\n" + "="*60)
    print("✅ 完成！")
    print("="*60)
    
    # 显示文件大小
    import subprocess
    for gif in ['walk_straight.gif', 'walk_left_turn.gif', 'walk_right_turn.gif']:
        path = f'{output_dir}/{gif}'
        if os.path.exists(path):
            size = os.path.getsize(path) / 1024  # KB
            print(f"  {gif}: {size:.1f} KB")


if __name__ == '__main__':
    main()
