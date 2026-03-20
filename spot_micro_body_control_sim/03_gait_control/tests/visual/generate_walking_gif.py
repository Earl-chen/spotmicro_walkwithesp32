#!/usr/bin/env python3
"""
四足机器人行走动画生成器

生成带转向功能的行走 GIF 动画
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle
import matplotlib.font_manager as fm

module_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, module_root)
from gait_algo_core.walk_gait import WalkGait

# 配置字体（优先使用系统中文字体）
font_list = [f.name for f in fm.fontManager.ttflist]
chinese_fonts = [f for f in font_list if any(k in f.lower() for k in ['noto', 'cjk', 'simhei', 'wqy', 'droid'])]

if chinese_fonts:
    plt.rcParams['font.sans-serif'] = chinese_fonts + ['DejaVu Sans']
    print(f"使用中文字体: {chinese_fonts[0]}")
else:
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Liberation Sans']
    print("使用英文标签")

plt.rcParams['axes.unicode_minus'] = False


def create_walking_animation(steering_angle=0.0, output_file=None):
    """
    创建行走动画
    
    Args:
        steering_angle: 转向角度（弧度）
        output_file: 输出文件名
    """
    # 默认输出到脚本所在目录
    if output_file is None:
        output_file = os.path.join(os.path.dirname(__file__), 'walking_animation.gif')
    
    print(f"\n生成行走动画（转向角度={steering_angle*180/np.pi:.1f}°）...")
    
    # 创建步态控制器
    gait = WalkGait(stride_length=0.05, step_height=0.03, frequency=0.8)
    gait.set_direction(steering_angle)
    
    dt = 0.02
    frames = 150  # 3秒动画
    
    # 创建图形
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    
    # === 左图：3D 俯视图 ===
    ax1.set_xlim(-200, 200)
    ax1.set_ylim(-150, 150)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3, linestyle='--')
    ax1.set_xlabel('X (mm)', fontsize=12)
    ax1.set_ylabel('Y (mm)', fontsize=12)
    
    # 标题（根据转向角度）
    if abs(steering_angle) < 0.01:
        title1 = 'Walk Gait - Straight (0°)'
    elif steering_angle > 0:
        title1 = f'Walk Gait - Left Turn (+{steering_angle*180/np.pi:.0f}°)'
    else:
        title1 = f'Walk Gait - Right Turn ({steering_angle*180/np.pi:.0f}°)'
    
    ax1.set_title(title1, fontsize=14, fontweight='bold')
    
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
    
    # 腿部初始位置
    leg_base_positions = {
        'LF': (body_length/2, body_width/2),
        'RF': (body_length/2, -body_width/2),
        'LB': (-body_length/2, body_width/2),
        'RB': (-body_length/2, -body_width/2)
    }
    
    leg_colors = {
        'LF': '#FF6B6B',
        'RF': '#4ECDC4',
        'LB': '#45B7D1',
        'RB': '#FFA07A'
    }
    
    leg_labels = {
        'LF': 'Left Front',
        'RF': 'Right Front',
        'LB': 'Left Back',
        'RB': 'Right Back'
    }
    
    # 初始化腿部绘图元素
    leg_points = {}
    leg_trails = {}
    
    for leg_name, (x, y) in leg_base_positions.items():
        # 当前位置点
        point, = ax1.plot(x, y, 'o', color=leg_colors[leg_name],
                         markersize=18, label=leg_labels[leg_name],
                         markeredgecolor='black', markeredgewidth=2)
        leg_points[leg_name] = point
        
        # 轨迹线
        trail, = ax1.plot([], [], '-', color=leg_colors[leg_name],
                         alpha=0.3, linewidth=2)
        leg_trails[leg_name] = trail
    
    ax1.legend(loc='upper left', fontsize=10, framealpha=0.9)
    
    # === 右图：轨迹曲线 ===
    ax2.set_xlim(0, frames * dt)
    ax2.set_ylim(-50, 50)
    ax2.grid(True, alpha=0.3, linestyle='--')
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Y Offset (mm)', fontsize=12)
    ax2.set_title('Lateral Trajectory (Y-axis)', fontsize=14, fontweight='bold')
    
    # 初始化轨迹曲线
    trajectory_lines = {}
    time_data = []
    y_data = {leg: [] for leg in leg_base_positions.keys()}
    
    for leg_name in leg_base_positions.keys():
        line, = ax2.plot([], [], '-', color=leg_colors[leg_name],
                        linewidth=2, label=leg_labels[leg_name])
        trajectory_lines[leg_name] = line
    
    ax2.legend(loc='upper left', fontsize=10, framealpha=0.9)
    
    # 文本显示
    info_text = ax1.text(0.02, 0.98, '', transform=ax1.transAxes,
                        fontsize=11, verticalalignment='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    def init():
        """初始化函数"""
        for leg_name in leg_base_positions.keys():
            leg_points[leg_name].set_data([], [])
            leg_trails[leg_name].set_data([], [])
            trajectory_lines[leg_name].set_data([], [])
        info_text.set_text('')
        return list(leg_points.values()) + list(leg_trails.values()) + \
               list(trajectory_lines.values()) + [info_text]
    
    # 存储历史轨迹
    trail_history = {leg: {'x': [], 'y': []} for leg in leg_base_positions.keys()}
    
    def update(frame):
        """更新函数"""
        nonlocal time_data, y_data, trail_history
        
        # 更新步态
        gait.update(dt)
        
        current_time = frame * dt
        time_data.append(current_time)
        
        # 更新每条腿
        for leg_name, (base_x, base_y) in leg_base_positions.items():
            # 获取轨迹点
            point = gait.get_foot_trajectory(leg_name)
            
            # 计算新位置（基准 + 偏移）
            new_x = base_x + point.x * 1000
            new_y = base_y + point.y * 1000
            
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
            
            # 更新Y轴轨迹曲线
            y_data[leg_name].append(point.y * 1000)
            trajectory_lines[leg_name].set_data(time_data, y_data[leg_name])
        
        # 更新信息文本
        state = gait.get_state()
        phase = gait.get_global_phase()
        support_legs = gait.count_support_legs()
        
        info = f'Phase: {phase:.2f}\n'
        info += f'Support Legs: {support_legs}/4\n'
        info += f'Steering: {steering_angle*180/np.pi:+.1f}°'
        
        info_text.set_text(info)
        
        # 动态调整X轴范围
        if current_time > 2.5:
            ax2.set_xlim(current_time - 2.5, current_time + 0.5)
        
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
    print("="*60)
    print("四足机器人行走动画生成器")
    print("="*60)
    
    # 确保输出目录存在
    output_dir = 'tests/visual'
    os.makedirs(output_dir, exist_ok=True)
    
    # 生成三种模式的动画
    animations = []
    
    # 1. 直行
    print("\n【1/3】生成直行动画...")
    file1 = create_walking_animation(0.0, f'{output_dir}/walk_straight.gif')
    animations.append(file1)
    
    # 2. 左转
    print("\n【2/3】生成左转动画...")
    file2 = create_walking_animation(np.pi/6, f'{output_dir}/walk_left_turn.gif')
    animations.append(file2)
    
    # 3. 右转
    print("\n【3/3】生成右转动画...")
    file3 = create_walking_animation(-np.pi/6, f'{output_dir}/walk_right_turn.gif')
    animations.append(file3)
    
    print("\n" + "="*60)
    print("✅ 所有动画生成完成！")
    print("="*60)
    
    for anim in animations:
        print(f"  {anim}")
    
    print("\n动画说明：")
    print("  - walk_straight.gif: 直行模式")
    print("  - walk_left_turn.gif: 左转30°")
    print("  - walk_right_turn.gif: 右转30°")


if __name__ == '__main__':
    main()
