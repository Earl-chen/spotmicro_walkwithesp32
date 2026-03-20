#!/usr/bin/env python3
"""
转向功能可视化脚本

生成转向轨迹的对比图和动画
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
import matplotlib

# 设置中文字体
matplotlib.rcParams['font.sans-serif'] = ['BabelStone Han', 'SimHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gait_algo_core.walk_gait import WalkGait

def generate_steering_comparison():
    """生成转向对比图"""
    print("生成转向对比图...")
    
    gait = WalkGait(stride_length=0.05, step_height=0.03, frequency=0.8)
    dt = 0.02
    steps_per_cycle = int(1.0 / (gait.frequency * dt))
    
    # 收集三种转向模式的数据
    modes = {
        '直行 (0°)': 0.0,
        '左转 (+30°)': np.pi/6,
        '右转 (-30°)': -np.pi/6
    }
    
    all_trajectories = {}
    
    for mode_name, angle in modes.items():
        gait.reset()
        gait.set_direction(angle)
        
        trajectories = {
            'left_front': {'x': [], 'y': [], 'z': []},
            'right_front': {'x': [], 'y': [], 'z': []},
            'left_back': {'x': [], 'y': [], 'z': []},
            'right_back': {'x': [], 'y': [], 'z': []}
        }
        
        for i in range(steps_per_cycle):
            gait.update(dt)
            
            for leg_name in trajectories.keys():
                point = gait.get_foot_trajectory(leg_name)
                trajectories[leg_name]['x'].append(point.x * 1000)  # 转换为mm
                trajectories[leg_name]['y'].append(point.y * 1000)
                trajectories[leg_name]['z'].append(point.z * 1000)
        
        all_trajectories[mode_name] = trajectories
    
    # 绘制对比图
    fig, axes = plt.subplots(3, 3, figsize=(15, 12))
    fig.suptitle('Walk步态转向功能对比', fontsize=16, fontweight='bold')
    
    leg_colors = {
        'left_front': '#FF6B6B',
        'right_front': '#4ECDC4',
        'left_back': '#45B7D1',
        'right_back': '#FFA07A'
    }
    
    for idx, (mode_name, trajectories) in enumerate(all_trajectories.items()):
        # XY平面（俯视图）
        ax_xy = axes[idx, 0]
        for leg_name, data in trajectories.items():
            ax_xy.plot(data['x'], data['y'], 
                      color=leg_colors[leg_name], 
                      label=leg_name, 
                      linewidth=2)
        ax_xy.set_xlabel('X (mm)')
        ax_xy.set_ylabel('Y (mm)')
        ax_xy.set_title(f'{mode_name}\nXY平面（俯视）')
        ax_xy.grid(True, alpha=0.3)
        ax_xy.axis('equal')
        ax_xy.legend(fontsize=8)
        
        # XZ平面（侧视图）
        ax_xz = axes[idx, 1]
        for leg_name, data in trajectories.items():
            ax_xz.plot(data['x'], data['z'], 
                      color=leg_colors[leg_name], 
                      linewidth=2)
        ax_xz.set_xlabel('X (mm)')
        ax_xz.set_ylabel('Z (mm)')
        ax_xz.set_title(f'{mode_name}\nXZ平面（侧视）')
        ax_xz.grid(True, alpha=0.3)
        
        # YZ平面（正视图）
        ax_yz = axes[idx, 2]
        for leg_name, data in trajectories.items():
            ax_yz.plot(data['y'], data['z'], 
                      color=leg_colors[leg_name], 
                      linewidth=2)
        ax_yz.set_xlabel('Y (mm)')
        ax_yz.set_ylabel('Z (mm)')
        ax_yz.set_title(f'{mode_name}\nYZ平面（正视）')
        ax_yz.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_file = 'tests/visual/steering_comparison.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"✅ 已保存: {output_file}")
    plt.close()
    
    return output_file


def generate_steering_animation():
    """生成转向动画"""
    print("生成转向动画...")
    
    gait = WalkGait(stride_length=0.05, step_height=0.03, frequency=0.8)
    gait.set_direction(np.pi/6)  # 左转30度
    
    dt = 0.02
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # 机器人身体（简化为矩形）
    body_width = 78  # mm
    body_length = 207.5  # mm
    
    # 腿部初始位置（相对于身体中心）
    leg_positions = {
        'left_front': (body_length/2, body_width/2),
        'right_front': (body_length/2, -body_width/2),
        'left_back': (-body_length/2, body_width/2),
        'right_back': (-body_length/2, -body_width/2)
    }
    
    leg_colors = {
        'left_front': '#FF6B6B',
        'right_front': '#4ECDC4',
        'left_back': '#45B7D1',
        'right_back': '#FFA07A'
    }
    
    # 初始化绘图元素
    body = plt.Rectangle((-body_length/2, -body_width/2), 
                        body_length, body_width, 
                        fill=False, edgecolor='black', linewidth=2)
    ax.add_patch(body)
    
    leg_points = {}
    for leg_name, (x, y) in leg_positions.items():
        point, = ax.plot(x, y, 'o', color=leg_colors[leg_name], 
                        markersize=10, label=leg_name)
        leg_points[leg_name] = point
    
    ax.set_xlim(-150, 150)
    ax.set_ylim(-100, 100)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_title('Walk步态转向动画（左转30°）')
    
    def update(frame):
        gait.update(dt)
        
        for leg_name, (base_x, base_y) in leg_positions.items():
            point = gait.get_foot_trajectory(leg_name)
            # 更新脚部位置（基准位置 + 轨迹偏移）
            new_x = base_x + point.x * 1000
            new_y = base_y + point.y * 1000
            leg_points[leg_name].set_data([new_x], [new_y])
        
        return list(leg_points.values())
    
    anim = FuncAnimation(fig, update, frames=100, 
                        interval=20, blit=True)
    
    output_file = 'tests/visual/steering_animation.gif'
    anim.save(output_file, writer='pillow', fps=50)
    print(f"✅ 已保存: {output_file}")
    plt.close()
    
    return output_file


def main():
    """主函数"""
    print("="*60)
    print("转向功能可视化")
    print("="*60)
    
    # 确保输出目录存在
    os.makedirs('tests/visual', exist_ok=True)
    
    # 生成对比图
    comparison_file = generate_steering_comparison()
    
    # 生成动画
    animation_file = generate_steering_animation()
    
    print("\n" + "="*60)
    print("✅ 可视化生成完成！")
    print("="*60)
    print(f"对比图: {comparison_file}")
    print(f"动画:   {animation_file}")


if __name__ == '__main__':
    main()
