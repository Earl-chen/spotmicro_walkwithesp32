#!/usr/bin/env python3
# analyze_gait.py - 步态详细分析

import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
import numpy as np

project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from app.gait.walk_gait import WalkGait

# 配置中文字体
font_path = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
if os.path.exists(font_path):
    fm.fontManager.addfont(font_path)
    font_prop = fm.FontProperties(fname=font_path)
    plt.rcParams['font.family'] = font_prop.get_name()
    plt.rcParams['axes.unicode_minus'] = False

def analyze_gait():
    """详细分析Walk步态"""
    
    # 创建步态控制器
    gait = WalkGait(stride_length=0.04, step_height=0.025, frequency=0.8)
    
    print("="*70)
    print("Walk步态详细分析报告")
    print("="*70)
    
    # 1. 步态参数
    print("\n【1. 步态参数】")
    print(f"  步长: {gait.stride_length*100:.1f} cm")
    print(f"  步高: {gait.step_height*100:.1f} cm")
    print(f"  步频: {gait.frequency} Hz")
    print(f"  周期: {1/gait.frequency:.2f} 秒")
    
    # 2. 相位安排
    print("\n【2. 相位安排】")
    print("  Walk步态特点：三足支撑，一足抬起")
    print("  相位偏移：")
    for leg, phase in gait.phase_offsets.items():
        angle = phase * 360
        print(f"    {leg:15s}: {phase:.2f} ({angle:5.1f}°)")
    
    # 3. 模拟一个完整周期
    print("\n【3. 一个完整周期的步态变化】")
    print("  (相位从0.0到1.0)")
    
    dt = 0.01
    total_steps = 100
    
    leg_names = list(gait.phase_offsets.keys())
    footprints = {leg: {'x': [], 'z': [], 'phase': [], 'state': []} for leg in leg_names}
    
    for step in range(total_steps):
        gait.global_phase = step / total_steps
        
        trajectories = gait.get_all_foot_trajectories()
        
        for leg_name, (x_offset, z_offset) in trajectories.items():
            footprints[leg_name]['x'].append(x_offset)
            footprints[leg_name]['z'].append(z_offset)
            footprints[leg_name]['phase'].append(gait.global_phase)
            
            # 判断状态：摆动相（前50%）或支撑相（后50%）
            leg_phase = gait.get_leg_phase(leg_name)
            state = 'swing' if leg_phase < 0.5 else 'stance'
            footprints[leg_name]['state'].append(state)
    
    # 4. 关键时刻分析（每10%一个点）
    print("\n【4. 关键时刻步态状态】")
    print("  (显示哪些腿在摆动，哪些在支撑)")
    
    key_phases = [0.0, 0.25, 0.5, 0.75]
    for phase in key_phases:
        gait.global_phase = phase
        print(f"\n  全局相位 {phase:.2f} ({phase*100:.0f}%):")
        
        for leg in leg_names:
            leg_phase = gait.get_leg_phase(leg)
            state = "🦵 摆动" if leg_phase < 0.5 else "🦶 支撑"
            x_off, z_off = gait.get_foot_trajectory(leg)
            print(f"    {leg:15s}: {state} (相位{leg_phase:.2f}, X偏移{x_off*100:+5.1f}cm, Z偏移{z_off*100:+5.1f}cm)")
    
    # 5. 落脚点分析
    print("\n【5. 落脚点位置变化】")
    print("  (每条腿的足端轨迹)")
    
    for leg in leg_names:
        x_data = np.array(footprints[leg]['x']) * 100  # 转换为cm
        z_data = np.array(footprints[leg]['z']) * 100
        
        print(f"\n  {leg}:")
        print(f"    X方向: {x_data.min():+.2f}cm 到 {x_data.max():+.2f}cm (范围: {x_data.max()-x_data.min():.2f}cm)")
        print(f"    Z方向: {z_data.min():+.2f}cm 到 {z_data.max():+.2f}cm (范围: {z_data.max()-z_data.min():.2f}cm)")
        
        # 找到着地点（Z最低点）
        min_z_idx = np.argmin(z_data)
        landing_phase = footprints[leg]['phase'][min_z_idx]
        landing_x = x_data[min_z_idx]
        print(f"    着地点: 相位{landing_phase:.2f}, X偏移{landing_x:+.2f}cm")
    
    # 6. 生成可视化图表
    print("\n【6. 生成可视化图表】")
    
    fig = plt.figure(figsize=(16, 10))
    
    # 子图1: X方向轨迹
    ax1 = fig.add_subplot(2, 2, 1)
    for leg in leg_names:
        x_data = np.array(footprints[leg]['x']) * 100
        phases = np.array(footprints[leg]['phase'])
        ax1.plot(phases, x_data, label=leg, linewidth=2)
    ax1.set_xlabel('全局相位')
    ax1.set_ylabel('X偏移 (cm)')
    ax1.set_title('足端X方向轨迹（前后移动）')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    # 子图2: Z方向轨迹
    ax2 = fig.add_subplot(2, 2, 2)
    for leg in leg_names:
        z_data = np.array(footprints[leg]['z']) * 100
        phases = np.array(footprints[leg]['phase'])
        ax2.plot(phases, z_data, label=leg, linewidth=2)
    ax2.set_xlabel('全局相位')
    ax2.set_ylabel('Z偏移 (cm)')
    ax2.set_title('足端Z方向轨迹（上下移动）')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    # 子图3: 相位关系图
    ax3 = fig.add_subplot(2, 2, 3)
    for i, leg in enumerate(leg_names):
        phase_offset = gait.phase_offsets[leg]
        ax3.barh(i, 0.5, left=phase_offset, height=0.6, 
                color='red', alpha=0.7, label='摆动相' if i==0 else '')
        ax3.barh(i, 0.5, left=phase_offset+0.5, height=0.6, 
                color='blue', alpha=0.7, label='支撑相' if i==0 else '')
    
    ax3.set_yticks(range(len(leg_names)))
    ax3.set_yticklabels(leg_names)
    ax3.set_xlabel('相位')
    ax3.set_title('各腿相位关系（红=摆动，蓝=支撑）')
    ax3.legend()
    ax3.grid(True, alpha=0.3, axis='x')
    
    # 子图4: 2D轨迹（俯视图）
    ax4 = fig.add_subplot(2, 2, 4)
    colors = ['red', 'green', 'orange', 'purple']
    for i, leg in enumerate(leg_names):
        x_data = np.array(footprints[leg]['x']) * 100
        z_data = np.array(footprints[leg]['z']) * 100
        ax4.plot(x_data, z_data, color=colors[i], label=leg, linewidth=2, alpha=0.7)
        
        # 标记起点和终点
        ax4.scatter(x_data[0], z_data[0], color=colors[i], s=100, marker='o', edgecolors='black', linewidths=2, zorder=5)
        ax4.scatter(x_data[-1], z_data[-1], color=colors[i], s=100, marker='s', edgecolors='black', linewidths=2, zorder=5)
    
    ax4.set_xlabel('X偏移 (cm)')
    ax4.set_ylabel('Z偏移 (cm)')
    ax4.set_title('足端2D轨迹（圆点=起点，方块=终点）')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    ax4.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax4.axvline(x=0, color='k', linestyle='--', alpha=0.3)
    ax4.set_aspect('equal')
    
    plt.tight_layout()
    
    output_path = os.path.join(os.path.dirname(__file__), 'gait_analysis.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"  ✅ 图表已保存: {output_path}")
    
    plt.close()
    
    # 7. 步态总结
    print("\n【7. 步态总结】")
    print("  Walk步态的特点：")
    print("    ✓ 任何时刻都有3只脚着地（稳定性高）")
    print("    ✓ 4条腿依次抬起（相位差90°）")
    print("    ✓ 占空比75%（每条腿75%时间着地）")
    print("    ✓ 适合复杂地形和慢速行走")
    print()
    print("  一个完整周期（0→1）的运动过程：")
    print("    0.00-0.25: 右前腿摆动，其他腿支撑")
    print("    0.25-0.50: 左后腿摆动，其他腿支撑")
    print("    0.50-0.75: 左前腿摆动，其他腿支撑")
    print("    0.75-1.00: 右后腿摆动，其他腿支撑")
    
    print("\n" + "="*70)

if __name__ == '__main__':
    analyze_gait()
