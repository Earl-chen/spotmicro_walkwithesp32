#!/usr/bin/env python3
# plot_gait_trajectory.py - 绘制Walk步态轨迹

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
from app.gait.trajectory import TrajectoryGenerator

# 配置中文字体
font_path = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
if os.path.exists(font_path):
    fm.fontManager.addfont(font_path)
    font_prop = fm.FontProperties(fname=font_path)
    plt.rcParams['font.family'] = font_prop.get_name()
    plt.rcParams['axes.unicode_minus'] = False

def plot_gait_trajectory():
    """绘制Walk步态轨迹"""
    
    gait = WalkGait(stride_length=0.04, step_height=0.025, frequency=0.8)
    
    legs = ['right_front', 'left_back', 'left_front', 'right_back']
    leg_names_cn = {
        'right_front': '右前腿',
        'left_back': '左后腿',
        'left_front': '左前腿',
        'right_back': '右后腿'
    }
    
    colors = {
        'right_front': '#FF6B6B',  # 红色
        'left_back': '#4ECDC4',     # 青色
        'left_front': '#FFD93D',    # 黄色
        'right_back': '#6BCF7F'     # 绿色
    }
    
    print("="*70)
    print("Walk步态轨迹绘制（修正版：占空比25%/75%）")
    print("="*70)
    
    # 创建图形
    fig = plt.figure(figsize=(16, 12))
    
    # 1. 绘制每条腿的X、Z轨迹（随相位变化）
    ax1 = fig.add_subplot(2, 3, 1)
    ax2 = fig.add_subplot(2, 3, 2)
    ax3 = fig.add_subplot(2, 3, 3)
    ax4 = fig.add_subplot(2, 3, 4)
    ax5 = fig.add_subplot(2, 3, 5)
    ax6 = fig.add_subplot(2, 3, 6)
    
    phases = np.linspace(0, 1, 100)
    
    for leg in legs:
        phase_offset = gait.phase_offsets[leg]
        
        x_values = []
        z_values = []
        states = []
        
        for global_phase in phases:
            gait.global_phase = global_phase
            leg_phase = gait.get_leg_phase(leg)
            
            x, z = TrajectoryGenerator.cycloid_trajectory(leg_phase, 0.04, 0.025)
            x_values.append(x * 100)
            z_values.append(z * 100)
            
            # 判断摆动相/支撑相
            is_swing = leg_phase < 0.25
            states.append(is_swing)
        
        # 绘制X轨迹
        ax1.plot(phases, x_values, label=leg_names_cn[leg], 
                color=colors[leg], linewidth=2.5, alpha=0.8)
        
        # 绘制Z轨迹
        ax2.plot(phases, z_values, label=leg_names_cn[leg], 
                color=colors[leg], linewidth=2.5, alpha=0.8)
        
        # 绘制2D轨迹（X-Z）
        ax3.plot(x_values, z_values, label=leg_names_cn[leg], 
                color=colors[leg], linewidth=2.5, alpha=0.8)
        
        # 标记起点和终点
        ax3.scatter(x_values[0], z_values[0], color=colors[leg], 
                   s=100, marker='o', edgecolors='black', linewidths=2, zorder=5)
        ax3.scatter(x_values[-1], z_values[-1], color=colors[leg], 
                   s=100, marker='s', edgecolors='black', linewidths=2, zorder=5)
    
    # 设置ax1（X轨迹）
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax1.axvline(x=0.25, color='red', linestyle=':', alpha=0.5, linewidth=2)
    ax1.axvline(x=0.5, color='gray', linestyle=':', alpha=0.3)
    ax1.axvline(x=0.75, color='gray', linestyle=':', alpha=0.3)
    ax1.set_xlabel('全局相位', fontsize=11)
    ax1.set_ylabel('X偏移 (cm)', fontsize=11)
    ax1.set_title('足端X方向轨迹（前后）', fontsize=12, fontweight='bold')
    ax1.legend(fontsize=9, loc='best')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim([0, 1])
    
    # 添加摆动相/支撑相标注
    ax1.axvspan(0, 0.25, alpha=0.1, color='red', label='摆动相')
    ax1.text(0.125, ax1.get_ylim()[1]*0.9, '摆动相\n25%', 
            ha='center', fontsize=9, color='red', fontweight='bold')
    ax1.text(0.625, ax1.get_ylim()[1]*0.9, '支撑相\n75%', 
            ha='center', fontsize=9, color='blue', fontweight='bold')
    
    # 设置ax2（Z轨迹）
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.axvline(x=0.25, color='red', linestyle=':', alpha=0.5, linewidth=2)
    ax2.set_xlabel('全局相位', fontsize=11)
    ax2.set_ylabel('Z偏移 (cm)', fontsize=11)
    ax2.set_title('足端Z方向轨迹（上下）', fontsize=12, fontweight='bold')
    ax2.legend(fontsize=9, loc='best')
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim([0, 1])
    
    # 添加摆动相/支撑相标注
    ax2.axvspan(0, 0.25, alpha=0.1, color='red')
    ax2.text(0.125, ax2.get_ylim()[1]*0.9, '摆动相\n25%', 
            ha='center', fontsize=9, color='red', fontweight='bold')
    ax2.text(0.625, ax2.get_ylim()[1]*0.9, '支撑相\n75%', 
            ha='center', fontsize=9, color='blue', fontweight='bold')
    
    # 设置ax3（2D轨迹）
    ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax3.axvline(x=0, color='k', linestyle='--', alpha=0.3)
    ax3.set_xlabel('X偏移 (cm)', fontsize=11)
    ax3.set_ylabel('Z偏移 (cm)', fontsize=11)
    ax3.set_title('足端2D轨迹（X-Z平面）', fontsize=12, fontweight='bold')
    ax3.legend(fontsize=9, loc='best')
    ax3.grid(True, alpha=0.3)
    ax3.axis('equal')
    
    # 4. 绘制单条腿详细轨迹（以右前腿为例）
    leg = 'right_front'
    phase_offset = gait.phase_offsets[leg]
    
    x_vals = []
    z_vals = []
    swing_phases = []
    stance_phases = []
    
    for global_phase in phases:
        gait.global_phase = global_phase
        leg_phase = gait.get_leg_phase(leg)
        
        x, z = TrajectoryGenerator.cycloid_trajectory(leg_phase, 0.04, 0.025)
        x_vals.append(x * 100)
        z_vals.append(z * 100)
        
        if leg_phase < 0.25:
            swing_phases.append((global_phase, x*100, z*100))
        else:
            stance_phases.append((global_phase, x*100, z*100))
    
    # 绘制摆动相
    if swing_phases:
        swing_x = [p[1] for p in swing_phases]
        swing_z = [p[2] for p in swing_phases]
        ax4.plot(swing_x, swing_z, color='#FF6B6B', linewidth=3, 
                label='摆动相（25%）', alpha=0.8)
        ax4.scatter(swing_x[0], swing_z[0], color='green', s=150, 
                   marker='o', edgecolors='black', linewidths=2, 
                   label='起点', zorder=5)
        ax4.scatter(swing_x[-1], swing_z[-1], color='red', s=150, 
                   marker='^', edgecolors='black', linewidths=2, 
                   label='摆动终点', zorder=5)
    
    # 绘制支撑相
    if stance_phases:
        stance_x = [p[1] for p in stance_phases]
        stance_z = [p[2] for p in stance_phases]
        ax4.plot(stance_x, stance_z, color='#4ECDC4', linewidth=3, 
                label='支撑相（75%）', alpha=0.8)
    
    ax4.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax4.axvline(x=0, color='k', linestyle='--', alpha=0.3)
    ax4.set_xlabel('X偏移 (cm)', fontsize=11)
    ax4.set_ylabel('Z偏移 (cm)', fontsize=11)
    ax4.set_title(f'{leg_names_cn[leg]}详细轨迹', fontsize=12, fontweight='bold')
    ax4.legend(fontsize=9, loc='best')
    ax4.grid(True, alpha=0.3)
    ax4.axis('equal')
    
    # 5. 绘制轨迹范围分析
    for i, leg in enumerate(legs):
        phase_offset = gait.phase_offsets[leg]
        
        x_vals = []
        z_vals = []
        
        for global_phase in phases:
            gait.global_phase = global_phase
            leg_phase = gait.get_leg_phase(leg)
            
            x, z = TrajectoryGenerator.cycloid_trajectory(leg_phase, 0.04, 0.025)
            x_vals.append(x * 100)
            z_vals.append(z * 100)
        
        x_min, x_max = min(x_vals), max(x_vals)
        z_min, z_max = min(z_vals), max(z_vals)
        
        # 绘制范围矩形
        rect = plt.Rectangle((x_min, z_min), x_max-x_min, z_max-z_min,
                             linewidth=2, edgecolor=colors[leg], 
                             facecolor=colors[leg], alpha=0.2,
                             label=leg_names_cn[leg])
        ax5.add_patch(rect)
        
        # 标记中心点
        ax5.scatter([(x_min+x_max)/2], [(z_min+z_max)/2], 
                   color=colors[leg], s=100, marker='x', linewidths=3)
    
    ax5.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax5.axvline(x=0, color='k', linestyle='--', alpha=0.3)
    ax5.set_xlabel('X偏移 (cm)', fontsize=11)
    ax5.set_ylabel('Z偏移 (cm)', fontsize=11)
    ax5.set_title('四条腿轨迹范围对比', fontsize=12, fontweight='bold')
    ax5.legend(fontsize=9, loc='best')
    ax5.grid(True, alpha=0.3)
    ax5.axis('equal')
    
    # 6. 绘制轨迹数据统计
    ax6.axis('off')
    
    # 输出统计数据
    stats_text = "轨迹数据统计\n" + "="*40 + "\n\n"
    
    for leg in legs:
        phase_offset = gait.phase_offsets[leg]
        
        x_vals = []
        z_vals = []
        swing_count = 0
        stance_count = 0
        
        for global_phase in phases:
            gait.global_phase = global_phase
            leg_phase = gait.get_leg_phase(leg)
            
            x, z = TrajectoryGenerator.cycloid_trajectory(leg_phase, 0.04, 0.025)
            x_vals.append(x * 100)
            z_vals.append(z * 100)
            
            if leg_phase < 0.25:
                swing_count += 1
            else:
                stance_count += 1
        
        x_min, x_max = min(x_vals), max(x_vals)
        z_min, z_max = min(z_vals), max(z_vals)
        
        stats_text += f"{leg_names_cn[leg]}:\n"
        stats_text += f"  X范围: {x_min:+.2f} ~ {x_max:+.2f} cm (宽度: {x_max-x_min:.2f} cm)\n"
        stats_text += f"  Z范围: {z_min:+.2f} ~ {z_max:+.2f} cm (高度: {z_max-z_min:.2f} cm)\n"
        stats_text += f"  摆动相: {swing_count}帧 ({swing_count}%)\n"
        stats_text += f"  支撑相: {stance_count}帧 ({stance_count}%)\n"
        stats_text += f"  相位偏移: {phase_offset:.2f} ({phase_offset*360:.0f}°)\n\n"
    
    ax6.text(0.1, 0.95, stats_text, transform=ax6.transAxes, 
            fontsize=9, verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))
    
    plt.tight_layout()
    
    # 保存图片
    output_path = 'gait_trajectory_plot.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n✅ 轨迹图已保存: {output_path}")
    
    plt.close()
    
    print("\n" + "="*70)
    print("轨迹分析完成！")
    print("="*70)

if __name__ == '__main__':
    plot_gait_trajectory()
