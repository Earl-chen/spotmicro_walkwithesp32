#!/usr/bin/env python3
# plot_gait_trajectory_v2.py - 绘制步态轨迹（使用项目字体）

import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
import numpy as np

# 使用脚本所在目录作为项目根目录
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from app.gait.walk_gait import WalkGait
from app.gait.trajectory import TrajectoryGenerator

print("="*70)
print("开始绘制步态轨迹图（修正版）")
print("="*70)

# 配置中文字体（兼容低版本matplotlib）
font_candidates = [
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf',
    '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
    '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc',
]

chinese_font = None
for font_path in font_candidates:
    if os.path.exists(font_path):
        try:
            # 尝试使用addfont方法（matplotlib 3.2+）
            if hasattr(fm.fontManager, 'addfont'):
                fm.fontManager.addfont(font_path)
            chinese_font = fm.FontProperties(fname=font_path)
            
            # 设置全局字体（兼容不同版本）
            font_name = chinese_font.get_name()
            plt.rcParams['font.family'] = font_name
            plt.rcParams['font.sans-serif'] = [font_name, 'DejaVu Sans']
            plt.rcParams['axes.unicode_minus'] = False
            
            print(f"✅ 已加载中文字体: {font_path}")
            break
        except Exception as e:
            print(f"⚠️  加载字体失败 {font_path}: {e}")
            continue

if chinese_font is None:
    print("❌ 未找到可用的中文字体，使用默认字体")
    chinese_font = fm.FontProperties()
    # 设置后备字体
    plt.rcParams['font.sans-serif'] = ['WenQuanYi Zen Hei', 'WenQuanYi Micro Hei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

# 创建步态控制器
gait = WalkGait(stride_length=0.04, step_height=0.025, frequency=0.8)

legs = ['right_front', 'left_back', 'left_front', 'right_back']
leg_names_cn = {
    'right_front': '右前腿',
    'left_back': '左后腿',
    'left_front': '左前腿',
    'right_back': '右后腿'
}

colors = {
    'right_front': 'red',
    'left_back': 'green',
    'left_front': 'blue',
    'right_back': 'orange'
}

# 生成相位数据
phases = np.linspace(0, 1, 100)

# 创建图形
fig, axes = plt.subplots(2, 2, figsize=(14, 10))

# 子图1: X轨迹
ax1 = axes[0, 0]
for leg in legs:
    x_values = []
    for phase in phases:
        gait.global_phase = phase
        leg_phase = gait.get_leg_phase(leg)
        x, z = TrajectoryGenerator.cycloid_trajectory(leg_phase, 0.04, 0.025)
        x_values.append(x * 100)
    
    ax1.plot(phases, x_values, label=leg_names_cn[leg], 
            color=colors[leg], linewidth=2)

ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
ax1.axvline(x=0.25, color='red', linestyle=':', alpha=0.5, linewidth=2, label='摆动/支撑分界')
ax1.set_xlabel('全局相位', fontsize=11)
ax1.set_ylabel('X偏移 (cm)', fontsize=11)
ax1.set_title('足端X方向轨迹（前后移动）', fontsize=12, fontweight='bold')
ax1.legend()
ax1.grid(True, alpha=0.3)

# 子图2: Z轨迹
ax2 = axes[0, 1]
for leg in legs:
    z_values = []
    for phase in phases:
        gait.global_phase = phase
        leg_phase = gait.get_leg_phase(leg)
        x, z = TrajectoryGenerator.cycloid_trajectory(leg_phase, 0.04, 0.025)
        z_values.append(z * 100)
    
    ax2.plot(phases, z_values, label=leg_names_cn[leg], 
            color=colors[leg], linewidth=2)

ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
ax2.axvline(x=0.25, color='red', linestyle=':', alpha=0.5, linewidth=2)
ax2.set_xlabel('全局相位', fontsize=11)
ax2.set_ylabel('Z偏移 (cm)', fontsize=11)
ax2.set_title('足端Z方向轨迹（上下移动）', fontsize=12, fontweight='bold')
ax2.legend()
ax2.grid(True, alpha=0.3)

# 子图3: 2D轨迹
ax3 = axes[1, 0]
for leg in legs:
    x_values = []
    z_values = []
    for phase in phases:
        gait.global_phase = phase
        leg_phase = gait.get_leg_phase(leg)
        x, z = TrajectoryGenerator.cycloid_trajectory(leg_phase, 0.04, 0.025)
        x_values.append(x * 100)
        z_values.append(z * 100)
    
    ax3.plot(x_values, z_values, label=leg_names_cn[leg], 
            color=colors[leg], linewidth=2)
    ax3.scatter(x_values[0], z_values[0], color=colors[leg], 
               s=100, marker='o', edgecolors='black', linewidths=2, zorder=5)

ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)
ax3.axvline(x=0, color='k', linestyle='--', alpha=0.3)
ax3.set_xlabel('X偏移 (cm)', fontsize=11)
ax3.set_ylabel('Z偏移 (cm)', fontsize=11)
ax3.set_title('足端2D轨迹（圆点=起点）', fontsize=12, fontweight='bold')
ax3.legend()
ax3.grid(True, alpha=0.3)
ax3.axis('equal')

# 子图4: 单腿详细轨迹（右前腿）
ax4 = axes[1, 1]
leg = 'right_front'

swing_x = []
swing_z = []
stance_x = []
stance_z = []

for phase in phases:
    gait.global_phase = phase
    leg_phase = gait.get_leg_phase(leg)
    x, z = TrajectoryGenerator.cycloid_trajectory(leg_phase, 0.04, 0.025)
    
    if leg_phase < 0.25:
        swing_x.append(x * 100)
        swing_z.append(z * 100)
    else:
        stance_x.append(x * 100)
        stance_z.append(z * 100)

# 绘制摆动相
ax4.plot(swing_x, swing_z, color='red', linewidth=3, 
        label='摆动相（25%）', alpha=0.8)
ax4.scatter(swing_x[0], swing_z[0], color='green', s=150, 
           marker='o', edgecolors='black', linewidths=2, 
           label='起点', zorder=5)

# 绘制支撑相
ax4.plot(stance_x, stance_z, color='blue', linewidth=3, 
        label='支撑相（75%）', alpha=0.8)

ax4.axhline(y=0, color='k', linestyle='--', alpha=0.3)
ax4.axvline(x=0, color='k', linestyle='--', alpha=0.3)
ax4.set_xlabel('X偏移 (cm)', fontsize=11)
ax4.set_ylabel('Z偏移 (cm)', fontsize=11)
ax4.set_title('右前腿详细轨迹（修正后）', fontsize=12, fontweight='bold')
ax4.legend()
ax4.grid(True, alpha=0.3)
ax4.axis('equal')

plt.tight_layout()

# 保存图片
output_path = os.path.join(project_root, 'gait_trajectory_v2.png')
plt.savefig(output_path, dpi=150, bbox_inches='tight')

file_size = os.path.getsize(output_path) / 1024
print(f"\n✅ 轨迹图已保存: {output_path}")
print(f"   文件大小: {file_size:.1f} KB")
print("="*70)

plt.close()
