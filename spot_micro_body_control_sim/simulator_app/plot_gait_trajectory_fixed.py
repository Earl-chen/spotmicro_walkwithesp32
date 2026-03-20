#!/usr/bin/env python3
# plot_gait_trajectory_fixed.py - 绘制步态轨迹（修复中文字体）

import sys
import os

# 设置项目路径
project_root = '/home/robot-01/work/spotmicro/spot_micro_body_control_sim/03_spot_micro_simulator_framework'
sys.path.insert(0, project_root)

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
import numpy as np

from app.gait.walk_gait import WalkGait
from app.gait.trajectory import TrajectoryGenerator

print("开始绘制步态轨迹...")

# ✅ 配置中文字体
font_path = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
if os.path.exists(font_path):
    fm.fontManager.addfont(font_path)
    font_prop = fm.FontProperties(fname=font_path)
    plt.rcParams['font.family'] = font_prop.get_name()
    plt.rcParams['axes.unicode_minus'] = False
    print(f"✅ 已加载中文字体")
else:
    print(f"⚠️ 字体不存在，使用默认字体")

# 创建步态控制器
gait = WalkGait(stride_length=0.04, step_height=0.025, frequency=0.8)

legs = ['right_front', 'left_back', 'left_front', 'right_back']
leg_names = {
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

# 生成数据
phases = np.linspace(0, 1, 100)

# 创建图形
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(14, 10))

# 绘制X轨迹
for leg in legs:
    x_vals = []
    for p in phases:
        gait.global_phase = p
        lp = gait.get_leg_phase(leg)
        x, z = TrajectoryGenerator.cycloid_trajectory(lp, 0.04, 0.025)
        x_vals.append(x * 100)
    ax1.plot(phases, x_vals, label=leg_names[leg], color=colors[leg], lw=2)

ax1.axvline(x=0.25, color='red', ls=':', lw=2, alpha=0.5)
ax1.axhline(y=0, color='k', ls='--', alpha=0.3)
ax1.set_xlabel('全局相位', fontsize=11)
ax1.set_ylabel('X偏移 (cm)', fontsize=11)
ax1.set_title('足端X方向轨迹', fontsize=12, fontweight='bold')
ax1.legend()
ax1.grid(True, alpha=0.3)

# 绘制Z轨迹
for leg in legs:
    z_vals = []
    for p in phases:
        gait.global_phase = p
        lp = gait.get_leg_phase(leg)
        x, z = TrajectoryGenerator.cycloid_trajectory(lp, 0.04, 0.025)
        z_vals.append(z * 100)
    ax2.plot(phases, z_vals, label=leg_names[leg], color=colors[leg], lw=2)

ax2.axvline(x=0.25, color='red', ls=':', lw=2, alpha=0.5)
ax2.axhline(y=0, color='k', ls='--', alpha=0.3)
ax2.set_xlabel('全局相位', fontsize=11)
ax2.set_ylabel('Z偏移 (cm)', fontsize=11)
ax2.set_title('足端Z方向轨迹', fontsize=12, fontweight='bold')
ax2.legend()
ax2.grid(True, alpha=0.3)

# 绘制2D轨迹
for leg in legs:
    x_vals = []
    z_vals = []
    for p in phases:
        gait.global_phase = p
        lp = gait.get_leg_phase(leg)
        x, z = TrajectoryGenerator.cycloid_trajectory(lp, 0.04, 0.025)
        x_vals.append(x * 100)
        z_vals.append(z * 100)
    ax3.plot(x_vals, z_vals, label=leg_names[leg], color=colors[leg], lw=2)
    ax3.scatter(x_vals[0], z_vals[0], color=colors[leg], s=100, 
               marker='o', edgecolors='black', lw=2, zorder=5)

ax3.axhline(y=0, color='k', ls='--', alpha=0.3)
ax3.axvline(x=0, color='k', ls='--', alpha=0.3)
ax3.set_xlabel('X偏移 (cm)', fontsize=11)
ax3.set_ylabel('Z偏移 (cm)', fontsize=11)
ax3.set_title('足端2D轨迹', fontsize=12, fontweight='bold')
ax3.legend()
ax3.grid(True, alpha=0.3)
ax3.axis('equal')

# 绘制单腿详细轨迹
leg = 'right_front'
swing_x, swing_z = [], []
stance_x, stance_z = [], []

for p in phases:
    gait.global_phase = p
    lp = gait.get_leg_phase(leg)
    x, z = TrajectoryGenerator.cycloid_trajectory(lp, 0.04, 0.025)
    
    if lp < 0.25:
        swing_x.append(x * 100)
        swing_z.append(z * 100)
    else:
        stance_x.append(x * 100)
        stance_z.append(z * 100)

ax4.plot(swing_x, swing_z, color='red', lw=3, label='摆动相（25%）', alpha=0.8)
ax4.scatter(swing_x[0], swing_z[0], color='green', s=150, 
           marker='o', edgecolors='black', lw=2, label='起点', zorder=5)
ax4.plot(stance_x, stance_z, color='blue', lw=3, label='支撑相（75%）', alpha=0.8)

ax4.axhline(y=0, color='k', ls='--', alpha=0.3)
ax4.axvline(x=0, color='k', ls='--', alpha=0.3)
ax4.set_xlabel('X偏移 (cm)', fontsize=11)
ax4.set_ylabel('Z偏移 (cm)', fontsize=11)
ax4.set_title('右前腿详细轨迹（修正后）', fontsize=12, fontweight='bold')
ax4.legend()
ax4.grid(True, alpha=0.3)
ax4.axis('equal')

plt.tight_layout()

# 保存图片
output_path = os.path.join(project_root, 'gait_trajectory.png')
plt.savefig(output_path, dpi=150, bbox_inches='tight')

file_size = os.path.getsize(output_path) / 1024
print(f"✅ 轨迹图已保存: {output_path}")
print(f"   文件大小: {file_size:.1f} KB")

plt.close()
