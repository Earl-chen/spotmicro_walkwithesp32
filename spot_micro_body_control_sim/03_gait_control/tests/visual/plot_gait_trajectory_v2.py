#!/usr/bin/env python3
# plot_gait_trajectory_v2.py - 绘制步态轨迹（使用项目字体）

import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
import numpy as np

# 获取模块根目录（上一级）
module_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, module_root)

from gait_algo_core.walk_gait import WalkGait
from gait_algo_core.trajectory import TrajectoryGenerator

print("="*70)
print("开始绘制步态轨迹图（修正版）")
print("="*70)

# 配置中文字体（兼容低版本matplotlib）
# 优先使用项目目录下的字体，然后检查其他候选字体
font_candidates = [
    os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fonts', 'BabelStoneHan.ttf'),  # 项目目录（优先）
    os.path.expanduser('~/BabelStoneHan.ttf'),  # 用户home目录
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
                _safe_addfont(font_path)
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
        x, y, z = TrajectoryGenerator.cycloid_trajectory(leg_phase, 0.04, 0.025)
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
        x, y, z = TrajectoryGenerator.cycloid_trajectory(leg_phase, 0.04, 0.025)
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
        x, y, z = TrajectoryGenerator.cycloid_trajectory(leg_phase, 0.04, 0.025)
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

# 子图4: 单腿详细轨迹（右前腿）- 改进版：添加箭头和关键点标注
ax4 = axes[1, 1]
leg = 'right_front'

# 收集所有轨迹点（按相位顺序）
all_x = []
all_z = []

for phase in phases:
    gait.global_phase = phase
    leg_phase = gait.get_leg_phase(leg)
    x, y, z = TrajectoryGenerator.cycloid_trajectory(leg_phase, 0.04, 0.025)
    all_x.append(x * 100)
    all_z.append(z * 100)

# 找到摆动相结束索引（相位>=0.25的第一个点）
swing_end_idx = 0
for i in range(len(phases)):
    if phases[i] >= 0.25:
        swing_end_idx = i
        break

# 绘制摆动相（红色）- 带箭头
# 找到最高点索引
max_z_idx = all_z.index(max(all_z))

for i in range(swing_end_idx):
    ax4.plot([all_x[i], all_x[i+1]], [all_z[i], all_z[i+1]], 
            color='red', linewidth=3, alpha=0.8)
    # 每隔4个点添加一个箭头，确保最后几段也有箭头
    if (i % 4 == 0 or i >= swing_end_idx - 3) and i < swing_end_idx - 1:
        ax4.annotate('', xy=(all_x[i+1], all_z[i+1]), xytext=(all_x[i], all_z[i]),
                    arrowprops=dict(arrowstyle='->', color='red', lw=2))

# 绘制支撑相（蓝色）- 带箭头
for i in range(swing_end_idx, len(all_x) - 1):
    ax4.plot([all_x[i], all_x[i+1]], [all_z[i], all_z[i+1]], 
            color='blue', linewidth=3, alpha=0.8)
    # 每隔10个点添加一个箭头
    if (i - swing_end_idx) % 10 == 0:
        dx = all_x[i+1] - all_x[i]
        dz = all_z[i+1] - all_z[i]
        ax4.annotate('', xy=(all_x[i+1], all_z[i+1]), xytext=(all_x[i], all_z[i]),
                    arrowprops=dict(arrowstyle='->', color='blue', lw=2))

# 从最后一点（支撑相）回到起点
ax4.plot([all_x[-1], all_x[0]], [all_z[-1], all_z[0]], 
        color='blue', linewidth=3, alpha=0.8)

# 标记关键点（用不同颜色和形状）
# 起点（绿色圆球）
ax4.scatter(all_x[0], all_z[0], s=200, color='green', 
           edgecolors='black', linewidths=2, zorder=10)
ax4.text(all_x[0]-0.3, all_z[0]+0.5, '起点\n相位0.00', fontsize=9, 
        ha='center', fontproperties=chinese_font,
        bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))

# 最高点（红色圆球）
ax4.scatter(all_x[max_z_idx], all_z[max_z_idx], s=200, color='red', 
           edgecolors='black', linewidths=2, zorder=10)
ax4.text(all_x[max_z_idx], all_z[max_z_idx]+0.5, '最高点\n相位0.12', fontsize=9, 
        ha='center', fontproperties=chinese_font,
        bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.8))

# 着地点（紫色方块）
ax4.scatter(all_x[swing_end_idx], all_z[swing_end_idx], s=200, color='purple', 
           marker='s', edgecolors='black', linewidths=2, zorder=10)
ax4.text(all_x[swing_end_idx]+0.3, all_z[swing_end_idx]+0.5, '着地点\n相位0.25', fontsize=9, 
        ha='center', fontproperties=chinese_font,
        bbox=dict(boxstyle='round', facecolor='plum', alpha=0.8))

# 中点（橙色三角形）
ax4.scatter(all_x[50], all_z[50], s=200, color='orange', 
           marker='^', edgecolors='black', linewidths=2, zorder=10)
ax4.text(all_x[50], all_z[50]-0.5, '中点\n0.50', fontsize=8, 
        ha='center', fontproperties=chinese_font)

# 后段（青色菱形）
ax4.scatter(all_x[75], all_z[75], s=200, color='cyan', 
           marker='D', edgecolors='black', linewidths=2, zorder=10)
ax4.text(all_x[75], all_z[75]-0.5, '后段\n0.75', fontsize=8, 
        ha='center', fontproperties=chinese_font)

# 添加文字说明（不使用 emoji）
ax4.text(-1.5, 2.8, '摆动相：抬腿向前', fontsize=10, 
        color='red', fontweight='bold', fontproperties=chinese_font,
        bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
ax4.text(-1.5, -0.5, '支撑相：着地向后', fontsize=10, 
        color='blue', fontweight='bold', fontproperties=chinese_font,
        bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

# 添加机器人前进方向箭头
ax4.annotate('', xy=(2.5, 0), xytext=(1.5, 0),
            arrowprops=dict(arrowstyle='->', color='green', lw=3))
ax4.text(2.0, 0.4, '前进方向', fontsize=10, ha='center', 
        color='green', fontweight='bold', fontproperties=chinese_font)

# 添加图例（右上角）
from matplotlib.lines import Line2D
legend_elements = [
    Line2D([0], [0], color='red', linewidth=3, label='摆动相（抬腿）'),
    Line2D([0], [0], color='blue', linewidth=3, label='支撑相（着地）'),
    Line2D([0], [0], marker='o', color='w', markerfacecolor='green', 
           markersize=10, markeredgecolor='black', label='起点 (0.00)'),
    Line2D([0], [0], marker='o', color='w', markerfacecolor='red', 
           markersize=10, markeredgecolor='black', label='最高点 (0.12)'),
    Line2D([0], [0], marker='s', color='w', markerfacecolor='purple', 
           markersize=10, markeredgecolor='black', label='着地点 (0.25)'),
    Line2D([0], [0], marker='^', color='w', markerfacecolor='orange', 
           markersize=10, markeredgecolor='black', label='中点 (0.50)'),
    Line2D([0], [0], marker='D', color='w', markerfacecolor='cyan', 
           markersize=10, markeredgecolor='black', label='后段 (0.75)'),
]
ax4.legend(handles=legend_elements, loc='upper right', fontsize=8, 
          prop=chinese_font, framealpha=0.9)

ax4.axhline(y=0, color='k', linestyle='--', alpha=0.3)
ax4.axvline(x=0, color='k', linestyle='--', alpha=0.3)
ax4.set_xlabel('X偏移 (cm)', fontsize=11, fontproperties=chinese_font)
ax4.set_ylabel('Z偏移 (cm)', fontsize=11, fontproperties=chinese_font)
ax4.set_title('右前腿详细轨迹（修正后）', fontsize=12, fontweight='bold', fontproperties=chinese_font)
ax4.grid(True, alpha=0.3)
ax4.axis('equal')

plt.tight_layout()

# 保存图片
output_path = os.path.join(os.getcwd(), 'gait_trajectory_v2.png')
plt.savefig(output_path, dpi=150, bbox_inches='tight')

file_size = os.path.getsize(output_path) / 1024
print(f"\n✅ 轨迹图已保存: {output_path}")
print(f"   文件大小: {file_size:.1f} KB")
print("="*70)

plt.close()
