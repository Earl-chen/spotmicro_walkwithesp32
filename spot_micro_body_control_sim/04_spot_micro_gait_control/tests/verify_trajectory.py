#!/usr/bin/env python3
# verify_trajectory.py - 验证轨迹生成正确性

import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
import numpy as np

# 添加项目根目录到 Python 路径
module_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, module_root)

from gait.trajectory import TrajectoryGenerator

# 配置中文字体（兼容低版本matplotlib）
font_candidates = [
    os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'fonts', 'BabelStoneHan.ttf'),
    os.path.expanduser('~/BabelStoneHan.ttf'),
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
            break
        except:
            continue

if chinese_font is None:
    chinese_font = fm.FontProperties()

def verify_trajectory():
    """验证修正后的轨迹"""
    
    print("="*70)
    print("轨迹生成器验证报告")
    print("="*70)
    
    stride_length = 0.04  # 4cm
    step_height = 0.025   # 2.5cm
    
    phases = np.linspace(0, 1, 100)
    x_values = []
    z_values = []
    
    for phase in phases:
        x, z = TrajectoryGenerator.cycloid_trajectory(phase, stride_length, step_height)
        x_values.append(x * 100)  # 转换为cm
        z_values.append(z * 100)
    
    # 输出关键点
    print("\n【关键点验证】")
    for phase in [0.0, 0.25, 0.5, 0.75, 1.0]:
        x, z = TrajectoryGenerator.cycloid_trajectory(phase, stride_length, step_height)
        print(f"  相位{phase:.2f}: x={x*100:+6.2f}cm, z={z*100:+6.2f}cm")
    
    # 验证范围
    x_min, x_max = min(x_values), max(x_values)
    z_min, z_max = min(z_values), max(z_values)
    
    print("\n【范围验证】")
    print(f"  X范围: {x_min:+.2f}cm 到 {x_max:+.2f}cm (范围: {x_max-x_min:.2f}cm)")
    print(f"  Z范围: {z_min:+.2f}cm 到 {z_max:+.2f}cm (范围: {z_max-z_min:.2f}cm)")
    
    # 验证标准
    print("\n【验证标准】")
    checks = []
    
    # 检查1：X范围对称
    if abs(x_min + x_max) < 0.1:  # 接近0
        print("  ✅ X范围对称（关于原点）")
        checks.append(True)
    else:
        print(f"  ❌ X范围不对称（min={x_min:.2f}, max={x_max:.2f}）")
        checks.append(False)
    
    # 检查2：X范围接近步长
    if abs((x_max - x_min) - stride_length * 100) < 0.5:
        print(f"  ✅ X范围接近步长（{stride_length*100:.1f}cm）")
        checks.append(True)
    else:
        print(f"  ❌ X范围不符合步长（期望{stride_length*100:.1f}cm，实际{x_max-x_min:.2f}cm）")
        checks.append(False)
    
    # 检查3：Z最大值接近步高
    if abs(z_max - step_height * 100) < 0.5:
        print(f"  ✅ Z最大值接近步高（{step_height*100:.1f}cm）")
        checks.append(True)
    else:
        print(f"  ❌ Z最大值不符合步高（期望{step_height*100:.1f}cm，实际{z_max:.2f}cm）")
        checks.append(False)
    
    # 检查4：起点=终点
    if abs(x_values[0] - x_values[-1]) < 0.01:
        print("  ✅ 起点=终点（轨迹闭合）")
        checks.append(True)
    else:
        print(f"  ❌ 起点≠终点（起点={x_values[0]:.2f}cm，终点={x_values[-1]:.2f}cm）")
        checks.append(False)
    
    # 绘制轨迹
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    
    # 子图1: X和Z随相位变化
    ax1.plot(phases, x_values, label='X偏移（前后）', linewidth=2, color='blue')
    ax1.plot(phases, z_values, label='Z偏移（上下）', linewidth=2, color='red')
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax1.axvline(x=0.5, color='gray', linestyle=':', alpha=0.5, label='摆动相/支撑相分界')
    ax1.set_xlabel('相位', fontsize=12)
    ax1.set_ylabel('偏移量 (cm)', fontsize=12)
    ax1.set_title('足端轨迹随相位变化（修正后）', fontsize=14, fontweight='bold')
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)
    
    # 子图2: 2D轨迹
    ax2.plot(x_values, z_values, linewidth=2, color='green')
    ax2.scatter(x_values[0], z_values[0], color='green', s=150, marker='o', 
                edgecolors='black', linewidths=2, label='起点（相位0）', zorder=5)
    ax2.scatter(x_values[25], z_values[25], color='red', s=150, marker='^', 
                edgecolors='black', linewidths=2, label='最高点（相位0.25）', zorder=5)
    ax2.scatter(x_values[50], z_values[50], color='blue', s=150, marker='s', 
                edgecolors='black', linewidths=2, label='着地点（相位0.5）', zorder=5)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.axvline(x=0, color='k', linestyle='--', alpha=0.3)
    ax2.set_xlabel('X偏移 (cm)', fontsize=12)
    ax2.set_ylabel('Z偏移 (cm)', fontsize=12)
    ax2.set_title('足端2D轨迹（修正后）', fontsize=14, fontweight='bold')
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    
    plt.tight_layout()
    
    output_path = os.path.join(os.path.dirname(__file__), 'trajectory_verification.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n✅ 验证图表已保存: {output_path}")
    
    plt.close()
    
    # 总结
    print("\n" + "="*70)
    if all(checks):
        print("✅ 所有验证通过！轨迹生成器修正成功！")
        return True
    else:
        print(f"⚠️  有 {checks.count(False)} 项验证失败，请检查代码！")
        return False
    
    print("="*70)

if __name__ == '__main__':
    verify_trajectory()
