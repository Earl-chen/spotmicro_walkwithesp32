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

from gait_algo_core.trajectory import TrajectoryGenerator

# 配置中文字体（直接使用 BabelStoneHan.ttf，仅使用相对路径）
print("=" * 70)
print("【字体加载调试信息】")
print("=" * 70)

# 仅使用相对路径
font_candidates = [
    # 从 tests/verify_trajectory.py 向上3级到 spot_micro_body_control_sim/fonts/
    os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'fonts', 'BabelStoneHan.ttf'),
]

print(f"字体候选路径数量: {len(font_candidates)}")
for i, path in enumerate(font_candidates):
    abs_path = os.path.abspath(path)
    print(f"  路径{i+1}: {abs_path}")
    print(f"    存在: {os.path.exists(abs_path)}")
    if os.path.exists(abs_path):
        print(f"    大小: {os.path.getsize(abs_path) / 1024 / 1024:.2f} MB")

chinese_font = None
font_loaded = False

for font_path in font_candidates:
    if os.path.exists(font_path):
        print(f"\n尝试加载字体: {font_path}")
        try:
            # 步骤1：清除字体缓存
            print("  步骤1: 清除字体缓存...")
            try:
                fm._load_fontmanager(try_read_cache=False)
                print("    ✅ 字体缓存已清除")
            except Exception as e:
                print(f"    ⚠️ 清除缓存失败: {e}")
            
            # 步骤2：注册字体
            print("  步骤2: 注册字体...")
            if hasattr(fm.fontManager, 'addfont'):
                fm.fontManager.addfont(font_path)
                print(f"    ✅ addfont 成功")
            else:
                print(f"    ⚠️ fontManager 没有 addfont 方法")
            
            # 步骤3：创建 FontProperties
            print("  步骤3: 创建 FontProperties...")
            chinese_font = fm.FontProperties(fname=font_path)
            font_name = chinese_font.get_name()
            print(f"    ✅ 字体名称: {font_name}")
            
            # 步骤4：检查字体管理器
            print("  步骤4: 检查字体管理器...")
            registered_fonts = [f.name for f in fm.fontManager.ttflist]
            if font_name in registered_fonts:
                print(f"    ✅ 字体已在字体管理器中")
            else:
                print(f"    ❌ 字体不在字体管理器中")
                print(f"    尝试手动添加...")
                # 手动添加到字体列表
                font_entry = fm.FontEntry(
                    fname=font_path,
                    name=font_name,
                    style='normal',
                    variant='normal',
                    weight='normal',
                    stretch='normal',
                    size='medium'
                )
                fm.fontManager.ttflist.append(font_entry)
                print(f"    ✅ 已手动添加到字体列表")
            
            # 步骤5：设置全局字体
            print("  步骤5: 设置全局字体...")
            plt.rcParams['font.family'] = font_name
            plt.rcParams['font.sans-serif'] = [font_name, 'DejaVu Sans', 'Arial Unicode MS']
            plt.rcParams['axes.unicode_minus'] = False
            print(f"    ✅ font.family = {font_name}")
            print(f"    ✅ font.sans-serif = {plt.rcParams['font.sans-serif']}")
            
            font_loaded = True
            print("\n✅ 字体加载完成！")
            break
        except Exception as e:
            print(f"  ❌ 字体加载失败: {e}")
            import traceback
            traceback.print_exc()
            continue

if not font_loaded:
    print("\n❌ 未找到中文字体，使用默认字体")
    chinese_font = fm.FontProperties()

print("=" * 70)

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
        x, y, z = TrajectoryGenerator.cycloid_trajectory(phase, stride_length, step_height)
        x_values.append(x * 100)  # 转换为cm
        z_values.append(z * 100)
    
    # 输出关键点
    print("\n【关键点验证】")
    for phase in [0.0, 0.25, 0.5, 0.75, 1.0]:
        x, y, z = TrajectoryGenerator.cycloid_trajectory(phase, stride_length, step_height)
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
    print("\n【绘图调试信息】")
    print(f"  当前 font.family: {plt.rcParams['font.family']}")
    print(f"  当前 font.sans-serif: {plt.rcParams['font.sans-serif']}")
    print(f"  chinese_font 对象: {chinese_font}")
    print(f"  chinese_font.get_name(): {chinese_font.get_name()}")
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    
    # 子图1: X和Z随相位变化
    ax1.plot(phases, x_values, label='X偏移（前后）', linewidth=2, color='blue')
    ax1.plot(phases, z_values, label='Z偏移（上下）', linewidth=2, color='red')
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax1.axvline(x=0.5, color='gray', linestyle=':', alpha=0.5, label='摆动相/支撑相分界')
    ax1.set_xlabel('相位', fontsize=12, fontproperties=chinese_font)
    ax1.set_ylabel('偏移量 (cm)', fontsize=12, fontproperties=chinese_font)
    ax1.set_title('足端轨迹随相位变化（修正后）', fontsize=14, fontweight='bold', fontproperties=chinese_font)
    ax1.legend(fontsize=10, prop=chinese_font)
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
    ax2.set_xlabel('X偏移 (cm)', fontsize=12, fontproperties=chinese_font)
    ax2.set_ylabel('Z偏移 (cm)', fontsize=12, fontproperties=chinese_font)
    ax2.set_title('足端2D轨迹（修正后）', fontsize=14, fontweight='bold', fontproperties=chinese_font)
    ax2.legend(fontsize=10, prop=chinese_font)
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
