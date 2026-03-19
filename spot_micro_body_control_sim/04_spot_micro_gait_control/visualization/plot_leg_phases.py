#!/usr/bin/env python3
# plot_leg_phases.py - 绘制每条腿的支撑相/摆动相图

import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
import numpy as np

# 添加项目根目录
module_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, module_root)

from gait.walk_gait import WalkGait

# 配置中文字体（兼容低版本matplotlib）
# 优先使用项目目录下的字体，然后检查其他候选字体
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

def plot_leg_phases():
    """绘制每条腿的支撑相/摆动相相位图"""
    
    # 创建步态控制器
    gait = WalkGait(stride_length=0.04, step_height=0.025, frequency=0.8)
    
    # 腿的名称和顺序
    leg_names = ['right_front', 'left_back', 'left_front', 'right_back']
    leg_names_cn = {
        'right_front': '右前腿',
        'left_back': '左后腿',
        'left_front': '左前腿',
        'right_back': '右后腿'
    }
    
    # 生成全局相位数据
    global_phases = np.linspace(0, 1, 100)
    
    # 创建图形
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
    
    # ========== 子图1: 相位条形图 ==========
    for i, leg_name in enumerate(leg_names):
        phase_offset = gait.phase_offsets[leg_name]
        
        # 计算摆动相和支撑相的区间
        swing_start = phase_offset
        swing_end = phase_offset + 0.5
        stance_start = swing_end % 1.0  # 支撑相起点（处理循环）
        stance_end = (swing_end + 0.5) % 1.0  # 支撑相终点
        
        # 处理相位跨越1.0的情况
        if swing_end <= 1.0:
            # 摆动相不跨越边界
            ax1.barh(i, 0.5, left=swing_start, height=0.7, 
                    color='#FF6B6B', alpha=0.8, edgecolor='black', linewidth=2)
            # 支撑相可能跨越边界
            if stance_end > stance_start:
                ax1.barh(i, 0.5, left=stance_start, height=0.7,
                        color='#4ECDC4', alpha=0.8, edgecolor='black', linewidth=2)
            else:
                # 支撑相跨越1.0
                ax1.barh(i, 1.0 - stance_start, left=stance_start, height=0.7,
                        color='#4ECDC4', alpha=0.8, edgecolor='black', linewidth=2)
                ax1.barh(i, stance_end, left=0, height=0.7,
                        color='#4ECDC4', alpha=0.8, edgecolor='black', linewidth=2)
        else:
            # 摆动相跨越1.0
            ax1.barh(i, 1.0 - swing_start, left=swing_start, height=0.7,
                    color='#FF6B6B', alpha=0.8, edgecolor='black', linewidth=2)
            ax1.barh(i, swing_end - 1.0, left=0, height=0.7,
                    color='#FF6B6B', alpha=0.8, edgecolor='black', linewidth=2)
            # 支撑相不跨越边界
            ax1.barh(i, 0.5, left=stance_start, height=0.7,
                    color='#4ECDC4', alpha=0.8, edgecolor='black', linewidth=2)
    
    ax1.set_yticks(range(len(leg_names)))
    ax1.set_yticklabels([leg_names_cn[leg] for leg in leg_names], fontsize=12)
    ax1.set_xlabel('全局相位', fontsize=12)
    ax1.set_title('Walk步态 - 各腿支撑相/摆动相分布', fontsize=14, fontweight='bold')
    ax1.set_xlim([0, 1])
    ax1.set_ylim([-0.5, len(leg_names) - 0.5])
    ax1.grid(True, alpha=0.3, axis='x')
    
    # 添加图例
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='#FF6B6B', alpha=0.8, edgecolor='black', label='摆动相（抬腿）'),
        Patch(facecolor='#4ECDC4', alpha=0.8, edgecolor='black', label='支撑相（着地）')
    ]
    ax1.legend(handles=legend_elements, loc='upper right', fontsize=11)
    
    # 添加关键相位线
    for phase in [0.25, 0.5, 0.75]:
        ax1.axvline(x=phase, color='gray', linestyle='--', alpha=0.5, linewidth=1.5)
    
    # ========== 子图2: 动态相位变化图 ==========
    for i, leg_name in enumerate(leg_names):
        phases = []
        states = []
        
        for global_phase in global_phases:
            gait.global_phase = global_phase
            leg_phase = gait.get_leg_phase(leg_name)
            is_swing = 1 if leg_phase < 0.5 else 0
            
            phases.append(global_phase)
            states.append(is_swing)
        
        # 绘制状态曲线
        ax2.plot(phases, [s + i * 2 for s in states], 
                linewidth=3, label=leg_names_cn[leg_name], alpha=0.8)
        
        # 填充区域
        ax2.fill_between(phases, i * 2, [s + i * 2 for s in states],
                        alpha=0.3)
    
    ax2.set_xlabel('全局相位', fontsize=12)
    ax2.set_ylabel('腿的状态（0=支撑相，1=摆动相）', fontsize=12)
    ax2.set_title('各腿相位随时间变化', fontsize=14, fontweight='bold')
    ax2.set_xlim([0, 1])
    ax2.set_ylim([-0.5, len(leg_names) * 2 - 0.5])
    ax2.set_yticks([i * 2 + 0.5 for i in range(len(leg_names))])
    ax2.set_yticklabels([leg_names_cn[leg] for leg in leg_names], fontsize=11)
    ax2.legend(loc='upper right', fontsize=10)
    ax2.grid(True, alpha=0.3)
    
    # 添加关键相位线和说明
    for phase in [0.25, 0.5, 0.75]:
        ax2.axvline(x=phase, color='gray', linestyle='--', alpha=0.5, linewidth=1.5)
    
    plt.tight_layout()
    
    # 保存图片
    output_path = os.path.join(os.path.dirname(__file__), 'leg_phase_diagram.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"✅ 相位图已保存: {output_path}")
    
    plt.close()
    
    # 输出详细说明
    print("\n" + "="*70)
    print("Walk步态相位分析")
    print("="*70)
    print("\n【相位安排】")
    for leg_name in leg_names:
        phase_offset = gait.phase_offsets[leg_name]
        print(f"  {leg_names_cn[leg_name]:8s}: 相位偏移 {phase_offset:.2f} ({phase_offset*360:.0f}°)")
    
    print("\n【关键相位时刻】")
    for phase in [0.0, 0.25, 0.5, 0.75]:
        gait.global_phase = phase
        print(f"\n  全局相位 {phase:.2f}:")
        
        swing_legs = []
        stance_legs = []
        
        for leg_name in leg_names:
            leg_phase = gait.get_leg_phase(leg_name)
            if leg_phase < 0.5:
                swing_legs.append(leg_names_cn[leg_name])
            else:
                stance_legs.append(leg_names_cn[leg_name])
        
        print(f"    摆动相（抬腿）: {', '.join(swing_legs)}")
        print(f"    支撑相（着地）: {', '.join(stance_legs)}")
        print(f"    着地腿数: {len(stance_legs)}/4")
    
    print("\n【Walk步态特点】")
    print("  ✓ 任何时刻都有3只脚着地（稳定性高）")
    print("  ✓ 4条腿依次抬起（相位差90°）")
    print("  ✓ 占空比75%（每条腿75%时间着地）")
    print("  ✓ 适合复杂地形和慢速行走")
    print("="*70)

if __name__ == '__main__':
    plot_leg_phases()
