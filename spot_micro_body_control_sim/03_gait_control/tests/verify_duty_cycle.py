#!/usr/bin/env python3
# verify_duty_cycle.py - 验证占空比和支撑腿数量

import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
import numpy as np

module_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, module_root)

from gait.walk_gait import WalkGait

# 配置中文字体（兼容低版本matplotlib）
font_candidates = [
    os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'fonts', 'BabelStoneHan.ttf'),
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

def verify_duty_cycle():
    """验证占空比和支撑腿数量"""
    
    gait = WalkGait()
    legs = ['right_front', 'left_back', 'left_front', 'right_back']
    leg_names_cn = {
        'right_front': '右前腿',
        'left_back': '左后腿',
        'left_front': '左前腿',
        'right_back': '右后腿'
    }
    
    print("="*70)
    print("Walk步态占空比验证报告")
    print("="*70)
    
    # 1. 验证占空比
    print("\n【1. 占空比验证】")
    print("  标准：摆动相25%，支撑相75%")
    print()
    
    all_duty_ok = True
    for leg in legs:
        swing_count = 0
        stance_count = 0
        
        for i in range(100):
            gait.global_phase = i / 100
            leg_p = gait.get_leg_phase(leg)
            
            if leg_p < 0.25:
                swing_count += 1
            else:
                stance_count += 1
        
        swing_pct = swing_count / 100 * 100
        stance_pct = stance_count / 100 * 100
        
        print(f"  {leg_names_cn[leg]:8s}: 摆动{swing_count}帧 ({swing_pct:.0f}%), "
              f"支撑{stance_count}帧 ({stance_pct:.0f}%)", end=' ')
        
        if abs(swing_pct - 25) < 1 and abs(stance_pct - 75) < 1:
            print('✅')
        else:
            print('❌')
            all_duty_ok = False
    
    # 2. 验证支撑腿数量
    print("\n【2. 支撑腿数量验证】")
    print("  标准：每时刻都有3条腿支撑")
    print()
    
    stance_counts = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0}
    
    for i in range(100):
        gait.global_phase = i / 100
        
        stance_count = 0
        for leg in legs:
            leg_p = gait.get_leg_phase(leg)
            if leg_p >= 0.25:
                stance_count += 1
        
        stance_counts[stance_count] += 1
    
    all_stance_ok = True
    for count in sorted(stance_counts.keys()):
        pct = stance_counts[count] / 100 * 100
        print(f"  {count}条腿支撑: {stance_counts[count]}帧 ({pct:.0f}%)", end=' ')
        
        if count == 3:
            print('✅ (正确)')
        elif stance_counts[count] > 0:  # 只有当帧数>0时才算错误
            print('❌ (错误)')
            all_stance_ok = False
        else:
            print('(未出现)')
    
    # 3. 关键相位验证
    print("\n【3. 关键相位验证】")
    
    key_phases = [0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875]
    
    for phase in key_phases:
        gait.global_phase = phase
        
        swing = []
        stance = []
        
        for leg in legs:
            leg_p = gait.get_leg_phase(leg)
            if leg_p < 0.25:
                swing.append(leg_names_cn[leg])
            else:
                stance.append(leg_names_cn[leg])
        
        print(f"\n  相位{phase:.3f}:")
        print(f"    摆动相（1条）: {swing[0] if swing else '无'}")
        print(f"    支撑相（3条）: {', '.join(stance)}")
        
        if len(stance) == 3:
            print(f"    ✅ 符合Walk步态")
        else:
            print(f"    ❌ 不符合Walk步态")
    
    # 4. 生成可视化
    print("\n【4. 生成可视化图表】")
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
    
    # 子图1: 相位条形图
    for i, leg in enumerate(legs):
        phase_offset = gait.phase_offsets[leg]
        
        # 摆动相（0-0.25）
        swing_start = phase_offset
        swing_end = phase_offset + 0.25
        
        # 支撑相（0.25-1.0）
        stance_start = (swing_end) % 1.0
        stance_end = (swing_end + 0.75) % 1.0
        
        # 绘制摆动相
        if swing_end <= 1.0:
            ax1.barh(i, 0.25, left=swing_start, height=0.7,
                    color='#FF6B6B', alpha=0.8, edgecolor='black', linewidth=2)
        else:
            ax1.barh(i, 1.0 - swing_start, left=swing_start, height=0.7,
                    color='#FF6B6B', alpha=0.8, edgecolor='black', linewidth=2)
            ax1.barh(i, swing_end - 1.0, left=0, height=0.7,
                    color='#FF6B6B', alpha=0.8, edgecolor='black', linewidth=2)
        
        # 绘制支撑相
        if stance_start < stance_end:
            ax1.barh(i, 0.75, left=stance_start, height=0.7,
                    color='#4ECDC4', alpha=0.8, edgecolor='black', linewidth=2)
        else:
            ax1.barh(i, 1.0 - stance_start, left=stance_start, height=0.7,
                    color='#4ECDC4', alpha=0.8, edgecolor='black', linewidth=2)
            ax1.barh(i, stance_end, left=0, height=0.7,
                    color='#4ECDC4', alpha=0.8, edgecolor='black', linewidth=2)
    
    ax1.set_yticks(range(len(legs)))
    ax1.set_yticklabels([leg_names_cn[leg] for leg in legs], fontsize=12)
    ax1.set_xlabel('全局相位', fontsize=12)
    ax1.set_title('Walk步态 - 修正后（摆动25%，支撑75%）', fontsize=14, fontweight='bold')
    ax1.set_xlim([0, 1])
    ax1.grid(True, alpha=0.3, axis='x')
    
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='#FF6B6B', alpha=0.8, edgecolor='black', label='摆动相（25%）'),
        Patch(facecolor='#4ECDC4', alpha=0.8, edgecolor='black', label='支撑相（75%）')
    ]
    ax1.legend(handles=legend_elements, loc='upper right', fontsize=11)
    
    # 子图2: 支撑腿数量随时间变化
    phases = np.linspace(0, 1, 100)
    stance_legs_count = []
    
    for phase in phases:
        gait.global_phase = phase
        count = sum(1 for leg in legs if gait.get_leg_phase(leg) >= 0.25)
        stance_legs_count.append(count)
    
    ax2.plot(phases, stance_legs_count, linewidth=3, color='#4ECDC4')
    ax2.fill_between(phases, 0, stance_legs_count, alpha=0.3, color='#4ECDC4')
    ax2.axhline(y=3, color='green', linestyle='--', linewidth=2, label='标准（3条腿）')
    ax2.set_xlabel('全局相位', fontsize=12)
    ax2.set_ylabel('支撑腿数量', fontsize=12)
    ax2.set_title('支撑腿数量随相位变化', fontsize=14, fontweight='bold')
    ax2.set_ylim([0, 4])
    ax2.set_yticks([0, 1, 2, 3, 4])
    ax2.legend(fontsize=11)
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    output_path = 'duty_cycle_verification.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"  ✅ 图表已保存: {output_path}")
    
    plt.close()
    
    print("\n" + "="*70)
    print("验证完成！")
    print("="*70)
    
    # 总结
    if all_duty_ok and all_stance_ok:
        print("\n🎉 所有验证通过！Walk步态修正成功！")
        return True
    else:
        print("\n⚠️  有验证失败，请检查！")
        return False

if __name__ == '__main__':
    success = verify_duty_cycle()
    sys.exit(0 if success else 1)
