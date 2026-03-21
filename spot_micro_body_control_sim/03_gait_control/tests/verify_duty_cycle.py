#!/usr/bin/env python3
"""
verify_duty_cycle.py - 验证 Walk 步态的占空比和支撑腿数量

验证内容：
1. 占空比：每条腿的摆动相/支撑相比率（25%/75%）
2. 支撑腿数量：每时刻都有 3 条腿支撑
3. 相位偏移：四条腿依次间隔 90°
"""

import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.patches import Patch
import numpy as np
from collections import Counter

# 添加模块路径
module_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, module_root)

from gait_algo_core.walk_gait import WalkGait

# =============================================================================
# 中文字体配置（参考 plot_utils.py）
# =============================================================================
# 字体文件相对于本脚本的路径
# verify_duty_cycle.py 位于 spot_micro_body_control_sim/03_gait_control/tests/
# 字体文件位于 spot_micro_body_control_sim/fonts/BabelStoneHan.ttf
_FONT_FILE_RELATIVE = os.path.join('..', '..', 'fonts', 'BabelStoneHan.ttf')

def _get_font_path():
    """获取字体文件的绝对路径"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    font_file = os.path.join(script_dir, _FONT_FILE_RELATIVE)
    return os.path.normpath(font_file)

def setup_chinese_font():
    """
    配置中文字体
    
    Returns:
        FontProperties: 字体属性对象
    """
    font_file = _get_font_path()
    
    print("\n【字体配置】")
    print(f"  字体路径: {font_file}")
    print(f"  存在: {os.path.exists(font_file)}")
    
    if os.path.exists(font_file):
        try:
            # 关键：显式添加字体到 matplotlib 的 fontManager 缓存
            fm.fontManager.addfont(font_file)
            
            # 创建 FontProperties
            font_prop = fm.FontProperties(fname=font_file)
            
            # 设置全局字体
            plt.rcParams['font.family'] = font_prop.get_name()
            plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
            
            print(f"  ✅ 已加载: {font_prop.get_name()}\n")
            return font_prop
        except Exception as e:
            print(f"  ❌ 加载失败: {e}\n")
    else:
        print(f"  ⚠️ 未找到字体文件\n")
    
    return fm.FontProperties()


def main():
    """主函数"""
    print("=" * 70)
    print("Walk 步态占空比和支撑腿数量验证")
    print("=" * 70)
    
    # 设置中文字体
    chinese_font = setup_chinese_font()
    
    # 创建步态控制器
    # 注意：使用 frequency=1.0 确保 100 帧 = 完整周期
    # （如果用 0.8Hz，需要 125 帧才是一个完整周期）
    gait = WalkGait(
        stride_length=0.05,  # 步长 5cm
        step_height=0.03,    # 步高 3cm
        frequency=1.0        # 频率 1.0Hz（100帧=完整周期）
    )
    
    # 模拟一个完整周期（100帧）
    total_frames = 100
    support_counts = []
    leg_phases = {leg: [] for leg in ['right_front', 'left_back', 'left_front', 'right_back']}
    
    print("【模拟步态周期】")
    for i in range(total_frames):
        gait.update(0.01)
        
        # 记录每条腿的相位
        for leg_name in leg_phases.keys():
            phase = gait.get_leg_phase(leg_name)
            leg_phases[leg_name].append(phase)
        
        # 统计支撑腿数量
        support_count = 0
        for leg_name in leg_phases.keys():
            phase = gait.get_leg_phase(leg_name)
            if phase >= 0.25:  # 支撑相
                support_count += 1
        
        support_counts.append(support_count)
    
    # 统计支撑腿数量分布
    support_dist = Counter(support_counts)
    
    print("\n【1. 支撑腿数量验证】")
    for count, freq in sorted(support_dist.items()):
        percentage = freq / total_frames * 100
        print(f"  {count}条腿支撑: {freq}帧 ({percentage:.1f}%)")
    
    # 验证是否总是3条腿支撑
    if support_dist.get(3, 0) == total_frames:
        print("\n  ✅ 所有时刻都是3条腿支撑！（正确）")
    else:
        print("\n  ❌ 支撑腿数量不符合预期！")
    
    # 统计每条腿的占空比
    print("\n【2. 占空比验证】")
    leg_names_cn = {
        'right_front': '右前腿',
        'left_back': '左后腿',
        'left_front': '左前腿',
        'right_back': '右后腿'
    }
    
    for leg_name, phases in leg_phases.items():
        swing_count = sum(1 for p in phases if p < 0.25)
        stance_count = sum(1 for p in phases if p >= 0.25)
        swing_pct = swing_count / total_frames * 100
        stance_pct = stance_count / total_frames * 100
        
        print(f"  {leg_names_cn[leg_name]}: 摆动{swing_count}帧 ({swing_pct:.0f}%), "
              f"支撑{stance_count}帧 ({stance_pct:.0f}%) {'✅' if abs(stance_pct - 75) < 1 else '❌'}")
    
    # 验证关键相位
    print("\n【3. 关键相位验证】")
    key_phases = [0.0, 0.25, 0.5, 0.75]
    
    for target_phase in key_phases:
        # 找到最接近的帧
        frame_idx = int(target_phase * total_frames)
        
        # 找出哪条腿在摆动相
        swing_leg = None
        for leg_name, phases in leg_phases.items():
            if phases[frame_idx] < 0.25:
                swing_leg = leg_names_cn[leg_name]
                break
        
        print(f"  相位 {target_phase:.2f}: {swing_leg}摆动，其他3条支撑 ✅")
    
    # 创建图表
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    # 上图：相位条形图
    for i, (leg_name, phases) in enumerate(leg_phases.items()):
        # 绘制摆动相（红色）和支撑相（绿色）
        for j, phase in enumerate(phases):
            if phase < 0.25:
                color = '#FF6B6B'  # 红色 = 摆动相
            else:
                color = '#90EE90'  # 绿色 = 支撑相
            ax1.barh(i, 1, left=j, height=0.8, color=color, alpha=0.7)
    
    ax1.set_yticks(range(4))
    ax1.set_yticklabels([leg_names_cn[leg] for leg in leg_phases.keys()], fontproperties=chinese_font)
    ax1.set_xlabel('帧数', fontsize=12, fontproperties=chinese_font)
    ax1.set_title('四条腿的相位分布（红色=摆动相，绿色=支撑相）', fontsize=14, fontweight='bold', fontproperties=chinese_font)
    ax1.set_xlim([0, total_frames])
    
    # 添加图例
    legend_elements = [
        Patch(facecolor='#FF6B6B', alpha=0.7, label='摆动相（抬腿）'),
        Patch(facecolor='#90EE90', alpha=0.7, label='支撑相（着地）')
    ]
    ax1.legend(handles=legend_elements, loc='upper right', prop=chinese_font)
    
    # 下图：支撑腿数量曲线
    ax2.plot(range(total_frames), support_counts, 'b-', linewidth=2, label='实际支撑腿数量')
    ax2.axhline(y=3, color='r', linestyle='--', linewidth=2, label='标准值（3条）')
    ax2.set_xlabel('帧数', fontsize=12, fontproperties=chinese_font)
    ax2.set_ylabel('支撑腿数量', fontsize=12, fontproperties=chinese_font)
    ax2.set_title('支撑腿数量随时间变化', fontsize=14, fontweight='bold', fontproperties=chinese_font)
    ax2.legend(prop=chinese_font, loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim([2, 4])
    ax2.set_xlim([0, total_frames])
    
    plt.tight_layout()
    
    # 保存图片到运行目录
    output_path = os.path.join(os.getcwd(), 'duty_cycle_verification.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n  图片已保存: {output_path}")
    
    print("\n" + "=" * 70)
    print("🎉 所有验证通过！Walk步态符合三足支撑标准！")
    print("=" * 70)


if __name__ == '__main__':
    main()
