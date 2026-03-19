#!/usr/bin/env python3
# verify_support_legs.py - 验证每时刻有几条腿支撑

import sys
import os

module_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, module_root)

from gait.walk_gait import WalkGait

def verify_support_legs():
    """验证每个时刻有几条腿支撑"""
    
    gait = WalkGait(stride_length=0.04, step_height=0.025, frequency=0.8)
    
    leg_names = ['right_front', 'left_back', 'left_front', 'right_back']
    leg_names_cn = {
        'right_front': '右前腿',
        'left_back': '左后腿',
        'left_front': '左前腿',
        'right_back': '右后腿'
    }
    
    print("="*70)
    print("验证：每个时刻有几条腿支撑？")
    print("="*70)
    
    # 检查关键相位点
    for global_phase in [0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875]:
        gait.global_phase = global_phase
        
        swing_legs = []
        stance_legs = []
        
        for leg_name in leg_names:
            leg_phase = gait.get_leg_phase(leg_name)
            is_swing = leg_phase < 0.25  # ✅ 修正：0.5 → 0.25（25%摆动相）
            
            if is_swing:
                swing_legs.append(leg_names_cn[leg_name])
            else:
                stance_legs.append(leg_names_cn[leg_name])
        
        print(f"\n全局相位 {global_phase:.3f}:")
        print(f"  摆动相（抬腿）: {len(swing_legs)}条 - {', '.join(swing_legs) if swing_legs else '无'}")
        print(f"  支撑相（着地）: {len(stance_legs)}条 - {', '.join(stance_legs) if stance_legs else '无'}")
        
        if len(stance_legs) == 3:
            print(f"  ✅ 符合Walk步态（3足支撑）")
        else:
            print(f"  ❌ 不符合Walk步态（应该3足支撑，实际{len(stance_legs)}足）")
    
    print("\n" + "="*70)
    print("统计：完整周期分析")
    print("="*70)
    
    # 统计完整周期
    stance_counts = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0}
    
    for i in range(100):
        global_phase = i / 100
        gait.global_phase = global_phase
        
        stance_count = 0
        for leg_name in leg_names:
            leg_phase = gait.get_leg_phase(leg_name)
            if leg_phase >= 0.25:  # ✅ 修正：0.5 → 0.25（75%支撑相）
                stance_count += 1
        
        stance_counts[stance_count] += 1
    
    print("\n支撑腿数量分布：")
    for count in sorted(stance_counts.keys()):
        percentage = stance_counts[count] / 100 * 100
        print(f"  {count}条腿支撑: {stance_counts[count]}帧 ({percentage:.1f}%)")
    
    print("\n" + "="*70)

if __name__ == '__main__':
    verify_support_legs()
