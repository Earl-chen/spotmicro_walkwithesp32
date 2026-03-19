#!/usr/bin/env python3
# test_walk_gait.py - Walk步态综合测试

import sys
import os

# 获取模块根目录（上一级）
module_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, module_root)

from gait.walk_gait import WalkGait
from gait.trajectory import TrajectoryGenerator

def test_trajectory():
    """测试轨迹生成器"""
    print("\n【测试1】轨迹生成器")
    print("-" * 60)
    
    stride = 0.04
    height = 0.025
    
    # 测试关键点
    test_cases = [
        (0.0, '摆动起点'),
        (0.125, '摆动中点'),
        (0.25, '摆动终点/支撑起点'),
        (0.5, '支撑中点'),
        (0.75, '支撑3/4'),
        (1.0, '支撑终点')
    ]
    
    all_pass = True
    
    for phase, desc in test_cases:
        x, z = TrajectoryGenerator.cycloid_trajectory(phase, stride, height)
        
        # 验证规则
        if 0.0 < phase < 0.25:
            # 摆动相中段：应该有Z高度
            if z < 0.001:
                print(f"  ❌ 相位{phase:.2f} ({desc}): 摆动相Z=0，应该有高度")
                all_pass = False
        elif phase >= 0.25:
            # 支撑相：Z应该为0
            if z > 0.001:
                print(f"  ❌ 相位{phase:.2f} ({desc}): 支撑相Z={z*100:.2f}cm，应该为0")
                all_pass = False
        
        print(f"  ✓ 相位{phase:.2f} ({desc}): x={x*100:+5.2f}cm, z={z*100:+5.2f}cm")
    
    if all_pass:
        print("\n  ✅ 轨迹生成器测试通过")
    else:
        print("\n  ❌ 轨迹生成器测试失败")
    
    return all_pass

def test_duty_cycle():
    """测试占空比"""
    print("\n【测试2】占空比")
    print("-" * 60)
    
    gait = WalkGait()
    legs = ['right_front', 'left_back', 'left_front', 'right_back']
    
    all_pass = True
    
    for leg in legs:
        swing = 0
        stance = 0
        
        for i in range(100):
            gait.global_phase = i / 100
            leg_p = gait.get_leg_phase(leg)
            
            if leg_p < 0.25:
                swing += 1
            else:
                stance += 1
        
        swing_pct = swing / 100 * 100
        stance_pct = stance / 100 * 100
        
        if abs(swing_pct - 25) < 1 and abs(stance_pct - 75) < 1:
            print(f"  ✓ {leg:15s}: 摆动{swing_pct:.0f}%, 支撑{stance_pct:.0f}% ✅")
        else:
            print(f"  ✗ {leg:15s}: 摆动{swing_pct:.0f}%, 支撑{stance_pct:.0f}% ❌")
            all_pass = False
    
    if all_pass:
        print("\n  ✅ 占空比测试通过")
    else:
        print("\n  ❌ 占空比测试失败")
    
    return all_pass

def test_stance_legs():
    """测试支撑腿数量"""
    print("\n【测试3】支撑腿数量")
    print("-" * 60)
    
    gait = WalkGait()
    legs = ['right_front', 'left_back', 'left_front', 'right_back']
    
    all_three_stance = True
    
    for i in range(100):
        gait.global_phase = i / 100
        
        stance_count = 0
        for leg in legs:
            leg_p = gait.get_leg_phase(leg)
            if leg_p >= 0.25:
                stance_count += 1
        
        if stance_count != 3:
            all_three_stance = False
            print(f"  ✗ 相位{i/100:.2f}: {stance_count}条腿支撑 ❌")
    
    if all_three_stance:
        print("  ✓ 所有时刻都是3条腿支撑 ✅")
        print("\n  ✅ 支撑腿数量测试通过")
    else:
        print("\n  ❌ 支撑腿数量测试失败")
    
    return all_three_stance

def main():
    """运行所有测试"""
    print("="*70)
    print("Walk步态综合测试")
    print("="*70)
    
    results = []
    
    # 运行测试
    results.append(("轨迹生成器", test_trajectory()))
    results.append(("占空比", test_duty_cycle()))
    results.append(("支撑腿数量", test_stance_legs()))
    
    # 总结
    print("\n" + "="*70)
    print("测试总结")
    print("="*70)
    
    for name, passed in results:
        status = "✅ 通过" if passed else "❌ 失败"
        print(f"  {name:20s}: {status}")
    
    all_passed = all(r[1] for r in results)
    
    print("\n" + "="*70)
    if all_passed:
        print("🎉 所有测试通过！Walk步态修正成功！")
    else:
        print("⚠️  有测试失败，请检查！")
    print("="*70)
    
    return all_passed

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
