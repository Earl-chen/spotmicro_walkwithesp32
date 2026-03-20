#!/usr/bin/env python3
# test_walk_gait.py - Walk 步态独立测试

"""
Walk 步态独立测试程序

这个文件演示如何使用 Walk 步态模块，不影响现有功能。

运行方式：
    cd tests/
    python3 test_walk_gait.py
"""

import sys
import os
import time

# 添加模块根目录到 Python 路径
module_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, module_root)

# 导入步态模块
from gait_algo_core.trajectory import TrajectoryGenerator
from gait_algo_core.walk_gait import WalkGait


def test_trajectory_generator():
    """测试轨迹生成器"""
    print("\n" + "="*60)
    print("测试 1: 轨迹生成器")
    print("="*60)
    
    # 测试摆线轨迹
    print("\n摆线轨迹（推荐）:")
    print("相位  | X偏移(mm) | Z偏移(mm) | 说明")
    print("-" * 50)
    for phase in [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]:
        x, z = TrajectoryGenerator.cycloid_trajectory(phase, 0.05, 0.03)
        desc = "摆动相" if phase < 0.5 else "支撑相"
        print(f"{phase:4.1f}  | {x*1000:9.1f} | {z*1000:9.1f} | {desc}")
    
    # 测试椭圆轨迹
    print("\n椭圆轨迹:")
    print("相位  | X偏移(mm) | Z偏移(mm)")
    print("-" * 40)
    for phase in [0.0, 0.25, 0.5, 0.75, 1.0]:
        x, z = TrajectoryGenerator.ellipse_trajectory(phase, 0.05, 0.03)
        print(f"{phase:4.2f}  | {x*1000:9.1f} | {z*1000:9.1f}")
    
    print("\n✅ 轨迹生成器测试通过")


def test_walk_gait():
    """测试 Walk 步态"""
    print("\n" + "="*60)
    print("测试 2: Walk 步态")
    print("="*60)
    
    # 创建 Walk 步态实例
    gait = WalkGait(stride_length=0.05, step_height=0.03, frequency=0.8)
    
    print(f"\n步态参数:")
    print(f"  步长: {gait.stride_length*1000:.1f}mm")
    print(f"  步高: {gait.step_height*1000:.1f}mm")
    print(f"  步频: {gait.frequency:.2f}Hz")
    print(f"  占空比: 0.75 (Walk步态)")
    
    print(f"\n相位偏移:")
    for leg_name, offset in gait.phase_offsets.items():
        print(f"  {leg_name:15s}: {offset:.2f} ({offset*90:.0f}°)")
    
    # 测试相位调度
    print("\n测试相位调度（一个完整周期）:")
    dt = 0.02  # 20ms 时间步
    steps_per_cycle = int(1.0 / (gait.frequency * dt))
    
    print(f"  每周期步数: {steps_per_cycle}")
    print(f"  周期时长: {1.0/gait.frequency:.2f}秒")
    
    # 模拟一个周期
    print("\n模拟一个完整周期:")
    for i in range(min(10, steps_per_cycle)):
        gait.update(dt)
        trajectories = gait.get_all_foot_trajectories()
        
        if i % 2 == 0:  # 每2步输出一次
            print(f"\n步骤 {i+1:2d}, 全局相位: {gait.global_phase:.2f}")
            for leg_name, (x, z) in trajectories.items():
                phase = gait.get_leg_phase(leg_name)
                state = "抬腿" if phase < 0.5 else "着地"
                print(f"  {leg_name:15s}: x={x*1000:6.1f}mm, z={z*1000:6.1f}mm, [{state}]")
    
    print("\n✅ Walk 步态测试通过")


def test_gait_parameter_adjustment():
    """测试参数调整"""
    print("\n" + "="*60)
    print("测试 3: 参数实时调整")
    print("="*60)
    
    gait = WalkGait()
    
    print("\n初始参数:")
    state = gait.get_state()
    print(f"  步长: {state['stride_length']*1000:.1f}mm")
    print(f"  步高: {state['step_height']*1000:.1f}mm")
    print(f"  步频: {state['frequency']:.2f}Hz")
    
    # 调整参数
    print("\n调整参数...")
    gait.set_parameters(stride_length=0.06, step_height=0.04, frequency=1.0)
    
    print("\n调整后参数:")
    state = gait.get_state()
    print(f"  步长: {state['stride_length']*1000:.1f}mm")
    print(f"  步高: {state['step_height']*1000:.1f}mm")
    print(f"  步频: {state['frequency']:.2f}Hz")
    
    # 测试轨迹类型切换
    print("\n切换轨迹类型...")
    gait.set_trajectory_type('ellipse')
    print(f"  轨迹类型: {gait.trajectory_type}")
    
    # 测试重置
    print("\n重置步态...")
    gait.reset()
    print(f"  全局相位: {gait.global_phase:.2f}")
    
    print("\n✅ 参数调整测试通过")


def test_continuous_gait():
    """测试连续步态运行"""
    print("\n" + "="*60)
    print("测试 4: 连续步态运行（5秒）")
    print("="*60)
    
    gait = WalkGait(stride_length=0.05, step_height=0.03, frequency=0.8)
    
    print("\n开始连续运行...")
    print("模拟 5 秒钟的步态运行，每秒输出一次状态")
    
    dt = 0.02  # 20ms
    total_time = 5.0
    steps = int(total_time / dt)
    
    start_time = time.time()
    last_report_time = start_time
    
    for i in range(steps):
        gait.update(dt)
        current_time = time.time()
        
        # 每秒输出一次
        if current_time - last_report_time >= 1.0:
            elapsed = current_time - start_time
            trajectories = gait.get_all_foot_trajectories()
            
            # 计算平均抬腿高度
            avg_z = np.mean([z for x, z in trajectories.values()])
            
            print(f"时间: {elapsed:.1f}s, 全局相位: {gait.global_phase:.2f}, 平均抬腿: {avg_z*1000:.1f}mm")
            last_report_time = current_time
    
    print("\n✅ 连续步态测试通过")


def main():
    """主测试函数"""
    print("="*60)
    print("Walk 步态独立测试程序")
    print("="*60)
    print("\n这个程序测试 Walk 步态模块的基本功能")
    print("不影响现有的 Python 仿真系统")
    
    try:
        # 运行所有测试
        test_trajectory_generator()
        test_walk_gait()
        test_gait_parameter_adjustment()
        test_continuous_gait()
        
        print("\n" + "="*60)
        print("🎉 所有测试通过！")
        print("="*60)
        print("\n下一步:")
        print("1. 查看测试结果，理解 Walk 步态的工作原理")
        print("2. 在 run_spot_micro.py 中集成步态控制器")
        print("3. 添加可视化界面控制")
        print("4. 测试实际机器人")
        
    except Exception as e:
        print(f"\n❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    import numpy as np
    main()
