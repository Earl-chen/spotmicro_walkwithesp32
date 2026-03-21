#!/usr/bin/env python3
"""
速度控制 API 演示脚本

展示新增的速度控制功能
"""

import sys
import os

# 添加模块路径
module_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, module_root)

from gait_algo_core.walk_gait import WalkGait

def demo_velocity_control():
    """演示速度控制 API"""
    
    print("=" * 70)
    print("速度控制 API 演示")
    print("=" * 70)
    
    # 创建步态控制器
    gait = WalkGait(stride_length=0.05, step_height=0.03, frequency=0.8)
    
    # ===========================================================================
    # 演示 1：set_velocity() - 统一速度控制
    # ===========================================================================
    print("\n【演示 1】set_velocity() - 统一速度控制")
    print("-" * 70)
    
    print("\n设置前进速度 = 0.05 m/s")
    gait.set_velocity(forward=0.05, lateral=0.0, yaw_rate=0.0)
    vel = gait.get_velocity()
    print(f"  当前速度: forward={vel['forward']:.3f}, lateral={vel['lateral']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")
    
    # 运行几步看看轨迹
    print("\n  运行 10 步，查看轨迹：")
    for i in range(10):
        gait.update(0.02)
        x, z = gait.get_foot_trajectory('right_front')
        print(f"    步 {i+1:2d}: x={x*1000:6.2f}mm, z={z*1000:6.2f}mm")
    
    # ===========================================================================
    # 演示 2：set_direction() - 简化方向控制
    # ===========================================================================
    print("\n【演示 2】set_direction() - 简化方向控制")
    print("-" * 70)
    
    print("\n设置方向 = 1.0（全速前进）")
    gait.set_direction(1.0)
    vel = gait.get_velocity()
    print(f"  当前速度: forward={vel['forward']:.3f}, lateral={vel['lateral']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")
    
    print("\n设置方向 = -1.0（全速后退）")
    gait.set_direction(-1.0)
    vel = gait.get_velocity()
    print(f"  当前速度: forward={vel['forward']:.3f}, lateral={vel['lateral']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")
    
    print("\n设置方向 = 0.0（停止）")
    gait.set_direction(0.0)
    vel = gait.get_velocity()
    print(f"  当前速度: forward={vel['forward']:.3f}, lateral={vel['lateral']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")
    
    # ===========================================================================
    # 演示 3：转向 + 前进组合
    # ===========================================================================
    print("\n【演示 3】转向 + 前进组合")
    print("-" * 70)
    
    print("\n设置：前进 0.05 m/s + 左转 0.3 rad/s")
    gait.set_velocity(forward=0.05, lateral=0.0, yaw_rate=0.3)
    vel = gait.get_velocity()
    print(f"  当前速度: forward={vel['forward']:.3f}, lateral={vel['lateral']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")
    
    print("\n  运行 10 步，查看左右腿轨迹差异：")
    for i in range(10):
        gait.update(0.02)
        x_left, z_left = gait.get_foot_trajectory('left_front')
        x_right, z_right = gait.get_foot_trajectory('right_front')
        print(f"    步 {i+1:2d}: 左腿 x={x_left*1000:6.2f}mm, 右腿 x={x_right*1000:6.2f}mm, 差值={(x_right-x_left)*1000:6.2f}mm")
    
    # ===========================================================================
    # 演示 4：速度限制
    # ===========================================================================
    print("\n【演示 4】速度限制功能")
    print("-" * 70)
    
    print("\n尝试设置超大速度：forward=1.0, lateral=0.5, yaw_rate=5.0")
    gait.set_velocity(forward=1.0, lateral=0.5, yaw_rate=5.0)
    vel = gait.get_velocity()
    print(f"  实际速度: forward={vel['forward']:.3f}, lateral={vel['lateral']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")
    print(f"  限制说明: 最大前进速度=0.100 m/s, 最大侧向速度=0.050 m/s, 最大角速度=1.000 rad/s")
    
    # ===========================================================================
    # 演示 5：动态速度变化
    # ===========================================================================
    print("\n【演示 5】动态速度变化")
    print("-" * 70)
    
    gait.reset()
    print("\n模拟加速过程：")
    for speed in [0.0, 0.02, 0.04, 0.06, 0.08, 0.1]:
        gait.set_velocity(forward=speed, lateral=0.0, yaw_rate=0.0)
        vel = gait.get_velocity()
        print(f"  设置速度 {speed:.2f} m/s → 实际速度 {vel['forward']:.3f} m/s")
    
    # ===========================================================================
    # 演示 6：get_state() 包含速度信息
    # ===========================================================================
    print("\n【演示 6】get_state() 包含完整速度信息")
    print("-" * 70)
    
    gait.set_velocity(forward=0.05, lateral=0.02, yaw_rate=0.3)
    state = gait.get_state()
    
    print("\n完整状态信息：")
    print(f"  步长: {state['stride_length']*1000:.1f} mm")
    print(f"  步高: {state['step_height']*1000:.1f} mm")
    print(f"  步频: {state['frequency']:.2f} Hz")
    print(f"  占空比: {state['duty_cycle']:.2f}")
    print(f"  转向角度: {state['steering_angle']:.3f} rad")
    print(f"  前进速度: {state['forward_speed']:.3f} m/s")
    print(f"  侧向速度: {state['lateral_speed']:.3f} m/s")
    print(f"  偏航角速度: {state['yaw_rate']:.3f} rad/s")
    
    print("\n" + "=" * 70)
    print("✅ 演示完成！")
    print("=" * 70)


if __name__ == '__main__':
    demo_velocity_control()
