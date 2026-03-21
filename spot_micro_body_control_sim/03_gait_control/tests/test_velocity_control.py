#!/usr/bin/env python3
"""
速度控制功能测试脚本

测试 Walk 步态的速度控制 API：
- set_velocity(forward, lateral, yaw_rate)
- set_direction(forward)
- get_velocity()
"""

import sys
import os
import numpy as np

# 添加模块路径
module_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, module_root)

from gait_algo_core.walk_gait import WalkGait


def test_set_velocity_basic():
    """测试 set_velocity 基本功能"""
    print("\n" + "="*60)
    print("测试1：set_velocity 基本功能")
    print("="*60)
    
    gait = WalkGait()
    
    # 测试1.1：前进
    print("\n【测试1.1】前进速度设置")
    gait.set_velocity(forward=0.05)
    vel = gait.get_velocity()
    print(f"  设置 forward=0.05")
    print(f"  实际值: forward={vel['forward']:.3f} m/s")
    assert abs(vel['forward'] - 0.05) < 0.001, "前进速度设置失败"
    print("  ✅ 前进速度设置正确")
    
    # 测试1.2：后退
    print("\n【测试1.2】后退速度设置")
    gait.set_velocity(forward=-0.05)
    vel = gait.get_velocity()
    print(f"  设置 forward=-0.05")
    print(f"  实际值: forward={vel['forward']:.3f} m/s")
    assert abs(vel['forward'] - (-0.05)) < 0.001, "后退速度设置失败"
    print("  ✅ 后退速度设置正确")
    
    # 测试1.3：转向
    print("\n【测试1.3】转向速度设置")
    gait.set_velocity(yaw_rate=0.5)
    vel = gait.get_velocity()
    print(f"  设置 yaw_rate=0.5")
    print(f"  实际值: yaw_rate={vel['yaw_rate']:.3f} rad/s")
    assert abs(vel['yaw_rate'] - 0.5) < 0.001, "转向速度设置失败"
    print("  ✅ 转向速度设置正确")
    
    # 测试1.4：复合运动
    print("\n【测试1.4】复合运动（前进+左转）")
    gait.set_velocity(forward=0.05, yaw_rate=0.3)
    vel = gait.get_velocity()
    print(f"  设置 forward=0.05, yaw_rate=0.3")
    print(f"  实际值: forward={vel['forward']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")
    assert abs(vel['forward'] - 0.05) < 0.001, "前进速度设置失败"
    assert abs(vel['yaw_rate'] - 0.3) < 0.001, "转向速度设置失败"
    print("  ✅ 复合运动设置正确")
    
    # 测试1.5：停止
    print("\n【测试1.5】停止")
    gait.set_velocity(0, 0, 0)
    vel = gait.get_velocity()
    print(f"  设置所有速度为0")
    print(f"  实际值: forward={vel['forward']:.3f}, lateral={vel['lateral']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")
    assert vel['forward'] == 0.0, "停止失败"
    assert vel['lateral'] == 0.0, "停止失败"
    assert vel['yaw_rate'] == 0.0, "停止失败"
    print("  ✅ 停止功能正确")
    
    print("\n✅ 测试1通过：set_velocity 基本功能正常")


def test_set_velocity_limits():
    """测试速度限制功能"""
    print("\n" + "="*60)
    print("测试2：速度限制功能")
    print("="*60)
    
    gait = WalkGait()
    
    # 测试2.1：超出最大前进速度
    print("\n【测试2.1】超出最大前进速度")
    max_forward = gait.max_forward_speed
    print(f"  最大前进速度: {max_forward:.3f} m/s")
    
    gait.set_velocity(forward=999.0)  # 超出限制
    vel = gait.get_velocity()
    print(f"  设置 forward=999.0")
    print(f"  实际值: forward={vel['forward']:.3f} m/s (应限制到 {max_forward:.3f})")
    assert abs(vel['forward'] - max_forward) < 0.001, "速度限制失败"
    print("  ✅ 正向速度限制正确")
    
    gait.set_velocity(forward=-999.0)  # 超出限制
    vel = gait.get_velocity()
    print(f"  设置 forward=-999.0")
    print(f"  实际值: forward={vel['forward']:.3f} m/s (应限制到 {-max_forward:.3f})")
    assert abs(vel['forward'] - (-max_forward)) < 0.001, "速度限制失败"
    print("  ✅ 负向速度限制正确")
    
    # 测试2.2：超出最大转向速度
    print("\n【测试2.2】超出最大转向速度")
    max_yaw = gait.max_yaw_rate
    print(f"  最大转向速度: {max_yaw:.3f} rad/s")
    
    gait.set_velocity(yaw_rate=999.0)
    vel = gait.get_velocity()
    print(f"  设置 yaw_rate=999.0")
    print(f"  实际值: yaw_rate={vel['yaw_rate']:.3f} rad/s (应限制到 {max_yaw:.3f})")
    assert abs(vel['yaw_rate'] - max_yaw) < 0.001, "转向速度限制失败"
    print("  ✅ 转向速度限制正确")
    
    print("\n✅ 测试2通过：速度限制功能正常")


def test_set_direction():
    """测试 set_direction 功能"""
    print("\n" + "="*60)
    print("测试3：set_direction 功能")
    print("="*60)
    
    gait = WalkGait()
    
    # 测试3.1：全速前进
    print("\n【测试3.1】全速前进 (1.0)")
    gait.set_direction(1.0)
    vel = gait.get_velocity()
    max_forward = gait.max_forward_speed
    print(f"  设置 direction=1.0")
    print(f"  实际值: forward={vel['forward']:.3f} m/s (应为 {max_forward:.3f})")
    assert abs(vel['forward'] - max_forward) < 0.001, "全速前进失败"
    assert vel['lateral'] == 0.0, "侧向速度应为0"
    assert vel['yaw_rate'] == 0.0, "转向速度应为0"
    print("  ✅ 全速前进正确")
    
    # 测试3.2：全速后退
    print("\n【测试3.2】全速后退 (-1.0)")
    gait.set_direction(-1.0)
    vel = gait.get_velocity()
    print(f"  设置 direction=-1.0")
    print(f"  实际值: forward={vel['forward']:.3f} m/s (应为 {-max_forward:.3f})")
    assert abs(vel['forward'] - (-max_forward)) < 0.001, "全速后退失败"
    print("  ✅ 全速后退正确")
    
    # 测试3.3：半速前进
    print("\n【测试3.3】半速前进 (0.5)")
    gait.set_direction(0.5)
    vel = gait.get_velocity()
    expected = 0.5 * max_forward
    print(f"  设置 direction=0.5")
    print(f"  实际值: forward={vel['forward']:.3f} m/s (应为 {expected:.3f})")
    assert abs(vel['forward'] - expected) < 0.001, "半速前进失败"
    print("  ✅ 半速前进正确")
    
    # 测试3.4：停止
    print("\n【测试3.4】停止 (0.0)")
    gait.set_direction(0.0)
    vel = gait.get_velocity()
    print(f"  设置 direction=0.0")
    print(f"  实际值: forward={vel['forward']:.3f} m/s (应为 0.000)")
    assert vel['forward'] == 0.0, "停止失败"
    print("  ✅ 停止正确")
    
    # 测试3.5：超出范围限制
    print("\n【测试3.5】超出范围限制 (2.0)")
    gait.set_direction(2.0)
    vel = gait.get_velocity()
    print(f"  设置 direction=2.0")
    print(f"  实际值: forward={vel['forward']:.3f} m/s (应限制到 {max_forward:.3f})")
    assert abs(vel['forward'] - max_forward) < 0.001, "方向限制失败"
    print("  ✅ 方向限制正确")
    
    print("\n✅ 测试3通过：set_direction 功能正常")


def test_velocity_trajectory_integration():
    """测试速度控制与轨迹生成的集成"""
    print("\n" + "="*60)
    print("测试4：速度控制与轨迹生成集成")
    print("="*60)
    
    gait = WalkGait(stride_length=0.05, step_height=0.03, frequency=0.8)
    
    # 测试4.1：前进轨迹
    print("\n【测试4.1】前进轨迹")
    gait.set_velocity(forward=0.05)
    
    dt = 0.02
    steps_per_cycle = int(1.0 / (gait.frequency * dt))
    
    print(f"  步频: {gait.frequency} Hz")
    print(f"  周期步数: {steps_per_cycle}")
    
    # 收集轨迹数据
    trajectories_forward = []
    for i in range(steps_per_cycle):
        gait.update(dt)
        x, z = gait.get_foot_trajectory('right_front')
        trajectories_forward.append((x, z))
    
    # 检查轨迹范围
    x_values = [x for x, z in trajectories_forward]
    x_range = max(x_values) - min(x_values)
    print(f"  X轴范围: {x_range*1000:.1f} mm")
    print(f"  步长: {gait.stride_length*1000:.1f} mm")
    assert x_range > 0, "前进轨迹应该有正向X偏移"
    print("  ✅ 前进轨迹生成正确")
    
    # 测试4.2：后退轨迹
    print("\n【测试4.2】后退轨迹")
    gait.reset()
    gait.set_velocity(forward=-0.05)
    
    trajectories_backward = []
    for i in range(steps_per_cycle):
        gait.update(dt)
        x, z = gait.get_foot_trajectory('right_front')
        trajectories_backward.append((x, z))
    
    x_values_back = [x for x, z in trajectories_backward]
    x_range_back = max(x_values_back) - min(x_values_back)
    print(f"  X轴范围: {x_range_back*1000:.1f} mm")
    assert x_range_back > 0, "后退轨迹应该有X偏移"
    print("  ✅ 后退轨迹生成正确")
    
    # 测试4.3：停止轨迹
    print("\n【测试4.3】停止轨迹")
    gait.reset()
    gait.set_velocity(forward=0.0)
    
    trajectories_stop = []
    for i in range(steps_per_cycle):
        gait.update(dt)
        x, z = gait.get_foot_trajectory('right_front')
        trajectories_stop.append((x, z))
    
    x_values_stop = [x for x, z in trajectories_stop]
    max_x = max(abs(x) for x in x_values_stop)
    print(f"  最大X偏移: {max_x*1000:.1f} mm")
    # 注意：即使速度为0，相位仍会更新，轨迹仍会生成
    # 但步长应该很小或为0
    print("  ✅ 停止时轨迹仍然生成（相位继续）")
    
    # 测试4.4：转向轨迹
    print("\n【测试4.4】前进+转向轨迹")
    gait.reset()
    gait.set_velocity(forward=0.05, yaw_rate=0.5)
    
    # 比较左右腿的轨迹差异
    left_trajectories = []
    right_trajectories = []
    
    for i in range(steps_per_cycle):
        gait.update(dt)
        x_left, z_left = gait.get_foot_trajectory('left_front')
        x_right, z_right = gait.get_foot_trajectory('right_front')
        left_trajectories.append(x_left)
        right_trajectories.append(x_right)
    
    left_range = max(left_trajectories) - min(left_trajectories)
    right_range = max(right_trajectories) - min(right_trajectories)
    
    print(f"  左前腿X范围: {left_range*1000:.1f} mm")
    print(f"  右前腿X范围: {right_range*1000:.1f} mm")
    print(f"  差异: {abs(left_range-right_range)*1000:.1f} mm")
    
    # 转向时左右腿步长应该不同
    assert abs(left_range - right_range) > 0.001, "转向时左右腿步长应该不同"
    print("  ✅ 转向轨迹生成正确（左右腿步长不同）")
    
    print("\n✅ 测试4通过：速度控制与轨迹生成集成正常")


def test_dynamic_velocity_change():
    """测试动态速度变化"""
    print("\n" + "="*60)
    print("测试5：动态速度变化")
    print("="*60)
    
    gait = WalkGait()
    
    # 测试5.1：从停止到前进
    print("\n【测试5.1】从停止到前进")
    gait.set_velocity(0, 0, 0)
    vel = gait.get_velocity()
    print(f"  初始速度: forward={vel['forward']:.3f}")
    
    gait.set_velocity(forward=0.05)
    vel = gait.get_velocity()
    print(f"  更新速度: forward={vel['forward']:.3f}")
    assert abs(vel['forward'] - 0.05) < 0.001, "速度更新失败"
    print("  ✅ 速度更新正确")
    
    # 测试5.2：从前进到后退
    print("\n【测试5.2】从前进到后退")
    gait.set_velocity(forward=0.05)
    vel = gait.get_velocity()
    print(f"  前进速度: forward={vel['forward']:.3f}")
    
    gait.set_velocity(forward=-0.05)
    vel = gait.get_velocity()
    print(f"  后退速度: forward={vel['forward']:.3f}")
    assert vel['forward'] < 0, "速度方向切换失败"
    print("  ✅ 方向切换正确")
    
    # 测试5.3：连续变化
    print("\n【测试5.3】连续速度变化")
    speeds = [0.0, 0.02, 0.04, 0.06, 0.08, 0.1, 0.08, 0.06, 0.04, 0.02, 0.0]
    
    for speed in speeds:
        gait.set_velocity(forward=speed)
        vel = gait.get_velocity()
        print(f"  设置: {speed:.2f} -> 实际: {vel['forward']:.3f} m/s")
        assert abs(vel['forward'] - speed) < 0.001, f"速度设置失败: {speed}"
    
    print("  ✅ 连续变化正确")
    
    print("\n✅ 测试5通过：动态速度变化正常")


def test_get_state_with_velocity():
    """测试 get_state 包含速度信息"""
    print("\n" + "="*60)
    print("测试6：get_state 包含速度信息")
    print("="*60)
    
    gait = WalkGait()
    
    # 设置速度
    gait.set_velocity(forward=0.05, lateral=0.02, yaw_rate=0.3)
    
    # 获取状态
    state = gait.get_state()
    
    print("\n状态信息:")
    print(f"  forward_speed: {state['forward_speed']:.3f} m/s")
    print(f"  lateral_speed: {state['lateral_speed']:.3f} m/s")
    print(f"  yaw_rate: {state['yaw_rate']:.3f} rad/s")
    print(f"  max_forward_speed: {state['max_forward_speed']:.3f} m/s")
    print(f"  max_lateral_speed: {state['max_lateral_speed']:.3f} m/s")
    print(f"  max_yaw_rate: {state['max_yaw_rate']:.3f} rad/s")
    
    # 验证
    assert 'forward_speed' in state, "状态缺少 forward_speed"
    assert 'lateral_speed' in state, "状态缺少 lateral_speed"
    assert 'yaw_rate' in state, "状态缺少 yaw_rate"
    assert abs(state['forward_speed'] - 0.05) < 0.001, "forward_speed 不正确"
    assert abs(state['lateral_speed'] - 0.02) < 0.001, "lateral_speed 不正确"
    assert abs(state['yaw_rate'] - 0.3) < 0.001, "yaw_rate 不正确"
    
    print("\n✅ 测试6通过：get_state 包含完整速度信息")


def test_reset_velocity():
    """测试 reset 重置速度参数"""
    print("\n" + "="*60)
    print("测试7：reset 重置速度参数")
    print("="*60)
    
    gait = WalkGait()
    
    # 设置速度
    print("\n设置速度:")
    gait.set_velocity(forward=0.05, lateral=0.02, yaw_rate=0.3)
    vel = gait.get_velocity()
    print(f"  forward={vel['forward']:.3f}, lateral={vel['lateral']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")
    
    # 重置
    print("\n执行 reset()...")
    gait.reset()
    
    # 检查速度
    vel = gait.get_velocity()
    print(f"  forward={vel['forward']:.3f}, lateral={vel['lateral']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")
    
    assert vel['forward'] == 0.0, "reset 未重置 forward_speed"
    assert vel['lateral'] == 0.0, "reset 未重置 lateral_speed"
    assert vel['yaw_rate'] == 0.0, "reset 未重置 yaw_rate"
    
    print("\n✅ 测试7通过：reset 正确重置所有速度参数")


def run_all_tests():
    """运行所有测试"""
    print("="*60)
    print("Walk 步态速度控制 API 测试")
    print("="*60)
    
    try:
        test_set_velocity_basic()
        test_set_velocity_limits()
        test_set_direction()
        test_velocity_trajectory_integration()
        test_dynamic_velocity_change()
        test_get_state_with_velocity()
        test_reset_velocity()
        
        print("\n" + "="*60)
        print("🎉 所有测试通过！")
        print("="*60)
        
        print("\n测试覆盖:")
        print("  ✅ set_velocity(forward, lateral, yaw_rate)")
        print("  ✅ set_direction(forward)")
        print("  ✅ get_velocity()")
        print("  ✅ 速度限制功能")
        print("  ✅ 轨迹生成集成")
        print("  ✅ 动态速度变化")
        print("  ✅ get_state() 包含速度信息")
        print("  ✅ reset() 重置速度")
        
        return True
        
    except AssertionError as e:
        print(f"\n❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    except Exception as e:
        print(f"\n❌ 测试异常: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == '__main__':
    success = run_all_tests()
    sys.exit(0 if success else 1)
