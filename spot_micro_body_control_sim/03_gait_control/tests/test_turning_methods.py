#!/usr/bin/env python3
"""
转向方法测试脚本

测试三个新增的转向方法：
1. cmd_vel(linear_x, linear_y, angular_z) - ROS 风格接口
2. zero_radius_turn(yaw_rate) - 零半径转向
3. cmd_vel_adaptive(linear_x, linear_y, angular_z) - 自适应转向
"""

import sys
import os

# 添加项目根目录到 Python 路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from gait_algo_core.walk_gait import WalkGait


def test_cmd_vel():
    """测试 cmd_vel 方法"""
    print("\n" + "="*60)
    print("测试 1: cmd_vel() 方法")
    print("="*60)
    
    gait = WalkGait()
    
    # 测试 1.1: 前进
    print("\n1.1 前进测试")
    gait.cmd_vel(linear_x=0.05)
    vel = gait.get_velocity()
    assert vel['forward'] == 0.05, f"前进速度错误: {vel['forward']}"
    assert vel['lateral'] == 0.0, f"侧向速度错误: {vel['lateral']}"
    assert vel['yaw_rate'] == 0.0, f"角速度错误: {vel['yaw_rate']}"
    print(f"✅ 前进速度设置成功: forward={vel['forward']}")
    
    # 测试 1.2: 后退
    print("\n1.2 后退测试")
    gait.cmd_vel(linear_x=-0.05)
    vel = gait.get_velocity()
    assert vel['forward'] == -0.05, f"后退速度错误: {vel['forward']}"
    print(f"✅ 后退速度设置成功: forward={vel['forward']}")
    
    # 测试 1.3: 侧移
    print("\n1.3 侧移测试")
    gait.cmd_vel(linear_y=0.03)
    vel = gait.get_velocity()
    assert vel['lateral'] == 0.03, f"侧向速度错误: {vel['lateral']}"
    print(f"✅ 侧向速度设置成功: lateral={vel['lateral']}")
    
    # 测试 1.4: 转向
    print("\n1.4 转向测试")
    gait.cmd_vel(angular_z=0.5)
    vel = gait.get_velocity()
    assert vel['yaw_rate'] == 0.5, f"角速度错误: {vel['yaw_rate']}"
    print(f"✅ 角速度设置成功: yaw_rate={vel['yaw_rate']}")
    
    # 测试 1.5: 组合运动（前进+转向）
    print("\n1.5 组合运动测试")
    gait.cmd_vel(linear_x=0.05, linear_y=0.02, angular_z=0.3)
    vel = gait.get_velocity()
    assert vel['forward'] == 0.05, f"前进速度错误: {vel['forward']}"
    assert vel['lateral'] == 0.02, f"侧向速度错误: {vel['lateral']}"
    assert vel['yaw_rate'] == 0.3, f"角速度错误: {vel['yaw_rate']}"
    print(f"✅ 组合运动设置成功: forward={vel['forward']}, lateral={vel['lateral']}, yaw_rate={vel['yaw_rate']}")
    
    # 测试 1.6: 停止
    print("\n1.6 停止测试")
    gait.cmd_vel(0, 0, 0)
    vel = gait.get_velocity()
    assert vel['forward'] == 0.0, f"前进速度应为0: {vel['forward']}"
    assert vel['lateral'] == 0.0, f"侧向速度应为0: {vel['lateral']}"
    assert vel['yaw_rate'] == 0.0, f"角速度应为0: {vel['yaw_rate']}"
    print(f"✅ 停止成功: 所有速度归零")
    
    # 测试 1.7: 速度限制
    print("\n1.7 速度限制测试")
    gait.cmd_vel(linear_x=100.0)  # 超出限制
    vel = gait.get_velocity()
    assert vel['forward'] == gait.max_forward_speed, f"前进速度应被限制: {vel['forward']}"
    print(f"✅ 前进速度被正确限制: {vel['forward']} (max={gait.max_forward_speed})")
    
    gait.cmd_vel(linear_x=-100.0)  # 负方向超出限制
    vel = gait.get_velocity()
    assert vel['forward'] == -gait.max_forward_speed, f"后退速度应被限制: {vel['forward']}"
    print(f"✅ 后退速度被正确限制: {vel['forward']} (min={-gait.max_forward_speed})")
    
    print("\n✅ cmd_vel() 所有测试通过！")
    return True


def test_zero_radius_turn():
    """测试 zero_radius_turn 方法"""
    print("\n" + "="*60)
    print("测试 2: zero_radius_turn() 方法")
    print("="*60)
    
    gait = WalkGait()
    
    # 测试 2.1: 左转
    print("\n2.1 左转测试")
    gait.zero_radius_turn(yaw_rate=0.5)
    vel = gait.get_velocity()
    assert vel['forward'] == 0.0, f"零半径转向不应有前进速度: {vel['forward']}"
    assert vel['lateral'] == 0.0, f"零半径转向不应有侧向速度: {vel['lateral']}"
    assert vel['yaw_rate'] == 0.5, f"角速度错误: {vel['yaw_rate']}"
    print(f"✅ 左转成功: yaw_rate={vel['yaw_rate']}, forward={vel['forward']}, lateral={vel['lateral']}")
    
    # 测试 2.2: 右转
    print("\n2.2 右转测试")
    gait.zero_radius_turn(yaw_rate=-0.5)
    vel = gait.get_velocity()
    assert vel['forward'] == 0.0, f"零半径转向不应有前进速度: {vel['forward']}"
    assert vel['lateral'] == 0.0, f"零半径转向不应有侧向速度: {vel['lateral']}"
    assert vel['yaw_rate'] == -0.5, f"角速度错误: {vel['yaw_rate']}"
    print(f"✅ 右转成功: yaw_rate={vel['yaw_rate']}, forward={vel['forward']}, lateral={vel['lateral']}")
    
    # 测试 2.3: 停止
    print("\n2.3 停止测试")
    gait.zero_radius_turn(yaw_rate=0.0)
    vel = gait.get_velocity()
    assert vel['yaw_rate'] == 0.0, f"角速度应为0: {vel['yaw_rate']}"
    print(f"✅ 停止成功: yaw_rate={vel['yaw_rate']}")
    
    # 测试 2.4: 验证 forward 和 lateral 始终为 0
    print("\n2.4 验证无前进和侧向速度")
    for yaw in [0.1, 0.5, 1.0, -0.3, -0.8]:
        gait.zero_radius_turn(yaw_rate=yaw)
        vel = gait.get_velocity()
        assert vel['forward'] == 0.0, f"yaw_rate={yaw} 时 forward 应为0: {vel['forward']}"
        assert vel['lateral'] == 0.0, f"yaw_rate={yaw} 时 lateral 应为0: {vel['lateral']}"
    print(f"✅ 所有角速度下 forward 和 lateral 均为 0")
    
    print("\n✅ zero_radius_turn() 所有测试通过！")
    return True


def test_cmd_vel_adaptive():
    """测试 cmd_vel_adaptive 方法"""
    print("\n" + "="*60)
    print("测试 3: cmd_vel_adaptive() 方法")
    print("="*60)
    
    gait = WalkGait()
    
    # 测试 3.1: 差速转向场景 1（前进+小角度转向）
    print("\n3.1 差速转向场景 1: 前进+小角度转向")
    gait.cmd_vel_adaptive(linear_x=0.05, angular_z=0.3)
    vel = gait.get_velocity()
    assert vel['forward'] == 0.05, f"前进速度错误: {vel['forward']}"
    assert vel['yaw_rate'] == 0.3, f"角速度错误: {vel['yaw_rate']}"
    print(f"✅ 差速转向: forward={vel['forward']}, yaw_rate={vel['yaw_rate']}")
    
    # 测试 3.2: 差速转向场景 2（小角度转向+几乎无前进）
    print("\n3.2 差速转向场景 2: 小角度转向+几乎无前进")
    gait.cmd_vel_adaptive(linear_x=0.01, angular_z=0.3)
    vel = gait.get_velocity()
    assert vel['forward'] == 0.01, f"前进速度错误: {vel['forward']}"
    assert vel['yaw_rate'] == 0.3, f"角速度错误: {vel['yaw_rate']}"
    print(f"✅ 差速转向: forward={vel['forward']}, yaw_rate={vel['yaw_rate']}")
    
    # 测试 3.3: 零半径转向场景 1（大角度转向+几乎无前进）
    print("\n3.3 零半径转向场景 1: 大角度转向+几乎无前进")
    gait.cmd_vel_adaptive(linear_x=0.01, angular_z=0.8)
    vel = gait.get_velocity()
    assert vel['forward'] == 0.0, f"零半径转向不应有前进速度: {vel['forward']}"
    assert vel['yaw_rate'] == 0.8, f"角速度错误: {vel['yaw_rate']}"
    print(f"✅ 零半径转向: yaw_rate={vel['yaw_rate']}, forward={vel['forward']}")
    
    # 测试 3.4: 零半径转向场景 2（大角度转向+完全无前进）
    print("\n3.4 零半径转向场景 2: 大角度转向+完全无前进")
    gait.cmd_vel_adaptive(linear_x=0.0, angular_z=0.6)
    vel = gait.get_velocity()
    assert vel['forward'] == 0.0, f"零半径转向不应有前进速度: {vel['forward']}"
    assert vel['yaw_rate'] == 0.6, f"角速度错误: {vel['yaw_rate']}"
    print(f"✅ 零半径转向: yaw_rate={vel['yaw_rate']}, forward={vel['forward']}")
    
    # 测试 3.5: 边界条件测试（angular_z = 0.5, linear_x = 0.02）
    print("\n3.5 边界条件测试")
    # 刚好不满足零半径转向条件（angular_z = 0.5 不是 > 0.5）
    gait.cmd_vel_adaptive(linear_x=0.01, angular_z=0.5)
    vel = gait.get_velocity()
    assert vel['forward'] == 0.01, f"前进速度错误: {vel['forward']}"
    assert vel['yaw_rate'] == 0.5, f"角速度错误: {vel['yaw_rate']}"
    print(f"✅ 差速转向（边界）: forward={vel['forward']}, yaw_rate={vel['yaw_rate']}")
    
    # 刚好满足零半径转向条件（angular_z > 0.5）
    gait.cmd_vel_adaptive(linear_x=0.01, angular_z=0.51)
    vel = gait.get_velocity()
    assert vel['forward'] == 0.0, f"零半径转向不应有前进速度: {vel['forward']}"
    assert vel['yaw_rate'] == 0.51, f"角速度错误: {vel['yaw_rate']}"
    print(f"✅ 零半径转向（边界）: yaw_rate={vel['yaw_rate']}, forward={vel['forward']}")
    
    # 测试 3.6: 多种场景组合
    print("\n3.6 多种场景组合测试")
    test_cases = [
        # (linear_x, angular_z, expected_forward, expected_yaw, mode)
        (0.05, 0.3, 0.05, 0.3, "差速"),
        (0.01, 0.6, 0.0, 0.6, "零半径"),
        (0.0, 0.8, 0.0, 0.8, "零半径"),
        (0.05, 0.6, 0.05, 0.6, "差速"),
        (0.01, 0.4, 0.01, 0.4, "差速"),
        (0.0, 0.4, 0.0, 0.4, "差速"),
    ]
    
    for i, (lx, az, exp_fwd, exp_yaw, mode) in enumerate(test_cases, 1):
        gait.cmd_vel_adaptive(linear_x=lx, angular_z=az)
        vel = gait.get_velocity()
        assert vel['forward'] == exp_fwd, f"测试用例 {i}: 前进速度错误，期望 {exp_fwd}，实际 {vel['forward']}"
        assert vel['yaw_rate'] == exp_yaw, f"测试用例 {i}: 角速度错误，期望 {exp_yaw}，实际 {vel['yaw_rate']}"
        print(f"  ✅ 用例 {i}: linear_x={lx}, angular_z={az} → {mode} (forward={vel['forward']}, yaw_rate={vel['yaw_rate']})")
    
    print("\n✅ cmd_vel_adaptive() 所有测试通过！")
    return True


def test_integration():
    """集成测试：测试所有方法的协同工作"""
    print("\n" + "="*60)
    print("测试 4: 集成测试")
    print("="*60)
    
    gait = WalkGait(stride_length=0.05, step_height=0.03, frequency=0.8)
    
    # 测试 4.1: 切换不同控制模式
    print("\n4.1 切换控制模式测试")
    
    # 模式 1: cmd_vel 前进
    gait.cmd_vel(linear_x=0.05)
    vel = gait.get_velocity()
    assert vel['forward'] == 0.05
    print(f"  ✅ cmd_vel 前进: {vel}")
    
    # 模式 2: 切换到零半径转向
    gait.zero_radius_turn(yaw_rate=0.5)
    vel = gait.get_velocity()
    assert vel['forward'] == 0.0
    assert vel['yaw_rate'] == 0.5
    print(f"  ✅ zero_radius_turn 转向: {vel}")
    
    # 模式 3: 切换到自适应控制
    gait.cmd_vel_adaptive(linear_x=0.05, angular_z=0.3)
    vel = gait.get_velocity()
    assert vel['forward'] == 0.05
    assert vel['yaw_rate'] == 0.3
    print(f"  ✅ cmd_vel_adaptive 差速: {vel}")
    
    # 模式 4: 停止
    gait.cmd_vel(0, 0, 0)
    vel = gait.get_velocity()
    assert vel['forward'] == 0.0
    assert vel['yaw_rate'] == 0.0
    print(f"  ✅ 停止: {vel}")
    
    # 测试 4.2: 轨迹生成验证
    print("\n4.2 轨迹生成验证")
    gait.cmd_vel(linear_x=0.05)
    gait.update(dt=0.02)
    trajectories = gait.get_all_foot_trajectories()
    assert len(trajectories) == 4, f"应该有4条腿的轨迹: {len(trajectories)}"
    for leg_name, (x, z) in trajectories.items():
        assert isinstance(x, (int, float)), f"{leg_name} x 应为数值"
        assert isinstance(z, (int, float)), f"{leg_name} z 应为数值"
    print(f"  ✅ 轨迹生成成功: {len(trajectories)} 条腿")
    
    # 测试 4.3: 状态信息
    print("\n4.3 状态信息验证")
    state = gait.get_state()
    assert 'forward_speed' in state
    assert 'yaw_rate' in state
    assert 'steering_angle' in state
    print(f"  ✅ 状态信息完整: forward={state['forward_speed']}, yaw_rate={state['yaw_rate']}")
    
    print("\n✅ 集成测试所有测试通过！")
    return True


def main():
    """运行所有测试"""
    print("\n" + "="*60)
    print("🚀 开始转向方法测试")
    print("="*60)
    
    all_passed = True
    
    try:
        # 测试 1: cmd_vel
        if not test_cmd_vel():
            all_passed = False
        
        # 测试 2: zero_radius_turn
        if not test_zero_radius_turn():
            all_passed = False
        
        # 测试 3: cmd_vel_adaptive
        if not test_cmd_vel_adaptive():
            all_passed = False
        
        # 测试 4: 集成测试
        if not test_integration():
            all_passed = False
        
        # 总结
        print("\n" + "="*60)
        if all_passed:
            print("🎉 所有测试通过！")
            print("="*60)
            print("\n📋 测试摘要:")
            print("  ✅ cmd_vel() - ROS 风格接口")
            print("  ✅ zero_radius_turn() - 零半径转向")
            print("  ✅ cmd_vel_adaptive() - 自适应转向")
            print("  ✅ 集成测试 - 协同工作")
            return 0
        else:
            print("❌ 部分测试失败！")
            print("="*60)
            return 1
            
    except Exception as e:
        print(f"\n❌ 测试过程发生错误: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    exit(main())
