#!/usr/bin/env python3
"""
转向功能测试脚本

测试 Walk 步态的转向能力
"""

import sys
import os
import numpy as np

# 添加模块路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gait_algo_core.walk_gait import WalkGait

def test_steering_functionality():
    """测试转向功能"""
    print("\n" + "="*60)
    print("测试：Walk 步态转向功能")
    print("="*60)
    
    # 创建步态控制器
    gait = WalkGait(stride_length=0.05, step_height=0.03, frequency=0.8)
    
    print("\n【测试1】直行模式（steering_angle = 0）")
    print("-"*60)
    
    gait.set_direction(0.0)
    
    # 测试一个完整周期
    dt = 0.02
    steps_per_cycle = int(1.0 / (gait.frequency * dt))
    
    print(f"周期步数: {steps_per_cycle}")
    
    # 收集轨迹数据
    trajectories = {
        'left_front': {'x': [], 'y': [], 'z': []},
        'right_front': {'x': [], 'y': [], 'z': []},
        'left_back': {'x': [], 'y': [], 'z': []},
        'right_back': {'x': [], 'y': [], 'z': []}
    }
    
    for i in range(steps_per_cycle):
        gait.update(dt)
        
        for leg_name in trajectories.keys():
            point = gait.get_foot_trajectory(leg_name)
            trajectories[leg_name]['x'].append(point.x)
            trajectories[leg_name]['y'].append(point.y)
            trajectories[leg_name]['z'].append(point.z)
    
    # 验证：y轴应该为0（无侧向偏移）
    print("Y轴最大偏移（应该≈0）:")
    for leg_name, data in trajectories.items():
        y_max = max(abs(y) for y in data['y'])
        print(f"  {leg_name:15s}: {y_max*1000:6.2f}mm")
    
    print("\n【测试2】左转模式（steering_angle = +30°）")
    print("-"*60)
    
    gait.reset()
    gait.set_direction(np.pi/6)  # 左转30度
    
    # 重新收集数据
    trajectories_left = {
        'left_front': {'x': [], 'y': [], 'z': []},
        'right_front': {'x': [], 'y': [], 'z': []},
        'left_back': {'x': [], 'y': [], 'z': []},
        'right_back': {'x': [], 'y': [], 'z': []}
    }
    
    for i in range(steps_per_cycle):
        gait.update(dt)
        
        for leg_name in trajectories_left.keys():
            point = gait.get_foot_trajectory(leg_name)
            trajectories_left[leg_name]['x'].append(point.x)
            trajectories_left[leg_name]['y'].append(point.y)
            trajectories_left[leg_name]['z'].append(point.z)
    
    # 验证：左右腿的y轴应该反向
    print("左右腿Y轴对比（左转30°）:")
    for leg_name in ['front', 'back']:
        left_y = trajectories_left[f'left_{leg_name}']['y']
        right_y = trajectories_left[f'right_{leg_name}']['y']
        
        left_max = max(abs(y) for y in left_y) * 1000
        right_max = max(abs(y) for y in right_y) * 1000
        
        print(f"  {leg_name:6s}: 左腿Y={left_max:6.2f}mm, 右腿Y={right_max:6.2f}mm, 差值={abs(left_max-right_max):6.2f}mm")
    
    print("\n【测试3】右转模式（steering_angle = -30°）")
    print("-"*60)
    
    gait.reset()
    gait.set_direction(-np.pi/6)  # 右转30度
    
    # 重新收集数据
    trajectories_right = {
        'left_front': {'x': [], 'y': [], 'z': []},
        'right_front': {'x': [], 'y': [], 'z': []},
        'left_back': {'x': [], 'y': [], 'z': []},
        'right_back': {'x': [], 'y': [], 'z': []}
    }
    
    for i in range(steps_per_cycle):
        gait.update(dt)
        
        for leg_name in trajectories_right.keys():
            point = gait.get_foot_trajectory(leg_name)
            trajectories_right[leg_name]['x'].append(point.x)
            trajectories_right[leg_name]['y'].append(point.y)
            trajectories_right[leg_name]['z'].append(point.z)
    
    # 验证：应该与左转相反
    print("左右腿Y轴对比（右转30°，应该与左转相反）:")
    for leg_name in ['front', 'back']:
        left_y = trajectories_right[f'left_{leg_name}']['y']
        right_y = trajectories_right[f'right_{leg_name}']['y']
        
        left_max = max(abs(y) for y in left_y) * 1000
        right_max = max(abs(y) for y in right_y) * 1000
        
        print(f"  {leg_name:6s}: 左腿Y={left_max:6.2f}mm, 右腿Y={right_max:6.2f}mm, 差值={abs(left_max-right_max):6.2f}mm")
    
    print("\n【测试4】动态转向（从直行到左转）")
    print("-"*60)
    
    gait.reset()
    
    # 前50步直行
    for i in range(50):
        gait.update(dt)
    
    state1 = gait.get_state()
    print(f"  直行状态: 转向角度 = {state1['steering_angle']*180/np.pi:.1f}°")
    
    # 切换到左转
    gait.set_direction(np.pi/6)
    
    for i in range(50):
        gait.update(dt)
    
    state2 = gait.get_state()
    print(f"  左转状态: 转向角度 = {state2['steering_angle']*180/np.pi:.1f}°")
    
    print("\n" + "="*60)
    print("✅ 转向功能测试完成")
    print("="*60)
    
    return True


if __name__ == '__main__':
    try:
        test_steering_functionality()
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
