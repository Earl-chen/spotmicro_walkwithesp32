# module_tests/test_kinematics_roundtrip.py
import sys
import os
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

import numpy as np
from robots.spot_micro.leg_kinematics import SpotLegKinematics
from core.types import LegJoints
import math

def test_fk_inverse_roundtrip():
    """测试正逆运动学往返计算精度"""
    print("开始FK-IK往返测试...")
    
    # 测试左腿
    kin_left = SpotLegKinematics(is_left=True)
    
    # 测试多组关节角度
    test_cases = [
        LegJoints(0.0, 0.0, 0.0),  # 零位
        LegJoints(0.2, -0.35, -0.8),  # 典型姿态
        LegJoints(-0.3, 0.5, -1.2),  # 极限姿态
        LegJoints(0.1, -0.1, -0.5),  # 小角度
    ]
    
    hip_pos = np.array([0.10375, 0.039, 0.0])  # 左前髋位置
    
    for i, joints in enumerate(test_cases):
        print(f"测试用例 {i+1}: hip_side={math.degrees(joints.hip_side):.1f}°, "
              f"hip_pitch={math.degrees(joints.hip_pitch):.1f}°, "
              f"knee_pitch={math.degrees(joints.knee_pitch):.1f}°")
        
        # 正运动学：关节角度 -> 脚部位置
        fk_result = kin_left.forward(hip_pos, joints)
        foot_world = fk_result[-1]  # 脚部世界位置
        foot_in_hip = foot_world - hip_pos  # 脚部相对髋的位置
        
        print(f"  FK结果: 脚部位置 = [{foot_in_hip[0]:.4f}, {foot_in_hip[1]:.4f}, {foot_in_hip[2]:.4f}] m")
        
        # 逆运动学：脚部位置 -> 关节角度
        ik_result = kin_left.inverse(foot_in_hip)
        
        if not ik_result.success:
            print(f"  ❌ IK失败: {ik_result.reason}")
            continue
        
        # 比较关节角度精度
        angle_errors = [
            abs(ik_result.joints.hip_side - joints.hip_side),
            abs(ik_result.joints.hip_pitch - joints.hip_pitch),
            abs(ik_result.joints.knee_pitch - joints.knee_pitch)
        ]
        
        max_error = max(angle_errors)
        print(f"  IK结果: hip_side={math.degrees(ik_result.joints.hip_side):.1f}°, "
              f"hip_pitch={math.degrees(ik_result.joints.hip_pitch):.1f}°, "
              f"knee_pitch={math.degrees(ik_result.joints.knee_pitch):.1f}°")
        print(f"  最大角度误差: {math.degrees(max_error):.3f}°")
        
        # 验证精度
        tolerance = 1e-2  # 约0.57度
        if max_error < tolerance:
            print(f"  ✓ 往返测试通过（误差 < {math.degrees(tolerance):.2f}°）")
        else:
            print(f"  ❌ 往返测试失败（误差 > {math.degrees(tolerance):.2f}°）")
            return False
        
        print()
    
    return True

def test_right_leg_kinematics():
    """测试右腿运动学"""
    print("测试右腿运动学...")
    
    kin_right = SpotLegKinematics(is_left=False)
    hip_pos = np.array([0.10375, -0.039, 0.0])  # 右前髋位置
    
    # 测试一个典型姿态
    joints = LegJoints(0.1, -0.3, -0.6)
    fk_result = kin_right.forward(hip_pos, joints)
    foot_world = fk_result[-1]
    foot_in_hip = foot_world - hip_pos
    
    ik_result = kin_right.inverse(foot_in_hip)
    
    if ik_result.success:
        print("✓ 右腿运动学测试通过")
        return True
    else:
        print(f"❌ 右腿运动学测试失败: {ik_result.reason}")
        return False

if __name__ == "__main__":
    success1 = test_fk_inverse_roundtrip()
    success2 = test_right_leg_kinematics()
    
    if success1 and success2:
        print("所有运动学测试通过! 🎉")
    else:
        print("运动学测试失败! ❌")
        exit(1)