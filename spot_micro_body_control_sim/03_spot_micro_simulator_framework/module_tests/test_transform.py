# module_tests/test_transform.py
import sys
import os
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

import numpy as np
from core.transform import WorldTransform

def test_matrix_roundtrip():
    """测试变换矩阵的往返转换"""
    wt = WorldTransform(0.1, 0.2, 0.3, 0.1, -0.2, 0.3)
    M = wt.matrix()
    inv = wt.inverse_matrix()
    I = M @ inv
    assert np.allclose(I, np.eye(4), atol=1e-9), "变换矩阵往返转换失败"
    print("✓ 变换矩阵往返转换测试通过")

def test_pose_conversion():
    """测试位姿角度单位转换"""
    wt = WorldTransform()
    
    # 测试弧度模式
    wt.set_pose_euler(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, radians=True)
    pos, roll, pitch, yaw = wt.get_pose_euler(radians=True)
    assert np.allclose(pos, [1.0, 2.0, 3.0]), "位置设置失败"
    assert np.isclose(roll, 0.1) and np.isclose(pitch, 0.2) and np.isclose(yaw, 0.3), "弧度角度设置失败"
    
    # 测试度数模式
    wt.set_pose_euler(1.0, 2.0, 3.0, 10.0, 20.0, 30.0, radians=False)
    pos, roll_deg, pitch_deg, yaw_deg = wt.get_pose_euler(radians=False)
    assert np.isclose(roll_deg, 10.0) and np.isclose(pitch_deg, 20.0) and np.isclose(yaw_deg, 30.0), "度数角度设置失败"
    print("✓ 位姿角度单位转换测试通过")

if __name__ == "__main__":
    test_matrix_roundtrip()
    test_pose_conversion()
    print("所有transform测试通过!")