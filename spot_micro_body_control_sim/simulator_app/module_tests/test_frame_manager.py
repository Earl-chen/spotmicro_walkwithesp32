# module_tests/test_frame_manager.py
import sys
import os
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from core.frame_manager import FrameManager
from core.transform import WorldTransform
import numpy as np

def test_transform_point_chain():
    """测试坐标系链式变换"""
    fm = FrameManager()
    fm.set_frame("world", None, WorldTransform(0,0,0,0,0,0))
    fm.set_frame("A", "world", WorldTransform(1,0,0,0,0,0))
    fm.set_frame("B", "A", WorldTransform(0,2,0,0,0,0))
    p = [0,0,0]
    p_world = fm.transform_point(p, "B", "world")
    assert np.allclose(p_world, [1,2,0]), f"期望 [1,2,0]，实际得到 {p_world}"
    print("✓ 坐标系链式变换测试通过")

def test_frame_hierarchy():
    """测试复杂坐标系层次结构"""
    fm = FrameManager()
    
    # 构建 world -> body -> hip_left -> foot 的层次结构
    fm.set_frame("world", None, WorldTransform(0,0,0,0,0,0))
    fm.set_frame("body", "world", WorldTransform(0.1, 0.2, 0.3, 0, 0, 0))  # 机体相对世界偏移
    fm.set_frame("hip_left", "body", WorldTransform(0.1, 0.05, 0, 0, 0, 0))  # 左髋相对机体偏移
    
    # 测试从hip到world的变换
    p_hip = [0, 0, -0.24]  # 脚部相对髋关节位置
    p_world = fm.transform_point(p_hip, "hip_left", "world")
    expected = [0.1 + 0.1, 0.2 + 0.05, 0.3 - 0.24]  # [0.2, 0.25, 0.06]
    assert np.allclose(p_world, expected, atol=1e-10), f"期望 {expected}，实际得到 {p_world}"
    print("✓ 复杂坐标系层次结构测试通过")

if __name__ == "__main__":
    test_transform_point_chain()
    test_frame_hierarchy()
    print("所有frame_manager测试通过!")