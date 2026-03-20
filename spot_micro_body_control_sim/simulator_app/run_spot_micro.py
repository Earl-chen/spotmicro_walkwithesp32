#!/usr/bin/env python3
# run_spot_micro.py - 四足机器人双模式控制系统主程序入口
import sys
import os
import math

# 添加项目根目录到 Python 路径
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from core.frame_manager import FrameManager
from core.transform import WorldTransform
from robots.spot_micro.leg_kinematics import SpotLegKinematics
from robots.spot_micro import geometry
from app.robot_model import RobotModel
from app.controller import Controller
from app.ui.ui_matplotlib import MatplotlibUI
from core.types import LegJoints

def build_system():
    """构建系统：坐标系 + 机器人模型 + 控制器"""
    
    # 1. 创建坐标系管理器
    fm = FrameManager()
    
    # 2. 设置世界坐标系（根坐标系）
    fm.set_frame("world", None, WorldTransform(0, 0, 0, 0, 0, 0))
    
    # 3. 设置机体坐标系（相对于世界坐标系）
    # 初始机体高度 Z = 0（将在脚步锁定后由滑块控制）
    body_tf = WorldTransform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    fm.set_frame("body", "world", body_tf)
    
    # 4. 设置各髋关节坐标系（相对于机体坐标系）
    for leg_name, offset in geometry.HIP_OFFSETS.items():
        x, y, z = offset
        hip_tf = WorldTransform(x, y, z, 0.0, 0.0, 0.0)
        fm.set_frame(f"hip_{leg_name}", "body", hip_tf)
    
    # 5. 创建机器人模型
    model = RobotModel(fm, body_frame="body")
    
    # 6. 为每条腿添加运动学求解器
    for leg_name in geometry.HIP_OFFSETS.keys():
        is_left = "left" in leg_name
        legkin = SpotLegKinematics(
            l1=geometry.L1, l2=geometry.L2, 
            l3=geometry.L3, l4=geometry.L4, 
            is_left=is_left
        )
        
        # 初始关节角度（零位，完全直立）
        # 滑块初始化时会通过逆运动学自动计算出站立姿态的关节角度
        initial_joints = LegJoints(0.0, 0.0, 0.0)
        model.add_leg(leg_name, legkin, hip_frame_name=f"hip_{leg_name}", initial_joints=initial_joints)
    
    # 7. 创建控制器
    controller = Controller(model)
    
    return model, controller

def run():
    """运行主程序"""
    print("=== 四足机器人世界坐标系控制系统 (模块化重构版) ===")
    print("功能特点:")
    print("* 模块化架构: core + robots + app 分离")
    print("* 机体6DOF控制: 位置(x,y,z) + 姿态(roll,pitch,yaw)")
    print("* 脚步世界坐标固定: 四只脚在世界坐标系中位置始终不变")
    print("* 逆运动学自动计算: 机体位姿变化时自动计算关节角度")
    print("* 实时坐标系显示: 世界坐标轴(红) + 机体坐标轴(蓝)")
    print("* 实时状态监控: 显示机体位姿、关节角度、脚部坐标")
    print()
    
    # 构建系统
    model, controller = build_system()
    
    # 显示初始状态
    pose = controller.get_body_pose()
    print("初始状态:")
    print(f"* 机体位置: ({pose['position'][0]:.3f}, {pose['position'][1]:.3f}, {pose['position'][2]:.3f}) m")
    print(f"* 机体姿态: ({pose['orientation_deg'][0]:.1f}°, {pose['orientation_deg'][1]:.1f}°, {pose['orientation_deg'][2]:.1f}°)")
    print("* 初始关节角度:")
    leg_names = {'left_front': '左前', 'left_back': '左后',
                 'right_front': '右前', 'right_back': '右后'}
    for leg_name, joints in model.joints.items():
        print(f"  {leg_names[leg_name]}: hip_side={math.degrees(joints.hip_side):.1f}°, "
              f"hip_pitch={math.degrees(joints.hip_pitch):.1f}°, "
              f"knee={math.degrees(joints.knee_pitch):.1f}°")
    print()
    
    foot_positions = controller.get_foot_positions_world()
    print("当前脚部世界坐标:")
    leg_names = {'left_front': '左前', 'left_back': '左后', 
                 'right_front': '右前', 'right_back': '右后'}
    for leg_name, pos in foot_positions.items():
        print(f"  {leg_names[leg_name]}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) m")
    
    print("\n启动交互式世界坐标系控制界面...")
    print("功能说明:")
    print("  * 四只脚的世界坐标位置始终保持不变 [LOCKED]")
    print("  * 拖动机体位姿滑块(X,Y,Z,Roll,Pitch,Yaw)")
    print("  * 系统自动通过逆运动学计算关节角度")
    print("  * 实时显示机体位姿、关节角度、脚部坐标")
    print("\n重构后的模块化架构确保了代码的可维护性和可扩展性!")
    
    # 创建并启动UI
    ui = MatplotlibUI(controller, model)
    ui.start()

if __name__ == "__main__":
    run()