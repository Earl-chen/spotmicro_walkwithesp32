#!/usr/bin/env python3
# gait_animation_demo_v5.py - 步态动画演示（修正版）

import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.animation import FuncAnimation
import numpy as np
import math

project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from core.frame_manager import FrameManager
from core.transform import WorldTransform
from robots.spot_micro.leg_kinematics import SpotLegKinematics
from robots.spot_micro import geometry
from app.robot_model import RobotModel
from app.controller import Controller
from app.gait.walk_gait import WalkGait
from core.types import LegJoints

# 配置中文字体
font_path = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
if os.path.exists(font_path):
    fm.fontManager.addfont(font_path)
    font_prop = fm.FontProperties(fname=font_path)
    plt.rcParams['font.family'] = font_prop.get_name()
    plt.rcParams['axes.unicode_minus'] = False

def build_system():
    """构建系统"""
    fm_mgr = FrameManager()
    fm_mgr.set_frame("world", None, WorldTransform(0, 0, 0, 0, 0, 0))
    
    body_tf = WorldTransform(0.0, 0.0, -0.1, 0.0, 0.0, 0.0)
    fm_mgr.set_frame("body", "world", body_tf)
    
    for leg_name, offset in geometry.HIP_OFFSETS.items():
        x, y, z = offset
        hip_tf = WorldTransform(x, y, z, 0.0, 0.0, 0.0)
        fm_mgr.set_frame(f"hip_{leg_name}", "body", hip_tf)
    
    model = RobotModel(fm_mgr, body_frame="body")
    
    for leg_name in geometry.HIP_OFFSETS.keys():
        is_left = "left" in leg_name
        legkin = SpotLegKinematics(
            l1=geometry.L1, l2=geometry.L2, 
            l3=geometry.L3, l4=geometry.L4, 
            is_left=is_left
        )
        initial_joints = LegJoints(0.0, 0.0, 0.0)
        model.add_leg(leg_name, legkin, hip_frame_name=f"hip_{leg_name}", initial_joints=initial_joints)
    
    controller = Controller(model)
    return model, controller

def set_standing_pose(model):
    """设置站立姿态"""
    hip_pitch_deg = 15.0
    knee_deg = -45.0
    
    for leg_name in model.legs.keys():
        joints = LegJoints(0.0, math.radians(hip_pitch_deg), math.radians(knee_deg))
        model.update_joint_angles(leg_name, joints)

def capture_initial_positions(model):
    """捕获初始足端位置"""
    initial_positions = {}
    for leg_name in model.legs.keys():
        joint_positions_body = model.get_leg_joints_body(leg_name)
        foot_pos_body = joint_positions_body[-1]
        
        legkin, hip_frame = model.legs[leg_name]
        foot_pos_hip = model.frame_manager.transform_point(
            foot_pos_body, "body", hip_frame
        )
        
        initial_positions[leg_name] = foot_pos_hip
    
    return initial_positions

def draw_robot(ax, model, controller, gait):
    """绘制机器人"""
    ax.clear()
    
    # 绘制机体
    body_outline = model.get_body_outline_world()
    body_x = [p[0] for p in body_outline] + [body_outline[0][0]]
    body_y = [p[1] for p in body_outline] + [body_outline[0][1]]
    body_z = [p[2] for p in body_outline] + [body_outline[0][2]]
    ax.plot(body_x, body_y, body_z, 'b-', linewidth=3)
    
    # 绘制四条腿
    colors = {
        'left_front': 'red',
        'left_back': 'green',
        'right_front': 'orange',
        'right_back': 'purple'
    }
    
    for leg_name in geometry.HIP_OFFSETS.keys():
        joint_positions = model.get_leg_joints_world(leg_name)
        
        # 绘制腿的各段
        for i in range(len(joint_positions) - 1):
            p1 = joint_positions[i]
            p2 = joint_positions[i + 1]
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 
                   c=colors[leg_name], linewidth=4, solid_capstyle='round')
        
        # 绘制关节点和足端
        for i, pos in enumerate(joint_positions):
            is_foot = (i == len(joint_positions) - 1)
            leg_phase = gait.get_leg_phase(leg_name)
            is_swing = leg_phase < 0.25  # ✅ 修正：从0.5改为0.25
            
            if is_foot:
                marker = 'o' if is_swing else 's'
                size = 120 if is_swing else 180
                alpha = 1.0 if is_swing else 0.7
            else:
                marker = 'o'
                size = 100
                alpha = 1.0
            
            ax.scatter([pos[0]], [pos[1]], [pos[2]], 
                      c=colors[leg_name], s=size, marker=marker, 
                      edgecolors='black', linewidths=1.5, alpha=alpha)
    
    # 设置坐标轴
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    
    # 视野跟随
    body_pos = model.frame_manager.transform_point(
        np.array([0, 0, 0]), "body", "world"
    )
    ax.set_xlim([body_pos[0] - 0.3, body_pos[0] + 0.3])
    ax.set_ylim([-0.3, 0.3])
    ax.set_zlim([-0.4, 0.2])
    ax.view_init(elev=20, azim=45)

def main():
    print("生成步态动画（修正版：占空比25%/75%）...")
    
    model, controller = build_system()
    set_standing_pose(model)
    
    gait = WalkGait(stride_length=0.04, step_height=0.025, frequency=0.8)
    controller.enable_forward_kinematics_mode()
    
    initial_foot_positions = capture_initial_positions(model)
    
    # 累计前进距离
    total_distance = 0.0
    previous_phase = 0.0
    
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    
    dt = 0.02
    total_frames = 150
    
    # IK成功统计
    ik_success = {leg: 0 for leg in model.legs.keys()}
    ik_total = {leg: 0 for leg in model.legs.keys()}
    
    def update(frame):
        nonlocal total_distance, previous_phase
        
        gait.update(dt)
        
        # 累计前进距离
        if gait.global_phase < previous_phase:
            total_distance += gait.stride_length
        
        previous_phase = gait.global_phase
        
        # 更新机体位置
        body_x = total_distance + gait.global_phase * gait.stride_length
        controller.set_body_pose(body_x, 0, -0.1, 0, 0, 0)
        
        # 更新所有腿的姿态
        trajectories = gait.get_all_foot_trajectories()
        
        for leg_name, (x_offset, z_offset) in trajectories.items():
            legkin, hip_frame = model.legs[leg_name]
            
            # 初始位置 + 步态偏移
            initial_pos = initial_foot_positions[leg_name]
            target_foot_pos_hip = initial_pos + np.array([x_offset, 0, z_offset])
            
            # 逆运动学
            ik_result = legkin.inverse(target_foot_pos_hip)
            
            ik_total[leg_name] += 1
            if ik_result.success:
                ik_success[leg_name] += 1
                model.update_joint_angles(leg_name, ik_result.joints)
        
        # 绘制机器人
        draw_robot(ax, model, controller, gait)
        
        # 统计支撑腿数量
        stance_count = sum(1 for leg in model.legs.keys() 
                          if gait.get_leg_phase(leg) >= 0.25)
        
        # 设置标题
        title = f'Walk步态（修正版）\n'
        title += f'相位: {gait.global_phase:.2f} | 支撑腿: {stance_count}条 | 前进: {body_x*100:.1f}cm'
        ax.set_title(title, fontsize=11, fontweight='bold')
        
        return []
    
    # 创建动画
    anim = FuncAnimation(fig, update, frames=total_frames, interval=50, blit=False)
    
    # 保存动画
    output_path = 'gait_animation_demo_v5.gif'
    print(f"正在保存动画到: {output_path}")
    print("（这可能需要几秒钟...）")
    
    try:
        anim.save(output_path, writer='pillow', fps=20)
        print(f"✅ 动画已保存: {output_path}")
        
        # 输出IK成功率
        print("\nIK成功率统计：")
        for leg in model.legs.keys():
            rate = ik_success[leg] / ik_total[leg] * 100 if ik_total[leg] > 0 else 0
            print(f"  {leg}: {rate:.1f}%")
        
        # 输出文件大小
        file_size = os.path.getsize(output_path) / 1024
        print(f"\n文件大小: {file_size:.1f} KB")
        
    except Exception as e:
        print(f"⚠️  保存动画失败: {e}")
    
    plt.close()

if __name__ == '__main__':
    main()
