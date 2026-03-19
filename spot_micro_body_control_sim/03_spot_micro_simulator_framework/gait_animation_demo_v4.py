#!/usr/bin/env python3
# gait_animation_demo_v4.py - 步态动画演示（累计前进距离版）

import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.animation import FuncAnimation
import numpy as np
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
    """设置站立姿态（膝关节弯曲45°）"""
    hip_pitch_deg = 15.0
    knee_deg = -45.0
    
    standing_joints = {}
    for leg_name in model.legs.keys():
        standing_joints[leg_name] = LegJoints(
            0.0,  
            math.radians(hip_pitch_deg),
            math.radians(knee_deg)
        )
        model.update_joint_angles(leg_name, standing_joints[leg_name])
    
    print(f"✅ 已设置站立姿态: hip_pitch={hip_pitch_deg}°, knee={knee_deg}°")
    return standing_joints

def draw_robot(ax, model, controller, gait, phase, total_distance):
    """绘制机器人"""
    ax.clear()
    
    # 绘制机体
    body_outline = model.get_body_outline_world()
    body_x = [p[0] for p in body_outline] + [body_outline[0][0]]
    body_y = [p[1] for p in body_outline] + [body_outline[0][1]]
    body_z = [p[2] for p in body_outline] + [body_outline[0][2]]
    ax.plot(body_x, body_y, body_z, 'b-', linewidth=3, label='机体')
    
    # 绘制四条腿
    colors = {
        'left_front': 'red',
        'left_back': 'green',
        'right_front': 'orange',
        'right_back': 'purple'
    }
    
    for leg_name in geometry.HIP_OFFSETS.keys():
        joint_positions = model.get_leg_joints_world(leg_name)
        
        for i in range(len(joint_positions) - 1):
            p1 = joint_positions[i]
            p2 = joint_positions[i + 1]
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 
                   c=colors[leg_name], linewidth=4, solid_capstyle='round')
        
        for i, pos in enumerate(joint_positions):
            is_foot = (i == len(joint_positions) - 1)
            leg_phase = gait.get_leg_phase(leg_name)
            is_swing = leg_phase < 0.5
            
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
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    
    # ========== 关键：视野跟随机体移动 ==========
    # 获取当前机体位置
    body_pos = model.frame_manager.transform_point(
        np.array([0, 0, 0]),
        from_frame="body",
        to_frame="world"
    )
    
    # 设置视野范围（以机体为中心）
    view_range = 0.3
    ax.set_xlim([body_pos[0] - view_range, body_pos[0] + view_range])
    ax.set_ylim([-view_range, view_range])
    ax.set_zlim([-0.4, 0.2])
    ax.view_init(elev=20, azim=45)
    ax.legend(loc='upper left')

def main():
    print("生成步态动画（累计前进距离版）...")
    
    # 构建系统
    model, controller = build_system()
    
    # ========== 关键修复：先设置站立姿态 ==========
    set_standing_pose(model)
    
    # 创建步态控制器
    gait = WalkGait(stride_length=0.04, step_height=0.025, frequency=0.8)
    
    # 切换到正向运动学模式
    controller.enable_forward_kinematics_mode()
    
    # 捕获初始足端位置（站立姿态）
    print("\n捕获初始足端位置（站立姿态）...")
    initial_foot_positions = {}
    for leg_name in model.legs.keys():
        joint_positions_body = model.get_leg_joints_body(leg_name)
        initial_foot_pos_body = joint_positions_body[-1]
        
        legkin, hip_frame = model.legs[leg_name]
        initial_foot_pos_hip = model.frame_manager.transform_point(
            initial_foot_pos_body, 
            from_frame="body", 
            to_frame=hip_frame
        )
        
        initial_foot_positions[leg_name] = initial_foot_pos_hip
        print(f"  {leg_name}: {initial_foot_pos_hip}")
    
    # 创建图形
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    
    # 动画参数
    dt = 0.02
    total_frames = 150  # 增加帧数，展示3个周期
    
    # ========== 关键修正：累计前进距离 ==========
    total_distance = 0.0
    previous_phase = 0.0
    body_z = -0.1
    
    # 数据收集
    ik_success_count = {leg: 0 for leg in model.legs.keys()}
    ik_total_count = {leg: 0 for leg in model.legs.keys()}
    
    def update(frame):
        nonlocal total_distance, previous_phase
        
        # 更新步态相位
        gait.update(dt)
        
        # ========== 关键修正：检测周期完成 ==========
        if gait.global_phase < previous_phase:
            # 一个完整周期结束，前进stride_length
            total_distance += gait.stride_length
            print(f"  周期完成，总距离: {total_distance*100:.1f}cm")
        
        previous_phase = gait.global_phase
        
        # 计算当前机体位置（累计距离 + 当前周期进度）
        body_x = total_distance + gait.global_phase * gait.stride_length
        
        # 更新机体位置
        controller.set_body_pose(
            body_x,  # X: 累计前进距离
            0, body_z, 0, 0, 0,
            radians=True
        )
        
        # 获取所有腿的足端轨迹
        trajectories = gait.get_all_foot_trajectories()
        
        # 对每条腿计算关节角度
        for leg_name, (x_offset, z_offset) in trajectories.items():
            legkin, hip_frame = model.legs[leg_name]
            
            # 初始位置 + 步态偏移
            initial_pos = initial_foot_positions[leg_name]
            target_foot_pos_hip = initial_pos + np.array([x_offset, 0, z_offset])
            
            # 使用逆运动学计算关节角度
            ik_result = legkin.inverse(target_foot_pos_hip)
            
            # 统计IK成功率
            ik_total_count[leg_name] += 1
            if ik_result.success:
                ik_success_count[leg_name] += 1
                joints = ik_result.joints
                model.update_joint_angles(leg_name, joints)
        
        # 绘制机器人
        draw_robot(ax, model, controller, gait, gait.global_phase, total_distance)
        
        # 设置标题（显示距离信息）
        title = f'四足机器人 Walk 步态\n'
        title += f'相位: {gait.global_phase:.2f} | '
        title += f'前进距离: {body_x*100:.1f}cm'
        ax.set_title(title, fontsize=12, fontweight='bold')
        
        return []
    
    # 创建动画
    anim = FuncAnimation(fig, update, frames=total_frames, interval=50, blit=False)
    
    # 保存动画
    output_path = os.path.join(os.path.dirname(__file__), 'gait_animation_demo_v4.gif')
    print(f"\n正在保存动画到: {output_path}")
    
    try:
        anim.save(output_path, writer='pillow', fps=20)
        print(f"✅ 动画已保存: {output_path}")
        
        # 显示IK成功率
        print("\nIK成功率统计：")
        for leg_name in model.legs.keys():
            success_rate = ik_success_count[leg_name] / ik_total_count[leg_name] * 100
            print(f"  {leg_name}: {success_rate:.1f}%")
        
        file_size = os.path.getsize(output_path) / 1024
        print(f"\n文件大小: {file_size:.1f} KB")
        
    except Exception as e:
        print(f"⚠️  保存动画失败: {e}")
    
    plt.close()

if __name__ == '__main__':
    main()
