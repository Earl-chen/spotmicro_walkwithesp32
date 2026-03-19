#!/usr/bin/env python3
# gait_animation_demo_v2.py - 步态动画演示（修复版 + 数据分析）

import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.animation import FuncAnimation
import numpy as np
import json

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

def analyze_gait_data(gait_data):
    """分析步态数据，检测异常"""
    print("\n" + "="*60)
    print("步态数据分析报告")
    print("="*60)
    
    issues = []
    
    # 1. 检查足端位置范围
    print("\n1. 足端位置范围（世界坐标系）：")
    for leg_name, positions in gait_data['foot_positions_world'].items():
        positions = np.array(positions)
        x_range = [positions[:, 0].min(), positions[:, 0].max()]
        y_range = [positions[:, 1].min(), positions[:, 1].max()]
        z_range = [positions[:, 2].min(), positions[:, 2].max()]
        
        print(f"  {leg_name}:")
        print(f"    X: [{x_range[0]:.3f}, {x_range[1]:.3f}] m (范围: {x_range[1]-x_range[0]:.3f}m)")
        print(f"    Y: [{y_range[0]:.3f}, {y_range[1]:.3f}] m (范围: {y_range[1]-y_range[0]:.3f}m)")
        print(f"    Z: [{z_range[0]:.3f}, {z_range[1]:.3f}] m (范围: {z_range[1]-z_range[0]:.3f}m)")
        
        # 检查异常
        if abs(x_range[1] - x_range[0]) > 0.15:
            issues.append(f"⚠️ {leg_name} X方向移动范围过大: {x_range[1]-x_range[0]:.3f}m")
        if abs(z_range[1] - z_range[0]) > 0.1:
            issues.append(f"⚠️ {leg_name} Z方向移动范围过大: {z_range[1]-z_range[0]:.3f}m")
    
    # 2. 检查关节角度范围
    print("\n2. 关节角度范围（度）：")
    for leg_name, angles in gait_data['joint_angles'].items():
        angles = np.array(angles)
        hip_side = np.degrees(angles[:, 0])
        hip_pitch = np.degrees(angles[:, 1])
        knee = np.degrees(angles[:, 2])
        
        print(f"  {leg_name}:")
        print(f"    hip_side: [{hip_side.min():.1f}°, {hip_side.max():.1f}°]")
        print(f"    hip_pitch: [{hip_pitch.min():.1f}°, {hip_pitch.max():.1f}°]")
        print(f"    knee: [{knee.min():.1f}°, {knee.max():.1f}°]")
        
        # 检查异常
        if abs(hip_side.max() - hip_side.min()) > 45:
            issues.append(f"⚠️ {leg_name} hip_side角度范围过大: {hip_side.max()-hip_side.min():.1f}°")
        if abs(knee.max() - knee.min()) > 90:
            issues.append(f"⚠️ {leg_name} knee角度范围过大: {knee.max()-knee.min():.1f}°")
    
    # 3. 检查IK成功率和失败原因
    print("\n3. 逆运动学成功率和失败原因：")
    ik_failure_reasons = gait_data.get('ik_failure_reasons', {})
    for leg_name, success_rate in gait_data['ik_success_rate'].items():
        print(f"  {leg_name}: {success_rate*100:.1f}%")
        if leg_name in ik_failure_reasons:
            reasons = ik_failure_reasons[leg_name]
            for reason, count in sorted(reasons.items(), key=lambda x: x[1], reverse=True)[:3]:
                print(f"    - {reason}: {count}次")
        if success_rate < 0.9:
            issues.append(f"⚠️ {leg_name} IK成功率低: {success_rate*100:.1f}%")
    
    # 4. 检查步态相位
    print("\n4. 步态相位分析：")
    for leg_name, phases in gait_data['leg_phases'].items():
        phases = np.array(phases)
        swing_count = np.sum(phases < 0.5)
        stance_count = np.sum(phases >= 0.5)
        print(f"  {leg_name}: 摆动相 {swing_count}帧, 支撑相 {stance_count}帧")
    
    # 输出问题列表
    if issues:
        print("\n" + "="*60)
        print("⚠️  检测到的问题：")
        for issue in issues:
            print(f"  {issue}")
        print("="*60)
    else:
        print("\n✅ 未检测到明显异常")
    
    return issues

def draw_robot(ax, model, controller, gait, phase):
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
        
        # 绘制腿的各段
        for i in range(len(joint_positions) - 1):
            p1 = joint_positions[i]
            p2 = joint_positions[i + 1]
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 
                   c=colors[leg_name], linewidth=4, solid_capstyle='round')
        
        # 绘制关节点
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
    
    # 设置坐标轴
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_xlim([-0.3, 0.3])
    ax.set_ylim([-0.3, 0.3])
    ax.set_zlim([-0.4, 0.2])
    ax.view_init(elev=20, azim=45)
    ax.legend(loc='upper left')

def main():
    print("生成步态动画（修复版 + 数据分析）...")
    
    # 构建系统
    model, controller = build_system()
    
    # 创建步态控制器（调整参数以提高IK成功率）
    gait = WalkGait(stride_length=0.04, step_height=0.025, frequency=0.8)
    
    # 切换到正向运动学模式
    controller.enable_forward_kinematics_mode()
    
    # ========== 关键修复：捕获初始足端位置 ==========
    print("\n捕获初始足端位置...")
    initial_foot_positions = {}
    for leg_name in model.legs.keys():
        # 获取初始足端在机体坐标系中的位置
        joint_positions_body = model.get_leg_joints_body(leg_name)
        initial_foot_pos_body = joint_positions_body[-1]  # 最后一维是足端
        
        # 转换到髋关节坐标系（从body坐标系转换到hip坐标系）
        legkin, hip_frame = model.legs[leg_name]
        # 使用逆变换：body -> hip
        initial_foot_pos_hip = model.frame_manager.transform_point(
            initial_foot_pos_body, 
            from_frame="body", 
            to_frame=hip_frame
        )
        
        initial_foot_positions[leg_name] = initial_foot_pos_hip
        print(f"  {leg_name}: {initial_foot_pos_hip}")
    
    # 数据收集
    gait_data = {
        'foot_positions_world': {leg: [] for leg in geometry.HIP_OFFSETS.keys()},
        'joint_angles': {leg: [] for leg in geometry.HIP_OFFSETS.keys()},
        'leg_phases': {leg: [] for leg in geometry.HIP_OFFSETS.keys()},
        'ik_success_rate': {leg: [] for leg in geometry.HIP_OFFSETS.keys()}
    }
    
    # 创建图形
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    
    # 动画参数
    dt = 0.02
    total_frames = 100
    
    # 记录初始机体位置
    initial_body_x = 0.0
    
    def update(frame):
        # 更新步态相位
        gait.update(dt)
        
        # ========== 新增：更新机体位置（向前移动）==========
        body_x_offset = gait.global_phase * gait.stride_length
        controller.set_body_pose(
            initial_body_x + body_x_offset,  # X: 向前移动
            0, -0.1, 0, 0, 0  # Y, Z, Roll, Pitch, Yaw
        )
        
        # 获取所有腿的足端轨迹
        trajectories = gait.get_all_foot_trajectories()
        
        # 对每条腿计算关节角度
        for leg_name, (x_offset, z_offset) in trajectories.items():
            legkin, hip_frame = model.legs[leg_name]
            
            # ========== 修复：初始位置 + 步态偏移 ==========
            initial_pos = initial_foot_positions[leg_name]
            target_foot_pos_hip = initial_pos + np.array([x_offset, 0, z_offset])
            
            # 使用逆运动学计算关节角度
            ik_result = legkin.inverse(target_foot_pos_hip)
            
            # 记录IK成功/失败
            gait_data['ik_success_rate'][leg_name].append(ik_result.success)
            
            if ik_result.success:
                joints = ik_result.joints
                model.update_joint_angles(leg_name, joints)
                
                # 记录关节角度
                gait_data['joint_angles'][leg_name].append([joints.hip_side, joints.hip_pitch, joints.knee_pitch])
            else:
                # IK失败，记录当前角度
                current_joints = model.joints[leg_name]
                gait_data['joint_angles'][leg_name].append([current_joints.hip_side, current_joints.hip_pitch, current_joints.knee_pitch])
            
            # 记录足端位置和相位
            foot_pos_world = model.get_leg_joints_world(leg_name)[-1]
            gait_data['foot_positions_world'][leg_name].append(foot_pos_world)
            gait_data['leg_phases'][leg_name].append(gait.get_leg_phase(leg_name))
        
        # 绘制机器人
        draw_robot(ax, model, controller, gait, gait.global_phase)
        
        # 设置标题
        title = f'四足机器人 Walk 步态\n相位: {gait.global_phase:.2f} | 步频: {gait.frequency}Hz'
        ax.set_title(title, fontsize=12, fontweight='bold')
        
        return []
    
    # 创建动画
    anim = FuncAnimation(fig, update, frames=total_frames, interval=50, blit=False)
    
    # 保存动画
    output_path = os.path.join(os.path.dirname(__file__), 'gait_animation_demo_v2.gif')
    print(f"\n正在保存动画到: {output_path}")
    
    try:
        anim.save(output_path, writer='pillow', fps=20)
        print(f"✅ 动画已保存: {output_path}")
        
        file_size = os.path.getsize(output_path) / 1024
        print(f"文件大小: {file_size:.1f} KB")
        
    except Exception as e:
        print(f"⚠️  保存动画失败: {e}")
        import traceback
        traceback.print_exc()
    
    plt.close()
    
    # 分析数据
    for leg_name in gait_data['ik_success_rate'].keys():
        success_list = gait_data['ik_success_rate'][leg_name]
        gait_data['ik_success_rate'][leg_name] = sum(success_list) / len(success_list) if success_list else 0
    
    analyze_gait_data(gait_data)

if __name__ == '__main__':
    main()
