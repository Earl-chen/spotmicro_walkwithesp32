#!/usr/bin/env python3
# gait_animation_demo.py - 步态动画演示

import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.animation import FuncAnimation
import numpy as np

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
    
    leg_names_cn = {
        'left_front': '左前',
        'left_back': '左后',
        'right_front': '右前',
        'right_back': '右后'
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
            # 判断是否着地（最后一个点）
            is_foot = (i == len(joint_positions) - 1)
            
            # 获取当前腿的相位，判断是否在摆动相
            leg_phase = gait.get_leg_phase(leg_name)
            is_swing = leg_phase < 0.5  # 摆动相（前50%）
            
            if is_foot:
                # 足端：摆动相用圆形，支撑相用方形
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
    print("生成步态动画...")
    
    # 构建系统
    model, controller = build_system()
    
    # 创建步态控制器
    gait = WalkGait(stride_length=0.05, step_height=0.03, frequency=0.8)
    
    # 切换到正向运动学模式（直接控制关节角度）
    controller.enable_forward_kinematics_mode()
    
    # 创建图形
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    
    # 动画参数
    dt = 0.02  # 时间步长（秒）
    total_frames = 100  # 总帧数（约5秒）
    
    def update(frame):
        # 更新步态相位
        gait.update(dt)
        
        # 获取所有腿的足端轨迹
        trajectories = gait.get_all_foot_trajectories()
        
        # 对每条腿计算关节角度（通过逆运动学）
        for leg_name, (x_offset, z_offset) in trajectories.items():
            # 获取该腿的运动学求解器
            legkin, hip_frame = model.legs[leg_name]
            
            # 构造足端在髋关节坐标系中的位置
            # 假设初始足端位置在 (0, 0, -geometry.L2 - geometry.L4)
            foot_x = x_offset
            foot_y = 0.0
            foot_z = -geometry.L2 - geometry.L4 + z_offset
            
            foot_pos_hip = np.array([foot_x, foot_y, foot_z])
            
            # 使用逆运动学计算关节角度
            ik_result = legkin.inverse(foot_pos_hip)
            
            if ik_result.success:
                # 更新关节角度
                joints = ik_result.joints
                model.update_joint_angles(leg_name, joints)
        
        # 绘制机器人
        draw_robot(ax, model, controller, gait, gait.global_phase)
        
        # 设置标题
        title = f'四足机器人 Walk 步态\n相位: {gait.global_phase:.2f} | 步频: {gait.frequency}Hz'
        ax.set_title(title, fontsize=12, fontweight='bold')
        
        return []
    
    # 创建动画
    anim = FuncAnimation(fig, update, frames=total_frames, interval=50, blit=False)
    
    # 保存动画
    output_path = os.path.join(os.path.dirname(__file__), 'gait_animation_demo.gif')
    print(f"正在保存动画到: {output_path}")
    print("（这可能需要几秒钟...）")
    
    try:
        anim.save(output_path, writer='pillow', fps=20)
        print(f"✅ 动画已保存: {output_path}")
        
        # 显示文件大小
        file_size = os.path.getsize(output_path) / 1024
        print(f"文件大小: {file_size:.1f} KB")
        
    except Exception as e:
        print(f"⚠️  保存动画失败: {e}")
        import traceback
        traceback.print_exc()
    
    plt.close()

if __name__ == '__main__':
    main()
