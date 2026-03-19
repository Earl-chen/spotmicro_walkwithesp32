#!/usr/bin/env python3
# gait_animation_demo_v6.py - 步态动画演示（带基座轨迹）

import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.animation import FuncAnimation
import numpy as np
import math

# 兼容低版本matplotlib：导入3D投影支持
try:
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
except ImportError:
    pass  # matplotlib 3.2+ 不需要显式导入

# 使用脚本所在目录作为项目根目录
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from core.frame_manager import FrameManager
from core.transform import WorldTransform
from robots.spot_micro.leg_kinematics import SpotLegKinematics
from robots.spot_micro import geometry
from app.robot_model import RobotModel
from app.controller import Controller
from gait.walk_gait import WalkGait
from core.types import LegJoints

# 配置中文字体（兼容低版本matplotlib）
# 获取项目根目录
script_dir = os.path.dirname(os.path.abspath(__file__))
font_candidates = [
    os.path.join(script_dir, 'fonts', 'BabelStoneHan.ttf'),  # 项目目录下的字体（优先）
    os.path.expanduser('~/BabelStoneHan.ttf'),                # 用户home目录
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf',
    '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
    '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc',
]

chinese_font = None
for font_path in font_candidates:
    if os.path.exists(font_path):
        try:
            # 尝试使用addfont方法（matplotlib 3.2+）
            if hasattr(fm.fontManager, 'addfont'):
                fm.fontManager.addfont(font_path)
            chinese_font = fm.FontProperties(fname=font_path)
            
            # 设置全局字体（兼容不同版本）
            font_name = chinese_font.get_name()
            plt.rcParams['font.family'] = font_name
            plt.rcParams['font.sans-serif'] = [font_name, 'DejaVu Sans']
            plt.rcParams['axes.unicode_minus'] = False
            break
        except:
            continue

if chinese_font is None:
    chinese_font = fm.FontProperties()  # 使用默认字体

def build_system():
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
        legkin = SpotLegKinematics(l1=geometry.L1, l2=geometry.L2, l3=geometry.L3, l4=geometry.L4, is_left=is_left)
        initial_joints = LegJoints(0.0, 0.0, 0.0)
        model.add_leg(leg_name, legkin, hip_frame_name=f"hip_{leg_name}", initial_joints=initial_joints)
    
    controller = Controller(model)
    return model, controller

def set_standing_pose(model):
    hip_pitch_deg = 15.0
    knee_deg = -45.0
    
    for leg_name in model.legs.keys():
        joints = LegJoints(0.0, math.radians(hip_pitch_deg), math.radians(knee_deg))
        model.update_joint_angles(leg_name, joints)

def capture_initial_positions(model):
    initial_positions = {}
    for leg_name in model.legs.keys():
        joint_positions_body = model.get_leg_joints_body(leg_name)
        foot_pos_body = joint_positions_body[-1]
        legkin, hip_frame = model.legs[leg_name]
        foot_pos_hip = model.frame_manager.transform_point(foot_pos_body, "body", hip_frame)
        initial_positions[leg_name] = foot_pos_hip
    return initial_positions

def main():
    print("="*70)
    print("生成步态动画（修正版 + 基座轨迹）")
    print("="*70)
    
    model, controller = build_system()
    set_standing_pose(model)
    
    gait = WalkGait(stride_length=0.04, step_height=0.025, frequency=0.8)
    controller.enable_forward_kinematics_mode()
    
    initial_foot_positions = capture_initial_positions(model)
    
    # 累计前进距离
    total_distance = 0.0
    previous_phase = 0.0
    
    # ✅ 记录基座轨迹
    body_trajectory = []
    
    fig = plt.figure(figsize=(10, 7))
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
        
        # ✅ 记录基座位置
        body_pos = model.frame_manager.transform_point(np.array([0, 0, 0]), "body", "world")
        body_trajectory.append(body_pos.copy())
        
        # 更新所有腿的姿态
        trajectories = gait.get_all_foot_trajectories()
        
        for leg_name, (x_offset, z_offset) in trajectories.items():
            legkin, hip_frame = model.legs[leg_name]
            
            initial_pos = initial_foot_positions[leg_name]
            target_foot_pos_hip = initial_pos + np.array([x_offset, 0, z_offset])
            
            ik_result = legkin.inverse(target_foot_pos_hip)
            
            ik_total[leg_name] += 1
            if ik_result.success:
                ik_success[leg_name] += 1
                model.update_joint_angles(leg_name, ik_result.joints)
        
        # 清除并重新绘制
        ax.clear()
        
        # ✅ 绘制基座轨迹
        if len(body_trajectory) > 1:
            traj = np.array(body_trajectory)
            ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], 
                   color='blue', linewidth=2, alpha=0.6, label='基座轨迹')
            ax.scatter(traj[-1, 0], traj[-1, 1], traj[-1, 2], 
                      color='blue', s=200, marker='^', 
                      edgecolors='black', linewidths=2, 
                      label='当前基座位置', zorder=10)
        
        # 绘制机体
        body_outline = model.get_body_outline_world()
        body_x_pts = [p[0] for p in body_outline] + [body_outline[0][0]]
        body_y_pts = [p[1] for p in body_outline] + [body_outline[0][1]]
        body_z_pts = [p[2] for p in body_outline] + [body_outline[0][2]]
        ax.plot(body_x_pts, body_y_pts, body_z_pts, 'b-', linewidth=3)
        
        # 绘制四条腿
        colors = {'left_front': 'red', 'left_back': 'green', 
                 'right_front': 'orange', 'right_back': 'purple'}
        
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
                is_swing = leg_phase < 0.25
                
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
        
        # 统计支撑腿数量
        stance_count = sum(1 for leg in model.legs.keys() 
                          if gait.get_leg_phase(leg) >= 0.25)
        
        # 设置坐标轴
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        
        # 视野跟随
        body_center = model.frame_manager.transform_point(np.array([0, 0, 0]), "body", "world")
        ax.set_xlim([body_center[0] - 0.3, body_center[0] + 0.3])
        ax.set_ylim([-0.3, 0.3])
        ax.set_zlim([-0.4, 0.2])
        ax.view_init(elev=20, azim=45)
        
        # 设置标题
        title = f'Walk步态（修正版 + 基座轨迹）\n'
        title += f'相位: {gait.global_phase:.2f} | 支撑腿: {stance_count}条 | 前进: {body_x*100:.1f}cm'
        ax.set_title(title, fontsize=11, fontweight='bold')
        ax.legend(loc='upper left')
        
        return []
    
    # 创建动画
    anim = FuncAnimation(fig, update, frames=total_frames, interval=50, blit=False)
    
    # 保存动画
    output_path = os.path.join(project_root, 'gait_animation_demo_v6.gif')
    print(f"正在保存动画到: {output_path}")
    
    try:
        anim.save(output_path, writer='pillow', fps=20)
        print(f"✅ 动画已保存: {output_path}")
        
        # 输出统计信息
        print("\nIK成功率统计：")
        for leg in model.legs.keys():
            rate = ik_success[leg] / ik_total[leg] * 100 if ik_total[leg] > 0 else 1
            print(f"  {leg}: {rate:.1f}%")
        
        print("\n基座轨迹统计：")
        if body_trajectory:
            traj = np.array(body_trajectory)
            print(f"  X范围: {traj[:, 0].min()*100:.1f} ~ {traj[:, 0].max()*100:.1f} cm")
            print(f"  Y范围: {traj[:, 1].min()*100:.1f} ~ {traj[:, 1].max()*100:.1f} cm")
            print(f"  Z范围: {traj[:, 2].min()*100:.1f} ~ {traj[:, 2].max()*100:.1f} cm")
            print(f"  总移动距离: {(traj[-1, 0] - traj[0, 0])*100:.1f} cm")
        
        file_size = os.path.getsize(output_path) / 1024
        print(f"\n文件大小: {file_size:.1f} KB")
        print("="*70)
        
    except Exception as e:
        print(f"⚠️  保存动画失败: {e}")
        import traceback
        traceback.print_exc()
    
    plt.close()

if __name__ == '__main__':
    main()
