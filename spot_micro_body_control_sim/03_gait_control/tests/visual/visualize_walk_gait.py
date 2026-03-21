#!/usr/bin/env python3
"""
Walk 步态可视化程序（修正版 - 使用真实正向运动学）

修正内容：
1. 使用 SpotLegKinematics 进行真实的正向运动学计算
2. 使用 RobotModel 和 Controller 管理机器人状态
3. 绘制完整的机器人形态（身体 + 所有关节连接）
4. 3D 视图展示真实的腿部运动
"""

import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.animation import FuncAnimation
import math

# 兼容低版本matplotlib：导入3D投影支持
try:
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
except ImportError:
    pass  # matplotlib 3.2+ 不需要显式导入

# 添加 simulator_app 到 Python 路径
simulator_app_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))), 'simulator_app')
sys.path.insert(0, simulator_app_path)

from core.frame_manager import FrameManager
from core.transform import WorldTransform
from robots.spot_micro.leg_kinematics import SpotLegKinematics
from robots.spot_micro import geometry
from app.robot_model import RobotModel
from app.controller import Controller
from core.types import LegJoints

# 导入本地的 WalkGait（03_gait_control 版本）
module_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, module_root)
from gait_algo_core.walk_gait import WalkGait

# 配置中文字体
def setup_chinese_font():
    # 获取脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    font_candidates = [
        # 向上3级到 spot_micro_body_control_sim/fonts/
        os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(script_dir))),
            'fonts',
            'BabelStoneHan.ttf'
        ),
    ]
    
    for font_path in font_candidates:
        abs_path = os.path.abspath(font_path)
        if os.path.exists(abs_path):
            try:
                try:
                    fm._load_fontmanager(try_read_cache=False)
                except:
                    pass
                
                if hasattr(fm.fontManager, 'addfont'):
                    fm.fontManager.addfont(abs_path)
                
                chinese_font = fm.FontProperties(fname=abs_path)
                font_name = chinese_font.get_name()
                
                plt.rcParams['font.family'] = font_name
                plt.rcParams['font.sans-serif'] = [font_name, 'DejaVu Sans']
                plt.rcParams['axes.unicode_minus'] = False
                
                print(f"✅ 已加载中文字体: {font_name}")
                return chinese_font
            except Exception as e:
                print(f"⚠️ 加载字体失败: {e}")
    
    print("⚠️ 使用默认字体")
    return fm.FontProperties()

def build_system():
    """构建机器人系统（从 simulator_app）"""
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
            l1=geometry.L1, 
            l2=geometry.L2, 
            l3=geometry.L3, 
            l4=geometry.L4, 
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
        foot_pos_hip = model.frame_manager.transform_point(foot_pos_body, "body", hip_frame)
        initial_positions[leg_name] = foot_pos_hip
    return initial_positions

def main():
    """主函数"""
    print("=" * 70)
    print("Walk 步态可视化程序（修正版 - 真实正向运动学）")
    print("=" * 70)
    print("\n修正内容：")
    print("  1. 使用 SpotLegKinematics 进行真实的运动学计算")
    print("  2. 绘制完整的机器人形态（身体 + 关节连接）")
    print("  3. 3D 视图展示真实的腿部运动")
    print()
    
    # 设置中文字体
    chinese_font = setup_chinese_font()
    
    # 构建系统
    print("构建机器人系统...")
    model, controller = build_system()
    set_standing_pose(model)
    
    # 创建步态控制器
    gait = WalkGait(
        stride_length=0.05,  # 50mm
        step_height=0.03,    # 30mm
        frequency=0.8
    )
    
    # 启用正向运动学模式
    controller.enable_forward_kinematics_mode()
    
    # 捕获初始足端位置
    initial_foot_positions = capture_initial_positions(model)
    
    print(f"初始足端位置（髋关节坐标系）：")
    for leg_name, pos in initial_foot_positions.items():
        print(f"  {leg_name}: ({pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}) mm")
    
    # 累计前进距离
    total_distance = 0.0
    previous_phase = 0.0
    
    # 记录基座轨迹
    body_trajectory = []
    
    # 创建图形
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    dt = 0.02
    total_frames = 100
    
    # IK 成功统计
    ik_success = {leg: 0 for leg in model.legs.keys()}
    ik_total = {leg: 0 for leg in model.legs.keys()}
    
    print(f"\n开始生成 {total_frames} 帧动画...")
    
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
        
        # 记录基座位置
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
        
        # 绘制基座轨迹
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
        
        # 绘制四条腿（完整关节连接）
        colors = {
            'left_front': 'red', 
            'left_back': 'green', 
            'right_front': 'orange', 
            'right_back': 'purple'
        }
        
        for leg_name in geometry.HIP_OFFSETS.keys():
            # 获取所有关节位置（世界坐标系）
            joint_positions = model.get_leg_joints_world(leg_name)
            
            # 绘制腿部连接（髋→膝→足）
            for i in range(len(joint_positions) - 1):
                p1 = joint_positions[i]
                p2 = joint_positions[i + 1]
                ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 
                       c=colors[leg_name], linewidth=4, solid_capstyle='round')
            
            # 绘制关节点
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
        ax.set_xlabel('X (m)', fontsize=11)
        ax.set_ylabel('Y (m)', fontsize=11)
        ax.set_zlabel('Z (m)', fontsize=11)
        
        # 视野跟随
        body_center = model.frame_manager.transform_point(np.array([0, 0, 0]), "body", "world")
        ax.set_xlim([body_center[0] - 0.3, body_center[0] + 0.3])
        ax.set_ylim([-0.3, 0.3])
        ax.set_zlim([-0.4, 0.2])
        ax.view_init(elev=20, azim=45)
        
        # 设置标题
        title = f'Walk步态（修正版）\n'
        title += f'相位: {gait.global_phase:.2f} | 支撑腿: {stance_count}条 | 前进: {body_x*100:.1f}cm'
        ax.set_title(title, fontsize=12, fontweight='bold')
        ax.legend(loc='upper left')
        
        # 进度显示
        if frame % 20 == 0:
            print(f"  已生成 {frame}/{total_frames} 帧")
        
        return []
    
    # 创建动画
    print("创建动画对象...")
    anim = FuncAnimation(fig, update, frames=total_frames, interval=50, blit=False)
    
    # 保存动画
    # 保存动画到当前工作目录
    output_file = os.path.join(os.getcwd(), 'walk_gait_animation.gif')
    print(f"\n正在保存动画到: {output_file}")
    print("（这可能需要几秒钟...）")
    
    # 使用 pillow 保存 GIF
    from PIL import Image
    
    # 手动渲染每一帧并保存
    frames_images = []
    for frame_idx in range(total_frames):
        update(frame_idx)
        fig.canvas.draw()
        img = np.array(fig.canvas.renderer.buffer_rgba())
        frames_images.append(Image.fromarray(img))
    
    frames_images[0].save(
        output_file,
        save_all=True,
        append_images=frames_images[1:],
        duration=50,
        loop=0
    )
    
    print(f"✅ 动画已保存: {output_file}")
    
    # 输出 IK 统计
    print("\n" + "=" * 70)
    print("IK 成功率统计：")
    for leg_name in model.legs.keys():
        success_rate = ik_success[leg_name] / ik_total[leg_name] * 100 if ik_total[leg_name] > 0 else 0
        print(f"  {leg_name}: {ik_success[leg_name]}/{ik_total[leg_name]} ({success_rate:.1f}%)")
    
    print("\n" + "=" * 70)
    print("🎉 可视化已完成！")
    print("=" * 70)

if __name__ == '__main__':
    main()
