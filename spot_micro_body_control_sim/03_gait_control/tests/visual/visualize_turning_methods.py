#!/usr/bin/env python3
"""
转向方法可视化程序（3D 机器人模型版）

展示不同转向方法的 3D 动画：
- cmd_vel() - ROS 风格接口
- zero_radius_turn() - 零半径转向
- cmd_vel_adaptive() - 自适应转向
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

# 导入通用中文字体配置模块
sys.path.insert(0, os.path.join(module_root, 'tests'))
from chinese_font_config import setup_chinese_font


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


def create_turning_animation(turning_config, output_file, title, frames=100, dt=0.02):
    """
    创建转向动画（3D 机器人模型）
    
    参数：
        turning_config: 转向配置字典
        output_file: 输出文件路径
        title: 动画标题
        frames: 帧数
        dt: 时间步长
    """
    # 设置中文字体
    chinese_font = setup_chinese_font()
    
    # 构建系统
    model, controller = build_system()
    set_standing_pose(model)
    
    # 创建步态控制器
    gait = WalkGait(
        stride_length=0.05,
        step_height=0.03,
        frequency=0.8
    )
    
    # 应用转向设置
    if 'cmd_vel' in turning_config:
        gait.cmd_vel(**turning_config['cmd_vel'])
    elif 'zero_radius_turn' in turning_config:
        gait.zero_radius_turn(**turning_config['zero_radius_turn'])
    elif 'cmd_vel_adaptive' in turning_config:
        gait.cmd_vel_adaptive(**turning_config['cmd_vel_adaptive'])
    
    # 启用正向运动学模式
    controller.enable_forward_kinematics_mode()
    
    # 捕获初始足端位置
    initial_foot_positions = capture_initial_positions(model)
    
    # 创建图形
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # IK 成功统计
    ik_success = {leg: 0 for leg in model.legs.keys()}
    ik_total = {leg: 0 for leg in model.legs.keys()}
    
    def draw_robot(ax, model, gait, initial_positions, chinese_font):
        """绘制机器人"""
        ax.clear()
        
        # 设置坐标轴
        ax.set_xlim(-0.3, 0.3)
        ax.set_ylim(-0.3, 0.3)
        ax.set_zlim(-0.3, 0.1)
        ax.set_xlabel('X (m)', fontsize=10, fontproperties=chinese_font)
        ax.set_ylabel('Y (m)', fontsize=10, fontproperties=chinese_font)
        ax.set_zlabel('Z (m)', fontsize=10, fontproperties=chinese_font)
        
        # 获取速度信息
        vel = gait.get_velocity()
        ax.set_title(f'{title}\n'
                    f'前进: {vel["forward"]*100:.1f} cm/s | '
                    f'转向: {vel["yaw_rate"]:.2f} rad/s',
                    fontsize=12, fontproperties=chinese_font)
        
        # 绘制身体（矩形）
        body_corners = np.array([
            [0.10375, 0.039, 0.0],
            [0.10375, -0.039, 0.0],
            [-0.10375, -0.039, 0.0],
            [-0.10375, 0.039, 0.0],
            [0.10375, 0.039, 0.0]
        ])
        ax.plot(body_corners[:, 0], body_corners[:, 1], body_corners[:, 2], 
                'b-', linewidth=2, label='身体')
        
        # 绘制腿部
        colors = {
            'left_front': '#FF6B6B',
            'right_front': '#4ECDC4',
            'left_back': '#45B7D1',
            'right_back': '#FFA07A'
        }
        
        for leg_name in model.legs.keys():
            # 获取步态偏移
            x_offset, z_offset = gait.get_foot_trajectory(leg_name)
            
            # 目标足端位置（髋关节坐标系）
            init_pos = initial_positions[leg_name]
            target_hip = np.array([
                init_pos[0] + x_offset,
                init_pos[1],
                init_pos[2] + z_offset
            ])
            
            # 获取腿的运动学对象
            legkin, hip_frame = model.legs[leg_name]
            
            # IK 求解（在髋关节坐标系）
            ik_total[leg_name] += 1
            ik_result = legkin.inverse(target_hip)
            
            if ik_result.success:
                ik_success[leg_name] += 1
                model.update_joint_angles(leg_name, ik_result.joints)
                
                # 获取关节位置（body 坐标系）
                joint_positions = model.get_leg_joints_body(leg_name)
                
                # 获取髋关节位置（直接从 geometry）
                hip_offset = geometry.HIP_OFFSETS[leg_name]
                hip_pos = np.array(hip_offset)
                
                # 绘制腿部关节连接
                shoulder_pos = joint_positions[1]
                elbow_pos = joint_positions[2]
                foot_pos = joint_positions[3]
                
                # 绘制连接线
                ax.plot([hip_pos[0], shoulder_pos[0]], 
                       [hip_pos[1], shoulder_pos[1]], 
                       [hip_pos[2], shoulder_pos[2]], 
                       'k-', linewidth=2)
                ax.plot([shoulder_pos[0], elbow_pos[0]], 
                       [shoulder_pos[1], elbow_pos[1]], 
                       [shoulder_pos[2], elbow_pos[2]], 
                       color=colors[leg_name], linewidth=2)
                ax.plot([elbow_pos[0], foot_pos[0]], 
                       [elbow_pos[1], foot_pos[1]], 
                       [elbow_pos[2], foot_pos[2]], 
                       color=colors[leg_name], linewidth=2)
                
                # 绘制关节点
                ax.scatter(*hip_pos, c='black', s=30, zorder=5)
                ax.scatter(*shoulder_pos, c='black', s=30, zorder=5)
                ax.scatter(*elbow_pos, c=colors[leg_name], s=30, zorder=5)
                ax.scatter(*foot_pos, c=colors[leg_name], s=50, marker='o', zorder=5)
        
        ax.legend(prop=chinese_font, loc='upper left')
        ax.view_init(elev=30, azim=-60)
    
    def update(frame):
        """更新动画帧"""
        gait.update(dt)
        draw_robot(ax, model, gait, initial_foot_positions, chinese_font)
        return []
    
    # 创建动画
    anim = FuncAnimation(fig, update, frames=frames, interval=dt*1000, blit=False)
    
    # 保存动画
    print(f"正在生成 {output_file}...")
    anim.save(output_file, writer='pillow', fps=30, dpi=100)
    print(f"✅ 已保存: {output_file}")
    
    # 打印 IK 成功率
    print("\nIK 成功率统计：")
    for leg_name in model.legs.keys():
        total = ik_total[leg_name]
        success = ik_success[leg_name]
        rate = success / total * 100 if total > 0 else 0
        print(f"  {leg_name}: {success}/{total} ({rate:.1f}%)")
    
    plt.close(fig)
    
    return output_file


def main():
    """主函数"""
    print("=" * 70)
    print("转向方法可视化（3D 机器人模型版）")
    print("=" * 70)
    print("\n修正内容：")
    print("  1. 使用 SpotLegKinematics 进行真实的运动学计算")
    print("  2. 绘制完整的机器人形态（身体 + 关节连接）")
    print("  3. 3D 视图展示真实的腿部运动")
    print("  4. 展示不同转向方法的效果")
    print()
    
    # 输出目录
    output_dir = os.getcwd()
    
    # 定义测试场景
    scenarios = [
        # === cmd_vel() 场景 ===
        {
            'name': 'cmd_vel_forward',
            'title': 'cmd_vel 前进',
            'config': {
                'cmd_vel': {'linear_x': 0.05, 'linear_y': 0.0, 'angular_z': 0.0}
            }
        },
        {
            'name': 'cmd_vel_turning',
            'title': 'cmd_vel 转向',
            'config': {
                'cmd_vel': {'linear_x': 0.05, 'linear_y': 0.0, 'angular_z': 0.3}
            }
        },
        
        # === zero_radius_turn() 场景 ===
        {
            'name': 'zero_radius_left',
            'title': '零半径左转',
            'config': {
                'zero_radius_turn': {'yaw_rate': 0.5}
            }
        },
        {
            'name': 'zero_radius_right',
            'title': '零半径右转',
            'config': {
                'zero_radius_turn': {'yaw_rate': -0.5}
            }
        },
        
        # === cmd_vel_adaptive() 场景 ===
        {
            'name': 'adaptive_differential',
            'title': '自适应差速转向',
            'config': {
                'cmd_vel_adaptive': {'linear_x': 0.05, 'linear_y': 0.0, 'angular_z': 0.3}
            }
        },
        {
            'name': 'adaptive_zero_radius',
            'title': '自适应零半径转向',
            'config': {
                'cmd_vel_adaptive': {'linear_x': 0.01, 'linear_y': 0.0, 'angular_z': 0.6}
            }
        },
    ]
    
    # 生成所有场景的动画
    generated_files = []
    for i, scenario in enumerate(scenarios, 1):
        print(f"\n【场景 {i}/{len(scenarios)}】{scenario['title']}")
        print("-" * 70)
        
        output_file = os.path.join(output_dir, f"turning_{scenario['name']}.gif")
        create_turning_animation(
            turning_config=scenario['config'],
            output_file=output_file,
            title=scenario['title'],
            frames=100,
            dt=0.02
        )
        generated_files.append(output_file)
    
    print("\n" + "=" * 70)
    print("✅ 所有动画生成完成！")
    print("=" * 70)
    print(f"\n生成的文件：")
    for i, file in enumerate(generated_files, 1):
        size = os.path.getsize(file) / 1024 / 1024  # MB
        print(f"  {i}. {os.path.basename(file)} ({size:.2f} MB)")
    
    print("\n📋 场景说明：")
    print("  1-2. cmd_vel() - ROS 风格接口")
    print("  3-4. zero_radius_turn() - 零半径转向")
    print("  5-6. cmd_vel_adaptive() - 自适应转向")


if __name__ == '__main__':
    main()
