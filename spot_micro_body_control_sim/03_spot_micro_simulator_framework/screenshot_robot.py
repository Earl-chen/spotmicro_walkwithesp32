#!/usr/bin/env python3
# screenshot_robot.py - 生成机器人界面截图

import sys
import os
import matplotlib
matplotlib.use('Agg')  # 使用非交互模式
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm

# 添加项目根目录到 Python 路径
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from core.frame_manager import FrameManager
from core.transform import WorldTransform
from robots.spot_micro.leg_kinematics import SpotLegKinematics
from robots.spot_micro import geometry
from app.robot_model import RobotModel
from app.controller import Controller
from core.types import LegJoints

# 配置中文字体
font_path = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
if os.path.exists(font_path):
    fm.fontManager.addfont(font_path)
    font_prop = fm.FontProperties(fname=font_path)
    plt.rcParams['font.family'] = font_prop.get_name()
    plt.rcParams['axes.unicode_minus'] = False

def build_system():
    """构建系统：坐标系 + 机器人模型 + 控制器"""
    fm_mgr = FrameManager()
    fm_mgr.set_frame("world", None, WorldTransform(0, 0, 0, 0, 0, 0))
    
    body_tf = WorldTransform(0.0, 0.0, -0.1, 0.0, 0.0, 0.0)  # 初始站立高度
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

def draw_robot_3d(ax, model, controller):
    """绘制3D机器人"""
    # 启用脚步锁定（捕获初始脚部位置）
    controller.capture_fixed_feet()
    
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
        'left_front': '左前腿',
        'left_back': '左后腿',
        'right_front': '右前腿',
        'right_back': '右后腿'
    }
    
    for leg_name in geometry.HIP_OFFSETS.keys():
        # 获取关节位置
        joint_positions = model.get_leg_joints_world(leg_name)
        
        # 绘制腿的各段
        for i in range(len(joint_positions) - 1):
            p1 = joint_positions[i]
            p2 = joint_positions[i + 1]
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 
                   c=colors[leg_name], linewidth=4, solid_capstyle='round')
        
        # 绘制关节点
        for i, pos in enumerate(joint_positions):
            size = 100 if i == 0 else 120 if i < len(joint_positions) - 1 else 150
            marker = 'o' if i < len(joint_positions) - 1 else 's'
            ax.scatter([pos[0]], [pos[1]], [pos[2]], 
                      c=colors[leg_name], s=size, marker=marker, 
                      edgecolors='black', linewidths=1.5)
    
    # 设置坐标轴
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_zlabel('Z (m)', fontsize=12)
    ax.set_title('四足机器人 - 站立姿态', fontsize=14, fontweight='bold')
    
    # 设置视角
    ax.set_xlim([-0.3, 0.3])
    ax.set_ylim([-0.3, 0.3])
    ax.set_zlim([-0.4, 0.2])
    ax.view_init(elev=20, azim=45)
    
    # 添加图例
    ax.legend(loc='upper left')

def main():
    print("生成机器人截图...")
    
    # 构建系统
    model, controller = build_system()
    
    # 创建图形
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制机器人
    draw_robot_3d(ax, model, controller)
    
    # 保存截图
    output_path = os.path.join(os.path.dirname(__file__), 'robot_screenshot.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"✅ 截图已保存: {output_path}")
    
    plt.close()

if __name__ == '__main__':
    main()
