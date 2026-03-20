#!/usr/bin/env python3
"""
Walk 步态可视化程序（改进版 - 完整机器人形态）

生成 Walk 步态的可视化图表和动画，包括完整的机器人形态
"""

import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.animation import FuncAnimation
from matplotlib import font_manager as fm

# 添加模块根目录到 Python 路径
module_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, module_root)

from gait_algo_core.walk_gait import WalkGait

# 机器人参数
BODY_LENGTH = 156  # mm
BODY_WIDTH = 78   # mm
L1 = 60.5   # 髋关节偏移
L2 = 10.0   # 髋关节侧摆连杆
L3 = 111.126  # 大腿长度
L4 = 118.5   # 小腿长度

# 髋关节位置（相对于身体中心）
HIP_POSITIONS = {
    'LF': (78, 39, 0),   # 左前
    'RF': (78, -39, 0),  # 右前
    'LB': (-78, 39, 0),  # 左后
    'RB': (-78, -39, 0)  # 右后
}

def setup_chinese_font():
    """配置中文字体（使用 BabelStoneHan.ttf）"""
    font_candidates = [
        os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))), 'fonts', 'BabelStoneHan.ttf'),
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
    return None

def forward_kinematics_2d(hip_pos, hip_side, hip_pitch, knee_pitch):
    """
    2D 正向运动学（侧视图）
    
    参数：
        hip_pos: 髋关节位置 (x, y, z)
        hip_side: 髋关节侧摆角度（弧度）
        hip_pitch: 髋关节俯仰角度（弧度）
        knee_pitch: 膝关节角度（弧度）
    
    返回：
        knee_pos: 膝关节位置 (x, y, z)
        foot_pos: 足端位置 (x, y, z)
    """
    # 简化计算：只考虑俯仰方向的2D投影
    # 大腿
    thigh_length = L3
    thigh_angle = hip_pitch
    
    knee_x = hip_pos[0] + thigh_length * np.cos(thigh_angle)
    knee_z = hip_pos[2] + thigh_length * np.sin(thigh_angle)
    knee_pos = (knee_x, hip_pos[1], knee_z)
    
    # 小腿
    calf_length = L4
    calf_angle = hip_pitch + knee_pitch
    
    foot_x = knee_x + calf_length * np.cos(calf_angle)
    foot_z = knee_z + calf_length * np.sin(calf_angle)
    foot_pos = (foot_x, hip_pos[1], foot_z)
    
    return knee_pos, foot_pos

def draw_robot_2d(ax, gait, phase):
    """
    绘制完整的机器人（2D侧视图）
    
    参数：
        ax: matplotlib 坐标轴
        gait: WalkGait 实例
        phase: 当前相位 (0-1)
    """
    # 清空坐标轴
    ax.clear()
    
    # 绘制身体（俯视图）
    body = Rectangle(
        (-BODY_LENGTH/2, -BODY_WIDTH/2),
        BODY_LENGTH,
        BODY_WIDTH,
        fill=False,
        edgecolor='navy',
        linewidth=2
    )
    ax.add_patch(body)
    
    # 绘制身体中心点
    ax.plot(0, 0, 'k+', markersize=10, markeredgewidth=2)
    
    # 腿部颜色
    leg_colors = {
        'LF': '#FF6B6B',  # 红色 - 左前
        'RF': '#4ECDC4',  # 青色 - 右前
        'LB': '#45B7D1',  # 蓝色 - 左后
        'RB': '#FFA07A'   # 橙色 - 右后
    }
    
    # 获取所有腿的轨迹
    trajectories = gait.get_all_foot_trajectories()
    
    # 绘制4条腿（俯视图）
    for leg_name, hip_pos in HIP_POSITIONS.items():
        # 获取足端轨迹偏移
        foot_offset_x, foot_offset_z = trajectories[leg_name]
        
        # 足端位置（世界坐标）
        foot_x = hip_pos[0] + foot_offset_x * 1000  # 转换为 mm
        foot_y = hip_pos[1]  # Y坐标不变
        foot_z = hip_pos[2] + foot_offset_z * 1000
        
        # 膝关节位置（简化：直线中点）
        knee_x = (hip_pos[0] + foot_x) / 2
        knee_y = hip_pos[1]
        knee_z = (hip_pos[2] + foot_z) / 2 + 30  # 抬高一点
        
        # 绘制腿部（俯视图：X-Y平面）
        color = leg_colors[leg_name]
        
        # 髋关节到膝关节（俯视图）
        ax.plot([hip_pos[0], knee_x], [hip_pos[1], knee_y], 
                color=color, linewidth=3, solid_capstyle='round')
        
        # 膝关节到足端（俯视图）
        ax.plot([knee_x, foot_x], [knee_y, foot_y], 
                color=color, linewidth=3, solid_capstyle='round')
        
        # 绘制关节点
        ax.plot(hip_pos[0], hip_pos[1], 'ko', markersize=5)  # 髋关节
        ax.plot(knee_x, knee_y, 'ko', markersize=5)  # 膝关节
        ax.plot(foot_x, foot_y, 'o', color=color, markersize=8)  # 足端
    
    # 设置坐标轴
    ax.set_xlim(-200, 200)
    ax.set_ylim(-150, 150)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_title(f'Walk Gait (Top View) - Phase: {phase:.2f}')

def main():
    """主函数"""
    print("=" * 60)
    print("Walk 步态可视化程序（完整机器人形态）")
    print("=" * 60)
    print("\n这个程序生成 Walk 步态的可视化动画\n")
    
    # 设置中文字体
    chinese_font = setup_chinese_font()
    
    # 创建步态控制器
    gait = WalkGait(
        stride_length=0.05,  # 50mm
        step_height=0.03,    # 30mm
        frequency=0.8
    )
    
    # 生成动画
    print("生成步态动画...")
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    frames = []
    for i in range(50):
        phase = i / 50
        gait.update(0.02)
        
        draw_robot_2d(ax, gait, phase)
        
        # 保存帧
        fig.canvas.draw()
        frames.append(np.array(fig.canvas.renderer.buffer_rgba()))
    
    # 保存动画
    output_file = os.path.join(os.getcwd(), 'walk_gait_animation.gif')
    print(f"正在保存动画到: {output_file}")
    print("（这可能需要几秒钟...）")
    
    # 使用 pillow 保存 GIF
    from PIL import Image
    images = [Image.fromarray(frame) for frame in frames]
    images[0].save(
        output_file,
        save_all=True,
        append_images=images[1:],
        duration=50,
        loop=0
    )
    
    print(f"✅ 动画已保存: {output_file}")
    
    print("\n" + "=" * 60)
    print("🎉 可视化已完成！")
    print("=" * 60)

if __name__ == '__main__':
    main()
