#!/usr/bin/env python3
"""
转向方法可视化脚本

生成不同转向方法的 GIF 动画：
- cmd_vel() 场景
- zero_radius_turn() 场景
- cmd_vel_adaptive() 场景
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle

# 导入通用中文字体配置模块
module_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, module_root)
from gait_algo_core.walk_gait import WalkGait
sys.path.insert(0, os.path.join(module_root, 'tests'))
from chinese_font_config import setup_chinese_font

# 配置中文字体
chinese_font = setup_chinese_font()


def create_turning_animation(gait_config, output_file, title, frames=100, dt=0.02):
    """
    创建转向动画
    
    参数：
        gait_config: 步态配置字典
        output_file: 输出文件路径
        title: 动画标题
        frames: 帧数
        dt: 时间步长
    """
    # 创建步态控制器
    gait = WalkGait(**gait_config.get('init', {}))
    
    # 应用转向设置
    if 'cmd_vel' in gait_config:
        gait.cmd_vel(**gait_config['cmd_vel'])
    elif 'zero_radius_turn' in gait_config:
        gait.zero_radius_turn(**gait_config['zero_radius_turn'])
    elif 'cmd_vel_adaptive' in gait_config:
        gait.cmd_vel_adaptive(**gait_config['cmd_vel_adaptive'])
    
    # 腿名称
    leg_names = ['right_front', 'left_back', 'left_front', 'right_back']
    
    # 创建图形
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    
    # === 左图：俯视图 ===
    ax1.set_xlim(-200, 200)
    ax1.set_ylim(-150, 150)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3, linestyle='--')
    ax1.set_xlabel('X 轴 (mm)', fontsize=12, fontproperties=chinese_font)
    ax1.set_ylabel('Y 轴 (mm)', fontsize=12, fontproperties=chinese_font)
    ax1.set_title(f'{title} - 俯视图', fontsize=14, fontweight='bold', fontproperties=chinese_font)
    
    # 机器人身体
    body_width = 78
    body_length = 121
    body = Rectangle((-body_length/2, -body_width/2), body_length, body_width,
                     fill=False, edgecolor='blue', linewidth=2)
    ax1.add_patch(body)
    
    # 绘制腿和轨迹的元素
    leg_lines = {}
    leg_points = {}
    trajectories = {leg: {'x': [], 'y': []} for leg in leg_names}
    
    colors = {
        'left_front': '#FF6B6B',
        'right_front': '#4ECDC4',
        'left_back': '#45B7D1',
        'right_back': '#FFA07A'
    }
    
    for leg_name in leg_names:
        line, = ax1.plot([], [], '-', color=colors[leg_name], linewidth=1.5, alpha=0.5)
        point, = ax1.plot([], [], 'o', color=colors[leg_name], markersize=8)
        leg_lines[leg_name] = line
        leg_points[leg_name] = point
    
    # 时间文本
    time_text = ax1.text(0.02, 0.95, '', transform=ax1.transAxes, fontsize=10,
                         verticalalignment='top', fontproperties=chinese_font)
    
    # === 右图：X 轴位置曲线 ===
    ax2.set_xlim(0, frames)
    ax2.set_ylim(-150, 150)
    ax2.grid(True, alpha=0.3, linestyle='--')
    ax2.set_xlabel('帧数', fontsize=12, fontproperties=chinese_font)
    ax2.set_ylabel('X 轴位置 (mm)', fontsize=12, fontproperties=chinese_font)
    ax2.set_title(f'{title} - 轨迹曲线', fontsize=14, fontweight='bold', fontproperties=chinese_font)
    
    # 轨迹曲线
    trajectory_lines = {}
    for leg_name in leg_names:
        line, = ax2.plot([], [], '-', color=colors[leg_name], linewidth=1.5, 
                        label=leg_name.replace('_', ' ').title())
        trajectory_lines[leg_name] = line
    
    ax2.legend(prop=chinese_font, loc='upper right')
    
    # 速度信息文本
    vel = gait.get_velocity()
    vel_text = ax2.text(0.02, 0.95, 
                       f"前进: {vel['forward']*100:.1f} cm/s\n"
                       f"侧向: {vel['lateral']*100:.1f} cm/s\n"
                       f"转向: {vel['yaw_rate']:.2f} rad/s",
                       transform=ax2.transAxes, fontsize=10,
                       verticalalignment='top', fontproperties=chinese_font,
                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # 髋关节位置
    hip_positions = {
        'left_front': (30.5, 60.5),
        'right_front': (30.5, -60.5),
        'left_back': (-30.5, 60.5),
        'right_back': (-30.5, -60.5)
    }
    
    # 存储完整轨迹用于右图
    full_trajectories = {leg: [] for leg in leg_names}
    
    def init():
        """初始化动画"""
        for leg_name in leg_names:
            leg_lines[leg_name].set_data([], [])
            leg_points[leg_name].set_data([], [])
            trajectory_lines[leg_name].set_data([], [])
        time_text.set_text('')
        return list(leg_lines.values()) + list(leg_points.values()) + list(trajectory_lines.values()) + [time_text]
    
    def update(frame):
        """更新动画帧"""
        gait.update(dt)
        
        for leg_name in leg_names:
            x, z = gait.get_foot_trajectory(leg_name)
            
            # 转换为毫米
            x_mm = x * 1000
            y_mm = 0  # 基础步态无侧向偏移
            
            # 髋关节位置
            hip_x, hip_y = hip_positions[leg_name]
            
            # 足端位置（髋关节 + 偏移）
            foot_x = hip_x + x_mm
            foot_y = hip_y + y_mm
            
            # 存储完整轨迹
            full_trajectories[leg_name].append(foot_x)
            
            # 更新最近轨迹（俯视图）
            trajectories[leg_name]['x'].append(foot_x)
            trajectories[leg_name]['y'].append(foot_y)
            
            # 保留最近 50 个点
            if len(trajectories[leg_name]['x']) > 50:
                trajectories[leg_name]['x'] = trajectories[leg_name]['x'][-50:]
                trajectories[leg_name]['y'] = trajectories[leg_name]['y'][-50:]
            
            # 更新轨迹线
            leg_lines[leg_name].set_data(trajectories[leg_name]['x'], trajectories[leg_name]['y'])
            
            # 更新足端点
            leg_points[leg_name].set_data([foot_x], [foot_y])
            
            # 更新轨迹曲线
            trajectory_lines[leg_name].set_data(range(len(full_trajectories[leg_name])), 
                                                full_trajectories[leg_name])
        
        # 更新时间文本
        time_text.set_text(f'时间: {frame * dt:.2f}s')
        
        return list(leg_lines.values()) + list(leg_points.values()) + list(trajectory_lines.values()) + [time_text]
    
    # 创建动画
    anim = FuncAnimation(fig, update, init_func=init, frames=frames, blit=True, interval=dt*1000)
    
    # 保存动画
    print(f"正在生成 {output_file}...")
    anim.save(output_file, writer='pillow', fps=30, dpi=100)
    print(f"✅ 已保存: {output_file}")
    
    plt.close(fig)
    
    return output_file


def main():
    """主函数"""
    print("=" * 70)
    print("转向方法可视化")
    print("=" * 70)
    
    # 输出目录
    output_dir = os.getcwd()
    
    # 定义测试场景
    scenarios = [
        # === cmd_vel() 场景 ===
        {
            'name': 'cmd_vel_forward',
            'title': 'cmd_vel 前进',
            'config': {
                'init': {'stride_length': 0.05, 'step_height': 0.03, 'frequency': 0.8},
                'cmd_vel': {'linear_x': 0.05, 'linear_y': 0.0, 'angular_z': 0.0}
            }
        },
        {
            'name': 'cmd_vel_turning',
            'title': 'cmd_vel 转向',
            'config': {
                'init': {'stride_length': 0.05, 'step_height': 0.03, 'frequency': 0.8},
                'cmd_vel': {'linear_x': 0.05, 'linear_y': 0.0, 'angular_z': 0.3}
            }
        },
        
        # === zero_radius_turn() 场景 ===
        {
            'name': 'zero_radius_left',
            'title': '零半径左转',
            'config': {
                'init': {'stride_length': 0.05, 'step_height': 0.03, 'frequency': 0.8},
                'zero_radius_turn': {'yaw_rate': 0.5}
            }
        },
        {
            'name': 'zero_radius_right',
            'title': '零半径右转',
            'config': {
                'init': {'stride_length': 0.05, 'step_height': 0.03, 'frequency': 0.8},
                'zero_radius_turn': {'yaw_rate': -0.5}
            }
        },
        
        # === cmd_vel_adaptive() 场景 ===
        {
            'name': 'adaptive_differential',
            'title': '自适应差速转向',
            'config': {
                'init': {'stride_length': 0.05, 'step_height': 0.03, 'frequency': 0.8},
                'cmd_vel_adaptive': {'linear_x': 0.05, 'linear_y': 0.0, 'angular_z': 0.3}
            }
        },
        {
            'name': 'adaptive_zero_radius',
            'title': '自适应零半径转向',
            'config': {
                'init': {'stride_length': 0.05, 'step_height': 0.03, 'frequency': 0.8},
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
            gait_config=scenario['config'],
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
        size = os.path.getsize(file) / 1024  # KB
        print(f"  {i}. {os.path.basename(file)} ({size:.1f} KB)")
    
    print("\n📋 场景说明：")
    print("  1-2. cmd_vel() - ROS 风格接口")
    print("  3-4. zero_radius_turn() - 零半径转向")
    print("  5-6. cmd_vel_adaptive() - 自适应转向")


if __name__ == '__main__':
    main()
