#!/usr/bin/env python3
# visualize_walk_gait.py - Walk 步态可视化

"""
Walk 步态可视化程序

生成步态动画和轨迹图，直观展示 Walk 步态的工作原理。

运行方式：
    cd visualization/
    python3 visualize_walk_gait.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib import font_manager
from mpl_toolkits.mplot3d import Axes3D

# 添加模块根目录到 Python 路径
module_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, module_root)

from gait_algo_core.walk_gait import WalkGait


def setup_chinese_font():
    """配置中文字体（使用 BabelStoneHan.ttf）"""
    # 仅使用相对路径
    font_candidates = [
        # 从 tests/visual/visualize_walk_gait.py 向上4级到 spot_micro_body_control_sim/fonts/
        os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))), 'fonts', 'BabelStoneHan.ttf'),
    ]
    
    chinese_font = None
    font_loaded = False
    
    for font_path in font_candidates:
        abs_path = os.path.abspath(font_path)
        if os.path.exists(abs_path):
            try:
                # 步骤1：清除字体缓存
                try:
                    font_manager._load_fontmanager(try_read_cache=False)
                except:
                    pass
                
                # 步骤2：注册字体
                if hasattr(font_manager.fontManager, 'addfont'):
                    font_manager.fontManager.addfont(abs_path)
                
                # 步骤3：创建 FontProperties
                chinese_font = font_manager.FontProperties(fname=abs_path)
                font_name = chinese_font.get_name()
                
                # 步骤4：检查字体管理器
                registered_fonts = [f.name for f in font_manager.fontManager.ttflist]
                if font_name not in registered_fonts:
                    # 手动添加到字体列表
                    font_entry = font_manager.FontEntry(
                        fname=abs_path,
                        name=font_name,
                        style='normal',
                        variant='normal',
                        weight='normal',
                        stretch='normal',
                        size='medium'
                    )
                    font_manager.fontManager.ttflist.append(font_entry)
                
                # 步骤5：设置全局字体
                plt.rcParams['font.family'] = font_name
                plt.rcParams['font.sans-serif'] = [font_name, 'DejaVu Sans', 'Arial Unicode MS']
                plt.rcParams['axes.unicode_minus'] = False
                
                print(f"✅ 已加载中文字体: {font_name}")
                font_loaded = True
                break
            except Exception as e:
                print(f"⚠️ 加载字体失败: {e}")
                continue
    
    if not font_loaded:
        print("⚠️ 使用默认字体")
    
    return chinese_font


def plot_trajectory_2d():
    """绘制2D轨迹图"""
    print("\n生成2D轨迹图...")
    
    gait = WalkGait(stride_length=0.05, step_height=0.03, frequency=0.8)
    
    # 生成一个完整周期的轨迹
    phases = np.linspace(0, 1, 100)
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Walk 步态轨迹分析', fontsize=16, fontweight='bold')
    
    # 绘制每条腿的轨迹
    for idx, (leg_name, offset) in enumerate(gait.phase_offsets.items()):
        ax = axes[idx // 2, idx % 2]
        
        # 计算轨迹
        x_traj = []
        z_traj = []
        
        for phase in phases:
            leg_phase = (phase + offset) % 1.0
            x, z = gait.get_foot_trajectory(leg_name)
            x_traj.append(x * 1000)  # 转换为 mm
            z_traj.append(z * 1000)
        
        # 绘制轨迹
        ax.plot(x_traj, z_traj, 'b-', linewidth=2, label='足端轨迹')
        
        # 标记摆动相和支撑相
        swing_end = int(len(phases) * 0.5)
        ax.plot(x_traj[:swing_end], z_traj[:swing_end], 'r-', linewidth=3, 
                label='摆动相（抬腿）', alpha=0.7)
        ax.plot(x_traj[swing_end:], z_traj[swing_end:], 'g-', linewidth=3, 
                label='支撑相（着地）', alpha=0.7)
        
        # 标记起点和终点
        ax.plot(x_traj[0], z_traj[0], 'go', markersize=10, label='起点')
        ax.plot(x_traj[-1], z_traj[-1], 'ro', markersize=10, label='终点')
        
        ax.set_xlabel('前后位置 X (mm)', fontsize=12)
        ax.set_ylabel('上下位置 Z (mm)', fontsize=12)
        ax.set_title(f'{leg_name}\n相位偏移: {offset*90:.0f}°', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=10)
        ax.set_aspect('equal')
    
    plt.tight_layout()
    
    # 保存图片
    output_path = os.path.join(os.getcwd(), 'walk_gait_trajectory_2d.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"✅ 2D轨迹图已保存: {output_path}")
    
    plt.close()  # 关闭图形，不显示


def plot_gait_sequence_3d():
    """绘制3D步态序列图"""
    print("\n生成3D步态序列图...")
    
    gait = WalkGait(stride_length=0.05, step_height=0.03, frequency=0.8)
    
    # 生成一个完整周期的步态序列
    phases = np.linspace(0, 1, 20)  # 20个关键帧
    
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle('Walk 步态序列（一个完整周期）', fontsize=16, fontweight='bold')
    
    # 绘制每个关键帧
    for i, phase in enumerate(phases[:16]):  # 只显示前16帧
        ax = fig.add_subplot(4, 4, i + 1, projection='3d')
        
        gait.global_phase = phase
        trajectories = gait.get_all_foot_trajectories()
        
        # 绘制机器人身体（简化为矩形）
        body_x = [-0.1, 0.1, 0.1, -0.1, -0.1]
        body_y = [-0.04, -0.04, 0.04, 0.04, -0.04]
        body_z = [0, 0, 0, 0, 0]
        ax.plot(body_x, body_y, body_z, 'b-', linewidth=2)
        
        # 绘制四条腿
        leg_positions = {
            'right_front': (0.1, -0.04, -0.2),
            'left_front': (0.1, 0.04, -0.2),
            'right_back': (-0.1, -0.04, -0.2),
            'left_back': (-0.1, 0.04, -0.2)
        }
        
        colors = {
            'right_front': 'red',
            'left_front': 'blue',
            'right_back': 'green',
            'left_back': 'orange'
        }
        
        for leg_name, (base_x, base_y, base_z) in leg_positions.items():
            x_offset, z_offset = trajectories[leg_name]
            
            # 足端位置
            foot_x = base_x + x_offset
            foot_y = base_y
            foot_z = base_z + z_offset
            
            # 计算中间关节点（髋关节和膝关节）
            # 髋关节：在基座和足端的中间偏上
            hip_x = base_x + (foot_x - base_x) * 0.3
            hip_y = base_y
            hip_z = base_z + (foot_z - base_z) * 0.3 + 0.05  # 稍微抬高
            
            # 膝关节：在髋关节和足端的中间
            knee_x = base_x + (foot_x - base_x) * 0.6
            knee_y = base_y
            knee_z = base_z + (foot_z - base_z) * 0.6
            
            # 绘制腿（三段）
            # 第一段：基座 -> 髋关节
            ax.plot([base_x, hip_x], [base_y, hip_y], [base_z, hip_z], 
                   c=colors[leg_name], linewidth=4, solid_capstyle='round')
            
            # 第二段：髋关节 -> 膝关节
            ax.plot([hip_x, knee_x], [hip_y, knee_y], [hip_z, knee_z], 
                   c=colors[leg_name], linewidth=4, solid_capstyle='round')
            
            # 第三段：膝关节 -> 足端
            ax.plot([knee_x, foot_x], [knee_y, foot_y], [knee_z, foot_z], 
                   c=colors[leg_name], linewidth=4, solid_capstyle='round')
            
            # 绘制关节点
            ax.scatter([base_x], [base_y], [base_z], c=colors[leg_name], s=80, marker='o', edgecolors='black', linewidths=1)
            ax.scatter([hip_x], [hip_y], [hip_z], c=colors[leg_name], s=100, marker='o', edgecolors='black', linewidths=1)
            ax.scatter([knee_x], [knee_y], [knee_z], c=colors[leg_name], s=100, marker='o', edgecolors='black', linewidths=1)
            
            # 绘制足端
            ax.scatter([foot_x], [foot_y], [foot_z], c=colors[leg_name], s=120, marker='o', edgecolors='black', linewidths=2)
            
            # 标记是否着地
            if z_offset < 0.001:  # 着地
                ax.scatter([foot_x], [foot_y], [foot_z], c='black', s=180, marker='s', alpha=0.4)
        
        ax.set_xlim([-0.2, 0.3])
        ax.set_ylim([-0.1, 0.1])
        ax.set_zlim([-0.3, 0.1])
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'相位: {phase:.2f}', fontsize=10)
    
    plt.tight_layout()
    
    # 保存图片
    output_path = os.path.join(os.getcwd(), 'walk_gait_sequence_3d.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"✅ 3D步态序列图已保存: {output_path}")
    
    plt.close()  # 关闭图形，不显示


def create_animation():
    """创建步态动画"""
    print("\n生成步态动画...")
    
    gait = WalkGait(stride_length=0.05, step_height=0.03, frequency=0.8)
    
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 腿的基座位置
    leg_positions = {
        'right_front': (0.1, -0.04, -0.2),
        'left_front': (0.1, 0.04, -0.2),
        'right_back': (-0.1, -0.04, -0.2),
        'left_back': (-0.1, 0.04, -0.2)
    }
    
    colors = {
        'right_front': 'red',
        'left_front': 'blue',
        'right_back': 'green',
        'left_back': 'orange'
    }
    
    def update(frame):
        ax.clear()
        
        # 更新相位
        gait.global_phase = frame / 100
        trajectories = gait.get_all_foot_trajectories()
        
        # 绘制机器人身体
        body_x = [-0.1, 0.1, 0.1, -0.1, -0.1]
        body_y = [-0.04, -0.04, 0.04, 0.04, -0.04]
        body_z = [0, 0, 0, 0, 0]
        ax.plot(body_x, body_y, body_z, 'b-', linewidth=3)
        
        # 绘制四条腿
        for leg_name, (base_x, base_y, base_z) in leg_positions.items():
            x_offset, z_offset = trajectories[leg_name]
            
            foot_x = base_x + x_offset
            foot_y = base_y
            foot_z = base_z + z_offset
            
            # 计算中间关节点（髋关节和膝关节）
            # 髋关节：在基座和足端的中间偏上
            hip_x = base_x + (foot_x - base_x) * 0.3
            hip_y = base_y
            hip_z = base_z + (foot_z - base_z) * 0.3 + 0.05  # 稍微抬高
            
            # 膝关节：在髋关节和足端的中间
            knee_x = base_x + (foot_x - base_x) * 0.6
            knee_y = base_y
            knee_z = base_z + (foot_z - base_z) * 0.6
            
            # 绘制腿（三段）
            # 第一段：基座 -> 髋关节
            ax.plot([base_x, hip_x], [base_y, hip_y], [base_z, hip_z], 
                   c=colors[leg_name], linewidth=5, solid_capstyle='round')
            
            # 第二段：髋关节 -> 膝关节
            ax.plot([hip_x, knee_x], [hip_y, knee_y], [hip_z, knee_z], 
                   c=colors[leg_name], linewidth=5, solid_capstyle='round')
            
            # 第三段：膝关节 -> 足端
            ax.plot([knee_x, foot_x], [knee_y, foot_y], [knee_z, foot_z], 
                   c=colors[leg_name], linewidth=5, solid_capstyle='round')
            
            # 绘制关节点
            ax.scatter([base_x], [base_y], [base_z], c=colors[leg_name], s=100, marker='o', edgecolors='black', linewidths=1.5)
            ax.scatter([hip_x], [hip_y], [hip_z], c=colors[leg_name], s=120, marker='o', edgecolors='black', linewidths=1.5)
            ax.scatter([knee_x], [knee_y], [knee_z], c=colors[leg_name], s=120, marker='o', edgecolors='black', linewidths=1.5)
            
            # 绘制足端
            marker = 'o' if z_offset > 0.001 else 's'
            size = 150 if z_offset > 0.001 else 250
            ax.scatter([foot_x], [foot_y], [foot_z], c=colors[leg_name], s=size, marker=marker, edgecolors='black', linewidths=2)
        
        ax.set_xlim([-0.2, 0.3])
        ax.set_ylim([-0.1, 0.1])
        ax.set_zlim([-0.3, 0.1])
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'Walk 步态动画\n相位: {gait.global_phase:.2f}', fontsize=14, fontweight='bold')
    
    # 创建动画
    anim = FuncAnimation(fig, update, frames=100, interval=50, blit=False)
    
    # 保存动画
    output_path = os.path.join(os.getcwd(), 'walk_gait_animation.gif')
    print(f"正在保存动画到: {output_path}")
    print("（这可能需要几秒钟...）")
    
    try:
        anim.save(output_path, writer='pillow', fps=20)
        print(f"✅ 动画已保存: {output_path}")
    except Exception as e:
        print(f"⚠️  保存动画失败: {e}")
        print("提示：需要安装 pillow 库: pip install pillow")
    
    plt.close()  # 关闭图形，不显示


def plot_phase_diagram():
    """绘制相位图"""
    print("\n生成相位图...")
    
    gait = WalkGait()
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # 绘制相位图
    phases = np.linspace(0, 1, 100)
    
    for leg_name, offset in gait.phase_offsets.items():
        # 计算该腿的相位曲线
        leg_phases = []
        for phase in phases:
            leg_phase = (phase + offset) % 1.0
            leg_phases.append(leg_phase)
        
        # 绘制
        ax.plot(phases, leg_phases, linewidth=2, label=f'{leg_name} (偏移 {offset*90:.0f}°)')
    
    ax.set_xlabel('全局相位', fontsize=12)
    ax.set_ylabel('各腿相位', fontsize=12)
    ax.set_title('Walk 步态相位图', fontsize=14, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_xlim([0, 1])
    ax.set_ylim([0, 1])
    
    # 标记摆动相和支撑相
    ax.axhline(y=0.5, color='r', linestyle='--', linewidth=2, alpha=0.5, label='摆动相/支撑相分界')
    ax.text(0.05, 0.25, '支撑相\n(着地)', fontsize=12, ha='center')
    ax.text(0.05, 0.75, '摆动相\n(抬腿)', fontsize=12, ha='center')
    
    plt.tight_layout()
    
    # 保存图片
    output_path = os.path.join(os.getcwd(), 'walk_gait_phase_diagram.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"✅ 相位图已保存: {output_path}")
    
    plt.close()  # 关闭图形，不显示


def main():
    """主函数"""
    print("="*60)
    print("Walk 步态可视化程序")
    print("="*60)
    print("\n这个程序生成 Walk 步态的可视化图表和动画")
    
    try:
        # 生成所有可视化
        plot_trajectory_2d()
        plot_phase_diagram()
        plot_gait_sequence_3d()
        create_animation()
        
        print("\n" + "="*60)
        print("🎉 所有可视化已完成！")
        print("="*60)
        print("\n生成的文件:")
        print("1. walk_gait_trajectory_2d.png - 2D轨迹图")
        print("2. walk_gait_phase_diagram.png - 相位图")
        print("3. walk_gait_sequence_3d.png - 3D步态序列图")
        print("4. walk_gait_animation.gif - 步态动画")
        
    except Exception as e:
        print(f"\n❌ 可视化失败: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    # 设置中文字体（使用更可靠的方法）
    setup_chinese_font()
    
    main()
