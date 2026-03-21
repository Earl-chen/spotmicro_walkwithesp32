#!/usr/bin/env python3
"""
转向功能可视化脚本

生成转向轨迹的对比图和动画
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
from matplotlib import font_manager as fm

# =============================================================================
# 中文字体配置
# =============================================================================
_FONT_FILE_RELATIVE = os.path.join('..', '..', '..', 'fonts', 'BabelStoneHan.ttf')

def _get_font_path():
    """获取字体文件的绝对路径"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    font_file = os.path.join(script_dir, _FONT_FILE_RELATIVE)
    return os.path.normpath(font_file)

def setup_chinese_font():
    """配置中文字体"""
    font_file = _get_font_path()
    
    if os.path.exists(font_file):
        try:
            # 关键：显式添加字体到 matplotlib 的 fontManager 缓存
            fm.fontManager.addfont(font_file)
            
            # 创建 FontProperties
            font_prop = fm.FontProperties(fname=font_file)
            
            # 设置全局字体
            plt.rcParams['font.family'] = font_prop.get_name()
            plt.rcParams['axes.unicode_minus'] = False
            
            return font_prop
        except Exception as e:
            print(f"⚠️ 字体加载失败: {e}")
    
    return fm.FontProperties()

# 配置中文字体
chinese_font = setup_chinese_font()
print(f"✅ 已加载中文字体: {chinese_font.get_name()}")

module_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, module_root)
from gait_algo_core.walk_gait import WalkGait

def generate_steering_comparison():
    """生成转向对比图"""
    print("生成转向对比图...")
    
    gait = WalkGait(stride_length=0.05, step_height=0.03, frequency=0.8)
    dt = 0.02
    steps_per_cycle = int(1.0 / (gait.frequency * dt))
    
    # 收集三种转向模式的数据
    # 修正：使用 set_steering() 替代 set_direction()
    modes = {
        '直行 (0°)': 0.0,
        '左转 (+30°)': np.pi/6,
        '右转 (-30°)': -np.pi/6
    }
    
    all_trajectories = {}
    
    for mode_name, angle in modes.items():
        gait.reset()
        gait.set_steering(angle)  # ✅ 修正：使用 set_steering()
        
        trajectories = {
            'left_front': {'x': [], 'y': [], 'z': []},
            'right_front': {'x': [], 'y': [], 'z': []},
            'left_back': {'x': [], 'y': [], 'z': []},
            'right_back': {'x': [], 'y': [], 'z': []}
        }
        
        for i in range(steps_per_cycle):
            gait.update(dt)
            
            for leg_name in trajectories.keys():
                x, z = gait.get_foot_trajectory(leg_name)  # ✅ 修正：返回 tuple (x, z)
                trajectories[leg_name]['x'].append(x * 1000)  # 转换为mm
                trajectories[leg_name]['y'].append(0.0)  # ✅ 修正：Y 轴偏移为 0
                trajectories[leg_name]['z'].append(z * 1000)
        
        all_trajectories[mode_name] = trajectories
    
    # 绘制对比图
    fig, axes = plt.subplots(3, 3, figsize=(15, 12))
    fig.suptitle('Walk步态转向功能对比', fontsize=16, fontweight='bold', fontproperties=chinese_font)
    
    leg_colors = {
        'left_front': '#FF6B6B',
        'right_front': '#4ECDC4',
        'left_back': '#45B7D1',
        'right_back': '#FFA07A'
    }
    
    for idx, (mode_name, trajectories) in enumerate(all_trajectories.items()):
        # XY平面（俯视图）
        ax_xy = axes[idx, 0]
        for leg_name, data in trajectories.items():
            ax_xy.plot(data['x'], data['y'], 
                      color=leg_colors[leg_name], 
                      label=leg_name, 
                      linewidth=2)
        ax_xy.set_xlabel('X (mm)', fontproperties=chinese_font)
        ax_xy.set_ylabel('Y (mm)', fontproperties=chinese_font)
        ax_xy.set_title(f'{mode_name}\nXY平面（俯视）', fontproperties=chinese_font)
        ax_xy.grid(True, alpha=0.3)
        ax_xy.axis('equal')
        ax_xy.legend(fontsize=8, prop=chinese_font)
        
        # XZ平面（侧视图）
        ax_xz = axes[idx, 1]
        for leg_name, data in trajectories.items():
            ax_xz.plot(data['x'], data['z'], 
                      color=leg_colors[leg_name], 
                      linewidth=2)
        ax_xz.set_xlabel('X (mm)', fontproperties=chinese_font)
        ax_xz.set_ylabel('Z (mm)', fontproperties=chinese_font)
        ax_xz.set_title(f'{mode_name}\nXZ平面（侧视）', fontproperties=chinese_font)
        ax_xz.grid(True, alpha=0.3)
        
        # YZ平面（正视图）
        ax_yz = axes[idx, 2]
        for leg_name, data in trajectories.items():
            ax_yz.plot(data['y'], data['z'], 
                      color=leg_colors[leg_name], 
                      linewidth=2)
        ax_yz.set_xlabel('Y (mm)', fontproperties=chinese_font)
        ax_yz.set_ylabel('Z (mm)', fontproperties=chinese_font)
        ax_yz.set_title(f'{mode_name}\nYZ平面（正视）', fontproperties=chinese_font)
        ax_yz.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_file = os.path.join(os.getcwd(), 'steering_comparison.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"✅ 已保存: {output_file}")
    plt.close()
    
    return output_file


def generate_steering_animation():
    """生成转向动画"""
    print("生成转向动画...")
    
    gait = WalkGait(stride_length=0.05, step_height=0.03, frequency=0.8)
    gait.set_steering(np.pi/6)  # ✅ 修正：左转30度
    
    dt = 0.02
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # 机器人身体（简化为矩形）
    body_width = 78  # mm
    body_length = 207.5  # mm
    
    # 腿部初始位置（相对于身体中心）
    leg_positions = {
        'left_front': (body_length/2, body_width/2),
        'right_front': (body_length/2, -body_width/2),
        'left_back': (-body_length/2, body_width/2),
        'right_back': (-body_length/2, -body_width/2)
    }
    
    leg_colors = {
        'left_front': '#FF6B6B',
        'right_front': '#4ECDC4',
        'left_back': '#45B7D1',
        'right_back': '#FFA07A'
    }
    
    # 初始化绘图元素
    body = plt.Rectangle((-body_length/2, -body_width/2), 
                        body_length, body_width, 
                        fill=False, edgecolor='black', linewidth=2)
    ax.add_patch(body)
    
    leg_points = {}
    for leg_name, (x, y) in leg_positions.items():
        point, = ax.plot(x, y, 'o', color=leg_colors[leg_name], 
                        markersize=10, label=leg_name)
        leg_points[leg_name] = point
    
    ax.set_xlim(-150, 150)
    ax.set_ylim(-100, 100)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', prop=chinese_font)
    ax.set_xlabel('X (mm)', fontproperties=chinese_font)
    ax.set_ylabel('Y (mm)', fontproperties=chinese_font)
    ax.set_title('Walk步态转向动画（左转30°）', fontproperties=chinese_font)
    
    def update(frame):
        gait.update(dt)
        
        for leg_name, (base_x, base_y) in leg_positions.items():
            x_offset, z_offset = gait.get_foot_trajectory(leg_name)  # ✅ 修正：返回 (x, z) 元组
            # 更新脚部位置（基准位置 + 轨迹偏移）
            new_x = base_x + x_offset * 1000
            new_y = base_y  # ✅ 修正：Y 轴偏移为 0
            leg_points[leg_name].set_data([new_x], [new_y])
        
        return list(leg_points.values())
    
    anim = FuncAnimation(fig, update, frames=100, 
                        interval=20, blit=True)
    
    output_file = os.path.join(os.getcwd(), 'steering_animation.gif')
    anim.save(output_file, writer='pillow', fps=50)
    print(f"✅ 已保存: {output_file}")
    plt.close()
    
    return output_file


def main():
    """主函数"""
    print("="*60)
    print("转向功能可视化")
    print("="*60)
    
    # 确保输出目录存在
    os.makedirs('tests/visual', exist_ok=True)
    
    # 生成对比图
    comparison_file = generate_steering_comparison()
    
    # 生成动画
    animation_file = generate_steering_animation()
    
    print("\n" + "="*60)
    print("✅ 可视化生成完成！")
    print("="*60)
    print(f"对比图: {comparison_file}")
    print(f"动画:   {animation_file}")


if __name__ == '__main__':
    main()
