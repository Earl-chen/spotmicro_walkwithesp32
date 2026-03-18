#!/usr/bin/env python3
"""
四足机器人运动学实验室 - 主程序入口
==================================

本程序是 Spot Micro 运动学实验室的主入口，提供：
- 交互式菜单系统
- 基础功能测试
- 可视化工具启动
- 批量计算演示

系统架构
--------
┌─────────────────────────────────────────────────────────────┐
│                      run_lab.py (本程序)                    │
│                    主程序入口 + 菜单系统                      │
└─────────────────────────────────────────────────────────────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
        ▼                  ▼                  ▼
┌───────────────┐  ┌───────────────┐  ┌───────────────┐
│ kinematics_   │  │ kinematics_   │  │ kinematics_   │
│ visualizer.py │  │   app.py      │  │   core.py     │
│  (可视化层)    │  │  (应用层)      │  │  (核心层)      │
└───────────────┘  └───────────────┘  └───────────────┘

运行方式
--------
1. 交互式模式（默认）：
   python run_lab.py

2. 快速演示模式：
   python run_lab.py --demo

功能菜单
--------
1. 基础功能测试         - IK/FK 精度验证
2. 左腿交互式可视化     - 实时角度↔坐标转换
3. 右腿交互式可视化     - 实时角度↔坐标转换
4. 双腿对比可视化       - 左右腿并排显示
5. 四足机器人控制器测试 - 站立姿态/步态测试
6. 批量计算演示         - 轨迹规划演示
7. 退出

依赖
----
- kinematics_app: 应用层控制器
- kinematics_visualizer: 可视化工具
- matplotlib: 3D 绘图（GUI 后端）

注意事项
--------
- 需要 GUI 环境（TkAgg 或 Qt5Agg）
- 无头服务器上只能运行基础测试，无法使用可视化
- 按 Ctrl+C 可随时退出
"""

import sys
import os
from kinematics_app import create_leg_controller, QuadrupedController
from kinematics_visualizer import create_visualizer


# ============================================================================
# 主程序
# ============================================================================

def main():
    """
    主程序入口

    显示欢迎信息和功能菜单，处理用户输入。
    使用 while 循环提供持续的交互体验。
    """
    # ====================================================================
    # 欢迎信息
    # ====================================================================
    print("=" * 60)
    print("Spot Micro 腿部运动学实验室")
    print("=" * 60)
    print("功能特点:")
    print("• 单腿正/逆运动学计算")
    print("• 批量计算支持（轨迹规划）")
    print("• 工作空间分析")
    print("• 交互式 3D 可视化")
    print("=" * 60)

    # ====================================================================
    # 初始化系统
    # ====================================================================
    print("\n初始化系统...")

    # 创建腿部控制器
    # 'spot_micro': 使用 Spot Micro 机器人的参数配置
    # verbose=True: 输出详细日志
    leg_controller = create_leg_controller('spot_micro', verbose=True)

    # ====================================================================
    # 功能菜单
    # ====================================================================
    options = [
        "基础功能测试",
        "左腿交互式可视化 (推荐)",
        "右腿交互式可视化",
        "双腿对比可视化",
        "四足机器人控制器测试",
        "批量计算演示",
        "退出"
    ]

    # ====================================================================
    # 主循环
    # ====================================================================
    while True:
        # 显示菜单
        print(f"\n{'='*40}")
        print("功能菜单:")
        for i, option in enumerate(options, 1):  # 从 1 开始编号
            print(f"{i}. {option}")
        print("="*40)

        try:
            # 获取用户输入
            choice = input("请选择功能 (1-7): ").strip()

            # 分发到对应的功能函数
            if choice == '1':
                test_basic_functionality(leg_controller)

            elif choice == '2':
                test_interactive_visualizer(leg_controller, 'left')

            elif choice == '3':
                test_interactive_visualizer(leg_controller, 'right')

            elif choice == '4':
                test_dual_visualizer(leg_controller)

            elif choice == '5':
                test_quadruped_controller()

            elif choice == '6':
                test_batch_calculation(leg_controller)

            elif choice == '7':
                print("感谢使用！")
                break  # 退出循环

            else:
                print("请输入 1-7")

        except KeyboardInterrupt:
            # 用户按 Ctrl+C
            print("\n\n程序被用户中断")
            break

        except Exception as e:
            # 捕获其他异常，防止程序崩溃
            print(f"发生错误: {e}")


# ============================================================================
# 功能测试函数
# ============================================================================

def test_basic_functionality(leg_controller):
    """
    基础功能测试

    测试运动学系统的核心功能：
    1. 单点逆运动学解算
    2. IK-FK 往返精度验证
    3. 预设位置测试套件

    参数
    ----
    leg_controller : LegController
        腿部控制器实例
    """
    print("\n" + "="*50)
    print("基础功能测试")
    print("="*50)

    # ====================================================================
    # 测试 1：单点逆运动学解算
    # ====================================================================
    print("\n1. 单点逆运动学解算:")

    # 测试点：向前伸展的位置
    x, y, z = 50, 60.5, -180

    # 执行逆运动学：位置 → 角度
    result = leg_controller.solve_position(x, y, z, 'left')

    if result:
        # 解算成功
        print(f"成功 ✓ 角度: {[f'{a:.1f}°' for a in result]}")

        # 验证正运动学：角度 → 位置
        # 这是 IK-FK 往返测试，确保往返误差在可接受范围内
        calc_pos = leg_controller.calculate_position(*result, 'left')

        # 计算往返误差（欧几里得距离）
        error = ((x-calc_pos[0])**2 + (y-calc_pos[1])**2 + (z-calc_pos[2])**2)**0.5
        print(f"验证误差: {error:.3f}mm {'✓' if error < 1.0 else '✗'}")
    else:
        # 解算失败
        print("失败 ✗")

    # ====================================================================
    # 测试 2：测试套件
    # ====================================================================
    print("\n2. 测试套件:")

    # 运行控制器内置的测试套件
    # 包含多个预设位置的测试
    leg_controller.test_basic_positions()


def test_interactive_visualizer(leg_controller, leg_side):
    """
    交互式可视化测试

    启动指定腿侧的交互式可视化工具，支持：
    - 实时角度↔坐标转换
    - 3D 腿部渲染
    - 手动输入测试

    参数
    ----
    leg_controller : LegController
        腿部控制器实例
    leg_side : str
        'left' 或 'right'，指定要显示的腿侧
    """
    print(f"\n启动{leg_side}腿交互式可视化工具...")
    print("提示: 将打开新窗口，支持实时角度-坐标转换")

    try:
        # 创建交互式可视化器
        visualizer = create_visualizer(leg_controller, 'interactive', leg_side)

        # 启动交互窗口（阻塞调用）
        visualizer.create_interactive_window()

    except Exception as e:
        # 可视化启动失败（常见原因：无 GUI 环境）
        print(f"可视化工具启动失败: {e}")
        print("可能的解决方案:")
        print("1. 检查 matplotlib 后端配置")
        print("2. 确保 GUI 环境正常")


def test_dual_visualizer(leg_controller):
    """
    双腿对比可视化测试

    生成左右腿的对比图，用于验证对称性或对比不同姿态。
    图片会保存到文件并在窗口中显示。

    参数
    ----
    leg_controller : LegController
        腿部控制器实例
    """
    print("\n启动双腿对比可视化...")

    try:
        # 创建双腿可视化器
        visualizer = create_visualizer(leg_controller, 'dual')

        # 定义测试姿态：对称姿态
        # 左腿和右腿的髋侧摆角度相反，其他角度相同
        left_angles = (15, -20, -60)   # 左腿：髋侧摆向外 15°
        right_angles = (-15, -20, -60)  # 右腿：髋侧摆向外 15°（角度为负）

        # 生成对比图
        fig = visualizer.create_comparison_plot(
            left_angles, right_angles,
            save_path='dual_leg_comparison.png'  # 保存到文件
        )

        # 显示图像窗口
        import matplotlib.pyplot as plt
        plt.show()

    except Exception as e:
        print(f"双腿可视化失败: {e}")


def test_quadruped_controller():
    """
    四足机器人控制器测试

    测试四条腿的协调运动：
    1. 默认站立姿态计算
    2. 对称步态测试

    使用独立的 QuadrupedController 实例。
    """
    print("\n" + "="*50)
    print("四足机器人控制器测试")
    print("="*50)

    try:
        # 创建四足控制器
        controller = QuadrupedController(verbose=True)

        # 测试默认站立姿态
        # 计算四条腿在默认站立位置时的关节角度
        stance_angles = controller.get_default_stance()

        # 测试对称步态（对角支撑相位）
        controller.test_symmetric_gait()

    except Exception as e:
        print(f"四足控制器测试失败: {e}")


def test_batch_calculation(leg_controller):
    """
    批量计算演示

    演示轨迹规划场景中的批量 IK/FK 计算：
    1. 生成圆形轨迹（16 个点）
    2. 批量逆运动学解算
    3. 批量正运动学验证
    4. 误差分析

    参数
    ----
    leg_controller : LegController
        腿部控制器实例
    """
    print("\n" + "="*50)
    print("批量计算演示")
    print("="*50)

    # ====================================================================
    # 生成测试轨迹
    # ====================================================================
    import math
    import numpy as np

    print("生成圆形轨迹...")

    # 轨迹参数
    center = (0, 60.5, -200)  # 圆心坐标 (x, y, z)
    radius = 30               # 半径 30mm
    num_points = 16           # 轨迹点数（16 个均匀分布的点）

    # 生成圆形轨迹点
    # 在 XZ 平面上生成圆形（Y 坐标固定）
    trajectory = []
    for i in range(num_points):
        # 计算当前点的角度（0 到 2π）
        angle = 2 * math.pi * i / num_points

        # 参数方程：x = r*cos(θ), z = r*sin(θ)
        x = center[0] + radius * math.cos(angle)
        y = center[1]  # Y 坐标不变
        z = center[2] + radius * math.sin(angle)

        trajectory.append((x, y, z))

    print(f"轨迹点数: {len(trajectory)}")

    # ====================================================================
    # 批量逆运动学解算
    # ====================================================================
    print("\n批量逆运动学解算...")

    # 调用核心层的批量 IK 函数
    # 返回：每个位置对应的角度或 None（不可达）
    results = leg_controller.kinematics.batch_inverse_kinematics(
        trajectory, is_left=True
    )

    # 统计成功率
    success_count = sum(1 for r in results if r is not None)
    print(f"解算成功: {success_count}/{len(trajectory)}")

    # ====================================================================
    # 显示部分结果
    # ====================================================================
    if success_count > 0:
        print("\n前5个成功解算的结果:")

        count = 0
        for i, (pos, angles) in enumerate(zip(trajectory, results)):
            if angles is not None and count < 5:
                # 将弧度转换为度数（便于阅读）
                angles_deg = [math.degrees(a) for a in angles]
                print(f"位置 {i+1}: {pos} → 角度: {[f'{a:.1f}°' for a in angles_deg]}")
                count += 1

        # ====================================================================
        # 批量正运动学验证
        # ====================================================================
        print("\n批量正运动学验证...")

        # 筛选出成功解算的角度
        valid_angles = [r for r in results if r is not None]

        if valid_angles:
            # 批量 FK：验证前 5 个成功解算
            verify_positions = leg_controller.kinematics.batch_forward_kinematics(
                valid_angles[:5], is_left=True
            )

            # 显示验证结果
            print("验证结果 (原位置 → 计算位置 → 误差):")
            for i, (orig, calc) in enumerate(zip(trajectory[:5], verify_positions)):
                if results[i] is not None:
                    # 计算误差
                    error = math.sqrt(sum((o-c)**2 for o, c in zip(orig, calc)))
                    print(f"  {orig} → {calc} → {error:.3f}mm")


# ============================================================================
# 快速演示模式
# ============================================================================

def run_quick_demo():
    """
    快速演示模式

    无需用户交互，快速运行几个关键位置的测试。
    适用于：
    - CI/CD 自动化测试
    - 快速验证系统功能
    - 无头环境测试

    使用方法
    --------
    python run_lab.py --demo
    """
    print("运行快速演示...")

    # 创建控制器（静默模式，不输出详细日志）
    controller = create_leg_controller('spot_micro', verbose=False)

    # 定义测试位置：(x, y, z, 描述)
    test_positions = [
        (0, 60.5, -200, "垂直下方"),
        (80, 60.5, -160, "向前伸展"),
        (-40, 60.5, -220, "向后收缩")
    ]

    # 输出表头
    print("\n快速测试结果:")
    print("-" * 40)

    # 逐个测试
    for x, y, z, desc in test_positions:
        result = controller.solve_position(x, y, z, 'left')
        status = "✓" if result else "✗"
        print(f"{desc:12} | {status} | 目标: ({x:3}, {y:5.1f}, {z:4})")

    # 输出表尾
    print("-" * 40)
    print("快速演示完成")


# ============================================================================
# 程序入口
# ============================================================================

if __name__ == "__main__":
    # 检查命令行参数
    if len(sys.argv) > 1 and sys.argv[1] == '--demo':
        # 快速演示模式
        run_quick_demo()
    else:
        # 交互式模式（默认）
        main()
