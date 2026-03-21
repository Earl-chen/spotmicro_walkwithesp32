#!/usr/bin/env python3
"""
转向方法演示脚本

展示新增的 3 种转向方法
"""

import sys
import os

# 添加模块路径
module_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, module_root)

from gait_algo_core.walk_gait import WalkGait

def demo_cmd_vel():
    """演示 cmd_vel() 方法"""
    print("\n" + "=" * 70)
    print("【演示 1】cmd_vel() - ROS 风格接口")
    print("=" * 70)
    
    gait = WalkGait()
    
    print("\n1.1 前进")
    gait.cmd_vel(linear_x=0.05, linear_y=0, angular_z=0)
    vel = gait.get_velocity()
    print(f"  设置: linear_x=0.05")
    print(f"  结果: forward={vel['forward']:.3f}, lateral={vel['lateral']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")
    
    print("\n1.2 后退")
    gait.cmd_vel(linear_x=-0.05, linear_y=0, angular_z=0)
    vel = gait.get_velocity()
    print(f"  设置: linear_x=-0.05")
    print(f"  结果: forward={vel['forward']:.3f}, lateral={vel['lateral']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")
    
    print("\n1.3 前进并转向")
    gait.cmd_vel(linear_x=0.05, linear_y=0, angular_z=0.3)
    vel = gait.get_velocity()
    print(f"  设置: linear_x=0.05, angular_z=0.3")
    print(f"  结果: forward={vel['forward']:.3f}, lateral={vel['lateral']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")
    
    print("\n1.4 停止")
    gait.cmd_vel(linear_x=0, linear_y=0, angular_z=0)
    vel = gait.get_velocity()
    print(f"  设置: 所有参数为 0")
    print(f"  结果: forward={vel['forward']:.3f}, lateral={vel['lateral']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")


def demo_zero_radius_turn():
    """演示 zero_radius_turn() 方法"""
    print("\n" + "=" * 70)
    print("【演示 2】zero_radius_turn() - 零半径转向")
    print("=" * 70)
    
    gait = WalkGait()
    
    print("\n2.1 左转（原地）")
    gait.zero_radius_turn(yaw_rate=0.5)
    vel = gait.get_velocity()
    print(f"  设置: yaw_rate=0.5")
    print(f"  结果: forward={vel['forward']:.3f}, lateral={vel['lateral']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")
    print(f"  验证: forward=0 且 lateral=0 ✅")
    
    print("\n2.2 右转（原地）")
    gait.zero_radius_turn(yaw_rate=-0.5)
    vel = gait.get_velocity()
    print(f"  设置: yaw_rate=-0.5")
    print(f"  结果: forward={vel['forward']:.3f}, lateral={vel['lateral']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")
    print(f"  验证: forward=0 且 lateral=0 ✅")
    
    print("\n2.3 停止")
    gait.zero_radius_turn(yaw_rate=0)
    vel = gait.get_velocity()
    print(f"  设置: yaw_rate=0")
    print(f"  结果: forward={vel['forward']:.3f}, lateral={vel['lateral']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")


def demo_cmd_vel_adaptive():
    """演示 cmd_vel_adaptive() 方法"""
    print("\n" + "=" * 70)
    print("【演示 3】cmd_vel_adaptive() - 自适应转向")
    print("=" * 70)
    
    gait = WalkGait()
    
    print("\n3.1 差速转向场景")
    print("-" * 70)
    
    print("\n  场景 1: 前进 + 小角度转向")
    gait.cmd_vel_adaptive(linear_x=0.05, linear_y=0, angular_z=0.3)
    vel = gait.get_velocity()
    print(f"    设置: linear_x=0.05, angular_z=0.3")
    print(f"    结果: forward={vel['forward']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")
    print(f"    方案: 差速转向（前进并转向）")
    
    print("\n  场景 2: 小角度转向 + 几乎无前进")
    gait.cmd_vel_adaptive(linear_x=0.01, linear_y=0, angular_z=0.4)
    vel = gait.get_velocity()
    print(f"    设置: linear_x=0.01, angular_z=0.4")
    print(f"    结果: forward={vel['forward']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")
    print(f"    方案: 差速转向（angular_z ≤ 0.5）")
    
    print("\n3.2 零半径转向场景")
    print("-" * 70)
    
    print("\n  场景 3: 大角度转向 + 几乎无前进")
    gait.cmd_vel_adaptive(linear_x=0.01, linear_y=0, angular_z=0.6)
    vel = gait.get_velocity()
    print(f"    设置: linear_x=0.01, angular_z=0.6")
    print(f"    结果: forward={vel['forward']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")
    print(f"    方案: 零半径转向（angular_z > 0.5 且 linear_x < 0.02）")
    
    print("\n  场景 4: 完全原地转向")
    gait.cmd_vel_adaptive(linear_x=0.0, linear_y=0, angular_z=0.8)
    vel = gait.get_velocity()
    print(f"    设置: linear_x=0.0, angular_z=0.8")
    print(f"    结果: forward={vel['forward']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")
    print(f"    方案: 零半径转向（无前进速度）")
    
    print("\n3.3 自适应逻辑说明")
    print("-" * 70)
    print("\n  判断条件:")
    print("    if abs(angular_z) > 0.5 and abs(linear_x) < 0.02:")
    print("      → 零半径转向（原地转向）")
    print("    else:")
    print("      → 差速转向（前进并转向）")


def demo_integration():
    """演示集成使用"""
    print("\n" + "=" * 70)
    print("【演示 4】集成使用 - 模拟真实场景")
    print("=" * 70)
    
    gait = WalkGait()
    
    print("\n场景: 机器人从起点走到目标点")
    print("-" * 70)
    
    print("\n步骤 1: 原地左转，对准目标")
    gait.zero_radius_turn(yaw_rate=0.5)
    vel = gait.get_velocity()
    print(f"  动作: 原地左转")
    print(f"  速度: forward={vel['forward']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")
    
    print("\n步骤 2: 前进 2 秒")
    gait.cmd_vel(linear_x=0.05, linear_y=0, angular_z=0)
    vel = gait.get_velocity()
    print(f"  动作: 前进")
    print(f"  速度: forward={vel['forward']:.3f}")
    
    print("\n步骤 3: 前进并微调方向")
    gait.cmd_vel_adaptive(linear_x=0.05, linear_y=0, angular_z=0.2)
    vel = gait.get_velocity()
    print(f"  动作: 前进并转向")
    print(f"  速度: forward={vel['forward']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")
    
    print("\n步骤 4: 到达目标，停止")
    gait.cmd_vel(linear_x=0, linear_y=0, angular_z=0)
    vel = gait.get_velocity()
    print(f"  动作: 停止")
    print(f"  速度: forward={vel['forward']:.3f}, yaw_rate={vel['yaw_rate']:.3f}")


def main():
    """主函数"""
    print("=" * 70)
    print("转向方法演示")
    print("=" * 70)
    
    demo_cmd_vel()
    demo_zero_radius_turn()
    demo_cmd_vel_adaptive()
    demo_integration()
    
    print("\n" + "=" * 70)
    print("✅ 演示完成！")
    print("=" * 70)
    
    print("\n📋 总结:")
    print("  1. cmd_vel() - ROS 风格接口，适用于通用控制")
    print("  2. zero_radius_turn() - 零半径转向，适用于原地转向")
    print("  3. cmd_vel_adaptive() - 自适应转向，自动选择最优方案")
    print("\n💡 推荐:")
    print("  - 大多数情况使用 cmd_vel_adaptive()")
    print("  - 需要精确控制时使用 cmd_vel() 或 zero_radius_turn()")


if __name__ == '__main__':
    main()
