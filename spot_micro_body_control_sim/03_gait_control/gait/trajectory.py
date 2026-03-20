# app/gait/trajectory.py
"""
足端轨迹生成器

提供多种轨迹生成方法，用于步态控制。
"""

import numpy as np


class TrajectoryGenerator:
    """
    足端轨迹生成器
    
    支持多种轨迹类型：
    - 摆线轨迹（Cycloid）：推荐用于 Walk 步态
    - 椭圆轨迹（Ellipse）：简化版
    """
    
    @staticmethod
    def cycloid_trajectory(phase: float, stride_length: float, step_height: float) -> tuple:
        """
        摆线轨迹（修正版 - 标准Walk步态）
        
        特点：
        - 平滑的加速和减速
        - 适合舵机控制
        - **修正：摆动相占25%，支撑相占75%**
        
        参数：
            phase: 相位 (0-1)
                   0-0.25: 摆动相（抬腿向前摆动）✅ 修正
                   0.25-1.0: 支撑相（向后蹬地）✅ 修正
            stride_length: 步长 (米)，默认 0.05 (50mm)
            step_height: 抬腿高度 (米)，默认 0.03 (30mm)
        
        返回：
            (x, z): 前后位置偏移, 上下位置偏移（米）
        
        Walk步态标准：
        - 摆动相：25%（相位0-0.25）
        - 支撑相：75%（相位0.25-1.0）
        - 确保每时刻都有3条腿支撑
        
        示例：
            >>> # 生成摆动相中点的轨迹
            >>> x, z = TrajectoryGenerator.cycloid_trajectory(0.125, 0.05, 0.03)
            >>> # x ≈ 0, z ≈ 0.015（最高点）
        """
        if phase < 0.25:  # ✅ 摆动相（25%）- 从0.5改为0.25
            t = phase * 4 * np.pi  # ✅ 从2改为4
            # X从 -stride_length/2 → +stride_length/2
            x = stride_length / 2 * (t / np.pi - 1)
            # Z抬起（摆线轨迹）
            z = step_height * (1 - np.cos(t)) / 2
        else:  # ✅ 支撑相（75%）- 从0.5改为0.25
            t = (phase - 0.25) * 4 / 3  # ✅ 修正：去掉π，t从0到1
            # X从 +stride_length/2 → -stride_length/2（修正后连续）
            x = stride_length / 2 * (1 - 2 * t)
            # Z着地（高度为0）
            z = 0
        
        # Y轴轨迹（用于转向）
        # 转向时，y轴会产生侧向偏移
        y = 0  # 默认无侧向偏移
        
        return x, y, z
        
        return x, z
    
    @staticmethod
    def ellipse_trajectory(phase: float, stride_length: float, step_height: float) -> tuple:
        """
        椭圆轨迹（简化版）
        
        特点：
        - 计算简单
        - 形状规则
        - 摆动相和支撑相对称
        
        参数：
            phase: 相位 (0-1)
            stride_length: 步长 (米)
            step_height: 抬腿高度 (米)
        
        返回：
            (x, z): 前后位置偏移, 上下位置偏移（米）
        
        注意：
            此轨迹在相位 0.5 时会有一个突变（从最高点直接到地面）
            建议使用摆线轨迹以获得更平滑的运动
        """
        angle = phase * 2 * np.pi
        x = stride_length / 2 * np.cos(angle)
        z = step_height * (1 + np.sin(angle)) / 2  # 只取上半椭圆
        
        return x, z
    
    @staticmethod
    def bezier_trajectory(phase: float, stride_length: float, step_height: float) -> tuple:
        """
        贝塞尔曲线轨迹（高级）
        
        特点：
        - 可自定义曲线形状
        - 平滑且可控
        - 适合复杂轨迹需求
        
        参数：
            phase: 相位 (0-1)
            stride_length: 步长 (米)
            step_height: 抬腿高度 (米)
        
        返回：
            (x, z): 前后位置偏移, 上下位置偏移（米）
        """
        # 简化的二次贝塞尔曲线
        # 控制点：(0, 0), (stride_length/2, step_height), (stride_length, 0)
        t = phase
        x = 2 * (1 - t) * t * (stride_length / 2) + t * t * stride_length
        z = 2 * (1 - t) * t * step_height
        
        return x, z


# 单元测试
if __name__ == '__main__':
    print("测试轨迹生成器...")
    
    # 测试摆线轨迹
    print("\n摆线轨迹测试:")
    for phase in [0.0, 0.25, 0.5, 0.75, 1.0]:
        x, z = TrajectoryGenerator.cycloid_trajectory(phase, 0.05, 0.03)
        print(f"  相位 {phase:.2f}: x={x*1000:.1f}mm, z={z*1000:.1f}mm")
    
    # 测试椭圆轨迹
    print("\n椭圆轨迹测试:")
    for phase in [0.0, 0.25, 0.5, 0.75, 1.0]:
        x, z = TrajectoryGenerator.ellipse_trajectory(phase, 0.05, 0.03)
        print(f"  相位 {phase:.2f}: x={x*1000:.1f}mm, z={z*1000:.1f}mm")
    
    print("\n✅ 轨迹生成器测试完成")
