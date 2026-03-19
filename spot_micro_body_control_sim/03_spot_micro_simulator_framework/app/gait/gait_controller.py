# app/gait/gait_controller.py
"""
步态控制器 - 集成到现有系统

提供步态控制的高级接口，集成到现有的 Controller 和 RobotModel。

使用示例：
    >>> from app.controller import Controller
    >>> from app.gait import GaitController
    >>> 
    >>> # 构建系统
    >>> model, controller = build_system()
    >>> 
    >>> # 创建步态控制器
    >>> gait_ctrl = GaitController(controller)
    >>> 
    >>> # 启用步态
    >>> gait_ctrl.enable_gait()
    >>> 
    >>> # 在主循环中更新
    >>> while True:
    ...     gait_ctrl.update()
    ...     time.sleep(0.02)
"""

import time
import numpy as np
from typing import Dict, Optional
from app.controller import Controller
from app.robot_model import RobotModel
from core.types import LegJoints
from .walk_gait import WalkGait


class GaitController:
    """
    步态控制器
    
    功能：
    - 管理 Walk 步态生成
    - 集成到现有 Controller
    - 提供步态控制接口
    - 保持与现有系统的兼容性
    
    特点：
    - 不修改现有 Controller 和 Model
    - 通过组合方式集成
    - 可以随时启用/禁用步态
    - 支持实时参数调整
    """
    
    def __init__(self, controller: Controller, verbose: bool = False):
        """
        初始化步态控制器
        
        参数：
            controller: 现有 Controller 实例
            verbose: 是否输出详细日志
        """
        self.controller = controller
        self.model = controller.model
        self.verbose = verbose
        
        # 创建 Walk 步态实例
        self.walk_gait = WalkGait()
        
        # 步态状态
        self.gait_enabled = False
        self.last_update_time: Optional[float] = None
        
        # 保存初始脚部位置（用于步态）
        self.initial_foot_positions: Optional[Dict[str, np.ndarray]] = None
        
        # 统计信息
        self.update_count = 0
        self.ik_success_count = 0
        self.ik_failure_count = 0
        
    def enable_gait(self):
        """
        启用步态控制
        
        注意：会禁用脚步锁定模式
        
        示例：
            >>> gait_ctrl.enable_gait()
        """
        # 捕获当前脚部位置作为初始位置
        self.initial_foot_positions = {}
        foot_positions = self.controller.get_foot_positions_world()
        
        for leg_name, pos in foot_positions.items():
            self.initial_foot_positions[leg_name] = np.array(pos)
        
        # 禁用脚步锁定（因为我们要主动控制脚部位置）
        self.controller.enable_forward_kinematics_mode()
        
        # 启用步态
        self.gait_enabled = True
        self.last_update_time = time.time()
        self.update_count = 0
        self.ik_success_count = 0
        self.ik_failure_count = 0
        
        if self.verbose:
            print("✅ 步态控制已启用")
            print(f"  初始脚部位置已捕获: {len(self.initial_foot_positions)} 条腿")
    
    def disable_gait(self):
        """
        禁用步态控制
        
        示例：
            >>> gait_ctrl.disable_gait()
        """
        self.gait_enabled = False
        
        if self.verbose:
            print("⏸️  步态控制已禁用")
            print(f"  统计: 更新次数={self.update_count}, "
                  f"IK成功={self.ik_success_count}, IK失败={self.ik_failure_count}")
    
    def update(self) -> bool:
        """
        更新步态（主循环调用）
        
        返回：
            bool: 是否成功更新
        
        示例：
            >>> while True:
            ...     if gait_ctrl.update():
            ...         # 步态已更新
            ...         pass
            ...     time.sleep(0.02)
        """
        if not self.gait_enabled:
            return False
        
        # 计算时间步长
        current_time = time.time()
        if self.last_update_time is None:
            dt = 0.02  # 默认 20ms
        else:
            dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # 限制最大时间步长（防止相位跳跃）
        if dt > 0.1:
            dt = 0.1
        
        # 更新步态相位
        self.walk_gait.update(dt)
        
        # 获取所有腿的轨迹
        trajectories = self.walk_gait.get_all_foot_trajectories()
        
        # 应用轨迹到每条腿
        success = True
        for leg_name, (x_offset, z_offset) in trajectories.items():
            # 获取初始脚部位置
            initial_pos = self.initial_foot_positions[leg_name]
            
            # 计算新的脚部位置（相对于初始位置）
            # 注意：这里假设机器人向前走（X方向）
            # 需要根据机器人朝向调整
            new_foot_pos = np.array([
                initial_pos[0] + x_offset,  # X方向偏移
                initial_pos[1],              # Y不变
                initial_pos[2] + z_offset    # Z方向偏移
            ])
            
            # 使用逆运动学计算关节角度
            ik_success = self._apply_ik_to_leg(leg_name, new_foot_pos)
            
            if not ik_success:
                success = False
        
        # 更新统计
        self.update_count += 1
        if success:
            self.ik_success_count += 1
        else:
            self.ik_failure_count += 1
        
        return success
    
    def _apply_ik_to_leg(self, leg_name: str, foot_pos_world: np.ndarray) -> bool:
        """
        应用逆运动学到指定腿
        
        参数：
            leg_name: 腿名称
            foot_pos_world: 脚部世界坐标 (3,)
        
        返回：
            bool: 是否成功
        """
        try:
            # 获取腿的运动学求解器和髋坐标系
            legkin, hip_frame = self.model.legs[leg_name]
            
            # 将脚部世界坐标转换到髋坐标系
            foot_in_hip = self.model.frame_manager.transform_point(
                foot_pos_world, 
                "world", 
                hip_frame
            )
            
            # 逆运动学计算
            ikr = legkin.inverse(foot_in_hip)
            
            if ikr.success and ikr.joints is not None:
                # 更新关节角度
                self.model.update_joint_angles(leg_name, ikr.joints)
                return True
            else:
                if self.verbose and self.update_count % 50 == 0:  # 每50次输出一次
                    print(f"⚠️  {leg_name} IK失败: {ikr.reason}")
                return False
                
        except Exception as e:
            if self.verbose and self.update_count % 50 == 0:
                print(f"❌ {leg_name} IK异常: {e}")
            return False
    
    def set_gait_parameters(self, 
                            stride_length: float = None,
                            step_height: float = None,
                            frequency: float = None):
        """
        设置步态参数
        
        参数：
            stride_length: 步长 (米)
            step_height: 抬腿高度 (米)
            frequency: 步频 (Hz)
        
        示例：
            >>> gait_ctrl.set_gait_parameters(stride_length=0.06, frequency=1.0)
        """
        self.walk_gait.set_parameters(stride_length, step_height, frequency)
        
        if self.verbose:
            state = self.walk_gait.get_state()
            print(f"🔧 步态参数已更新:")
            print(f"  步长: {state['stride_length']*1000:.1f}mm")
            print(f"  步高: {state['step_height']*1000:.1f}mm")
            print(f"  步频: {state['frequency']:.2f}Hz")
    
    def set_trajectory_type(self, trajectory_type: str):
        """
        设置轨迹类型
        
        参数：
            trajectory_type: 'cycloid', 'ellipse', 'bezier'
        
        示例：
            >>> gait_ctrl.set_trajectory_type('ellipse')
        """
        self.walk_gait.set_trajectory_type(trajectory_type)
        
        if self.verbose:
            print(f"🔧 轨迹类型已更改: {trajectory_type}")
    
    def get_gait_status(self) -> Dict:
        """
        获取步态状态
        
        返回：
            dict: 步态状态信息
        
        示例：
            >>> status = gait_ctrl.get_gait_status()
            >>> print(f"步态启用: {status['enabled']}")
        """
        gait_state = self.walk_gait.get_state()
        
        return {
            'enabled': self.gait_enabled,
            'update_count': self.update_count,
            'ik_success_count': self.ik_success_count,
            'ik_failure_count': self.ik_failure_count,
            'ik_success_rate': (self.ik_success_count / max(self.update_count, 1)) * 100,
            **gait_state
        }
    
    def reset_gait(self):
        """重置步态（相位归零）"""
        self.walk_gait.reset()
        
        if self.verbose:
            print("🔄 步态已重置")


# 单元测试
if __name__ == '__main__':
    print("测试步态控制器...")
    
    # 这里需要构建完整的系统才能测试
    # 建议在集成测试中进行
    
    print("✅ 步态控制器模块已创建")
    print("提示：在 run_spot_micro.py 中集成测试")
