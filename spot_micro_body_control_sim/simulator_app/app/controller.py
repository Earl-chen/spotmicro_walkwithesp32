# app/controller.py
"""
四足机器人控制器

负责协调机器人模型和用户控制，提供：
- 双模式控制（逆运动学/正向运动学）
- 脚步锁定功能
- 原子操作保证状态一致性
"""
from typing import Dict
import numpy as np
from app.robot_model import RobotModel
from core.types import LegJoints
import math


class Controller:
    """
    四足机器人控制器

    参数：
        model: RobotModel 实例
        verbose: 是否输出详细日志，默认 False
    """

    def __init__(self, model: RobotModel, verbose: bool = False):
        self.model = model
        self.verbose = verbose
        self.fixed_foot_world: Dict[str, np.ndarray] = {}
        self.foot_lock_enabled = False

    def capture_fixed_feet(self):
        """捕获当前脚部世界坐标位置，用于脚步锁定"""
        self.fixed_foot_world.clear()
        for leg_name in self.model.legs.keys():
            foot_world = self.model.get_leg_joints_world(leg_name)[-1].copy()
            self.fixed_foot_world[leg_name] = foot_world
        self.foot_lock_enabled = True

        if self.verbose:
            print(f"脚步锁定已启用，固定了 {len(self.fixed_foot_world)} 只脚的世界坐标")

    def set_body_pose(self, x, y, z, roll, pitch, yaw, *, radians=True):
        """
        设置机体位姿，如果启用了脚步锁定，会自动通过逆运动学保持脚步位置不变

        使用原子操作：如果任何一条腿的 IK 失败，将回滚机体位姿，保持状态一致

        参数：
            x, y, z: 位置 (米)
            roll, pitch, yaw: 姿态角度 (弧度或度)
            radians: True 表示弧度，False 表示度
        """
        body_node = self.model.frame_manager.nodes[self.model.body_frame]

        # 如果启用了脚步锁定，使用原子操作确保状态一致性
        if self.foot_lock_enabled and self.fixed_foot_world:
            # 步骤 1: 保存旧的机体位姿（用于可能的回滚）
            old_pos, old_roll, old_pitch, old_yaw = body_node.rel.get_pose_euler(radians=True)

            # 步骤 2: 临时设置新的机体位姿（用于 IK 计算）
            body_node.rel.set_pose_euler(x, y, z, roll, pitch, yaw, radians=radians)

            # 步骤 3: 计算所有腿的 IK，但不立即更新
            new_joints = {}
            failed_legs = []

            for leg_name, foot_world in self.fixed_foot_world.items():
                legkin, hip_frame = self.model.legs[leg_name]

                # 将脚部世界坐标转换到髋坐标系
                foot_in_hip = self.model.frame_manager.transform_point(foot_world, "world", hip_frame)

                # 逆运动学计算
                ikr = legkin.inverse(foot_in_hip)
                if ikr.success and ikr.joints is not None:
                    new_joints[leg_name] = ikr.joints
                else:
                    failed_legs.append((leg_name, ikr.reason))

            # 步骤 4: 检查是否所有 IK 都成功
            if not failed_legs:
                # 所有 IK 成功，原子更新所有关节角度
                for leg_name, joints in new_joints.items():
                    self.model.update_joint_angles(leg_name, joints)
            else:
                # 有 IK 失败，回滚机体位姿
                body_node.rel.set_pose_euler(old_pos[0], old_pos[1], old_pos[2],
                                              old_roll, old_pitch, old_yaw, radians=True)
                if self.verbose:
                    print(f"IK 失败: {len(failed_legs)} 条腿，机体位姿已回滚")
        else:
            # 脚步锁定未启用，直接更新机体位姿
            body_node.rel.set_pose_euler(x, y, z, roll, pitch, yaw, radians=radians)

    def set_joint_angles(self, leg_name: str, hip_side: float, hip_pitch: float, knee_pitch: float, *, radians=True):
        """
        直接设置关节角度

        参数：
            leg_name: 腿名称
            hip_side: 髋侧摆角度
            hip_pitch: 髋俯仰角度
            knee_pitch: 膝俯仰角度
            radians: True 表示弧度，False 表示度
        """
        if not radians:
            hip_side = math.radians(hip_side)
            hip_pitch = math.radians(hip_pitch)
            knee_pitch = math.radians(knee_pitch)

        joints = LegJoints(hip_side, hip_pitch, knee_pitch)
        self.model.update_joint_angles(leg_name, joints)

        # 如果设置了关节角度，禁用脚步锁定
        self.foot_lock_enabled = False

    def get_body_pose(self):
        """
        获取当前机体位姿

        返回：
            dict: {'position': [x,y,z], 'orientation_rad': [...], 'orientation_deg': [...]}
        """
        body_node = self.model.frame_manager.nodes[self.model.body_frame]
        pos, roll, pitch, yaw = body_node.rel.get_pose_euler(radians=True)
        return {
            'position': pos.tolist(),
            'orientation_rad': [roll, pitch, yaw],
            'orientation_deg': [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]
        }

    def get_foot_positions_world(self):
        """
        获取所有脚部在世界坐标系中的位置

        返回：
            dict: {'leg_name': [x, y, z], ...}
        """
        foot_positions = {}
        for leg_name in self.model.legs.keys():
            foot_pos = self.model.get_leg_joints_world(leg_name)[-1]
            foot_positions[leg_name] = foot_pos.tolist()
        return foot_positions

    def set_all_joint_angles(self, joint_angles_dict: Dict, *, radians=True):
        """
        设置所有腿的关节角度

        参数：
            joint_angles_dict: {'left_front': [hip_side, hip_pitch, knee_pitch], ...}
            radians: True 表示弧度，False 表示度
        """
        for leg_name, angles in joint_angles_dict.items():
            if leg_name in self.model.legs:
                self.set_joint_angles(leg_name, angles[0], angles[1], angles[2], radians=radians)

    def get_all_joint_angles(self, *, radians=True):
        """
        获取所有腿的关节角度

        参数：
            radians: True 返回弧度，False 返回度

        返回：
            dict: {'left_front': [hip_side, hip_pitch, knee_pitch], ...}
        """
        angles_dict = {}
        for leg_name in self.model.legs.keys():
            joints = self.model.joints[leg_name]
            if radians:
                angles_dict[leg_name] = [joints.hip_side, joints.hip_pitch, joints.knee_pitch]
            else:
                angles_dict[leg_name] = [
                    math.degrees(joints.hip_side),
                    math.degrees(joints.hip_pitch),
                    math.degrees(joints.knee_pitch)
                ]
        return angles_dict

    def enable_forward_kinematics_mode(self):
        """启用正向运动学模式（关节控制模式）"""
        self.foot_lock_enabled = False

    def enable_inverse_kinematics_mode(self):
        """启用逆运动学模式（脚步锁定模式）"""
        self.capture_fixed_feet()

    def is_inverse_kinematics_mode(self):
        """
        检查是否为逆运动学模式

        返回：
            bool: True 表示逆运动学模式
        """
        return self.foot_lock_enabled
