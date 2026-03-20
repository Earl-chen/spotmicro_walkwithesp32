# robots/spot_micro/leg_kinematics.py
import numpy as np
from core.kinematics.ik_base import IKSolverBase
from core.types import LegJoints, IKResult
from robots.spot_micro import geometry
from core.utils.rotations import rot_x, rot_y, rot_z
from core.config import IK_S_MIN
from typing import List
import math

class SpotLegKinematics(IKSolverBase):
    def __init__(self, l1=geometry.L1, l2=geometry.L2, l3=geometry.L3, l4=geometry.L4, is_left=True):
        self.l1 = l1; self.l2 = l2; self.l3 = l3; self.l4 = l4
        self.is_left = is_left

    def forward(self, hip_pos_body: np.ndarray, joints: LegJoints) -> List[np.ndarray]:
        """
        正运动学计算 - 从原代码_calculate_leg_joints方法迁移
        输入：
            hip_pos_body: 髋关节在机体坐标系中的位置 (3,) numpy array (米)
            joints: 关节角度 (弧度)
        返回：
            list of numpy arrays: [hip_pos, extension_pos, hip_pitch_pos, knee_pos, foot_pos]
        """
        hip_side_angle = joints.hip_side
        hip_pitch_angle = joints.hip_pitch
        knee_pitch_angle = joints.knee_pitch
        
        # 髋关节延伸段计算
        if self.is_left:
            l1_base = np.array([0, self.l1, 0])
        else:
            l1_base = np.array([0, -self.l1, 0])
        
        rot_side = rot_x(hip_side_angle)
        extension_pos = hip_pos_body + rot_side @ l1_base
        
        # 髋关节俯仰段计算
        l2_base = np.array([0, 0, -self.l2])
        hip_pitch_pos = extension_pos + rot_side @ l2_base
        
        # 大腿段计算
        l3_base = np.array([0, 0, -self.l3])
        rot_pitch = rot_y(hip_pitch_angle)
        knee_pos = hip_pitch_pos + (rot_side @ rot_pitch) @ l3_base
        
        # 小腿段计算
        l4_base = np.array([0, 0, -self.l4])
        rot_knee = rot_y(knee_pitch_angle)
        foot_pos = knee_pos + (rot_side @ rot_pitch @ rot_knee) @ l4_base
        
        return [hip_pos_body.copy(), extension_pos, hip_pitch_pos, knee_pos, foot_pos]

    def inverse(self, foot_pos_in_hip_frame: np.ndarray) -> IKResult:
        """
        逆运动学计算 - 从原代码inverse_kinematics_leg方法迁移
        输入：foot_pos_in_hip_frame: 脚部在髋坐标系中的位置 (3,) (米)
        返回：IKResult(success, LegJoints(...), reason)
        """
        try:
            x, y, z = foot_pos_in_hip_frame[0], foot_pos_in_hip_frame[1], foot_pos_in_hip_frame[2]
            
            # 几何计算
            H = math.sqrt(y**2 + z**2)

            # 检查基本几何约束
            if H < self.l1:
                return IKResult(False, None, f"Target too close to hip: H={H:.4f} < l1={self.l1:.4f}")

            G = math.sqrt(H**2 - self.l1**2)
            F = G - self.l2

            # 检查 F 是否为正（脚部太靠近髋俯仰关节）
            if F <= 0:
                return IKResult(False, None, f"Target too close to hip pitch joint: F={F:.4f} <= 0 (G={G:.4f}, l2={self.l2:.4f})")

            S = math.sqrt(F**2 + x**2)

            # 检查 S 是否过小（防止除零和数值不稳定）
            if S < IK_S_MIN:
                return IKResult(False, None, f"Target too close to knee joint: S={S:.6f} < {IK_S_MIN}")

            # 检查可达性约束
            if S > (self.l3 + self.l4):
                return IKResult(False, None, f"Target too far: S={S:.4f} > l3+l4={self.l3+self.l4:.4f}")
            
            # 髋侧摆角度
            if self.is_left:
                hip_side_angle = math.atan2(z, y) + math.acos(self.l1 / H)
            else:
                hip_side_angle = math.pi + math.atan2(z, y) - math.acos(self.l1 / H)
            
            # 髋俯仰角度
            cos_hip = (S**2 + self.l3**2 - self.l4**2) / (2 * S * self.l3)
            cos_hip = max(-1.0, min(1.0, cos_hip))  # 数值稳定性
            hip_pitch_angle = math.acos(cos_hip) - math.atan2(x, F)
            
            # 膝俯仰角度
            cos_knee = (S**2 - self.l3**2 - self.l4**2) / (2 * self.l3 * self.l4)
            cos_knee = max(-1.0, min(1.0, cos_knee))  # 数值稳定性
            knee_pitch_angle = -math.acos(cos_knee)
            
            # 约束角度范围（弧度）
            hip_side_angle = max(-math.pi/2, min(math.pi/2, hip_side_angle))
            hip_pitch_angle = max(-math.pi/2, min(math.pi/2, hip_pitch_angle))
            knee_pitch_angle = max(-math.pi, min(0, knee_pitch_angle))
            
            return IKResult(True, LegJoints(hip_side_angle, hip_pitch_angle, knee_pitch_angle), None)
            
        except (ValueError, ZeroDivisionError) as e:
            return IKResult(False, None, f"Math error: {e}")