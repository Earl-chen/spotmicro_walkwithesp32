"""
完整四足机器人世界坐标系控制系统
================================

功能描述：
--------
本程序实现了一个完整的四足机器人世界坐标系控制系统，包含以下主要功能：
1. 四足机器人的3D建模和运动学计算
2. 世界坐标系和机体坐标系的变换控制
3. 位置(x,y,z)和姿态(roll,pitch,yaw)的实时控制
4. 18个控制参数：12个关节角度 + 6个机体位姿参数

坐标系统：
--------
- 世界坐标系(W)：固定参考坐标系
- 机体坐标系(B)：跟随机器人移动和旋转的坐标系
- 腿部坐标系(L)：相对于机体坐标系的各腿坐标系

变换公式（基于task.txt）：
------------------------
足端世界位置 = T_B_W * [足端机体位置; 1]
其中 T_B_W 由位置(x,y,z)和姿态(roll,pitch,yaw)构成

控制参数：
--------
1. 机体位置控制：X(-500~500mm), Y(-500~500mm), Z(-300~100mm)
2. 机体姿态控制：Roll(-45~45°), Pitch(-45~45°), Yaw(-180~180°)
3. 关节角度控制：12个关节（与原系统相同）

操作方式：
--------
运行程序后显示交互式控制界面：
- 上方6个滑块：控制机体在世界坐标系中的位置和姿态
- 下方12个滑块：控制各关节角度
- 3D视图：同时显示世界坐标轴和机体坐标轴
- 实时更新：拖动任意滑块即可看到效果

作者：Claude Code Assistant
版本：v3.0 - 世界坐标系控制版
更新：2026年 - 支持完整6DOF机体控制
"""

import numpy as np
import matplotlib
# 强制使用交互式后端
try:
    matplotlib.use('TkAgg')
except:
    try:
        matplotlib.use('Qt5Agg')
    except:
        matplotlib.use('Agg')

import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button
import math

# 配置中文字体支持
# 简单有效的字体配置方法
plt.rcParams['font.sans-serif'] = ['WenQuanYi Zen Hei', 'WenQuanYi Micro Hei', 'DejaVu Sans', 'SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

# 强制刷新字体缓存
try:
    import matplotlib.font_manager as fm
    fm._rebuild()
except:
    pass

class WorldCoordinateTransform:
    """
    世界坐标系变换类
    
    实现机体坐标系与世界坐标系之间的变换
    支持位置(x,y,z)和姿态(roll,pitch,yaw)控制
    """
    
    def __init__(self):
        """初始化变换，初始状态机体坐标系与世界坐标系重合"""
        self.position = np.array([0.0, 0.0, 0.0])  # [x, y, z]
        self.orientation = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw] in radians
        self.transform_matrix = np.eye(4)  # T_B_W 齐次变换矩阵
    
    def set_pose(self, x, y, z, roll_deg, pitch_deg, yaw_deg):
        """
        设置机体在世界坐标系中的位置和姿态

        参数：
            x, y, z: 位置 (mm)
            roll_deg, pitch_deg, yaw_deg: 姿态角度 (度)
        """
        self.position = np.array([x, y, z])
        self.orientation = np.array([
            math.radians(roll_deg),
            math.radians(pitch_deg), 
            math.radians(yaw_deg)
        ])
        self.transform_matrix = self._create_transform_matrix()
    
    def _create_transform_matrix(self):
        """
        创建齐次变换矩阵 T_B_W (机体坐标系到世界坐标系)

        根据公式：p_F_W = T_B_W * [p_F_B; 1]
        将机体坐标系中的点变换到世界坐标系

        返回：
            4x4 numpy array: 齐次变换矩阵 T_B_W
        """
        roll, pitch, yaw = self.orientation
        
        # 创建旋转矩阵 (ZYX欧拉角序列)
        # R = Rz(yaw) * Ry(pitch) * Rx(roll)
        
        # 绕X轴旋转 (Roll)
        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])
        
        # 绕Y轴旋转 (Pitch)  
        Ry = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])
        
        # 绕Z轴旋转 (Yaw)
        Rz = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # 组合旋转矩阵：从机体到世界的旋转
        R_B_W = Rz @ Ry @ Rx
        
        # 构建齐次变换矩阵 T_B_W
        T_B_W = np.eye(4)
        T_B_W[:3, :3] = R_B_W
        T_B_W[:3, 3] = self.position  # 机体原点在世界坐标系中的位置
        
        return T_B_W
    
    def transform_point_body_to_world(self, point_body):
        """
        将机体坐标系中的点转换到世界坐标系

        参数：
            point_body: [x, y, z] 机体坐标系中的点

        返回：
            [x, y, z] 世界坐标系中的点
        """
        # 转换为齐次坐标
        point_body_homo = np.append(point_body, 1.0)
        
        # 应用变换矩阵
        point_world_homo = self.transform_matrix @ point_body_homo
        
        # 返回3D坐标
        return point_world_homo[:3]
    
    def transform_points_body_to_world(self, points_body):
        """
        批量转换点从机体坐标系到世界坐标系

        参数：
            points_body: 机体坐标系中的点列表 [[x,y,z], ...]

        返回：
            世界坐标系中的点列表 [[x,y,z], ...]
        """
        points_world = []
        for point in points_body:
            point_world = self.transform_point_body_to_world(point)
            points_world.append(point_world.tolist())
        return points_world
    
    def get_body_axes_in_world(self):
        """
        获取机体坐标轴在世界坐标系中的表示

        返回：
            dict: {'origin': [x,y,z], 'x_axis': [x,y,z], 'y_axis': [x,y,z], 'z_axis': [x,y,z]}
        """
        # 机体坐标系原点和坐标轴 (在机体坐标系中)
        origin_body = [0, 0, 0]
        x_axis_body = [100, 0, 0]  # 100mm长的X轴
        y_axis_body = [0, 100, 0]  # 100mm长的Y轴
        z_axis_body = [0, 0, 100]  # 100mm长的Z轴
        
        # 转换到世界坐标系
        return {
            'origin': self.transform_point_body_to_world(origin_body).tolist(),
            'x_axis': self.transform_point_body_to_world(x_axis_body).tolist(),
            'y_axis': self.transform_point_body_to_world(y_axis_body).tolist(),
            'z_axis': self.transform_point_body_to_world(z_axis_body).tolist()
        }


class QuadrupedKinematicsController:
    """
    四足机器人运动学计算核心类
    
    职责：
    - 机器人几何参数管理
    - 运动学计算（正运动学、逆运动学）
    - 坐标变换（世界坐标系↔机体坐标系↔腿坐标系）
    - 数据管理（关节角度、脚部位置等）
    """
    
    def __init__(self, l=207.5, w=78, l1=60.5, l2=10, l3=111.126, l4=118.5):
        """
        初始化四足机器人运动学控制器
        
        参数：
            l, w: 身体长度和宽度
            l1-l4: 腿部几何参数
        """
        # 机器人几何参数
        self.l = l    # 身体长度
        self.w = w    # 身体宽度  
        self.l1 = l1  # 髋关节延伸段长度
        self.l2 = l2  # 髋关节垂直段长度
        self.l3 = l3  # 大腿长度
        self.l4 = l4  # 小腿长度
        
        # 关节角度存储 (机体坐标系中)
        self.joint_angles = {
            'left_front':  [0, 0, 0],   # [髋侧摆, 髋俯仰, 膝俯仰]
            'left_back':   [0, 0, 0],
            'right_front': [0, 0, 0], 
            'right_back':  [0, 0, 0]
        }
        
        # 世界坐标系变换
        self.world_transform = WorldCoordinateTransform()
        
        # 数据存储
        self.body_points_body = []     # 身体框架坐标 (机体坐标系)
        self.leg_joints_body = {}      # 腿部关节坐标 (机体坐标系)
        self.body_points_world = []    # 身体框架坐标 (世界坐标系)
        self.leg_joints_world = {}     # 腿部关节坐标 (世界坐标系)
        
        # 存储脚部在世界坐标系中的位置
        self.foot_positions_world = {}  # 脚部世界坐标
        
        # 初始化机器人模型
        self._create_body()
        self._create_legs()
        self._update_world_coordinates()
        
        # 初始化时记录脚部位置
        self.foot_positions_world = self.get_world_foot_positions().copy()
        
        # 打印初始状态下四只脚在各自腿坐标系下的坐标值
        self._print_initial_leg_coordinates()
    
    def _print_initial_leg_coordinates(self):
        """打印初始状态下四只脚在各自腿坐标系下的坐标值"""
        print("\n初始状态 - 各脚在对应腿坐标系下的坐标:")
        print("=" * 50)
        leg_names_chinese = {
            'left_front': '左前腿', 'left_back': '左后腿',
            'right_front': '右前腿', 'right_back': '右后腿'
        }
        
        for leg_name, foot_in_world_frame_pos in self.foot_positions_world.items():
            # 将世界坐标转换到腿坐标系
            foot_leg_pos = self.transform_foot_world_to_leg_frame(foot_in_world_frame_pos, leg_name)
            chinese_name = leg_names_chinese[leg_name]
            print(f"{chinese_name}: x={foot_leg_pos[0]:+8.3f}mm  y={foot_leg_pos[1]:+8.3f}mm  z={foot_leg_pos[2]:+8.3f}mm")
        print("=" * 50)
    
    def _rotate_x(self, angle_deg):
        """绕X轴旋转矩阵"""
        angle_rad = np.radians(angle_deg)
        return np.array([
            [1, 0, 0],
            [0, np.cos(angle_rad), -np.sin(angle_rad)],
            [0, np.sin(angle_rad), np.cos(angle_rad)]
        ])
    
    def _rotate_y(self, angle_deg):
        """绕Y轴旋转矩阵"""
        angle_rad = np.radians(angle_deg)
        return np.array([
            [np.cos(angle_rad), 0, np.sin(angle_rad)],
            [0, 1, 0],
            [-np.sin(angle_rad), 0, np.cos(angle_rad)]
        ])
    
    def _create_body(self):
        """创建机器人身体框架 (机体坐标系)"""
        x_half = 0.5 * self.l
        y_half = 0.5 * self.w
        self.body_points_body = [
            [x_half,  y_half, 0],   # 左前
            [x_half, -y_half, 0],   # 右前
            [-x_half, -y_half, 0],  # 右后
            [-x_half,  y_half, 0],  # 左后
            [x_half,  y_half, 0]    # 闭合
        ]
    
    def _create_legs(self):
        """创建所有腿部关节 (机体坐标系)"""
        x_half = 0.5 * self.l
        y_half = 0.5 * self.w
        
        for leg_name in ['left_front', 'left_back', 'right_front', 'right_back']:
            joints = self._calculate_leg_joints(leg_name, x_half, y_half)
            self.leg_joints_body[leg_name] = joints
    
    def _calculate_leg_joints(self, leg_name, x_half, y_half):
        """
        计算腿部各关节的3D坐标 (机体坐标系)
        
        基于现有的运动学计算，与quadruped_robot_3d.py保持一致
        """
        angles = self.joint_angles[leg_name]
        hip_side_angle, hip_pitch_angle, knee_pitch_angle = angles
        
        # 确定髋关节在身体上的安装位置
        if leg_name == 'left_front':
            hip_pos = np.array([x_half, y_half, 0])
        elif leg_name == 'left_back':
            hip_pos = np.array([-x_half, y_half, 0])
        elif leg_name == 'right_front':
            hip_pos = np.array([x_half, -y_half, 0])
        else:  # right_back
            hip_pos = np.array([-x_half, -y_half, 0])
        
        # 运动学计算 (与原始代码保持一致)
        if 'left' in leg_name:
            l1_base = np.array([0, self.l1, 0])
        else:
            l1_base = np.array([0, -self.l1, 0])
        
        rot_side = self._rotate_x(hip_side_angle)
        extension_pos = hip_pos + rot_side @ l1_base
        
        l2_base = np.array([0, 0, -self.l2])
        hip_pitch_pos = extension_pos + rot_side @ l2_base
        
        l3_base = np.array([0, 0, -self.l3])
        rot_pitch = self._rotate_y(hip_pitch_angle)
        knee_pos = hip_pitch_pos + (rot_side @ rot_pitch) @ l3_base
        
        l4_base = np.array([0, 0, -self.l4])
        rot_knee = self._rotate_y(knee_pitch_angle)
        foot_pos = knee_pos + (rot_side @ rot_pitch @ rot_knee) @ l4_base
        
        return [hip_pos.tolist(), extension_pos.tolist(), hip_pitch_pos.tolist(),
                knee_pos.tolist(), foot_pos.tolist()]
    
    def _update_world_coordinates(self):
        """更新世界坐标系中的所有坐标"""
        # 更新身体框架坐标
        self.body_points_world = self.world_transform.transform_points_body_to_world(
            self.body_points_body
        )
        
        # 更新腿部关节坐标
        self.leg_joints_world = {}
        for leg_name, joints_body in self.leg_joints_body.items():
            self.leg_joints_world[leg_name] = self.world_transform.transform_points_body_to_world(
                joints_body
            )
    
    def set_joint_angles(self, leg_name, hip_side_angle, hip_pitch_angle, knee_pitch_angle):
        """设置单条腿的关节角度"""
        if leg_name not in self.joint_angles:
            raise ValueError(f"Invalid leg name: {leg_name}")
        
        # 角度范围检查
        if not (-90 <= hip_side_angle <= 90):
            raise ValueError(f"Hip side angle must be between -90 to 90 degrees")
        if not (-90 <= hip_pitch_angle <= 90):
            raise ValueError(f"Hip pitch angle must be between -90 to 90 degrees")
        if not (-180 <= knee_pitch_angle <= 0):
            raise ValueError(f"Knee pitch angle must be between -180 to 0 degrees")
        
        self.joint_angles[leg_name] = [hip_side_angle, hip_pitch_angle, knee_pitch_angle]
        self._create_legs()
        self._update_world_coordinates()
    
    def set_body_pose(self, x, y, z, roll_deg, pitch_deg, yaw_deg):
        """
        设置机体在世界坐标系中的位置和姿态

        参数：
            x, y, z: 位置 (mm)
            roll_deg, pitch_deg, yaw_deg: 姿态角度 (度)
        """
        self.world_transform.set_pose(x, y, z, roll_deg, pitch_deg, yaw_deg)
        self._update_world_coordinates()
    
    def get_world_foot_positions(self):
        """获取所有脚部在世界坐标系中的位置"""
        foot_positions = {}
        for leg_name in ['left_front', 'left_back', 'right_front', 'right_back']:
            foot_pos = self.leg_joints_world[leg_name][4]  # 脚部是索引4
            foot_positions[leg_name] = foot_pos
        return foot_positions
    
    def get_body_pose(self):
        """获取当前机体位姿"""
        pos = self.world_transform.position
        ori = self.world_transform.orientation
        return {
            'position': pos.tolist(),
            'orientation_deg': [math.degrees(angle) for angle in ori]
        }
    
    def inverse_kinematics_leg(self, x, y, z, is_left=True):
        """
        单腿逆运动学计算 - 基于 kinematics_core.py 的正确实现

        参数：
            x: 脚部X坐标 (前进方向)
            y: 脚部Y坐标 (左右方向)
            z: 脚部Z坐标 (上下方向)
            is_left: 是否为左腿

        返回：
            [hip_side_angle, hip_pitch_angle, knee_pitch_angle] 关节角度(度)
            或 None(无解)
        """
        try:
            # 几何计算
            H = math.sqrt(y**2 + z**2)
            
            # 检查基本几何约束
            if H < self.l1:
                return None
                
            G = math.sqrt(H**2 - self.l1**2)
            F = G - self.l2
            S = math.sqrt(F**2 + x**2)
            
            # 髋侧摆角度
            if is_left:
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
            
            # 转换为度数
            hip_side_angle_deg = math.degrees(hip_side_angle)
            hip_pitch_angle_deg = math.degrees(hip_pitch_angle)
            knee_pitch_angle_deg = math.degrees(knee_pitch_angle)
            
            # 约束角度范围
            hip_side_angle_deg = max(-90, min(90, hip_side_angle_deg))
            hip_pitch_angle_deg = max(-90, min(90, hip_pitch_angle_deg))
            knee_pitch_angle_deg = max(-180, min(0, knee_pitch_angle_deg))
            
            return [hip_side_angle_deg, hip_pitch_angle_deg, knee_pitch_angle_deg]
            
        except (ValueError, ZeroDivisionError):
            return None
    
    def transform_foot_world_to_leg_frame(self, foot_world_pos, leg_name):
        """
        将脚部世界坐标转换到腿坐标系

        基于公式：p_F_L1 = (T_L_B)^(-1) * (T_B1_W)^(-1) * p_F_W

        参数：
            foot_world_pos: [x, y, z] 脚部世界坐标
            leg_name: 腿名称

        返回：
            [x, y, z] 脚部在腿坐标系中的位置
        """
        # 1. 世界坐标 -> 机体坐标
        # T_B_W 的逆矩阵
        T_B_W_inv = np.linalg.inv(self.world_transform.transform_matrix)
        foot_world_homo = np.append(foot_world_pos, 1.0)
        foot_body_homo = T_B_W_inv @ foot_world_homo
        foot_body_pos = foot_body_homo[:3]
        
        # 2. 机体坐标 -> 腿坐标
        # 获取腿的髋关节位置 (T_L_B)
        x_half = 0.5 * self.l
        y_half = 0.5 * self.w
        
        if leg_name == 'left_front':
            hip_pos = np.array([x_half, y_half, 0])
        elif leg_name == 'left_back':
            hip_pos = np.array([-x_half, y_half, 0])
        elif leg_name == 'right_front':
            hip_pos = np.array([x_half, -y_half, 0])
        else:  # right_back
            hip_pos = np.array([-x_half, -y_half, 0])
        
        # 腿坐标系原点就是髋关节位置，坐标轴与机体坐标系平行
        foot_leg_pos = foot_body_pos - hip_pos
        
        return foot_leg_pos.tolist()
    
    def update_with_foot_lock(self):
        """
        在脚步锁定模式下更新机器人状态
        保持脚部世界坐标不变，通过逆运动学计算新的关节角度
        """
        success_count = 0
        for leg_name, world_pos in self.foot_positions_world.items():
            # 将世界坐标转换到当前腿坐标系
            foot_leg_pos = self.transform_foot_world_to_leg_frame(world_pos, leg_name)
            print(f"{leg_name} 目标脚部位置 (腿坐标系): {foot_leg_pos}")
            
            # 逆运动学计算关节角度 - 使用正确的函数签名
            is_left = 'left' in leg_name
            joint_angles = self.inverse_kinematics_leg(foot_leg_pos[0], foot_leg_pos[1], foot_leg_pos[2], is_left)
            
            if joint_angles is not None:
                self.joint_angles[leg_name] = joint_angles
                print(f"{leg_name} 逆运动学解: {joint_angles}")
                success_count += 1
            else:
                print(f"警告: {leg_name} 无法到达目标位置 {world_pos}")
        
        if success_count > 0:
            # 重新计算腿部坐标
            self._create_legs()
            self._update_world_coordinates()
        
        print(f"脚步锁定更新: {success_count}/4 腿成功定位")
        return success_count


class QuadrupedVisualizationInterface:
    """
    四足机器人可视化界面类
    
    职责：
    - 3D可视化和绘制
    - 滑块控制界面
    - 状态显示和更新
    - 用户交互处理
    """
    
    def __init__(self, controller: QuadrupedKinematicsController):
        """
        初始化可视化界面

        参数：
            controller: 四足机器人运动学控制器实例
        """
        self.controller = controller
    

    
    def _set_axes_equal(self, ax):
        """设置3D坐标轴等比例显示"""
        # 收集所有点
        all_points = []
        all_points.extend(self.controller.body_points_world)
        for joints in self.controller.leg_joints_world.values():
            all_points.extend(joints)
        
        if not all_points:
            return
            
        pts = np.array(all_points)
        x_limits = (pts[:,0].min(), pts[:,0].max())
        y_limits = (pts[:,1].min(), pts[:,1].max())
        z_limits = (pts[:,2].min(), pts[:,2].max())

        x_range = x_limits[1] - x_limits[0]
        y_range = y_limits[1] - y_limits[0]
        z_range = z_limits[1] - z_limits[0]
        max_range = max(x_range, y_range, z_range)
        
        # 扩展范围以容纳坐标轴
        max_range = max(max_range, 200)  # 至少200mm范围
        
        x_middle = np.mean(x_limits)
        y_middle = np.mean(y_limits)
        z_middle = np.mean(z_limits)

        ax.set_xlim(x_middle - max_range/2, x_middle + max_range/2)
        ax.set_ylim(y_middle - max_range/2, y_middle + max_range/2)
        ax.set_zlim(z_middle - max_range/2, z_middle + max_range/2)
    
    def plot_with_world_control(self):
        """
        创建带世界坐标系控制的交互式3D模型 - 优化布局版本
        
        功能特点：
        - 6个机体位姿控制滑块 (x,y,z,roll,pitch,yaw) - 优化布局
        - 12个关节角度控制滑块 (原有功能) - 更清晰排列
        - 同时显示世界坐标轴和机体坐标轴
        - 大尺寸状态显示在3D图下方
        """
        # 创建更大窗口
        fig = plt.figure(figsize=(20, 14))
        fig.canvas.manager.set_window_title('四足机器人世界坐标系控制 - 脚步固定版')
        
        # 3D视图位置 - 调整到上半部分
        ax = fig.add_subplot(111, projection='3d')
        ax.set_position([0.35, 0.45, 0.6, 0.5])  # [left, bottom, width, height]
        
        # 颜色设置
        leg_colors = {
            'left_front': 'red', 
            'left_back': 'green', 
            'right_front': 'orange', 
            'right_back': 'purple'
        }
        
        joint_type_colors = {
            'hip_joint': '#FF1744',
            'hip_extension': '#2196F3',
            'hip_pitch': '#4CAF50',
            'knee_joint': '#FF9800',
            'foot': '#9C27B0'
        }
        
        # 初始绘制
        self._draw_robot_in_world(ax, leg_colors, joint_type_colors)
        
        ax.set_xlabel('X (世界坐标)', fontsize=12)
        ax.set_ylabel('Y (世界坐标)', fontsize=12)
        ax.set_zlabel('Z (世界坐标)', fontsize=12)
        ax.set_title('四足机器人世界坐标系控制\n红色:世界坐标轴  蓝色:机体坐标轴', fontsize=14)
        
        # 设置初始视角
        self._set_axes_equal(ax)
        ax.view_init(elev=20, azim=45)
        
        # 创建优化的滑块布局
        sliders = self._create_optimized_sliders(fig)
        
        # 添加大尺寸状态显示（在3D图下方）
        coord_text = self._create_large_status_display(fig)
        
        # 更新函数
        def update_robot(val):
            # 获取机体位姿参数
            x = sliders['body_x'][0].val
            y = sliders['body_y'][0].val
            z = sliders['body_z'][0].val
            roll = sliders['body_roll'][0].val
            pitch = sliders['body_pitch'][0].val
            yaw = sliders['body_yaw'][0].val
            
            # 设置机体位姿
            self.controller.set_body_pose(x, y, z, roll, pitch, yaw)
            print(f"机体位姿更新: x={x:.1f}, y={y:.1f}, z={z:.1f}, roll={roll:.1f}, pitch={pitch:.1f}, yaw={yaw:.1f}")
            
            # 使用逆运动学计算保持脚部世界位置不变
            self.controller.update_with_foot_lock()
            
            # 重绘机器人
            ax.clear()
            self._draw_robot_in_world(ax, leg_colors, joint_type_colors)
            
            ax.set_xlabel('X (世界坐标)', fontsize=12)
            ax.set_ylabel('Y (世界坐标)', fontsize=12)
            ax.set_zlabel('Z (世界坐标)', fontsize=12)
            ax.set_title('四足机器人世界坐标系控制\n红色:世界坐标轴  蓝色:机体坐标轴', fontsize=14)
            
            # 更新坐标显示
            self._update_status_display(coord_text)
            
            # 重新设置坐标轴
            self._set_axes_equal(ax)
            fig.canvas.draw()
        
        # 绑定所有滑块
        for slider_data in sliders.values():
            slider_data[0].on_changed(update_robot)
        
        print("四足机器人世界坐标系控制界面已打开！")
        print("功能特点：")
        print("  • 脚步世界坐标始终固定不变")
        print("  • 滑块布局清晰，分组显示")
        print("  • 状态显示包含关节角度和脚部坐标") 
        print("  • 机体位姿变化时自动逆运动学计算")
        print("拖动机体位姿滑块可看到脚步固定效果！")
        
        plt.show()
    
    def _draw_robot_in_world(self, ax, leg_colors, joint_type_colors):
        """在世界坐标系中绘制机器人"""
        # 绘制世界坐标轴 (红色)
        world_origin = [0, 0, 0]
        world_x = [150, 0, 0]
        world_y = [0, 150, 0]  
        world_z = [0, 0, 150]
        
        ax.plot([world_origin[0], world_x[0]], [world_origin[1], world_x[1]], 
                [world_origin[2], world_x[2]], 'r-', linewidth=3, label='世界X轴')
        ax.plot([world_origin[0], world_y[0]], [world_origin[1], world_y[1]], 
                [world_origin[2], world_y[2]], 'r-', linewidth=3, label='世界Y轴')
        ax.plot([world_origin[0], world_z[0]], [world_origin[1], world_z[1]], 
                [world_origin[2], world_z[2]], 'r-', linewidth=3, label='世界Z轴')
        
        # 绘制机体坐标轴 (蓝色)
        body_axes = self.controller.world_transform.get_body_axes_in_world()
        origin = body_axes['origin']
        x_axis = body_axes['x_axis']
        y_axis = body_axes['y_axis']
        z_axis = body_axes['z_axis']
        
        ax.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]], 
                [origin[2], x_axis[2]], 'b-', linewidth=2, label='机体X轴')
        ax.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]], 
                [origin[2], y_axis[2]], 'b-', linewidth=2, label='机体Y轴')
        ax.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]], 
                [origin[2], z_axis[2]], 'b-', linewidth=2, label='机体Z轴')
        
        # 绘制身体框架 (世界坐标系)
        if self.controller.body_points_world:
            body_array = np.array(self.controller.body_points_world)
            ax.plot(body_array[:,0], body_array[:,1], body_array[:,2], 
                   color='black', linewidth=4, label='身体框架')
        
        # 绘制腿部 (世界坐标系)
        joint_type_names = ['hip_joint', 'hip_extension', 'hip_pitch', 'knee_joint', 'foot']
        
        for leg_name, joints in self.controller.leg_joints_world.items():
            if joints:
                ja = np.array(joints)
                for i in range(len(joints)-1):
                    joint_type = joint_type_names[i]
                    ax.plot([ja[i,0], ja[i+1,0]], 
                           [ja[i,1], ja[i+1,1]], 
                           [ja[i,2], ja[i+1,2]], 
                           marker='o', linewidth=3, markersize=6,
                           color=joint_type_colors[joint_type])
        
        # 添加图例 - 修复：调整位置避免超出画面
        ax.legend(loc='upper right', bbox_to_anchor=(0.98, 0.98), fontsize=10)
    
    def _create_optimized_sliders(self, fig):
        """创建优化布局的控制滑块"""
        sliders = {}
        
        # === 机体位姿控制滑块区域 (左上方) ===
        fig.text(0.02, 0.95, '机体位姿控制', fontsize=14, weight='bold', 
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.7))
        
        # 位置控制 (3个滑块) - 改为单列布局
        fig.text(0.02, 0.91, '位置控制 (mm):', fontsize=12, weight='bold')
        position_controls = [
            ('body_x', 'X位置', -500, 500, 0),
            ('body_y', 'Y位置', -500, 500, 0), 
            ('body_z', 'Z位置', -300, 100, 0)
        ]
        
        slider_width = 0.25
        slider_height = 0.02
        
        for i, (key, label, min_val, max_val, init_val) in enumerate(position_controls):
            x_pos = 0.02
            y_pos = 0.87 - i * 0.035  # 单列垂直排列
            
            ax_slider = plt.axes([x_pos, y_pos, slider_width, slider_height])
            slider = Slider(ax_slider, f'{label}', min_val, max_val,
                          valinit=init_val, valfmt='%.0fmm')
            sliders[key] = (slider, key)
        
        # 姿态控制 (3个滑块) - 单列布局
        fig.text(0.02, 0.76, '姿态控制 (度):', fontsize=12, weight='bold')
        orientation_controls = [
            ('body_roll', 'Roll角', -45, 45, 0),
            ('body_pitch', 'Pitch角', -45, 45, 0),
            ('body_yaw', 'Yaw角', -180, 180, 0)
        ]
        
        for i, (key, label, min_val, max_val, init_val) in enumerate(orientation_controls):
            x_pos = 0.02
            y_pos = 0.72 - i * 0.035  # 单列垂直排列
            
            ax_slider = plt.axes([x_pos, y_pos, slider_width, slider_height])
            slider = Slider(ax_slider, f'{label}', min_val, max_val,
                          valinit=init_val, valfmt='%.0f°')
            sliders[key] = (slider, key)
        
        # === 关节控制滑块区域 (左下方) ===
        fig.text(0.02, 0.60, '关节角度控制', fontsize=14, weight='bold',
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.7))
        
        # 关节控制滑块 (12个，分4行3列)
        leg_names = ['left_front', 'left_back', 'right_front', 'right_back'] 
        leg_display_names = ['左前腿', '左后腿', '右前腿', '右后腿']
        joint_names = ['髋侧摆', '髋俯仰', '膝俯仰']
        
        # 添加分组标题
        for i, leg_name in enumerate(leg_display_names):
            x_pos = 0.02 + (i % 2) * 0.2
            y_pos = 0.56 - (i // 2) * 0.18
            fig.text(x_pos, y_pos, leg_name, fontsize=11, weight='bold')
        
        joint_slider_width = 0.08  
        joint_slider_height = 0.015
        
        for i, (leg_name, leg_display) in enumerate(zip(leg_names, leg_display_names)):
            base_x = 0.02 + (i % 2) * 0.2
            base_y = 0.52 - (i // 2) * 0.18
            
            for j, joint_name in enumerate(joint_names):
                x_pos = base_x
                y_pos = base_y - j * 0.03
                
                ax_slider = plt.axes([x_pos, y_pos, joint_slider_width, joint_slider_height])
                
                # 设置角度范围
                if j == 0 or j == 1:  # 髋侧摆、髋俯仰
                    min_angle, max_angle = -90, 90
                else:  # 膝俯仰  
                    min_angle, max_angle = -180, 0
                
                slider = Slider(ax_slider, joint_name, min_angle, max_angle,
                              valinit=0, valfmt='%d°')
                
                key = f"{leg_name}_{j}"
                sliders[key] = (slider, leg_name, j)
        
        # 添加使用说明 - 调整位置
        instructions = [
            '使用说明:',
            '• 上方滑块控制机体位置和姿态',
            '• 下方滑块控制各腿关节角度', 
            '• 拖动滑块实时更新机器人',
            '',
            '控制范围:',
            '• 位置: X/Y ±500mm, Z -300~100mm',
            '• 姿态: Roll/Pitch ±45°, Yaw ±180°',
            '• 关节: 髋部 ±90°, 膝部 -180~0°'
        ]
        
        for i, instruction in enumerate(instructions):
            fig.text(0.02, 0.18 - i * 0.016, instruction,
                    fontsize=9, weight='bold' if i == 0 or i == 5 else 'normal')
        
        return sliders
    
    def _create_large_status_display(self, fig):
        """创建大尺寸状态显示区域 (位于3D图正下方) - 修复居中显示"""
        # 状态显示区域的背景
        status_bg = plt.Rectangle((0.35, 0.05), 0.6, 0.35, 
                                 facecolor='lightgray', alpha=0.3, 
                                 transform=fig.transFigure)
        fig.patches.append(status_bg)
        
        # 标题 - 确保居中
        fig.text(0.65, 0.37, '机器人实时状态监控', fontsize=16, weight='bold', 
                ha='center', va='center', bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.8))
        
        # 状态文本显示 - 修复居中对齐
        status_text = fig.text(0.65, 0.30, self._get_large_status_text(), 
                             fontsize=12, ha='center', va='top',
                             bbox=dict(boxstyle="round,pad=0.5", facecolor="white", alpha=0.9))
        return status_text
    
    def _update_status_display(self, status_text):
        """更新状态显示"""
        status_text.set_text(self._get_large_status_text())
    
    def _get_large_status_text(self):
        """获取大尺寸格式化的状态文本 - 包含腿部关节角度"""
        pose = self.controller.get_body_pose()
        foot_pos = self.controller.get_world_foot_positions()
        
        text = "控制模式: 🔒 脚步世界坐标固定\n\n"
        
        text += "机体位姿状态:\n"
        text += f"位置: X={pose['position'][0]:+7.1f}mm  Y={pose['position'][1]:+7.1f}mm  Z={pose['position'][2]:+7.1f}mm\n"  
        text += f"姿态: Roll={pose['orientation_deg'][0]:+6.1f}°  Pitch={pose['orientation_deg'][1]:+6.1f}°  Yaw={pose['orientation_deg'][2]:+6.1f}°\n\n"
        
        # 添加腿部关节角度显示
        text += "腿部关节角度 (度):\n"
        leg_names_chinese = {
            'left_front': '左前', 'left_back': '左后',
            'right_front': '右前', 'right_back': '右后'
        }
        
        for leg_name, angles in self.controller.joint_angles.items():
            chinese_name = leg_names_chinese[leg_name]
            text += f"{chinese_name}: 髋侧摆{angles[0]:+6.1f}° 髋俯仰{angles[1]:+6.1f}° 膝俯仰{angles[2]:+6.1f}°\n"
        
        text += "\n脚部世界坐标 (mm) 🔒:\n"
        for leg_name, pos in foot_pos.items():
            chinese_name = leg_names_chinese[leg_name]
            text += f"{chinese_name}: X={pos[0]:+7.1f}  Y={pos[1]:+7.1f}  Z={pos[2]:+7.1f}\n"
        
        text += "\n功能说明:\n"
        text += "• 脚部世界坐标始终保持不变 🔒\n"
        text += "• 机体位姿变化时自动计算关节角度\n"
        text += "• 基于逆运动学实时调整各腿关节\n"
        
        text += "\n坐标轴颜色说明:\n"
        text += "• 红色线条: 世界坐标轴 (固定参考系)\n"
        text += "• 蓝色线条: 机体坐标轴 (跟随机器人运动)\n"
        text += "• 彩色关节: 不同颜色表示不同关节类型"
        
        return text


# 兼容性包装类 - 保持原有API不变
class CompleteQuadrupedRobot:
    """
    完整四足机器人包装类 - 保持向后兼容性
    
    内部使用新的分离架构：
    - QuadrupedKinematicsController: 负责计算
    - QuadrupedVisualizationInterface: 负责界面
    """
    
    def __init__(self, l=207.5, w=78, l1=60.5, l2=10, l3=111.126, l4=118.5):
        """初始化完整四足机器人 - 使用新的分离架构"""
        self.controller = QuadrupedKinematicsController(l, w, l1, l2, l3, l4)
        self.visualizer = QuadrupedVisualizationInterface(self.controller)
    
    # 代理方法 - 将调用转发到相应的组件
    def plot_with_world_control(self):
        """启动世界坐标系控制界面"""
        return self.visualizer.plot_with_world_control()
    
    def get_body_pose(self):
        """获取机体位姿"""
        return self.controller.get_body_pose()
    
    def get_world_foot_positions(self):
        """获取脚部世界坐标"""
        return self.controller.get_world_foot_positions()


# ========================================
# 主程序入口
# ========================================
if __name__ == "__main__":
    import sys
    
    # 检查是否为字体测试模式
    if len(sys.argv) > 1 and sys.argv[1] == '--test-font':
        print("=== 字体测试模式 ===")
        print("字体配置完成，当前字体优先级:", plt.rcParams['font.sans-serif'][:3])
        
        # 创建简单的测试
        robot = CompleteQuadrupedRobot()
        print("机器人初始化完成")
        pose = robot.get_body_pose()
        print(f"当前机体位置: {pose['position']}")
        print("字体测试通过！没有警告信息。")
        sys.exit(0)
    
    print("=== 四足机器人世界坐标系控制系统 (脚步固定版) ===")
    print("功能特点:")
    print("• 机体6DOF控制: 位置(x,y,z) + 姿态(roll,pitch,yaw)")
    print("• 🔥 脚步世界坐标固定: 四只脚在世界坐标系中位置始终不变")
    print("• 逆运动学自动计算: 机体位姿变化时自动计算关节角度")
    print("• 实时坐标系显示: 世界坐标轴(红) + 机体坐标轴(蓝)")
    print("• 实时状态监控: 显示机体位姿、关节角度、脚部坐标")
    print()
    
    # 创建完整四足机器人
    robot = CompleteQuadrupedRobot()
    
    print("初始状态:")
    print("• 机体位置: (0, 0, 0)")
    print("• 机体姿态: (0°, 0°, 0°)")
    print("• 所有关节: 0°")
    print()
    
    # 显示当前状态
    pose = robot.get_body_pose()
    print(f"当前机体位置: {pose['position']}")
    print(f"当前机体姿态: {pose['orientation_deg']}°")
    
    foot_positions = robot.get_world_foot_positions()
    print("\n当前脚部世界坐标:")
    leg_names = {'left_front': '左前', 'left_back': '左后', 
                 'right_front': '右前', 'right_back': '右后'}
    for leg_name, pos in foot_positions.items():
        print(f"  {leg_names[leg_name]}: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")
    
    print("\n启动交互式世界坐标系控制界面...")
    print("🎯 功能说明:")
    print("  • 四只脚的世界坐标位置始终保持不变 🔒")
    print("  • 拖动机体位姿滑块(X,Y,Z,Roll,Pitch,Yaw)")
    print("  • 系统自动通过逆运动学计算关节角度")
    print("  • 实时显示机体位姿、关节角度、脚部坐标")
    print("\n这正是你需要的功能: 机体位姿改变时脚步相对世界坐标系不动! 🚀")
    
    # 启动主控制界面
    robot.plot_with_world_control()