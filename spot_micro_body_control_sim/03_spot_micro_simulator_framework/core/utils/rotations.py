# core/utils/rotations.py
"""
旋转矩阵工具模块

提供 3D 旋转矩阵的生成和转换功能。
"""
import numpy as np
import math
from typing import Tuple


def rot_x(roll: float) -> np.ndarray:
    """
    绕 X 轴旋转矩阵

    参数：
        roll: 旋转角度 (弧度)

    返回：
        np.ndarray: 3x3 旋转矩阵
    """
    c, s = math.cos(roll), math.sin(roll)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(pitch: float) -> np.ndarray:
    """
    绕 Y 轴旋转矩阵

    参数：
        pitch: 旋转角度 (弧度)

    返回：
        np.ndarray: 3x3 旋转矩阵
    """
    c, s = math.cos(pitch), math.sin(pitch)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(yaw: float) -> np.ndarray:
    """
    绕 Z 轴旋转矩阵

    参数：
        yaw: 旋转角度 (弧度)

    返回：
        np.ndarray: 3x3 旋转矩阵
    """
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def euler_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    欧拉角转旋转矩阵

    使用 ZYX 旋转顺序：R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

    参数：
        roll: 横滚角 (弧度)
        pitch: 俯仰角 (弧度)
        yaw: 偏航角 (弧度)

    返回：
        np.ndarray: 3x3 旋转矩阵
    """
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def matrix_to_euler(R: np.ndarray) -> Tuple[float, float, float]:
    """
    旋转矩阵转欧拉角

    使用 ZYX 旋转顺序的逆运算。

    参数：
        R: 3x3 旋转矩阵

    返回：
        tuple: (roll, pitch, yaw) 欧拉角 (弧度)
    """
    sy = (R[0, 0]**2 + R[1, 0]**2)**0.5
    singular = sy < 1e-9

    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        # 万向节锁死情况
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0

    return roll, pitch, yaw
