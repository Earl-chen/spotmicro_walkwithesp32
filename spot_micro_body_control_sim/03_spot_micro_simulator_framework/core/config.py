# core/config.py
"""
项目级配置：单位、几何默认值（注意：项目内部使用米和弧度）。
"""
# Units: internal SI
LENGTH_UNIT = "m"
ANGLE_UNIT = "rad"

# Numerical tolerances
EPS = 1e-9

# IK tolerances (预留配置，可用于未来的精度检查)
IK_POS_TOL = 1e-4    # 0.1 mm position tolerance
IK_ANGLE_TOL = 1e-3  # ~0.057 deg angle tolerance

# Debug flag (预留配置，可用于启用详细日志)
DEBUG = False

# IK 边界阈值
IK_S_MIN = 1e-6  # 逆运动学中最小距离 S 的阈值，防止除零