# robots/spot_micro/geometry.py
"""
Geometry parameters for Spot-like robot. Units: meters.
Replace values with your actual robot geometry.
"""
# 从原代码提取并转换单位（mm -> m）
L1 = 0.0605    # 60.5 mm
L2 = 0.010     # 10 mm  
L3 = 0.111126  # 111.126 mm
L4 = 0.1185    # 118.5 mm

# 身体尺寸（mm -> m）
BODY_LENGTH = 0.2075   # 207.5 mm
BODY_WIDTH = 0.078     # 78 mm

# 髋关节在机体坐标系中的位置
HIP_OFFSETS = {
    "left_front":  (BODY_LENGTH/2,  BODY_WIDTH/2, 0.0),   # 左前
    "left_back":   (-BODY_LENGTH/2, BODY_WIDTH/2, 0.0),   # 左后
    "right_front": (BODY_LENGTH/2, -BODY_WIDTH/2, 0.0),   # 右前
    "right_back":  (-BODY_LENGTH/2, -BODY_WIDTH/2, 0.0),  # 右后
}