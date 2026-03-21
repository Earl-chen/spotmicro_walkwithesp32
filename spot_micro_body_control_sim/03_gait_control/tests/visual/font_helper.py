#!/usr/bin/env python3
"""
通用字体配置辅助模块 - 兼容所有 matplotlib 版本

使用方法：
    from font_helper import setup_chinese_font
    font_prop = setup_chinese_font()
"""

import os
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm


def setup_chinese_font(font_relative_path=None):
    """
    配置中文字体 - 兼容所有 matplotlib 版本
    
    参数：
        font_relative_path: 字体文件相对路径（可选）
    
    返回：
        FontProperties 对象
    """
    # 默认字体路径（从 visual/ 向上3级到 spot_micro_body_control_sim/fonts/）
    if font_relative_path is None:
        current_dir = os.path.dirname(os.path.abspath(__file__))
        font_relative_path = os.path.join('..', '..', '..', 'fonts', 'BabelStoneHan.ttf')
    
    # 计算绝对路径
    if not os.path.isabs(font_relative_path):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        font_path = os.path.normpath(os.path.join(current_dir, font_relative_path))
    else:
        font_path = font_relative_path
    
    if not os.path.exists(font_path):
        print(f"⚠️ 字体文件不存在: {font_path}")
        return fm.FontProperties()
    
    # 创建 FontProperties
    font_prop = fm.FontProperties(fname=font_path)
    font_name = font_prop.get_name()
    
    # 注册字体（兼容新旧版本）
    try:
        # 方法1：matplotlib 3.2+ 使用 addfont
        if hasattr(fm.fontManager, 'addfont'):
            fm.fontManager.addfont(font_path)
        else:
            # 方法2：旧版本 matplotlib，手动添加到字体列表
            try:
                fm.fontManager.ttflist.append(fm.FontEntry(
                    fname=font_path,
                    name=font_name,
                    style=font_prop.get_style(),
                    variant=font_prop.get_variant(),
                    weight=font_prop.get_weight(),
                    stretch=font_prop.get_stretch(),
                    size=font_prop.get_size()
                ))
            except Exception:
                pass
    except Exception:
        pass
    
    # 设置全局字体
    plt.rcParams['font.family'] = font_name
    plt.rcParams['axes.unicode_minus'] = False
    
    return font_prop
