#!/usr/bin/env python3
"""
中文字体配置模块 - 统一配置 BabelStoneHan.ttf

使用方法：
    from chinese_font_config import setup_chinese_font
    setup_chinese_font()
"""

import os
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm


def setup_chinese_font(verbose=True):
    """
    配置中文字体 - 直接使用 BabelStoneHan.ttf
    
    参数：
        verbose: 是否输出详细信息
    
    返回：
        FontProperties 对象
    """
    # 字体文件路径（从当前文件位置向上查找）
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 尝试多个可能的字体路径（仅相对路径）
    font_candidates = [
        # 从 tests/ 向上2级到 spot_micro_body_control_sim/fonts/
        os.path.join(current_dir, '..', '..', 'fonts', 'BabelStoneHan.ttf'),
    ]
    
    font_path = None
    for candidate in font_candidates:
        candidate_abs = os.path.abspath(candidate)
        if os.path.exists(candidate_abs):
            font_path = candidate_abs
            break
    
    if font_path is None:
        if verbose:
            print("⚠️ 未找到 BabelStoneHan.ttf，使用默认字体")
        return fm.FontProperties()
    
    if verbose:
        print(f"✅ 找到字体: {font_path}")
    
    # 清除字体缓存
    try:
        fm._load_fontmanager(try_read_cache=False)
    except:
        pass
    
    # 使用 addfont 方法注册字体
    if hasattr(fm.fontManager, 'addfont'):
        fm.fontManager.addfont(font_path)
        if verbose:
            print(f"✅ 字体已注册")
    
    # 创建 FontProperties
    font_prop = fm.FontProperties(fname=font_path)
    font_name = font_prop.get_name()
    
    if verbose:
        print(f"✅ 字体名称: {font_name}")
    
    # 设置全局字体
    plt.rcParams['font.family'] = font_name
    plt.rcParams['font.sans-serif'] = [font_name, 'DejaVu Sans', 'Arial Unicode MS']
    plt.rcParams['axes.unicode_minus'] = False
    
    return font_prop


# 测试代码
if __name__ == '__main__':
    print("=" * 60)
    print("字体配置测试")
    print("=" * 60)
    
    font_prop = setup_chinese_font()
    
    if font_prop:
        print("\n✅ 字体配置成功！")
        print(f"字体属性: {font_prop}")
    else:
        print("\n❌ 字体配置失败")
