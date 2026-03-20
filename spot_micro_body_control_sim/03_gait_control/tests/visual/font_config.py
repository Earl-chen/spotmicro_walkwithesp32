#!/usr/bin/env python3
"""
字体配置模块 - 直接使用 BabelStoneHan.ttf

使用方法：
    from font_config import setup_chinese_font
    setup_chinese_font()
"""

import os
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm


def setup_chinese_font():
    """
    配置中文字体 - 直接使用 BabelStoneHan.ttf
    """
    # 字体文件路径（从当前文件位置向上查找）
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 尝试多个可能的字体路径（仅相对路径）
    font_candidates = [
        # 从 tests/visual/ 向上3级到 spot_micro_body_control_sim/fonts/
        os.path.join(current_dir, '..', '..', '..', 'fonts', 'BabelStoneHan.ttf'),
        # 从 tests/ 向上2级到 spot_micro_body_control_sim/fonts/
        os.path.join(current_dir, '..', '..', 'fonts', 'BabelStoneHan.ttf'),
    ]
    
    font_path = None
    for candidate in font_candidates:
        candidate_abs = os.path.abspath(candidate)
        if os.path.exists(candidate_abs):
            font_path = candidate_abs
            print(f"✅ 找到字体: {font_path}")
            break
    
    if font_path is None:
        print("⚠️ 未找到 BabelStoneHan.ttf，使用默认字体")
        return None
    
    # 方法1：使用 fontManager.addfont（推荐）
    try:
        if hasattr(fm.fontManager, 'addfont'):
            fm.fontManager.addfont(font_path)
            print("✅ 使用 addfont 方法注册字体")
    except Exception as e:
        print(f"⚠️ addfont 失败: {e}")
    
    # 方法2：直接创建 FontProperties
    font_prop = fm.FontProperties(fname=font_path)
    font_name = font_prop.get_name()
    print(f"✅ 字体名称: {font_name}")
    
    # 方法3：设置全局字体
    plt.rcParams['font.family'] = font_name
    plt.rcParams['axes.unicode_minus'] = False
    
    # 方法4：备用设置
    plt.rcParams['font.sans-serif'] = [font_name, 'DejaVu Sans']
    
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
