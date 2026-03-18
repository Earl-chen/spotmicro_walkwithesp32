#!/bin/bash
# ESP32 编译脚本
# 用法: ./build.sh [目标] [串口]
# 示例: ./build.sh esp32 /dev/ttyUSB0

set -e

# 设置ESP-IDF环境
echo "🔧 设置ESP-IDF环境..."
source /home/cm/esp/v5.5/esp-idf/export.sh 2>&1 | grep -E "Done|Error" || true

# 进入项目目录
cd "$(dirname "$0")"
PROJECT_DIR=$(pwd)
echo "📁 项目目录: $PROJECT_DIR"

# 检查idf.py
if ! command -v idf.py &> /dev/null; then
    echo "❌ idf.py 未找到，尝试直接使用..."
    alias idf.py='/home/cm/esp/v5.5/esp-idf/tools/idf.py'
fi

# 设置目标芯片
TARGET="${1:-esp32}"
echo "🎯 目标芯片: $TARGET"

# 设置串口
PORT="${2:-/dev/ttyUSB0}"

# 编译
echo "🔨 开始编译..."
idf.py set-target "$TARGET" 2>&1 | tail -3 || true
idf.py build 2>&1

echo "✅ 编译完成!"
