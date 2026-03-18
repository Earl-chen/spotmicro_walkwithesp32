#!/bin/bash
set -e

# 设置ESP-IDF环境
echo "=== 设置ESP-IDF环境 ==="
source /home/cm/esp/v5.5/esp-idf/export.sh 2>&1 | grep -E "Done|Error|编译"

# 进入项目目录
cd /home/cm/1_work/2_Robot/2_Quadruped_robot/0_SpotMicroAI/Ai_Code/spot_micro_esp32/test_pca9685_standalone

# 检查编译器
echo "=== 检查编译器 ==="
which xtensa-esp32-elf-gcc || echo "编译器未找到"

# 设置目标并编译
echo "=== 设置目标芯片 ==="
idf.py set-target esp32 2>&1 | tail -3

echo "=== 开始编译 ==="
idf.py build 2>&1 | tail -50

echo "=== 编译完成 ==="
