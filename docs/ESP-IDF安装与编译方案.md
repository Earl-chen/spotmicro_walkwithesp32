# ESP-IDF 安装与编译方案

## 概述

本文档提供完整的 ESP-IDF 安装指南，用于编译 `spot_micro_esp32/07_body_control_web` 项目。

**创建日期**：2026-03-20
**创建者**：太子（AI 助手）
**项目**：SpotMicro 四足机器人
**状态**：待执行

---

## 一、前置条件

### 系统要求
- **操作系统**：Linux (Ubuntu 20.04+ 推荐)
- **内存**：≥ 8GB RAM
- **磁盘空间**：≥ 10GB 可用空间
- **权限**：sudo 权限
- **网络**：稳定的互联网连接

### 检查命令
```bash
# 检查系统
uname -a
cat /etc/os-release

# 检查内存
free -h

# 检查磁盘
df -h
```

---

## 二、安装方案

### 方案 A：官方安装（推荐）

**优点**：官方支持，稳定性高
**缺点**：下载较慢（国外服务器）
**时间**：60-90 分钟

#### 步骤 1：安装依赖

```bash
sudo apt-get update
sudo apt-get install -y git wget flex bison gperf python3 python3-pip \
    python3-venv cmake ninja-build ccache libffi-dev libssl-dev \
    dfu-util libusb-1.0-0
```

**预期输出**：
```
Reading package lists... Done
Building dependency tree... Done
...
```

#### 步骤 2：克隆 ESP-IDF

```bash
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v5.3  # 使用稳定版本
git submodule update --init --recursive
```

**预期输出**：
```
Cloning into 'esp-idf'...
remote: Enumerating objects: 100%, done.
...
```

**时间**：10-20 分钟（取决于网络速度）

#### 步骤 3：安装工具链

```bash
cd ~/esp/esp-idf
./install.sh esp32
```

**预期输出**：
```
Setting up Python environment...
Installing ESP-IDF tools...
...
All done! You can now run:
. ~/esp/esp-idf/export.sh
```

**时间**：15-25 分钟

#### 步骤 4：配置环境变量

```bash
# 临时配置（当前终端）
source ~/esp/esp-idf/export.sh

# 永久配置（添加到 ~/.bashrc）
echo 'alias get_idf=". ~/esp/esp-idf/export.sh"' >> ~/.bashrc
echo 'alias idf="idf.py"' >> ~/.bashrc

# 重新加载配置
source ~/.bashrc
```

**验证安装**：
```bash
idf.py --version
# 应该输出：ESP-IDF v5.3
```

---

### 方案 B：国内镜像源（速度快）

**优点**：下载速度快
**缺点**：可能不是最新版本
**时间**：30-45 分钟

#### 步骤 1：使用国内镜像

```bash
mkdir -p ~/esp
cd ~/esp

# 使用 Gitee 镜像
git clone --recursive https://gitee.com/EspressifSystems/esp-idf.git
cd esp-idf
git checkout v5.3
```

#### 步骤 2：配置国内下载源

```bash
# 设置环境变量
export IDF_GITHUB_ASSETS_URL="https://dl.espressif.com/github_assets"
export IDF_TOOLS_PATH=~/esp/tools

# 安装
./install.sh esp32
```

**预期输出**：
```
Setting up Python environment...
Installing ESP-IDF tools...
Downloading from mirrors...
```

**时间**：10-15 分钟

---

## 三、编译项目

### 步骤 1：进入项目目录

```bash
cd /home/robot-01/work/spotmicro/spot_micro_esp32/07_body_control_web
```

### 步骤 2：配置环境（每次编译前执行）

```bash
source ~/esp/esp-idf/export.sh
# 或使用别名
get_idf
```

### 步骤 3：首次编译

```bash
idf.py build
```

**预期输出**：
```
Executing action: all
...
[100%] Built target __idf_libs
Project build complete. To flash, run this command:
idf.py -p (PORT) flash
```

**时间**：15-25 分钟（首次编译较慢）

### 步骤 4：后续编译（增量编译）

```bash
# 增量编译（快）
idf.py build

# 完全重新编译
idf.py fullclean
idf.py build
```

**时间**：2-5 分钟（增量编译）

---

## 四、常见问题与解决方案

### 问题 1：找不到 `gait/WalkGait.hpp`

**错误信息**：
```
fatal error: gait/WalkGait.hpp: No such file or directory
```

**原因**：CMakeLists.txt 中未包含 `gait` 目录

**解决方案**：
```bash
# 检查文件是否存在
ls main/gait/WalkGait.hpp

# 检查 CMakeLists.txt
grep "gait" main/CMakeLists.txt

# 如果没有，手动添加
# 编辑 main/CMakeLists.txt，在 INCLUDE_DIRS 中添加 "gait"
```

**预期输出**：
```
main/gait/WalkGait.hpp
```

---

### 问题 2：命名空间错误

**错误信息**：
```
error: 'WalkGait' is not a member of 'Robot::Gait'
```

**原因**：命名空间定义不匹配

**解决方案**：
```bash
# 检查命名空间定义
grep "namespace Robot" main/gait/WalkGait.hpp

# 确保使用正确的命名空间
# 在 .cpp 文件中添加：
using namespace Robot::Gait;
```

---

### 问题 3：链接错误

**错误信息**：
```
undefined reference to `LegKinematics::inverse_kinematics(Vector3 const&)'
```

**原因**：函数声明但未实现

**解决方案**：
```bash
# 检查函数实现
grep "inverse_kinematics" main/kinematics/LegKinematics.cpp

# 如果没有实现，检查是否有其他可用函数
# 可能需要使用不同的 IK 函数名
```

---

### 问题 4：Python 版本不兼容

**错误信息**：
```
ModuleNotFoundError: No module named 'cmake'
```

**原因**：ESP-IDF Python 环境未激活

**解决方案**：
```bash
# 重新激活环境
source ~/esp/esp-idf/export.sh

# 验证 Python
idf.py --version
which python3

# 如果仍然失败，重新安装
cd ~/esp/esp-idf
./install.sh esp32
```

---

## 五、烧录到 ESP32（可选）

### 步骤 1：连接 ESP32

```bash
# 检查设备
ls /dev/ttyUSB* /dev/ttyACM*

# 添加权限（如果需要）
sudo usermod -a -G dialout $USER
# 注销并重新登录
```

### 步骤 2：配置串口

```bash
idf.py -p /dev/ttyUSB0 monitor
```

### 步骤 3：烧录并监控

```bash
# 烧录并打开监控
idf.py -p /dev/ttyUSB0 flash monitor

# 或分步执行
idf.py -p /dev/ttyUSB0 flash
idf.py -p /dev/ttyUSB0 monitor
```

**预期输出**：
```
Connecting........
Detecting chip type... ESP32
...
Hash of data verified.
Leaving...
Hard resetting via RTS pin...
```

---

## 六、验证清单

### 执行前检查
- [ ] 系统是 Linux (Ubuntu 20.04+)
- [ ] 有 sudo 权限
- [ ] 磁盘空间 > 10GB
- [ ] 网络连接正常

### 安装后检查
- [ ] `idf.py --version` 输出正确版本
- [ ] `which idf.py` 显示正确路径
- [ ] Python 环境正常

### 编译后检查
- [ ] 无编译错误
- [ ] 生成 `.bin` 文件
- [ ] 生成 `build/` 目录

### 烧录后检查
- [ ] 串口连接成功
- [ ] 烧录进度 100%
- [ ] 设备正常启动

---

## 七、时间预估

| 阶段 | 任务 | 预计时间 | 累计时间 |
|------|------|----------|----------|
| 准备 | 检查系统要求 | 5分钟 | 5分钟 |
| 安装依赖 | apt-get install | 10分钟 | 15分钟 |
| 克隆 ESP-IDF | git clone | 20分钟 | 35分钟 |
| 安装工具链 | ./install.sh | 20分钟 | 55分钟 |
| 首次编译 | idf.py build | 20分钟 | 75分钟 |
| 问题修复 | 预留缓冲 | 15分钟 | 90分钟 |

**总计**：60-90 分钟

---

## 八、执行者检查清单

### 执行前检查
- [ ] 系统是 Linux (Ubuntu 20.04+)
- [ ] 有 sudo 权限
- [ ] 磁盘空间 > 10GB
- [ ] 网络连接正常

### 执行中检查
- [ ] 每个步骤完成后检查输出
- [ ] 遇到错误先查看"问题与解决方案"章节
- [ ] 记录所有错误信息

### 执行后检查
- [ ] `idf.py --version` 正常
- [ ] `idf.py build` 成功
- [ ] 生成 `.bin` 文件

---

## 九、联系支持

**遇到问题时提供以下信息**：
1. 系统信息：`uname -a`
2. ESP-IDF 版本：`idf.py --version`
3. 完整错误日志
4. 项目路径：`/home/robot-01/work/spotmicro/spot_micro_esp32/07_body_control_web`

---

## 十、参考链接

- **ESP-IDF 官方文档**：https://docs.espressif.com/projects/esp-idf/
- **ESP-IDF GitHub**：https://github.com/espressif/esp-idf
- **项目 GitHub**：https://github.com/Earl-chen/spotmicro_walkwithesp32

---

**文档版本**：v1.0
**创建日期**：2026-03-20
**创建者**：太子（AI 助手）
**状态**：待执行
