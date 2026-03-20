# ESP-IDF 安装与 Walk 步态编译方案

**项目**：SpotMicro 四足机器人步态控制
**目标**：在当前机器安装 ESP-IDF 并编译 07_body_control_web 项目
**执行者**：技术人员
**预计时间**：60-90 分钟

---

## 一、前置条件检查

### 1.1 系统要求
- 操作系统：Linux (Ubuntu 20.04+ 或 Debian)
- 内存：至少 4GB RAM
- 磁盘：至少 10GB 可用空间
- 权限：sudo 权限

### 1.2 检查命令
```bash
# 检查系统版本
lsb_release -a

# 检查内存
free -h

# 检查磁盘
df -h ~

# 检查权限
sudo -v
```

---

## 二、安装方案

### 方案 A：官方安装（推荐）

**优点**：
- 官方支持
- 版本稳定
- 文档完善

**缺点**：
- 下载时间长（约 1-2GB）
- 编译时间长

**步骤**：

#### 2.1 安装依赖包（10分钟）
```bash
# 更新包列表
sudo apt-get update

# 安装必要依赖
sudo apt-get install -y \
    git \
    wget \
    flex \
    bison \
    gperf \
    python3 \
    python3-pip \
    python3-setuptools \
    cmake \
    ninja-build \
    ccache \
    libffi-dev \
    libssl-dev \
    dfu-util \
    libusb-1.0-0-dev \
    pkg-config
```

#### 2.2 克隆 ESP-IDF（20分钟）
```bash
# 创建目录
mkdir -p ~/esp
cd ~/esp

# 克隆 ESP-IDF（使用稳定版本 v5.3）
git clone --recursive --branch v5.3 https://github.com/espressif/esp-idf.git

# 进入目录
cd esp-idf
```

#### 2.3 安装 ESP-IDF（20分钟）
```bash
# 安装 ESP32 工具链
./install.sh esp32

# 配置环境变量（每次使用前需要执行）
source export.sh

# 验证安装
idf.py --version
# 预期输出：ESP-IDF v5.3
```

#### 2.4 永久配置环境变量（可选）
```bash
# 添加到 .bashrc
echo 'alias get_idf="source ~/esp/esp-idf/export.sh"' >> ~/.bashrc
source ~/.bashrc

# 使用方式
get_idf  # 等同于 source ~/esp/esp-idf/export.sh
```

---

### 方案 B：使用镜像源（国内推荐）

**优点**：
- 下载速度快
- 稳定性高

**步骤**：
```bash
# 使用清华大学镜像
mkdir -p ~/esp
cd ~/esp
git clone --recursive --branch v5.3 https://gitee.com/EspressifSystems/esp-idf.git

cd esp-idf

# 设置镜像源
export IDF_GITHUB_ASSETS="dl.espressif.com/github_assets"

# 安装
./install.sh esp32
source export.sh
```

---

## 三、编译计划

### 3.1 首次编译（预计 10-20分钟）

```bash
# 1. 配置环境
source ~/esp/esp-idf/export.sh

# 2. 进入项目目录
cd /home/robot-01/work/spotmicro/spot_micro_esp32/07_body_control_web

# 3. 清理（可选）
rm -rf build sdkconfig

# 4. 配置项目（可选，默认配置通常够用）
idf.py menuconfig

# 5. 编译
idf.py build

# 6. 查看编译结果
echo "编译状态: $?"
ls -lh build/*.bin
```

### 3.2 预期输出

**成功标志**：
```
Project build complete. To flash, run this command:
/home/robot-01/esp/esp-idf/components/esptool_py/esptool/esptool.py -p (PORT) -b 460800 --before default_reset --after hard_reset --chip esp32 write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x10000 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/body_control_web.bin
```

**生成的文件**：
- `build/bootloader/bootloader.bin` - 引导程序
- `build/partition_table/partition-table.bin` - 分区表
- `build/body_control_web.bin` - 主程序

---

## 四、可能遇到的问题与解决方案

### 4.1 头文件找不到

**错误示例**：
```
fatal error: gait/WalkGait.hpp: No such file or directory
```

**解决方案**：
```bash
# 检查文件是否存在
ls -la main/gait/WalkGait.hpp

# 检查 CMakeLists.txt 中的 INCLUDE_DIRS
grep "gait" main/CMakeLists.txt

# 如果缺失，添加到 INCLUDE_DIRS
# 在 main/CMakeLists.txt 中找到 INCLUDE_DIRS，添加 "gait"
```

### 4.2 命名空间错误

**错误示例**：
```
error: 'WalkGait' is not a member of 'Robot::Gait'
```

**解决方案**：
```bash
# 检查命名空间定义
grep "namespace Robot" main/gait/WalkGait.hpp

# 确保使用正确的命名空间
# 在 robot_bridge.cpp 中：using namespace Robot::Gait;
```

### 4.3 Python 版本问题

**错误示例**：
```
 ImportError: No module named 'cmake'
```

**解决方案**：
```bash
# 安装 Python 依赖
python3 -m pip install --user cmake ninja

# 或重新运行 ESP-IDF 安装
cd ~/esp/esp-idf
./install.sh esp32
```

### 4.4 权限问题

**错误示例**：
```
Permission denied: '/dev/ttyUSB0'
```

**解决方案**：
```bash
# 添加用户到 dialout 组
sudo usermod -a -G dialout $USER

# 重新登录生效
logout
# 重新登录
```

---

## 五、烧录到 ESP32（可选）

### 5.1 连接硬件
```bash
# 查看连接的设备
ls /dev/ttyUSB* /dev/ttyACM*

# 预期输出：/dev/ttyUSB0 或 /dev/ttyACM0
```

### 5.2 烧录命令
```bash
# 配置环境
source ~/esp/esp-idf/export.sh

# 烧录（替换 /dev/ttyUSB0 为实际端口）
idf.py -p /dev/ttyUSB0 flash

# 查看串口输出
idf.py -p /dev/ttyUSB0 monitor

# 退出 monitor：Ctrl + ]
```

### 5.3 一键烧录并监控
```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## 六、验证清单

### 6.1 安装验证
- [ ] `idf.py --version` 输出正确版本
- [ ] `which idf.py` 显示正确路径
- [ ] `python3 --version` 版本 >= 3.6

### 6.2 编译验证
- [ ] `idf.py build` 无错误完成
- [ ] `build/body_control_web.bin` 文件存在
- [ ] 文件大小合理（通常 1-2MB）

### 6.3 功能验证（烧录后）
- [ ] ESP32 启动正常
- [ ] 串口输出正常
- [ ] WiFi 连接成功
- [ ] Web 界面可访问

---

## 七、执行时间表

| 阶段 | 任务 | 预计时间 | 累计时间 |
|------|------|---------|---------|
| **准备** | 检查系统要求 | 5分钟 | 5分钟 |
| **安装依赖** | apt-get install | 10分钟 | 15分钟 |
| **克隆 ESP-IDF** | git clone | 20分钟 | 35分钟 |
| **安装工具链** | ./install.sh | 20分钟 | 55分钟 |
| **首次编译** | idf.py build | 20分钟 | 75分钟 |
| **问题修复** | 预留缓冲 | 15分钟 | 90分钟 |

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
- [ ] 生成 .bin 文件

---

## 九、联系支持

**遇到问题时提供以下信息**：

1. **系统信息**
   ```bash
   lsb_release -a
   uname -a
   ```

2. **完整错误日志**
   ```bash
   idf.py build > build.log 2>&1
   # 发送 build.log 文件
   ```

3. **ESP-IDF 版本**
   ```bash
   idf.py --version
   ```

4. **Python 版本**
   ```bash
   python3 --version
   pip3 list | grep -i esp
   ```

---

## 十、附录

### 10.1 有用的命令

```bash
# 清理编译
idf.py fullclean

# 重新配置
idf.py reconfigure

# 查看编译大小
idf.py size

# 查看组件大小
idf.py size-components

# 生成烧录脚本
idf.py gen-flash-cmd
```

### 10.2 参考链接

- ESP-IDF 官方文档：https://docs.espressif.com/projects/esp-idf/
- ESP-IDF GitHub：https://github.com/espressif/esp-idf
- 项目 GitHub：https://github.com/Earl-chen/spotmicro_walkwithesp32

---

**文档版本**：v1.0
**创建日期**：2026-03-20
**创建者**：太子（AI 助手）
**状态**：待执行

---

**执行者签名**：________________  **日期**：________________

**完成时间**：________________  **结果**：________________
