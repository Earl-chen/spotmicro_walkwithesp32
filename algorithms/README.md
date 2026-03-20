# 核心算法 (Algorithms)

本目录包含四足机器人控制的核心算法实现，使用 **C++** 编写。

---

## 📁 目录结构

```
algorithms/
├── kinematics/         # 运动学算法
│   ├── kinematics/     # FK/IK 求解器
│   ├── tests/          # 测试框架
│   └── kinematics_test # 可执行文件
│
└── gait/               # 步态算法
    ├── gait/           # 轨迹生成、步态控制
    ├── tests/          # 测试框架
    └── gait_test       # 可执行文件
```

---

## 🔄 开发流程

```
1️⃣ Python 原型
   spot_micro_body_control_sim/
   ↓ 算法验证通过
   
2️⃣ C++ 实现（本目录）
   algorithms/
   ├── kinematics/      ← 运动学 C++ 版本
   └── gait/            ← 步态 C++ 版本
   ↓ C++ 验证通过
   
3️⃣ ESP32 部署
   spot_micro_esp32/
   └── 复用 algorithms/ 中的 C++ 代码
```

---

## 🚀 快速使用

### 运动学测试

```bash
cd algorithms/kinematics

# 编译（如果需要）
make -f Makefile.kinematics_test

# 快速验证
./kinematics_test --quick

# 批量测试
./kinematics_test --batch

# 交互式菜单
./kinematics_test
```

### 步态测试

```bash
cd algorithms/gait

# 编译（如果需要）
make -f Makefile.gait_test

# 快速验证
./gait_test --quick

# 批量测试
./gait_test --batch

# 交互式菜单
./gait_test
```

---

## 🔗 与 ESP32 的关系

| 模块 | algorithms/ | spot_micro_esp32/ |
|------|-------------|-------------------|
| **运动学** | `kinematics/kinematics/` | `main/kinematics/` |
| **步态** | `gait/gait/` | `main/gait/`（未来） |

**代码复用方式**：
1. 在 `algorithms/` 中开发和验证算法
2. 验证通过后，复制到 `spot_micro_esp32/`
3. 在 ESP32 中集成到控制系统

---

## 📋 测试结果

### 运动学测试
- ✅ FK-IK 往返误差 < 0.001°
- ✅ 20个测试用例全部通过
- ✅ 与 Python 参考实现一致

### 步态测试
- ✅ 轨迹闭合（起点=终点）
- ✅ 占空比 25%/75%
- ✅ 每时刻 3 条腿支撑

---

## 📚 文档

- [运动学测试文档](kinematics/KINEMATICS_TEST_README.md)
- [步态测试文档](gait/README.md)

---

## 🛠️ 技术规格

| 项目 | 说明 |
|------|------|
| **语言** | C++17 |
| **编译器** | GCC 9+ |
| **依赖** | 标准C++库（无外部依赖） |
| **兼容性** | Linux / ESP32 |

---

## 📝 开发指南

### 修改算法流程

1. **修改 Python 原型**
   - 在 `spot_micro_body_control_sim/` 中修改
   - 运行测试验证

2. **更新 C++ 实现**
   - 在 `algorithms/` 中更新对应代码
   - 运行测试验证

3. **部署到 ESP32**
   - 复制验证通过的代码到 `spot_micro_esp32/`
   - 在实机上测试

### 添加新算法

1. 在对应目录下创建新文件
2. 编写算法实现
3. 编写测试用例
4. 运行测试验证
5. 更新文档

---

## ⚠️ 注意事项

1. **所有算法修改必须先通过测试验证**
2. **保持与 Python 版本的算法一致性**
3. **ESP32 部署前必须在 PC 上验证通过**
