/**
 * @file simple_test_main.cpp
 * @brief ESP32四足机器人模块化测试系统主程序
 *
 * 📋 测试模式总览：
 *
 * 🔧 测试模式 0: PCA9685硬件驱动测试
 *    ✅ I2C通信稳定性验证
 *    ✅ PWM输出精度测试
 *    ✅ 舵机控制响应测试
 *    ✅ 交互式PWM调试
 *    📊 测试目标: 验证硬件层基础功能
 *
 * 📐 测试模式 1: ServoDriver精度测试
 *    ✅ 角度到PWM转换精度验证
 *    ✅ 基于实际标定数据的控制测试
 *    ✅ 角度插值算法准确性测试
 *    ✅ 配置管理器功能验证
 *    📊 测试目标: 验证舵机驱动层功能
 *
 * 🎯 测试模式 2: 基于实际数据的角度控制测试
 *    ✅ ServoDriver交互式角度控制
 *    ✅ 0-180度全范围控制验证
 *    ✅ 舵机响应特性分析
 *    ✅ 实际标定数据精度验证
 *    📊 测试目标: 验证实际应用中的角度控制
 *
 * 🤖 测试模式 3: JointController精度测试
 *    ✅ 关节到舵机映射关系验证
 *    ✅ 反向映射精度测试
 *    ✅ 关节角度限制验证
 *    ✅ 预设姿势功能测试
 *    📊 测试目标: 验证关节控制层功能
 *
 * 🎮 测试模式 4: 交互式关节控制测试
 *    ✅ 四足机器人运动学映射验证
 *    ✅ 机器人坐标系转换准确性
 *    ✅ 交互式关节调试
 *    ✅ 实时映射关系验证
 *    📊 测试目标: 验证完整的关节控制系统
 *
 * 🌊 测试模式 5: 平滑运动控制测试
 *    ✅ 姿态平滑插值验证 (PoseInterpolator)
 *    ✅ 关节平滑输出测试 (JointSmoother)
 *    ✅ 统一平滑运动控制器
 *    ✅ 性能基准测试和动作协调性
 *    📊 测试目标: 验证平滑运动控制系统
 * 🔄 测试顺序建议：
 * 1. 模式0 → 验证硬件基础
 * 2. 模式1 → 验证舵机驱动
 * 3. 模式2 → 验证角度控制
 * 4. 模式3 → 验证关节映射
 * 5. 模式4 → 综合功能测试
 * 6. 模式5 → 平滑运动控制测试
 *
 * 📊 测试数据：
 * - 12个舵机的实际标定数据
 * - 四足机器人精确映射关系
 * - 机器人坐标系定义
 * - 关节角度限制范围
 *
 * 💡 修改app_main()中的函数调用来切换测试模式
 */

#include <stdio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// 定义测试模式宏（与CMakeLists.txt中的TEST_MODE保持一致）
#define PWM_CONTROLLER_TEST_MODE      0
#define SERVOANGLE_CONTROLLER_TEST_MODE  1
#define JOINT_CONTROLLER_TEST_MODE    2
#define KINEMATICS_CONTROLLER_TEST_MODE  3
#define SMOOTH_MOTION_TEST_MODE       4

// 根据TEST_MODE宏定义声明对应的函数
#if TEST_MODE == PWM_CONTROLLER_TEST_MODE
    // 测试模式 0: PCA9685硬件驱动测试
    extern void test_pca9685_driver();
    void test_mode_0_pca9685_hardware();

#elif TEST_MODE == SERVOANGLE_CONTROLLER_TEST_MODE
    // 测试模式 1: ServoDriver角度控制测试
    extern void test_servo_angle_control();
    void test_mode_1_servo_angle_control();

#elif TEST_MODE == JOINT_CONTROLLER_TEST_MODE
    // 测试模式 2: 关节控制器测试
    extern void test_joint_controller();
    void test_mode_2_joint_controller();

#elif TEST_MODE == KINEMATICS_CONTROLLER_TEST_MODE
    // 测试模式 3: 运动学控制器测试
    extern void test_kinematics_controller();
    void test_mode_3_kinematics_controller();

#elif TEST_MODE == SMOOTH_MOTION_TEST_MODE
    // 测试模式 4: 平滑运动控制测试
    extern "C" void test_smooth_motion_controller();
    void test_mode_4_smooth_motion_controller();
    
#else
    #error "未定义的测试模式"
#endif



extern "C" void app_main() {
    printf("\n=== ESP32四足机器人模块化测试系统 ===\n");

#if TEST_MODE == PWM_CONTROLLER_TEST_MODE
    printf("🔧 当前测试模式: PWM_CONTROLLER_TEST_MODE (PCA9685硬件驱动测试)\n");
    printf("📋 测试内容:\n");
    printf("  ✅ I2C通信稳定性验证\n");
    printf("  ✅ PWM输出精度测试\n");
    printf("  ✅ 舵机控制响应测试\n");
    printf("  ✅ 交互式PWM调试\n\n");
    printf("🚀 启动测试...\n\n");

    test_mode_0_pca9685_hardware();

#elif TEST_MODE == SERVOANGLE_CONTROLLER_TEST_MODE
    printf("📐 当前测试模式: SERVOANGLE_CONTROLLER_TEST_MODE (舵机角度控制测试)\n");
    printf("📋 测试内容:\n");
    printf("  ✅ 角度到PWM转换精度验证\n");
    printf("  ✅ 基于实际标定数据的控制测试\n");
    printf("  ✅ 角度插值算法准确性测试\n");
    printf("  ✅ 配置管理器功能验证\n\n");
    printf("🚀 启动测试...\n\n");

    test_mode_1_servo_angle_control();

#elif TEST_MODE == JOINT_CONTROLLER_TEST_MODE
    printf("🎯 当前测试模式: JOINT_CONTROLLER_TEST_MODE (关节控制器测试)\n");
    printf("📋 测试内容:\n");
    printf("  ✅ 关节到舵机映射关系验证\n");
    printf("  ✅ 反向映射精度测试\n");
    printf("  ✅ 关节角度限制验证\n");
    printf("  ✅ 预设姿势功能测试\n\n");
    printf("🚀 启动测试...\n\n");

    test_mode_2_joint_controller();

#elif TEST_MODE == KINEMATICS_CONTROLLER_TEST_MODE
    printf("🦾 当前测试模式: KINEMATICS_CONTROLLER_TEST_MODE (运动学控制器测试)\n");
    printf("📋 测试内容:\n");
    printf("  ✅ 正向/逆向运动学计算\n");
    printf("  ✅ 坐标系变换验证\n");
    printf("  ✅ 四腿协调控制\n");
    printf("  ✅ 机体位姿控制\n");
    printf("  ✅ 交互式运动学调试\n\n");
    printf("🚀 启动测试...\n\n");

    test_mode_3_kinematics_controller();

#elif TEST_MODE == SMOOTH_MOTION_TEST_MODE
    printf("🌊 当前测试模式: SMOOTH_MOTION_TEST_MODE (平滑运动控制测试)\n");
    printf("📋 测试内容:\n");
    printf("  ✅ 姿态平滑插值验证\n");
    printf("  ✅ 舵机平滑输出测试\n");
    printf("  ✅ 统一平滑运动控制\n");
    printf("  ✅ 性能基准测试\n\n");
    printf("🚀 启动测试...\n\n");

    test_mode_4_smooth_motion_controller();

#else
    #error "未支持的测试模式"
#endif

    printf("\n💡 要切换测试模式，请修改CMakeLists.txt中的TEST_MODE值:\n");
    printf("   set(TEST_MODE 0)  # PCA9685硬件测试\n");
    printf("   set(TEST_MODE 1)  # 舵机角度控制测试\n");
    printf("   set(TEST_MODE 2)  # 关节控制器测试\n");
    printf("   set(TEST_MODE 3)  # 运动学控制器测试\n");
    printf("   set(TEST_MODE 4)  # 平滑运动控制测试\n");
    printf("然后重新编译: idf.py build\n\n");
}

// ==================== 测试模式实现 ====================

#if TEST_MODE == PWM_CONTROLLER_TEST_MODE
void test_mode_0_pca9685_hardware() {
    printf("🔧 === 测试模式 0: PCA9685硬件驱动测试 ===\n");
    printf("📋 测试内容:\n");
    printf("  - PCA9685硬件初始化 (100Hz频率)\n");
    printf("  - I2C通信验证\n");
    printf("  - 交互式PWM控制\n");
    printf("  - 实时舵机角度调试\n\n");

    printf("💡 预期结果:\n");
    printf("  - 初始化成功\n");
    printf("  - 进入交互模式\n");
    printf("  - 舵机响应PWM命令 (350≈90°)\n\n");

    // 调用PCA9685测试
    test_pca9685_driver();
}

#elif TEST_MODE == SERVOANGLE_CONTROLLER_TEST_MODE
void test_mode_1_servo_angle_control() {
    printf("📐 === 测试模式 1: ServoDriver角度控制测试 ===\n");
    printf("📋 测试内容:\n");
    printf("  - 基于您实际测试数据的舵机控制\n");
    printf("  - 交互式角度输入 (支持小数角度)\n");
    printf("  - 自动PWM计算和线性插值\n");
    printf("  - 单个/批量舵机角度设置\n");
    printf("  - 快速测试序列 (0°→90°→180°)\n\n");

    printf("💡 预期结果:\n");
    printf("  - 角度精确转换为PWM值\n");
    printf("  - 舵机准确移动到指定角度\n");
    printf("  - 交互响应流畅\n\n");

    printf("📈 使用您的实际测试数据:\n");
    printf("  - 舵机0-2 (左前腿): 170-615/625-1060/1080\n");
    printf("  - 舵机3-5 (右前腿): 240/170/185-685/630/635-1130/1090/1085\n");
    printf("  - 舵机6-8 (左后腿): 190/200/195-655-1120/1110\n");
    printf("  - 舵机9-11 (右后腿): 210/180-670/630-1130/1080\n\n");

    // 调用角度控制测试
    test_servo_angle_control();
}

#elif TEST_MODE == JOINT_CONTROLLER_TEST_MODE
void test_mode_2_joint_controller() {
    printf("🎯 === 测试模式 2: 关节控制器测试 ===\n");
    printf("📋 测试内容:\n");
    printf("  - JointController初始化和映射配置\n");
    printf("  - 关节到舵机角度转换精度验证\n");
    printf("  - 反向映射精度测试\n");
    printf("  - 关节角度限制验证\n");
    printf("  - 预设姿势测试\n\n");

    printf("💡 预期结果:\n");
    printf("  - 映射转换精度在容差范围内\n");
    printf("  - 反向映射误差<1度\n");
    printf("  - 关节限制正确执行\n");
    printf("  - 自动化测试全部通过\n\n");

    printf("📊 测试基于您的映射关系:\n");
    printf("  - 左前腿: 舵机0(髋侧摆) 舵机1(髋俯仰) 舵机2(膝俯仰)\n");
    printf("  - 右前腿: 舵机3(髋侧摆) 舵机4(髋俯仰) 舵机5(膝俯仰)\n");
    printf("  - 左后腿: 舵机6(髋侧摆) 舵机7(髋俯仰) 舵机8(膝俯仰)\n");
    printf("  - 右后腿: 舵机9(髋侧摆) 舵机10(髋俯仰) 舵机11(膝俯仰)\n\n");

    // 调用关节控制器测试
    test_joint_controller();
}

#elif TEST_MODE == KINEMATICS_CONTROLLER_TEST_MODE
void test_mode_3_kinematics_controller() {
    printf("🦾 === 测试模式 3: 运动学控制器测试 ===\n");
    printf("📋 测试内容:\n");
    printf("  - SpotMicro单腿运动学求解器\n");
    printf("  - 正向/逆向运动学计算验证\n");
    printf("  - 3D坐标变换和旋转矩阵\n");
    printf("  - 四腿协调控制算法\n");
    printf("  - 机体位姿控制系统\n");
    printf("  - 交互式运动学调试终端\n\n");

    printf("💡 预期结果:\n");
    printf("  - 运动学计算精度验证通过\n");
    printf("  - 坐标变换误差在容差范围内\n");
    printf("  - 工作空间边界检查正确\n");
    printf("  - 交互式命令响应流畅\n\n");

    printf("📐 基于SpotMicro几何参数:\n");
    printf("  - L1=60.5mm L2=10mm L3=111.126mm L4=118.5mm\n");
    printf("  - 身体长度=207.5mm 宽度=78mm\n");
    printf("  - 支持0-180°关节角度范围\n");
    printf("  - 工作空间: 最大伸展≈229mm\n\n");

    printf("🎮 可用交互命令:\n");
    printf("  - test [all|fk|ik|transform|workspace] : 自动化测试\n");
    printf("  - leg <leg_id> <x> <y> <z> : 设置腿部位置\n");
    printf("  - pose <x> <y> <z> <roll> <pitch> <yaw> : 机体位姿\n");
    printf("  - action [stand|sit|crouch|tilt] : 预设动作\n");
    printf("  - info [legs|workspace|geometry] : 状态查询\n");
    printf("  - calc <leg_id> <hip_side> <hip_pitch> <knee_pitch> : 运动学计算\n\n");

    // 创建大堆栈任务来运行运动学控制器，避免栈溢出
    xTaskCreate([](void* pvParameters) {
        test_kinematics_controller();
        vTaskDelete(NULL);
    }, "kinematics_task", 16384, NULL, 5, NULL);  // 16KB堆栈
}

#elif TEST_MODE == SMOOTH_MOTION_TEST_MODE
void test_mode_4_smooth_motion_controller() {
    printf("🌊 === 测试模式 4: 平滑运动控制测试 ===\n");
    printf("📋 测试内容:\n");
    printf("  - 姿态平滑插值系统 (PoseInterpolator)\n");
    printf("  - 关节平滑输出控制 (JointSmoother)\n");
    printf("  - 统一平滑运动控制器\n");
    printf("  - 性能基准测试和验证\n\n");

    printf("💡 预期结果:\n");
    printf("  - 姿态过渡平滑无突变\n");
    printf("  - 舵机运动连续无抖动\n");
    printf("  - 整体动作协调自然\n\n");

    printf("🎯 测试基于新增功能.txt的三个核心功能:\n");
    printf("  1. iterate_to_position → PoseInterpolator\n");
    printf("  2. set_leg_servos_in_steps → JointSmoother\n");

    printf("📊 配置参数:\n");
    printf("  - MOTION_STEP_ANGLE: 0.05rad (≈2.86°)\n");
    printf("  - MOTION_STEP_MOVEMENT: 2.0mm\n");
    printf("  - SERVO_STEP_ANGLE: 2.0°\n");
    printf("  - 更新频率: 50Hz (20ms)\n\n");

    // 创建大堆栈任务来运行平滑运动控制器，避免栈溢出
    xTaskCreate([](void* pvParameters) {
        test_smooth_motion_controller();
        vTaskDelete(NULL);
    }, "smooth_motion_task", 20480, NULL, 5, NULL);  // 20KB堆栈 (比运动学稍大)
}

#endif
