/**
 * @file main.cpp
 * @brief 04_test_kinematics_controller - 运动学控制器测试程序入口
 */

#include <cstdio>

extern "C" void app_main(void);

// 声明测试函数
extern void test_kinematics_controller();

void app_main(void) {
    test_kinematics_controller();
}
