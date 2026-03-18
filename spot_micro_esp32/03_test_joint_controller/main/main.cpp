/**
 * @file main.cpp
 * @brief 03_test_joint_controller - 关节控制器测试程序入口
 */

#include <cstdio>

extern "C" void app_main(void);

// 声明测试函数
extern void test_joint_controller();

void app_main(void) {
    test_joint_controller();
}
