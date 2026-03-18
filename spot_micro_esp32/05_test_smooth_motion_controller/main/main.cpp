/**
 * @file main.cpp
 * @brief 05_test_smooth_motion_controller - 平滑运动控制器测试程序入口
 */

#include <cstdio>

extern "C" void app_main(void);

// 声明测试函数
extern void test_smooth_motion_controller();

void app_main(void) {
    test_smooth_motion_controller();
}
