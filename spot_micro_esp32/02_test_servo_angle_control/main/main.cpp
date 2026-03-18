/**
 * @file main.cpp
 * @brief 02_test_servo_angle_control - 舵机角度控制测试程序入口
 */

#include <cstdio>

extern "C" void app_main(void);

// 声明测试函数
extern void test_servo_angle_control();

void app_main(void) {
    test_servo_angle_control();
}
