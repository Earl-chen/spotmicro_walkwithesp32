/**
 * @file gait_test.cpp
 * @brief 步态测试命令行工具
 * 
 * 仿照 kinematics_test_standalone 风格
 * 用于在 PC 上验证步态算法的正确性
 * 
 * 使用方式:
 *     ./gait_test --quick       # 快速验证
 *     ./gait_test --batch       # 批量测试
 *     ./gait_test --continuous  # 连续运行测试
 *     ./gait_test               # 交互式菜单
 */

#include <iostream>
#include <string>
#include <limits>
#include <cstdlib>
#include "gait/GaitTypes.hpp"
#include "gait/TrajectoryGenerator.hpp"
#include "gait/WalkGait.hpp"
#include "tests/GaitTestFramework.hpp"

using namespace Robot::Gait;
using namespace Robot::Testing;

/**
 * @brief 打印欢迎横幅
 */
void print_banner() {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "   四足机器狗步态测试程序" << std::endl;
    std::cout << "   SpotMicro Gait Test Suite" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
}

/**
 * @brief 打印主菜单
 */
void print_main_menu() {
    std::cout << "\n📋 测试选项菜单:" << std::endl;
    std::cout << "1. 快速验证测试" << std::endl;
    std::cout << "2. 轨迹生成器验证" << std::endl;
    std::cout << "3. Walk步态验证" << std::endl;
    std::cout << "4. 占空比验证" << std::endl;
    std::cout << "5. 支撑腿验证" << std::endl;
    std::cout << "6. 连续步态运行（5秒）" << std::endl;
    std::cout << "0. 退出程序" << std::endl;
    std::cout << "请选择 (0-6): ";
}

/**
 * @brief 打印帮助信息
 */
void print_help(const char* program_name) {
    std::cout << "使用方法:" << std::endl;
    std::cout << "  " << program_name << " [选项]" << std::endl;
    std::cout << std::endl;
    std::cout << "选项:" << std::endl;
    std::cout << "  -q, --quick       快速验证测试" << std::endl;
    std::cout << "  -b, --batch       批量预定义测试" << std::endl;
    std::cout << "  -c, --continuous  连续步态运行测试（5秒）" << std::endl;
    std::cout << "  -h, --help        显示此帮助信息" << std::endl;
    std::cout << "  无参数            启动交互式菜单" << std::endl;
}

/**
 * @brief 交互式菜单
 */
void interactive_menu() {
    GaitTestFramework framework;
    
    int choice;
    do {
        print_main_menu();
        
        while (!(std::cin >> choice)) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "无效输入，请输入数字 (0-6): ";
        }
        
        switch (choice) {
            case 1:
                framework.run_quick_test();
                break;
            case 2:
                framework.test_trajectory_generator();
                break;
            case 3:
                framework.test_walk_gait();
                break;
            case 4:
                framework.test_duty_cycle();
                break;
            case 5:
                framework.test_support_legs();
                break;
            case 6:
                framework.test_continuous_gait();
                break;
            case 0:
                std::cout << "\n👋 感谢使用四足机器狗步态测试程序！" << std::endl;
                break;
            default:
                std::cout << "❌ 无效选择，请重新输入。" << std::endl;
                break;
        }
        
        if (choice != 0) {
            std::cout << "\n按回车键继续...";
            std::cin.ignore();
            std::cin.get();
        }
        
    } while (choice != 0);
}

int main(int argc, char* argv[]) {
    print_banner();
    
    // 检查命令行参数
    if (argc > 1) {
        std::string arg = argv[1];
        
        if (arg == "--quick" || arg == "-q") {
            std::cout << "\n运行快速验证模式..." << std::endl;
            GaitTestFramework framework;
            framework.run_quick_test();
            return 0;
            
        } else if (arg == "--batch" || arg == "-b") {
            std::cout << "\n运行批量测试模式..." << std::endl;
            GaitTestFramework framework;
            framework.run_batch_test();
            return 0;
            
        } else if (arg == "--continuous" || arg == "-c") {
            std::cout << "\n运行连续步态测试模式..." << std::endl;
            GaitTestFramework framework;
            framework.test_continuous_gait();
            return 0;
            
        } else if (arg == "--help" || arg == "-h") {
            print_help(argv[0]);
            return 0;
            
        } else {
            std::cout << "❌ 未知选项: " << arg << std::endl;
            print_help(argv[0]);
            return 1;
        }
    }
    
    // 无参数，启动交互式菜单
    try {
        interactive_menu();
    } catch (const std::exception& e) {
        std::cerr << "❌ 程序运行出错: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
