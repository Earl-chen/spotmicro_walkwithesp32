#include "tests/KinematicsTestFramework.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <sstream>
#include <algorithm>

using namespace Robot::Testing;
using namespace Robot;

void printWelcome() {
    std::cout << "\n";
    std::cout << "========================================" << std::endl;
    std::cout << "   四足机器狗运动学测试程序" << std::endl;
    std::cout << "   SpotMicro Kinematics Test Suite" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
}

void printMainMenu() {
    std::cout << "📋 测试选项菜单:" << std::endl;
    std::cout << "1. FK-IK 往返精度测试" << std::endl;
    std::cout << "2. 工作空间边界测试" << std::endl;
    std::cout << "3. 批量预定义测试" << std::endl;
    std::cout << "4. 交互式单个测试" << std::endl;
    std::cout << "5. 查看腿部信息" << std::endl;
    std::cout << "6. 查看工作空间信息" << std::endl;
    std::cout << "0. 退出程序" << std::endl;
    std::cout << "请选择 (0-6): ";
}

int getUserChoice() {
    int choice;
    while (!(std::cin >> choice)) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "无效输入，请输入数字 (0-6): ";
    }
    return choice;
}

void runFkIkRoundtripTest(KinematicsTestFramework& framework) {
    std::cout << "\n🔄 执行 FK-IK 往返测试..." << std::endl;

    auto results = framework.executeFkIkRoundtripTest();
    auto report = framework.generateReport(TestType::FK_IK_ROUNDTRIP);

    framework.printReportSummary(report);

    std::cout << "是否保存测试报告? (y/n): ";
    char save_choice;
    std::cin >> save_choice;
    if (save_choice == 'y' || save_choice == 'Y') {
        framework.saveReport(report, "txt");
    }
}

void runBatchTest(KinematicsTestFramework& framework) {
    std::cout << "\n📦 执行批量预定义测试..." << std::endl;

    framework.startTestingSession();

    auto test_cases = framework.createPredefinedTestCases();
    std::vector<TestResult> results;

    for (const auto& test_case : test_cases) {
        std::cout << "执行测试: " << test_case.name << std::endl;
        auto result = framework.executeTestCase(test_case);
        results.push_back(result);

        if (result.success) {
            std::cout << "✓ 通过" << std::endl;
        } else {
            std::cout << "❌ 失败: " << result.error_message << std::endl;
        }
    }

    framework.endTestingSession();

    auto report = framework.generateReport(TestType::BATCH_TESTING);
    framework.printReportSummary(report);

    std::cout << "是否保存测试报告? (y/n): ";
    char save_choice;
    std::cin >> save_choice;
    if (save_choice == 'y' || save_choice == 'Y') {
        framework.saveReport(report, "txt");
    }
}

void runInteractiveTest(KinematicsTestFramework& framework) {
    std::cout << "\n🎮 交互式测试模式" << std::endl;
    std::cout << "选择测试类型:" << std::endl;
    std::cout << "1. 正运动学测试 (关节角度 -> 脚部位置)" << std::endl;
    std::cout << "2. 逆运动学测试 (脚部位置 -> 关节角度)" << std::endl;
    std::cout << "请选择 (1-2): ";

    int test_type;
    std::cin >> test_type;

    if (test_type == 1) {
        // 正运动学测试
        std::cout << "\n正运动学测试设置:" << std::endl;

        std::cout << "选择腿部 (0=左前, 1=右前, 2=左后, 3=右后): ";
        int leg_choice;
        std::cin >> leg_choice;

        if (leg_choice < 0 || leg_choice > 3) {
            std::cout << "❌ 无效的腿部选择" << std::endl;
            return;
        }

        std::cout << "输入关节角度 (度):" << std::endl;
        std::cout << "髋侧摆角度: ";
        float hip_side_deg;
        std::cin >> hip_side_deg;

        std::cout << "髋俯仰角度: ";
        float hip_pitch_deg;
        std::cin >> hip_pitch_deg;

        std::cout << "膝俯仰角度: ";
        float knee_pitch_deg;
        std::cin >> knee_pitch_deg;

        // 转换为弧度
        ThreeJointAngles joints;
        joints.hip_side = hip_side_deg * M_PI / 180.0f;
        joints.hip_pitch = hip_pitch_deg * M_PI / 180.0f;
        joints.knee_pitch = knee_pitch_deg * M_PI / 180.0f;

        TestCase test_case;
        test_case.name = "交互式FK测试";
        test_case.leg_type = static_cast<LegType>(leg_choice);
        test_case.input_joints = joints;
        test_case.use_joints_input = true;

        framework.startTestingSession();
        auto result = framework.executeForwardKinematicsTest(test_case);
        framework.endTestingSession();

        std::cout << "\n📊 测试结果:" << std::endl;
        if (result.success) {
            std::cout << "✓ 正运动学计算成功" << std::endl;
            std::cout << "脚部位置: [" << result.actual_position.x << ", "
                      << result.actual_position.y << ", "
                      << result.actual_position.z << "] m" << std::endl;
        } else {
            std::cout << "❌ 正运动学计算失败: " << result.error_message << std::endl;
        }

    } else if (test_type == 2) {
        // 逆运动学测试
        std::cout << "\n逆运动学测试设置:" << std::endl;

        std::cout << "选择腿部 (0=左前, 1=右前, 2=左后, 3=右后): ";
        int leg_choice;
        std::cin >> leg_choice;

        if (leg_choice < 0 || leg_choice > 3) {
            std::cout << "❌ 无效的腿部选择" << std::endl;
            return;
        }

        std::cout << "输入脚部目标位置 (米, 相对髋关节):" << std::endl;
        std::cout << "X (前后): ";
        float x;
        std::cin >> x;

        std::cout << "Y (左右): ";
        float y;
        std::cin >> y;

        std::cout << "Z (上下): ";
        float z;
        std::cin >> z;

        Vector3 target_pos(x, y, z);

        TestCase test_case;
        test_case.name = "交互式IK测试";
        test_case.leg_type = static_cast<LegType>(leg_choice);
        test_case.input_position = target_pos;
        test_case.use_joints_input = false;

        framework.startTestingSession();
        auto result = framework.executeInverseKinematicsTest(test_case);
        framework.endTestingSession();

        std::cout << "\n📊 测试结果:" << std::endl;
        if (result.success) {
            std::cout << "✓ 逆运动学计算成功" << std::endl;
            std::cout << "关节角度: 髋侧摆=" << result.actual_joints.hip_side * 180/M_PI << "°, "
                      << "髋俯仰=" << result.actual_joints.hip_pitch * 180/M_PI << "°, "
                      << "膝俯仰=" << result.actual_joints.knee_pitch * 180/M_PI << "°" << std::endl;
        } else {
            std::cout << "❌ 逆运动学计算失败: " << result.error_message << std::endl;
        }

    } else {
        std::cout << "❌ 无效的测试类型选择" << std::endl;
    }
}

void runQuickTest(KinematicsTestFramework& framework) {
    std::cout << "\n⚡ 快速验证测试..." << std::endl;

    framework.startTestingSession();

    // 简单的FK-IK往返测试
    TestCase test_case;
    test_case.name = "快速验证测试";
    test_case.leg_type = LegType::LEFT_FRONT;
    test_case.input_joints = {0.1f, -0.3f, -0.6f}; // 典型姿态
    test_case.use_joints_input = true;

    std::cout << "测试关节角度: 髋侧摆=" << test_case.input_joints.hip_side * 180/M_PI << "°, "
              << "髋俯仰=" << test_case.input_joints.hip_pitch * 180/M_PI << "°, "
              << "膝俯仰=" << test_case.input_joints.knee_pitch * 180/M_PI << "°" << std::endl;

    // 步骤1：正运动学
    auto leg_solver = framework.getLegSolver(test_case.leg_type);
    auto hip_pos = framework.getLegHipPosition(test_case.leg_type);

    auto fk_result = leg_solver->forwardKinematics(hip_pos, test_case.input_joints);
    if (!fk_result.success) {
        std::cout << "❌ 正运动学失败: " << fk_result.error_message << std::endl;
        return;
    }

    Vector3 foot_in_hip = {
        fk_result.foot_position.x - hip_pos.x,
        fk_result.foot_position.y - hip_pos.y,
        fk_result.foot_position.z - hip_pos.z
    };

    std::cout << "✓ 正运动学成功，脚部位置: [" << foot_in_hip.x << ", "
              << foot_in_hip.y << ", " << foot_in_hip.z << "] m" << std::endl;

    // 步骤2：逆运动学
    auto ik_result = leg_solver->inverseKinematics(foot_in_hip);
    if (!ik_result.success) {
        std::cout << "❌ 逆运动学失败: " << ik_result.error_message << std::endl;
        return;
    }

    std::cout << "✓ 逆运动学成功，关节角度: 髋侧摆=" << ik_result.joint_angles.hip_side * 180/M_PI << "°, "
              << "髋俯仰=" << ik_result.joint_angles.hip_pitch * 180/M_PI << "°, "
              << "膝俯仰=" << ik_result.joint_angles.knee_pitch * 180/M_PI << "°" << std::endl;

    // 步骤3：计算误差
    float hip_side_error = std::abs(test_case.input_joints.hip_side - ik_result.joint_angles.hip_side);
    float hip_pitch_error = std::abs(test_case.input_joints.hip_pitch - ik_result.joint_angles.hip_pitch);
    float knee_pitch_error = std::abs(test_case.input_joints.knee_pitch - ik_result.joint_angles.knee_pitch);
    float max_error = std::max(std::max(hip_side_error, hip_pitch_error), knee_pitch_error);

    std::cout << "往返误差: " << max_error * 180/M_PI << "°" << std::endl;

    if (max_error < 1e-2f) {  // 0.57度容差
        std::cout << "✅ 快速验证测试通过！运动学计算正常工作。" << std::endl;
    } else {
        std::cout << "⚠️  快速验证测试未通过，误差较大。" << std::endl;
    }

    framework.endTestingSession();
}

int main(int argc, char* argv[]) {
    printWelcome();

    // 检查命令行参数
    if (argc > 1) {
        std::string arg = argv[1];
        if (arg == "--quick" || arg == "-q") {
            std::cout << "运行快速验证模式..." << std::endl;
            KinematicsTestFramework framework;
            runQuickTest(framework);
            return 0;
        } else if (arg == "--roundtrip" || arg == "-r") {
            std::cout << "运行FK-IK往返测试模式..." << std::endl;
            KinematicsTestFramework framework;
            runFkIkRoundtripTest(framework);
            return 0;
        } else if (arg == "--batch" || arg == "-b") {
            std::cout << "运行批量测试模式..." << std::endl;
            KinematicsTestFramework framework;
            runBatchTest(framework);
            return 0;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "使用方法:" << std::endl;
            std::cout << "  " << argv[0] << " [选项]" << std::endl;
            std::cout << std::endl;
            std::cout << "选项:" << std::endl;
            std::cout << "  -q, --quick      快速验证测试" << std::endl;
            std::cout << "  -r, --roundtrip  FK-IK往返测试" << std::endl;
            std::cout << "  -b, --batch      批量预定义测试" << std::endl;
            std::cout << "  -h, --help       显示此帮助信息" << std::endl;
            std::cout << "  无参数           启动交互式菜单" << std::endl;
            return 0;
        }
    }

    // 创建测试框架
    try {
        KinematicsTestFramework framework;

        // 主菜单循环
        int choice;
        do {
            printMainMenu();
            choice = getUserChoice();

            switch (choice) {
                case 1:
                    runFkIkRoundtripTest(framework);
                    break;

                case 2:
                    std::cout << "\n🚧 工作空间边界测试功能正在开发中..." << std::endl;
                    break;

                case 3:
                    runBatchTest(framework);
                    break;

                case 4:
                    runInteractiveTest(framework);
                    break;

                case 5:
                    framework.printLegInfo();
                    break;

                case 6:
                    std::cout << "\n🚧 工作空间信息显示功能正在开发中..." << std::endl;
                    break;

                case 0:
                    std::cout << "\n👋 感谢使用四足机器狗运动学测试程序！" << std::endl;
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

    } catch (const std::exception& e) {
        std::cerr << "❌ 程序运行出错: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}