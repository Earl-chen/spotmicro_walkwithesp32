#include "KinematicsTestFramework.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>

namespace Robot {
namespace Testing {

using Vector3 = Robot::Kinematics::CoordinateTransform::Vector3;

KinematicsTestFramework::KinematicsTestFramework() {
    // 初始化四条腿的运动学求解器
    leg_solvers_.resize(4);
    leg_solvers_[0] = std::make_unique<Kinematics::SpotLegKinematics>(true);  // 左前腿
    leg_solvers_[1] = std::make_unique<Kinematics::SpotLegKinematics>(false); // 右前腿
    leg_solvers_[2] = std::make_unique<Kinematics::SpotLegKinematics>(true);  // 左后腿
    leg_solvers_[3] = std::make_unique<Kinematics::SpotLegKinematics>(false); // 右后腿

    // 初始化髋关节位置
    hip_positions_.resize(4);
    auto hip_offset_lf = Kinematics::SpotMicroGeometry::getHipOffset(Kinematics::SpotMicroGeometry::LEFT_FRONT);
    auto hip_offset_rf = Kinematics::SpotMicroGeometry::getHipOffset(Kinematics::SpotMicroGeometry::RIGHT_FRONT);
    auto hip_offset_lb = Kinematics::SpotMicroGeometry::getHipOffset(Kinematics::SpotMicroGeometry::LEFT_BACK);
    auto hip_offset_rb = Kinematics::SpotMicroGeometry::getHipOffset(Kinematics::SpotMicroGeometry::RIGHT_BACK);

    hip_positions_[0] = Vector3(hip_offset_lf.x, hip_offset_lf.y, hip_offset_lf.z);
    hip_positions_[1] = Vector3(hip_offset_rf.x, hip_offset_rf.y, hip_offset_rf.z);
    hip_positions_[2] = Vector3(hip_offset_lb.x, hip_offset_lb.y, hip_offset_lb.z);
    hip_positions_[3] = Vector3(hip_offset_rb.x, hip_offset_rb.y, hip_offset_rb.z);

    std::cout << "🤖 四足机器狗运动学测试框架已初始化" << std::endl;
    std::cout << "📏 腿部几何参数: L1=" << Kinematics::SpotMicroGeometry::L1*1000 << "mm, "
              << "L2=" << Kinematics::SpotMicroGeometry::L2*1000 << "mm, "
              << "L3=" << Kinematics::SpotMicroGeometry::L3*1000 << "mm, "
              << "L4=" << Kinematics::SpotMicroGeometry::L4*1000 << "mm" << std::endl;
    std::cout << std::endl;
}

KinematicsTestFramework::~KinematicsTestFramework() = default;

void KinematicsTestFramework::startTestingSession() {
    session_start_time_ = std::chrono::steady_clock::now();
    current_results_.clear();
    std::cout << "🚀 测试会话开始: " << getCurrentTimestamp() << std::endl;
}

void KinematicsTestFramework::endTestingSession() {
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - session_start_time_);
    std::cout << "🏁 测试会话结束: " << getCurrentTimestamp() << std::endl;
    std::cout << "⏱️  总执行时间: " << duration.count() << "ms" << std::endl;
}

bool KinematicsTestFramework::isLeftLeg(LegType leg_type) const {
    return (leg_type == LegType::LEFT_FRONT || leg_type == LegType::LEFT_BACK);
}

std::string KinematicsTestFramework::getLegName(LegType leg_type) const {
    switch (leg_type) {
        case LegType::LEFT_FRONT: return "左前腿";
        case LegType::RIGHT_FRONT: return "右前腿";
        case LegType::LEFT_BACK: return "左后腿";
        case LegType::RIGHT_BACK: return "右后腿";
        default: return "未知腿部";
    }
}

std::string KinematicsTestFramework::getCurrentTimestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

float KinematicsTestFramework::calculatePositionError(const Vector3& expected,
                                                     const Vector3& actual) const {
    float dx = expected.x - actual.x;
    float dy = expected.y - actual.y;
    float dz = expected.z - actual.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

float KinematicsTestFramework::calculateMaxAngleError(const ThreeJointAngles& expected,
                                                     const ThreeJointAngles& actual) const {
    float error1 = std::abs(expected.hip_side - actual.hip_side);
    float error2 = std::abs(expected.hip_pitch - actual.hip_pitch);
    float error3 = std::abs(expected.knee_pitch - actual.knee_pitch);
    return std::max({error1, error2, error3});
}

TestResult KinematicsTestFramework::executeTestCase(const TestCase& test_case) {
    if (test_case.use_joints_input) {
        return executeForwardKinematicsTest(test_case);
    } else {
        return executeInverseKinematicsTest(test_case);
    }
}

TestResult KinematicsTestFramework::executeForwardKinematicsTest(const TestCase& test_case) {
    auto start_time = std::chrono::steady_clock::now();

    TestResult result;
    result.test_case = test_case;

    try {
        auto solver = getLegSolver(test_case.leg_type);
        auto hip_position = getLegHipPosition(test_case.leg_type);

        // 执行正运动学计算
        auto fk_result = solver->forwardKinematics(hip_position, test_case.input_joints);

        if (fk_result.success) {
            result.actual_position = fk_result.foot_position;
            result.actual_joints = fk_result.joint_angles;
            result.success = true;

            // 计算位置误差
            result.position_error = calculatePositionError(test_case.expected_position, result.actual_position);

            // 检查是否满足容差要求
            if (result.position_error <= test_case.position_tolerance) {
                result.success = true;
            } else {
                result.success = false;
                result.error_message = "位置误差超出容差范围";
            }
        } else {
            result.success = false;
            result.error_message = std::string("正运动学计算失败: ") + fk_result.error_message;
        }
    } catch (const std::exception& e) {
        result.success = false;
        result.error_message = std::string("异常: ") + e.what();
    }

    auto end_time = std::chrono::steady_clock::now();
    result.execution_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    current_results_.push_back(result);
    return result;
}

TestResult KinematicsTestFramework::executeInverseKinematicsTest(const TestCase& test_case) {
    auto start_time = std::chrono::steady_clock::now();

    TestResult result;
    result.test_case = test_case;

    try {
        auto solver = getLegSolver(test_case.leg_type);

        // 执行逆运动学计算
        auto ik_result = solver->inverseKinematics(test_case.input_position);

        if (ik_result.success) {
            result.actual_joints = ik_result.joint_angles;
            result.actual_position = ik_result.foot_position;
            result.success = true;

            // 计算角度误差
            result.max_angle_error = calculateMaxAngleError(test_case.expected_joints, result.actual_joints);

            // 检查是否满足容差要求
            if (result.max_angle_error <= test_case.angle_tolerance) {
                result.success = true;
            } else {
                result.success = false;
                result.error_message = "角度误差超出容差范围";
            }
        } else {
            result.success = test_case.expect_success == false; // 如果期望失败，则失败也是成功
            result.error_message = std::string("逆运动学计算失败: ") + ik_result.error_message;
        }
    } catch (const std::exception& e) {
        result.success = false;
        result.error_message = std::string("异常: ") + e.what();
    }

    auto end_time = std::chrono::steady_clock::now();
    result.execution_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    current_results_.push_back(result);
    return result;
}

std::vector<TestResult> KinematicsTestFramework::executeFkIkRoundtripTest() {
    std::cout << "开始 FK-IK 往返测试..." << std::endl;
    std::cout << std::string(50, '=') << std::endl;

    startTestingSession();

    // 创建往返测试用例
    std::vector<TestCase> test_cases;

    // 为每条腿创建测试用例
    std::vector<LegType> leg_types = {
        LegType::LEFT_FRONT, LegType::RIGHT_FRONT,
        LegType::LEFT_BACK, LegType::RIGHT_BACK
    };

    // 典型关节角度测试用例
    std::vector<ThreeJointAngles> joint_test_cases = {
        {0.0f, 0.0f, 0.0f},           // 零位
        {0.2f, -0.35f, -0.8f},        // 典型姿态
        {-0.3f, 0.5f, -1.2f},         // 极限姿态
        {0.1f, -0.1f, -0.5f},         // 小角度
        {-0.1f, -0.6f, -0.9f}         // 其他姿态
    };

    for (auto leg_type : leg_types) {
        for (size_t i = 0; i < joint_test_cases.size(); ++i) {
            TestCase test_case;
            test_case.name = getLegName(leg_type) + "_往返测试_" + std::to_string(i+1);
            test_case.description = "FK-IK往返精度测试";
            test_case.leg_type = leg_type;
            test_case.input_joints = joint_test_cases[i];
            test_case.use_joints_input = true;
            test_case.angle_tolerance = 1e-2f;  // 约0.57度
            test_case.position_tolerance = 1e-3f; // 1mm

            test_cases.push_back(test_case);
        }
    }

    std::vector<TestResult> results;

    for (const auto& test_case : test_cases) {
        std::cout << "\n测试用例: " << test_case.name << std::endl;
        std::cout << "关节角度: hip_side=" << test_case.input_joints.hip_side * 180/M_PI << "°, "
                  << "hip_pitch=" << test_case.input_joints.hip_pitch * 180/M_PI << "°, "
                  << "knee_pitch=" << test_case.input_joints.knee_pitch * 180/M_PI << "°" << std::endl;

        // 第一步：正运动学
        auto solver = getLegSolver(test_case.leg_type);
        auto hip_position = getLegHipPosition(test_case.leg_type);

        auto fk_result = solver->forwardKinematics(hip_position, test_case.input_joints);
        if (!fk_result.success) {
            TestResult result;
            result.test_case = test_case;
            result.success = false;
            result.error_message = "正运动学失败: " + std::string(fk_result.error_message);
            results.push_back(result);
            std::cout << "❌ 正运动学失败: " << fk_result.error_message << std::endl;
            continue;
        }

        Vector3 foot_in_hip = {
            fk_result.foot_position.x - hip_position.x,
            fk_result.foot_position.y - hip_position.y,
            fk_result.foot_position.z - hip_position.z
        };

        std::cout << "FK结果: 脚部位置 = [" << foot_in_hip.x << ", "
                  << foot_in_hip.y << ", " << foot_in_hip.z << "] m" << std::endl;

        // 第二步：逆运动学
        auto ik_result = solver->inverseKinematics(foot_in_hip);
        if (!ik_result.success) {
            TestResult result;
            result.test_case = test_case;
            result.success = false;
            result.error_message = "逆运动学失败: " + std::string(ik_result.error_message);
            results.push_back(result);
            std::cout << "❌ 逆运动学失败: " << ik_result.error_message << std::endl;
            continue;
        }

        // 第三步：比较精度
        float max_angle_error = calculateMaxAngleError(test_case.input_joints, ik_result.joint_angles);

        std::cout << "IK结果: hip_side=" << ik_result.joint_angles.hip_side * 180/M_PI << "°, "
                  << "hip_pitch=" << ik_result.joint_angles.hip_pitch * 180/M_PI << "°, "
                  << "knee_pitch=" << ik_result.joint_angles.knee_pitch * 180/M_PI << "°" << std::endl;
        std::cout << "最大角度误差: " << max_angle_error * 180/M_PI << "°" << std::endl;

        TestResult result;
        result.test_case = test_case;
        result.actual_joints = ik_result.joint_angles;
        result.actual_position = foot_in_hip;
        result.max_angle_error = max_angle_error;
        result.success = (max_angle_error <= test_case.angle_tolerance);

        if (result.success) {
            std::cout << "✓ 往返测试通过（误差 < " << test_case.angle_tolerance * 180/M_PI << "°）" << std::endl;
        } else {
            result.error_message = "角度误差超出容差";
            std::cout << "❌ 往返测试失败（误差 > " << test_case.angle_tolerance * 180/M_PI << "°）" << std::endl;
        }

        results.push_back(result);
    }

    endTestingSession();
    return results;
}

std::vector<TestCase> KinematicsTestFramework::createPredefinedTestCases() const {
    std::vector<TestCase> cases;

    // FK测试用例
    TestCase fk_case1;
    fk_case1.name = "FK_左前腿_零位";
    fk_case1.description = "左前腿零位正运动学测试";
    fk_case1.leg_type = LegType::LEFT_FRONT;
    fk_case1.input_joints = {0.0f, 0.0f, 0.0f};
    fk_case1.use_joints_input = true;
    fk_case1.position_tolerance = 1e-3f;
    cases.push_back(fk_case1);

    // IK测试用例
    TestCase ik_case1;
    ik_case1.name = "IK_左前腿_典型位置";
    ik_case1.description = "左前腿典型位置逆运动学测试";
    ik_case1.leg_type = LegType::LEFT_FRONT;
    ik_case1.input_position = {0.15f, 0.02f, -0.18f};
    ik_case1.use_joints_input = false;
    ik_case1.angle_tolerance = 1e-2f;
    ik_case1.expect_success = true;
    cases.push_back(ik_case1);

    return cases;
}

TestReport KinematicsTestFramework::generateReport(TestType test_type) const {
    TestReport report;
    report.test_type = test_type;
    report.timestamp = getCurrentTimestamp();
    report.results = current_results_;
    report.total_tests = current_results_.size();

    report.passed_tests = 0;
    report.failed_tests = 0;
    float total_position_error = 0.0f;
    float total_angle_error = 0.0f;

    for (const auto& result : current_results_) {
        if (result.success) {
            report.passed_tests++;
        } else {
            report.failed_tests++;
        }

        report.max_position_error = std::max(report.max_position_error, result.position_error);
        report.max_angle_error = std::max(report.max_angle_error, result.max_angle_error);
        total_position_error += result.position_error;
        total_angle_error += result.max_angle_error;
    }

    report.success_rate = report.total_tests > 0 ?
        static_cast<float>(report.passed_tests) / report.total_tests * 100.0f : 0.0f;

    report.average_position_error = report.total_tests > 0 ?
        total_position_error / report.total_tests : 0.0f;

    report.average_angle_error = report.total_tests > 0 ?
        total_angle_error / report.total_tests : 0.0f;

    return report;
}

void KinematicsTestFramework::printReportSummary(const TestReport& report) const {
    std::cout << "\n📋 测试报告摘要" << std::endl;
    std::cout << "=" << std::string(50, '=') << std::endl;
    std::cout << "测试时间: " << report.timestamp << std::endl;
    std::cout << "总测试数: " << report.total_tests << std::endl;
    std::cout << "通过数量: " << report.passed_tests << std::endl;
    std::cout << "失败数量: " << report.failed_tests << std::endl;
    std::cout << "成功率: " << std::fixed << std::setprecision(1) << report.success_rate << "%" << std::endl;

    if (report.max_angle_error > 0) {
        std::cout << "最大角度误差: " << report.max_angle_error * 180/M_PI << "°" << std::endl;
        std::cout << "平均角度误差: " << report.average_angle_error * 180/M_PI << "°" << std::endl;
    }

    if (report.max_position_error > 0) {
        std::cout << "最大位置误差: " << report.max_position_error * 1000 << "mm" << std::endl;
        std::cout << "平均位置误差: " << report.average_position_error * 1000 << "mm" << std::endl;
    }

    if (report.failed_tests > 0) {
        std::cout << "\n❌ 失败的测试用例:" << std::endl;
        for (const auto& result : report.results) {
            if (!result.success) {
                std::cout << "  - " << result.test_case.name << ": " << result.error_message << std::endl;
            }
        }
    }

    std::cout << std::endl;
}

bool KinematicsTestFramework::saveReport(const TestReport& report, const std::string& format) const {
    std::string timestamp = getCurrentTimestamp();
    // 替换时间字符串中的不合法字符
    std::replace(timestamp.begin(), timestamp.end(), ' ', '_');
    std::replace(timestamp.begin(), timestamp.end(), ':', '-');

    std::string filename = "kinematics_test_report_" + timestamp + "." + format;

    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cout << "❌ 无法保存报告到文件: " << filename << std::endl;
        return false;
    }

    file << "四足机器狗运动学测试报告" << std::endl;
    file << "==============================" << std::endl;
    file << "生成时间: " << report.timestamp << std::endl;
    file << "总测试数: " << report.total_tests << std::endl;
    file << "通过数量: " << report.passed_tests << std::endl;
    file << "失败数量: " << report.failed_tests << std::endl;
    file << "成功率: " << std::fixed << std::setprecision(1) << report.success_rate << "%" << std::endl;
    file << std::endl;

    file << "详细结果:" << std::endl;
    file << "----------" << std::endl;
    for (size_t i = 0; i < report.results.size(); ++i) {
        const auto& result = report.results[i];
        file << i+1 << ". " << result.test_case.name << std::endl;
        file << "   状态: " << (result.success ? "✓ 通过" : "❌ 失败") << std::endl;
        if (!result.success) {
            file << "   错误: " << result.error_message << std::endl;
        }
        if (result.max_angle_error > 0) {
            file << "   角度误差: " << result.max_angle_error * 180/M_PI << "°" << std::endl;
        }
        if (result.position_error > 0) {
            file << "   位置误差: " << result.position_error * 1000 << "mm" << std::endl;
        }
        file << std::endl;
    }

    file.close();
    std::cout << "📄 测试报告已保存: " << filename << std::endl;
    return true;
}

void KinematicsTestFramework::printLegInfo() const {
    std::cout << "🦾 腿部运动学信息" << std::endl;
    std::cout << "==================" << std::endl;

    for (int i = 0; i < 4; ++i) {
        LegType leg_type = static_cast<LegType>(i);
        std::cout << getLegName(leg_type) << ":" << std::endl;

        auto hip_pos = hip_positions_[i];
        std::cout << "  髋关节位置: [" << hip_pos.x << ", " << hip_pos.y << ", " << hip_pos.z << "] m" << std::endl;

        auto solver = leg_solvers_[i].get();
        auto bounds = solver->getWorkspaceBounds();
        std::cout << "  工作空间: X[" << bounds.min_x << "," << bounds.max_x
                  << "] Y[" << bounds.min_y << "," << bounds.max_y
                  << "] Z[" << bounds.min_z << "," << bounds.max_z << "] m" << std::endl;

        std::cout << std::endl;
    }
}

} // namespace Testing
} // namespace Robot