#pragma once

#include "../kinematics/LegKinematics.hpp"
#include "../kinematics/KinematicsGeometry.hpp"
#include "../kinematics/CoordinateTransform.hpp"
#include "../RobotTypes.hpp"
#include <vector>
#include <string>
#include <chrono>
#include <memory>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Robot {
namespace Testing {

using Vector3 = Robot::Kinematics::CoordinateTransform::Vector3;

/**
 * @brief 测试类型枚举
 */
enum class TestType {
    FK_IK_ROUNDTRIP,      // FK-IK往返测试
    WORKSPACE_BOUNDARY,    // 工作空间边界测试
    SINGULARITY_DETECTION, // 奇异点检测
    ACCURACY_VERIFICATION, // 精度验证
    BATCH_TESTING,        // 批量测试
    INTERACTIVE           // 交互式测试
};

/**
 * @brief 腿部类型枚举
 */
enum class LegType {
    LEFT_FRONT = 0,
    RIGHT_FRONT = 1,
    LEFT_BACK = 2,
    RIGHT_BACK = 3
};

/**
 * @brief 测试用例结构
 */
struct TestCase {
    std::string name;
    std::string description;
    LegType leg_type;

    // 输入数据 (二选一)
    ThreeJointAngles input_joints;     // 正运动学输入：关节角度
    Vector3 input_position;  // 逆运动学输入：脚部位置
    bool use_joints_input;  // true: 使用关节角度输入，false: 使用位置输入

    // 期望结果
    Vector3 expected_position;
    ThreeJointAngles expected_joints;
    bool expect_success;

    // 容差
    float position_tolerance;  // 位置容差 (米)
    float angle_tolerance;     // 角度容差 (弧度)

    TestCase() : use_joints_input(true), expect_success(true),
                 position_tolerance(1e-3f), angle_tolerance(1e-2f) {}
};

/**
 * @brief 测试结果结构
 */
struct TestResult {
    TestCase test_case;
    bool success;
    std::string error_message;

    // 实际结果
    Vector3 actual_position;
    ThreeJointAngles actual_joints;

    // 精度指标
    float position_error;    // 位置误差 (米)
    float max_angle_error;   // 最大角度误差 (弧度)

    // 执行时间
    std::chrono::microseconds execution_time;

    TestResult() : success(false), position_error(0.0f), max_angle_error(0.0f) {}
};

/**
 * @brief 测试报告结构
 */
struct TestReport {
    TestType test_type;
    std::string timestamp;

    int total_tests;
    int passed_tests;
    int failed_tests;
    float success_rate;

    std::chrono::milliseconds total_execution_time;
    std::chrono::microseconds average_execution_time;

    std::vector<TestResult> results;

    // 统计信息
    float max_position_error;
    float max_angle_error;
    float average_position_error;
    float average_angle_error;

    TestReport() : total_tests(0), passed_tests(0), failed_tests(0), success_rate(0.0f),
                   max_position_error(0.0f), max_angle_error(0.0f),
                   average_position_error(0.0f), average_angle_error(0.0f) {}
};

/**
 * @brief 四足机器狗运动学测试框架
 *
 * 基于Python版本kinematics_test_framework.py设计
 * 支持命令行交互和批量测试
 */
class KinematicsTestFramework {
private:
    // 运动学求解器
    std::vector<std::unique_ptr<Kinematics::SpotLegKinematics>> leg_solvers_;

    // 髋关节位置偏移
    std::vector<Vector3> hip_positions_;

    // 测试结果存储
    std::vector<TestResult> current_results_;
    std::chrono::steady_clock::time_point session_start_time_;

    // 工具方法
    bool isLeftLeg(LegType leg_type) const;
    std::string getLegName(LegType leg_type) const;
    std::string getCurrentTimestamp() const;

    // 计算误差
    float calculatePositionError(const Vector3& expected,
                               const Vector3& actual) const;
    float calculateMaxAngleError(const ThreeJointAngles& expected,
                               const ThreeJointAngles& actual) const;

public:
    /**
     * @brief 构造函数
     */
    KinematicsTestFramework();

    /**
     * @brief 析构函数
     */
    ~KinematicsTestFramework();

    /**
     * @brief 开始测试会话
     */
    void startTestingSession();

    /**
     * @brief 结束测试会话
     */
    void endTestingSession();

    /**
     * @brief 执行单个测试用例
     */
    TestResult executeTestCase(const TestCase& test_case);

    /**
     * @brief 执行正运动学测试
     */
    TestResult executeForwardKinematicsTest(const TestCase& test_case);

    /**
     * @brief 执行逆运动学测试
     */
    TestResult executeInverseKinematicsTest(const TestCase& test_case);

    /**
     * @brief 执行FK-IK往返测试
     */
    std::vector<TestResult> executeFkIkRoundtripTest();

    /**
     * @brief 执行工作空间边界测试
     */
    std::vector<TestResult> executeWorkspaceBoundaryTest();

    /**
     * @brief 执行批量预定义测试
     */
    std::vector<TestResult> executeBatchTest();

    /**
     * @brief 生成测试报告
     */
    TestReport generateReport(TestType test_type) const;

    /**
     * @brief 保存报告到文件
     */
    bool saveReport(const TestReport& report, const std::string& format = "txt") const;

    /**
     * @brief 打印报告摘要
     */
    void printReportSummary(const TestReport& report) const;

    /**
     * @brief 打印详细结果
     */
    void printDetailedResults(const TestReport& report) const;

    /**
     * @brief 交互式命令行界面
     */
    void runInteractiveMode();

    /**
     * @brief 创建预定义测试用例
     */
    std::vector<TestCase> createPredefinedTestCases() const;

    /**
     * @brief 打印腿部信息
     */
    void printLegInfo() const;

    /**
     * @brief 打印工作空间信息
     */
    void printWorkspaceInfo() const;

    /**
     * @brief 公开方法：获取腿部运动学求解器（用于测试）
     */
    Kinematics::SpotLegKinematics* getLegSolver(LegType leg_type) const {
        return leg_solvers_[static_cast<int>(leg_type)].get();
    }

    /**
     * @brief 公开方法：获取腿部髋关节位置（用于测试）
     */
    Vector3 getLegHipPosition(LegType leg_type) const {
        return hip_positions_[static_cast<int>(leg_type)];
    }
};

} // namespace Testing
} // namespace Robot