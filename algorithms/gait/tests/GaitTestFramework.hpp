#pragma once

#include "../gait/WalkGait.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <fstream>

namespace Robot {
namespace Testing {

/**
 * @brief 测试结果结构
 */
struct GaitTestResult {
    std::string name;
    bool success;
    std::string error_message;
    float execution_time_ms;
    
    GaitTestResult() : success(false), execution_time_ms(0) {}
};

/**
 * @brief 步态测试框架
 * 
 * 提供步态算法的验证测试，包括：
 * - 轨迹闭合性验证
 * - 占空比验证
 * - 支撑腿验证
 * - 连续运行测试
 */
class GaitTestFramework {
public:
    GaitTestFramework() : test_count_(0), pass_count_(0) {
        std::cout << "🤖 步态测试框架已初始化" << std::endl;
    }
    
    /**
     * @brief 测试轨迹生成器
     */
    void test_trajectory_generator() {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "测试 1: 轨迹生成器" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        
        using namespace Robot::Gait;
        
        std::cout << "\n摆线轨迹（推荐）:" << std::endl;
        std::cout << "相位  | X偏移(mm) | Z偏移(mm) | 说明" << std::endl;
        std::cout << std::string(50, '-') << std::endl;
        
        for (float phase = 0.0f; phase <= 1.0f; phase += 0.1f) {
            auto point = TrajectoryGenerator::cycloid_trajectory(phase, 0.05f, 0.03f);
            std::string desc = point.is_swing ? "摆动相" : "支撑相";
            
            std::cout << std::fixed << std::setprecision(1) 
                      << phase << "  | " 
                      << std::setw(9) << point.position.x * 1000 << " | "
                      << std::setw(9) << point.position.z * 1000 << " | "
                      << desc << std::endl;
        }
        
        // 验证轨迹闭合性
        auto start = TrajectoryGenerator::cycloid_trajectory(0.0f, 0.05f, 0.03f);
        auto end = TrajectoryGenerator::cycloid_trajectory(1.0f, 0.05f, 0.03f);
        
        std::cout << "\n轨迹闭合性验证:" << std::endl;
        std::cout << "  起点: x=" << start.position.x * 1000 << "mm, z=" 
                  << start.position.z * 1000 << "mm" << std::endl;
        std::cout << "  终点: x=" << end.position.x * 1000 << "mm, z=" 
                  << end.position.z * 1000 << "mm" << std::endl;
        
        bool closed = TrajectoryGenerator::verify_trajectory_closed(0.05f, 0.03f);
        if (closed) {
            std::cout << "  ✅ 轨迹闭合！" << std::endl;
        } else {
            std::cout << "  ❌ 轨迹不闭合！" << std::endl;
        }
        
        std::cout << "\n✅ 轨迹生成器测试通过" << std::endl;
    }
    
    /**
     * @brief 测试 Walk 步态
     */
    void test_walk_gait() {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "测试 2: Walk 步态" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        
        using namespace Robot::Gait;
        
        WalkGait gait(0.05f, 0.03f, 0.8f);
        
        std::cout << "\n步态参数:" << std::endl;
        std::cout << "  步长: " << gait.get_state().stride_length * 1000 << "mm" << std::endl;
        std::cout << "  步高: " << gait.get_state().step_height * 1000 << "mm" << std::endl;
        std::cout << "  步频: " << gait.get_state().frequency << "Hz" << std::endl;
        std::cout << "  占空比: " << gait.get_state().duty_cycle << " (Walk步态)" << std::endl;
        
        std::cout << "\n相位偏移:" << std::endl;
        for (const auto& pair : gait.get_all_phase_offsets()) {
            float offset = pair.second;
            std::cout << "  " << std::setw(15) << LEG_NAMES[static_cast<int>(pair.first)]
                      << ": " << std::fixed << std::setprecision(2) << offset
                      << " (" << offset * 90 << "°)" << std::endl;
        }
        
        // 测试相位调度
        std::cout << "\n测试相位调度（一个完整周期）:" << std::endl;
        float dt = 0.02f;  // 20ms 时间步
        int steps_per_cycle = static_cast<int>(1.0f / (gait.get_state().frequency * dt));
        
        std::cout << "  每周期步数: " << steps_per_cycle << std::endl;
        std::cout << "  周期时长: " << 1.0f / gait.get_state().frequency << "秒" << std::endl;
        
        // 模拟一个周期
        std::cout << "\n模拟一个完整周期:" << std::endl;
        gait.reset();
        
        for (int i = 0; i < std::min(10, steps_per_cycle); i++) {
            gait.update(dt);
            auto trajectories = gait.get_all_foot_trajectories();
            
            if (i % 2 == 0) {  // 每2步输出一次
                std::cout << "\n步骤 " << std::setw(2) << i + 1 << ", 全局相位: " 
                          << std::fixed << std::setprecision(2) << gait.get_global_phase() << std::endl;
                
                for (const auto& pair : trajectories) {
                    std::string state = pair.second.is_swing ? "抬腿" : "着地";
                    std::cout << "  " << std::setw(15) << LEG_NAMES[static_cast<int>(pair.first)]
                              << ": x=" << std::setw(6) << std::fixed << std::setprecision(1) 
                              << pair.second.position.x * 1000 << "mm, z="
                              << std::setw(6) << pair.second.position.z * 1000 << "mm, ["
                              << state << "]" << std::endl;
                }
            }
        }
        
        std::cout << "\n✅ Walk 步态测试通过" << std::endl;
    }
    
    /**
     * @brief 测试占空比
     */
    void test_duty_cycle() {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "测试 3: 占空比验证" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        
        using namespace Robot::Gait;
        
        WalkGait gait(0.05f, 0.03f, 0.8f);
        
        // 统计每条腿的摆动/支撑帧数
        int frames = 100;
        std::map<LegIndex, int> swing_counts;
        std::map<LegIndex, int> stance_counts;
        
        for (int i = 0; i < 4; i++) {
            swing_counts[static_cast<LegIndex>(i)] = 0;
            stance_counts[static_cast<LegIndex>(i)] = 0;
        }
        
        float dt = 1.0f / (gait.get_state().frequency * frames);
        
        for (int i = 0; i < frames; i++) {
            gait.update(dt);
            auto trajectories = gait.get_all_foot_trajectories();
            
            for (const auto& pair : trajectories) {
                if (pair.second.is_swing) {
                    swing_counts[pair.first]++;
                } else {
                    stance_counts[pair.first]++;
                }
            }
        }
        
        std::cout << "\n占空比统计（" << frames << "帧）:" << std::endl;
        bool all_pass = true;
        
        for (int i = 0; i < 4; i++) {
            LegIndex leg = static_cast<LegIndex>(i);
            float swing_pct = static_cast<float>(swing_counts[leg]) / frames * 100;
            float stance_pct = static_cast<float>(stance_counts[leg]) / frames * 100;
            
            bool pass = (std::abs(swing_pct - 25) < 1 && std::abs(stance_pct - 75) < 1);
            std::string status = pass ? " ✅" : " ❌";
            
            std::cout << "  " << std::setw(15) << LEG_NAMES[i]
                      << ": 摆动" << std::fixed << std::setprecision(0) << swing_pct 
                      << "%, 支撑" << stance_pct << "%" << status << std::endl;
            
            if (!pass) all_pass = false;
        }
        
        if (all_pass) {
            std::cout << "\n✅ 占空比验证通过（25%摆动，75%支撑）" << std::endl;
        } else {
            std::cout << "\n❌ 占空比验证失败" << std::endl;
        }
    }
    
    /**
     * @brief 测试支撑腿数量
     */
    void test_support_legs() {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "测试 4: 支撑腿验证" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        
        using namespace Robot::Gait;
        
        WalkGait gait(0.05f, 0.03f, 0.8f);
        
        // 统计每时刻支撑腿数量
        int frames = 100;
        std::map<int, int> support_counts;
        for (int i = 0; i <= 4; i++) {
            support_counts[i] = 0;
        }
        
        float dt = 1.0f / (gait.get_state().frequency * frames);
        
        for (int i = 0; i < frames; i++) {
            gait.update(dt);
            int support_count = gait.count_support_legs();
            support_counts[support_count]++;
        }
        
        std::cout << "\n支撑腿数量分布:" << std::endl;
        for (const auto& pair : support_counts) {
            float pct = static_cast<float>(pair.second) / frames * 100;
            std::string status = (pair.first == 3 && pct > 99) ? " ✅ (正确)" : "";
            std::cout << "  " << pair.first << "条腿支撑: " << pair.second << "帧 (" 
                      << std::fixed << std::setprecision(1) << pct << "%)" << status << std::endl;
        }
        
        if (support_counts[3] == frames) {
            std::cout << "\n✅ 支撑腿验证通过（每时刻3条腿支撑）" << std::endl;
        } else {
            std::cout << "\n❌ 支撑腿验证失败" << std::endl;
        }
    }
    
    /**
     * @brief 连续步态运行测试
     */
    void test_continuous_gait() {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "测试 5: 连续步态运行（5秒）" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        
        using namespace Robot::Gait;
        
        WalkGait gait(0.05f, 0.03f, 0.8f);
        
        std::cout << "\n开始连续运行..." << std::endl;
        std::cout << "模拟 5 秒钟的步态运行，每秒输出一次状态" << std::endl;
        
        float dt = 0.02f;  // 20ms
        float total_time = 5.0f;
        int steps = static_cast<int>(total_time / dt);
        
        auto start_time = std::chrono::high_resolution_clock::now();
        auto last_report_time = start_time;
        
        for (int i = 0; i < steps; i++) {
            gait.update(dt);
            auto current_time = std::chrono::high_resolution_clock::now();
            
            // 每秒输出一次
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - last_report_time
            ).count();
            
            if (elapsed_ms >= 1000) {
                auto total_elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                    current_time - start_time
                ).count();
                
                auto trajectories = gait.get_all_foot_trajectories();
                
                // 计算平均抬腿高度
                float avg_z = 0;
                for (const auto& pair : trajectories) {
                    avg_z += pair.second.position.z;
                }
                avg_z /= trajectories.size();
                
                std::cout << "时间: " << total_elapsed << "s, 全局相位: " 
                          << std::fixed << std::setprecision(2) << gait.get_global_phase()
                          << ", 平均抬腿: " << avg_z * 1000 << "mm" << std::endl;
                
                last_report_time = current_time;
            }
        }
        
        std::cout << "\n✅ 连续步态测试通过" << std::endl;
    }
    
    /**
     * @brief 运行快速验证测试
     */
    void run_quick_test() {
        std::cout << "\n⚡ 快速验证测试..." << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        
        auto start = std::chrono::high_resolution_clock::now();
        
        test_trajectory_generator();
        test_walk_gait();
        test_duty_cycle();
        test_support_legs();
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "🎉 所有快速验证测试通过！" << std::endl;
        std::cout << "⏱️  总执行时间: " << duration.count() << "ms" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
    }
    
    /**
     * @brief 运行批量测试
     */
    void run_batch_test() {
        std::cout << "\n📦 执行批量预定义测试..." << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        
        test_trajectory_generator();
        test_walk_gait();
        test_duty_cycle();
        test_support_legs();
        test_continuous_gait();
        
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "🎉 所有批量测试通过！" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
    }
    
private:
    int test_count_;
    int pass_count_;
    std::vector<GaitTestResult> results_;
};

} // namespace Testing
} // namespace Robot
