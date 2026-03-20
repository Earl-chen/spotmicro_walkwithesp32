#pragma once

#include <array>
#include <string>
#include <map>

namespace Robot {
namespace Gait {

/**
 * @brief 腿部索引枚举
 */
enum class LegIndex {
    LEFT_FRONT = 0,    // 左前腿
    RIGHT_FRONT = 1,   // 右前腿
    LEFT_BACK = 2,     // 左后腿
    RIGHT_BACK = 3     // 右后腿
};

/**
 * @brief 腿部名称数组
 */
static const std::array<std::string, 4> LEG_NAMES = {
    "left_front", "right_front", "left_back", "right_back"
};

/**
 * @brief 3D向量结构
 */
struct Vector3 {
    float x;
    float y;
    float z;
    
    Vector3(float x = 0.0f, float y = 0.0f, float z = 0.0f) 
        : x(x), y(y), z(z) {}
};

/**
 * @brief 轨迹点结构
 */
struct TrajectoryPoint {
    Vector3 position;     // 足端位置
    float phase;          // 当前相位 [0, 1]
    bool is_swing;        // 是否摆动相
};

/**
 * @brief 步态参数结构
 */
struct GaitParams {
    float stride_length;  // 步长 (米)
    float step_height;    // 抬腿高度 (米)
    float frequency;      // 步频 (Hz)
    float duty_cycle;     // 占空比 (Walk: 0.75)
    
    GaitParams() 
        : stride_length(0.05f)
        , step_height(0.03f)
        , frequency(0.8f)
        , duty_cycle(0.75f) {}
};

/**
 * @brief 轨迹类型枚举
 */
enum class TrajectoryType {
    CYCLOID,    // 摆线轨迹（推荐）
    ELLIPSE,    // 椭圆轨迹
    BEZIER      // 贝塞尔曲线
};

/**
 * @brief 步态类型枚举
 */
enum class GaitType {
    WALK,       // Walk步态（三足支撑）
    TROT,       // Trot步态（对角腿）
    STAND       // 站立
};

/**
 * @brief 腿部状态结构
 */
struct LegState {
    float phase_offset;       // 相位偏移
    float current_phase;      // 当前相位
    Vector3 foot_position;    // 足端位置
    bool is_swing;            // 是否摆动相
    
    LegState() : phase_offset(0.0f), current_phase(0.0f), is_swing(false) {}
};

} // namespace Gait
} // namespace Robot
