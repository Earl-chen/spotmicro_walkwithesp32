#pragma once

#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Robot {
namespace Kinematics {

/**
 * @brief 3D坐标变换工具类
 *
 * 提供旋转矩阵计算和坐标系变换功能
 * 与Python版本core/utils/rotations.py和core/transform.py保持一致
 */
class CoordinateTransform {
public:
    // 3x3旋转矩阵类型定义
    using Matrix3x3 = std::array<std::array<float, 3>, 3>;

    // 3D向量类型定义
    struct Vector3 {
        float x, y, z;
        Vector3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}

        Vector3 operator+(const Vector3& other) const {
            return Vector3(x + other.x, y + other.y, z + other.z);
        }

        Vector3 operator-(const Vector3& other) const {
            return Vector3(x - other.x, y - other.y, z - other.z);
        }

        float& operator[](int index) {
            return (&x)[index];
        }

        const float& operator[](int index) const {
            return (&x)[index];
        }
    };

    // 6DOF姿态结构
    struct Pose6DOF {
        Vector3 position;      // 位置 (x, y, z)
        Vector3 orientation;   // 姿态 (roll, pitch, yaw) 弧度

        Pose6DOF(float x = 0, float y = 0, float z = 0,
                 float roll = 0, float pitch = 0, float yaw = 0)
            : position(x, y, z), orientation(roll, pitch, yaw) {}
    };

    /**
     * @brief 创建绕X轴的旋转矩阵
     * @param angle 旋转角度 (弧度)
     * @return 3x3旋转矩阵
     */
    static Matrix3x3 rotationX(float angle) {
        float c = cos(angle);
        float s = sin(angle);

        return {{
            {{1, 0, 0}},
            {{0, c, -s}},
            {{0, s, c}}
        }};
    }

    /**
     * @brief 创建绕Y轴的旋转矩阵
     * @param angle 旋转角度 (弧度)
     * @return 3x3旋转矩阵
     */
    static Matrix3x3 rotationY(float angle) {
        float c = cos(angle);
        float s = sin(angle);

        return {{
            {{c, 0, s}},
            {{0, 1, 0}},
            {{-s, 0, c}}
        }};
    }

    /**
     * @brief 创建绕Z轴的旋转矩阵
     * @param angle 旋转角度 (弧度)
     * @return 3x3旋转矩阵
     */
    static Matrix3x3 rotationZ(float angle) {
        float c = cos(angle);
        float s = sin(angle);

        return {{
            {{c, -s, 0}},
            {{s, c, 0}},
            {{0, 0, 1}}
        }};
    }

    /**
     * @brief 创建Roll-Pitch-Yaw复合旋转矩阵
     * @param roll 绕X轴旋转角度 (弧度)
     * @param pitch 绕Y轴旋转角度 (弧度)
     * @param yaw 绕Z轴旋转角度 (弧度)
     * @return 3x3旋转矩阵
     */
    static Matrix3x3 rotationRPY(float roll, float pitch, float yaw) {
        // 按照Roll(X) -> Pitch(Y) -> Yaw(Z)的顺序组合
        Matrix3x3 Rx = rotationX(roll);
        Matrix3x3 Ry = rotationY(pitch);
        Matrix3x3 Rz = rotationZ(yaw);

        // 计算 Rz * Ry * Rx
        return matrixMultiply(Rz, matrixMultiply(Ry, Rx));
    }

    /**
     * @brief 矩阵乘法 A * B
     */
    static Matrix3x3 matrixMultiply(const Matrix3x3& A, const Matrix3x3& B) {
        Matrix3x3 result = {{{0}}};

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    result[i][j] += A[i][k] * B[k][j];
                }
            }
        }

        return result;
    }

    /**
     * @brief 矩阵乘向量 M * v
     */
    static Vector3 matrixVectorMultiply(const Matrix3x3& M, const Vector3& v) {
        return Vector3(
            M[0][0]*v.x + M[0][1]*v.y + M[0][2]*v.z,
            M[1][0]*v.x + M[1][1]*v.y + M[1][2]*v.z,
            M[2][0]*v.x + M[2][1]*v.y + M[2][2]*v.z
        );
    }

    /**
     * @brief 单位矩阵
     */
    static Matrix3x3 identity() {
        return {{
            {{1, 0, 0}},
            {{0, 1, 0}},
            {{0, 0, 1}}
        }};
    }

    /**
     * @brief 矩阵转置
     */
    static Matrix3x3 transpose(const Matrix3x3& M) {
        Matrix3x3 result;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                result[i][j] = M[j][i];
            }
        }
        return result;
    }

    /**
     * @brief 角度转弧度
     */
    static float degreesToRadians(float degrees) {
        return degrees * M_PI / 180.0f;
    }

    /**
     * @brief 弧度转角度
     */
    static float radiansToDegrees(float radians) {
        return radians * 180.0f / M_PI;
    }
};

/**
 * @brief 坐标系变换管理器
 *
 * 管理世界坐标系 -> 机体坐标系 -> 髋坐标系的层次化变换
 */
class FrameManager {
private:
    CoordinateTransform::Pose6DOF body_pose_;  // 机体相对于世界的位姿

public:
    /**
     * @brief 设置机体位姿
     */
    void setBodyPose(const CoordinateTransform::Pose6DOF& pose);

    /**
     * @brief 获取机体位姿
     */
    CoordinateTransform::Pose6DOF getBodyPose() const;

    /**
     * @brief 将点从世界坐标系变换到机体坐标系
     */
    CoordinateTransform::Vector3 transformWorldToBody(const CoordinateTransform::Vector3& world_point) const;

    /**
     * @brief 将点从机体坐标系变换到世界坐标系
     */
    CoordinateTransform::Vector3 transformBodyToWorld(const CoordinateTransform::Vector3& body_point) const;

    /**
     * @brief 将点从机体坐标系变换到指定髋坐标系
     */
    CoordinateTransform::Vector3 transformBodyToHip(const CoordinateTransform::Vector3& body_point,
                                                   const CoordinateTransform::Vector3& hip_offset) const;

    /**
     * @brief 将点从髋坐标系变换到机体坐标系
     */
    CoordinateTransform::Vector3 transformHipToBody(const CoordinateTransform::Vector3& hip_point,
                                                   const CoordinateTransform::Vector3& hip_offset) const;

    /**
     * @brief 将点从世界坐标系变换到指定髋坐标系
     */
    CoordinateTransform::Vector3 transformWorldToHip(const CoordinateTransform::Vector3& world_point,
                                                    const CoordinateTransform::Vector3& hip_offset) const;

    /**
     * @brief 将点从髋坐标系变换到世界坐标系
     */
    CoordinateTransform::Vector3 transformHipToWorld(const CoordinateTransform::Vector3& hip_point,
                                                    const CoordinateTransform::Vector3& hip_offset) const;

    /**
     * @brief 调试输出当前变换状态
     */
    void printTransformInfo() const;
};

} // namespace Kinematics
} // namespace Robot