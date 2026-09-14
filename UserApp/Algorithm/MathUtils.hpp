/**
 * @file MathUtils.hpp
 * @brief 6DOF 数学工具：矩阵类型别名、旋转矩阵、Fossen 模型基础
 *
 * 依赖：Eigen 3.4+ (仅 Core 模块，固定大小矩阵，零动态内存)
 *
 * 提供：
 * 1. 6DOF 固定大小矩阵/向量类型别名 (栈分配)
 * 2. 欧拉角 → 6×6 运动学变换矩阵 R(η)
 * 3. 6DOF 索引常量与辅助转换函数
 */

#pragma once

#include <Eigen/Core>
#include <array>
#include <cmath>

namespace auv {
namespace algorithm {
namespace math {

// ============================================================================
// 6DOF 固定大小矩阵类型别名（栈分配，零动态内存）
// ============================================================================

/// 6-DOF 向量：位姿 η、速度 ν、力/力矩 τ
using Vector6f = Eigen::Matrix<float, 6, 1>;

/// 6×6 矩阵：旋转矩阵、惯量矩阵、阻尼矩阵等
using Matrix6f = Eigen::Matrix<float, 6, 6>;

/// 6×N 矩阵：推进器配置矩阵（N 为推进器数量，运行时确定）
using Matrix6xNf = Eigen::Matrix<float, 6, Eigen::Dynamic>;

// ============================================================================
// 6DOF 索引常量与数值常量（编译期可求值）
// ============================================================================

/// 6-DOF 空间维度
constexpr int kDim = 6;

/// 3-DOF 旋转子空间维度
constexpr int kRotDim = 3;

/// 数值阈值：用于前向 T 矩阵万向锁检测的 cosθ 下限
constexpr float kCosThetaEps = 1e-6f;

/// 角度归一化常量
constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 6.28318530717958647692f;

/**
 * @brief 6DOF 轴索引枚举
 *
 * 遵循 SNAME 标准符号：
 *   0 - X     (Surge)  前进
 *   1 - Y     (Sway)   横移
 *   2 - Z     (Heave)  升降
 *   3 - ROLL  (Roll)   横滚 (绕 X 轴)
 *   4 - PITCH (Pitch)  俯仰 (绕 Y 轴)
 *   5 - YAW   (Yaw)    偏航 (绕 Z 轴)
 */
enum Axis : int { X = 0, Y = 1, Z = 2, ROLL = 3, PITCH = 4, YAW = 5 };

/**
 * @brief std::array<float, 6> → Eigen::Vector6f
 *
 * 零拷贝映射（前提是 array 内存布局连续）。
 * 返回的 Vector6f 与 arr 共享内存，修改 vec 会影响 arr。
 */
inline Vector6f arrayToVector(const std::array<float, 6> &arr) noexcept {
  return Eigen::Map<const Vector6f>(arr.data());
}

/**
 * @brief Eigen::Vector6f → std::array<float, 6>
 *
 * 将向量数据复制到数组中。
 */
inline void vectorToArray(const Vector6f &vec,
                          std::array<float, 6> &arr) noexcept {
  Eigen::Map<Vector6f>(arr.data()) = vec;
}

// ============================================================================
// 6×6 运动学变换矩阵 R(η)
// ============================================================================

/**
 * @brief 从欧拉角生成 6×6 运动学变换矩阵 R(η)
 *
 * 矩阵结构（分块）：
 * @code
 *        |  R_ZYX(φ,θ,ψ)     0₃ₓ₃   |
 * R(η) = |                             |
 *        |     0₃ₓ₃         T(φ,θ)    |
 * @endcode
 *
 *
 * @param roll  横滚角 φ (rad)
 * @param pitch 俯仰角 θ (rad)
 * @param yaw   偏航角 ψ (rad)
 * @return Matrix6f 6×6 运动学变换矩阵
 */
inline Matrix6f eulerToRotationMatrix(float roll, float pitch,
                                      float yaw) noexcept {
  Matrix6f R = Matrix6f::Zero();

  float cr = std::cos(roll);  // cφ
  float sr = std::sin(roll);  // sφ
  float cp = std::cos(pitch); // cθ
  float sp = std::sin(pitch); // sθ
  float cy = std::cos(yaw);   // cψ
  float sy = std::sin(yaw);   // sψ

  // ---- 左上 3×3：线速度旋转矩阵 R_ZYX (World = R · Body) ----
  auto R_rot = R.topLeftCorner<kRotDim, kRotDim>();
  R_rot(0, 0) = cy * cp;
  R_rot(0, 1) = cy * sp * sr - sy * cr;
  R_rot(0, 2) = cy * sp * cr + sy * sr;

  R_rot(1, 0) = sy * cp;
  R_rot(1, 1) = sy * sp * sr + cy * cr;
  R_rot(1, 2) = sy * sp * cr - cy * sr;

  R_rot(2, 0) = -sp;
  R_rot(2, 1) = cp * sr;
  R_rot(2, 2) = cp * cr;

  // ---- 右下 3×3：欧拉角速率变换矩阵 T(φ,θ) ----
  auto R_T = R.bottomRightCorner<kRotDim, kRotDim>();
  if (std::abs(cp) > kCosThetaEps) {
    float tp = sp / cp; // tan(θ)
    R_T(0, 0) = 1.0f;
    R_T(0, 1) = sr * tp;
    R_T(0, 2) = cr * tp;

    R_T(1, 0) = 0.0f;
    R_T(1, 1) = cr;
    R_T(1, 2) = -sr;

    R_T(2, 0) = 0.0f;
    R_T(2, 1) = sr / cp;
    R_T(2, 2) = cr / cp;
  } else {
    R_T(0, 0) = 1.0f;
    R_T(1, 1) = 1.0f;
    R_T(2, 2) = 1.0f;
  }

  return R;
}

/**
 * @brief 从欧拉角生成 6×6 运动学逆变换矩阵 R(η)⁻¹
 *
 * 用于将世界系向量变换到机体系：ν_body = R(η)⁻¹ · η_dot_world
 *
 * 分块结构：
 * @code
 *           |  R_ZYX(φ,θ,ψ)ᵀ   0₃ₓ₃   |
 * R(η)⁻¹  = |                           |
 *           |     0₃ₓₓ       T(φ,θ)⁻¹  |
 * @endcode
 *
 * - 左上 3×3：R_ZYXᵀ = R_ZYX⁻¹（旋转矩阵是正交矩阵）
 * - 右下 3×3：T⁻¹ 欧拉角速率逆变换矩阵
 *
 * @param roll  横滚角 φ (rad)
 * @param pitch 俯仰角 θ (rad)
 * @param yaw   偏航角 ψ (rad)
 * @return Matrix6f 6×6 运动学逆变换矩阵
 */
inline Matrix6f eulerToRotationMatrixInverse(float roll, float pitch,
                                             float yaw) noexcept {
  Matrix6f R = Matrix6f::Zero();

  float cr = std::cos(roll);  // cφ
  float sr = std::sin(roll);  // sφ
  float cp = std::cos(pitch); // cθ
  float sp = std::sin(pitch); // sθ
  float cy = std::cos(yaw);   // cψ
  float sy = std::sin(yaw);   // sψ

  // ---- 左上 3×3：R_ZYX 的转置 (Body = R_ZYXᵀ · World) ----
  auto R_rot = R.topLeftCorner<kRotDim, kRotDim>();
  R_rot(0, 0) = cy * cp;
  R_rot(1, 0) = cy * sp * sr - sy * cr;
  R_rot(2, 0) = cy * sp * cr + sy * sr;

  R_rot(0, 1) = sy * cp;
  R_rot(1, 1) = sy * sp * sr + cy * cr;
  R_rot(2, 1) = sy * sp * cr - cy * sr;

  R_rot(0, 2) = -sp;
  R_rot(1, 2) = cp * sr;
  R_rot(2, 2) = cp * cr;

  // ---- 右下 3×3：T⁻¹ 欧拉角速率逆变换矩阵 ----
  auto R_T_inv = R.bottomRightCorner<kRotDim, kRotDim>();
  R_T_inv(0, 0) = 1.0f;
  R_T_inv(0, 1) = 0.0f;
  R_T_inv(0, 2) = -sp;

  R_T_inv(1, 0) = 0.0f;
  R_T_inv(1, 1) = cr;
  R_T_inv(1, 2) = cp * sr;

  R_T_inv(2, 0) = 0.0f;
  R_T_inv(2, 1) = -sr;
  R_T_inv(2, 2) = cp * cr;

  return R;
}

/**
 * @brief 对 6DOF 向量应用旋转矩阵 R(η)：ν_world = R(η) · ν_body
 *
 * 使用 Eigen::Map 零拷贝映射，.noalias() 避免隐式临时变量。
 * 输入输出指向不同的物理缓冲区，无内存重叠风险。
 */
inline void applyRotationToWorld(const float body_in[6], float world_out[6],
                                 float roll, float pitch, float yaw) noexcept {
  Eigen::Map<const Vector6f> v_in(body_in);
  Eigen::Map<Vector6f> v_out(world_out);
  v_out.noalias() = eulerToRotationMatrix(roll, pitch, yaw) * v_in;
}

/**
 * @brief 对 6DOF 向量应用逆变换矩阵：ν_body = R(η)⁻¹ · η_world
 *
 * 将世界系向量变换到机体系。
 * 内部使用 eulerToRotationMatrixInverse()，正确处理 T⁻¹ 而非 Tᵀ。
 *
 * 使用 Eigen::Map 零拷贝映射，.noalias() 避免隐式临时变量。
 */
inline void applyRotationToBody(const float world_in[6], float body_out[6],
                                float roll, float pitch, float yaw) noexcept {
  Eigen::Map<const Vector6f> v_in(world_in);
  Eigen::Map<Vector6f> v_out(body_out);
  v_out.noalias() = eulerToRotationMatrixInverse(roll, pitch, yaw) * v_in;
}

/**
 * @brief 将世界系 6DoF 力/力矩变换到机体系。
 *
 * Wrench 的线力和力矩都按同一个旋转矩阵变换；它不是欧拉角速度，
 * 因此不能复用上面的 T(roll,pitch) 角速度逆变换。
 */
inline void applyWrenchToBody(const float world_in[6], float body_out[6],
                              float roll, float pitch, float yaw) noexcept {
  const auto full_rotation = eulerToRotationMatrix(roll, pitch, yaw);
  const auto R = full_rotation.topLeftCorner<3, 3>();
  Eigen::Map<const Eigen::Matrix<float, 3, 1>> force_world(world_in);
  Eigen::Map<const Eigen::Matrix<float, 3, 1>> moment_world(world_in + 3);
  Eigen::Map<Eigen::Matrix<float, 3, 1>> force_body(body_out);
  Eigen::Map<Eigen::Matrix<float, 3, 1>> moment_body(body_out + 3);
  force_body.noalias() = R.transpose() * force_world;
  moment_body.noalias() = R.transpose() * moment_world;
}

} // namespace math
} // namespace algorithm
} // namespace auv
