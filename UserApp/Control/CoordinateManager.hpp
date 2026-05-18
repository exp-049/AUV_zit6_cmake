/**
 * @file CoordinateManager.hpp
 * @brief 坐标系变换管理工具类
 * 
 * 职责：
 * 1. 提供北东地 (NED) 世界坐标系与潜器机体系 (Body) 之间的向量投影变换。
 * 2. 处理 Yaw (航向角) 在二维平面上的旋转矩阵。
 */

#pragma once

#include <cmath>

namespace auv {
namespace control {

/**
 * @class CoordinateManager
 * @brief 静态坐标变换工具
 */
class CoordinateManager {
public:
    /**
     * @brief 世界系 (NED) 向量 -> 机体系 (Body) 向量 
     * 
     * 公式：
     * Body_X = NED_X * cos(yaw) + NED_Y * sin(yaw)
     * Body_Y = -NED_X * sin(yaw) + NED_Y * cos(yaw)
     * 
     * @param yaw 当前航向角 (rad)
     * @param world_x 世界系 X (北向) 向量分量
     * @param world_y 世界系 Y (东向) 向量分量
     * @param[out] body_x 机体系前进方向向量分量
     * @param[out] body_y 机体系横移方向向量分量
     */
    static void worldToBody(float yaw, float world_x, float world_y, float& body_x, float& body_y);

    /**
     * @brief 机体系 (Body) 向量 -> 世界系 (NED) 向量
     * 
     * 公式：旋转矩阵的逆（即转置矩阵）
     * 
     * @param yaw 当前航向角 (rad)
     * @param body_x 机体系前进方向向量分量
     * @param body_y 机体系横移方向向量分量
     * @param[out] world_x 世界系 X (北向) 向量分量
     * @param[out] world_y 世界系 Y (东向) 向量分量
     */
    static void bodyToWorld(float yaw, float body_x, float body_y, float& world_x, float& world_y);
    /**
     * @brief 角度归一化到 [-PI, PI]
     * @param angle 输入角度 (rad)
     * @return 归一化后的角度 (rad)
     */
    static float normalizeAngle(float angle);
};

} // namespace control
} // namespace auv
