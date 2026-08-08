#ifndef __MICROROS_PUBLISHER_HPP
#define __MICROROS_PUBLISHER_HPP

#include "AppContext.hpp"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <stdint.h>

#include <rcl_interfaces/msg/log.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/u_int32.h>
#include <zit6_interfaces/msg/zit_status.h>
#include <zit6_interfaces/msg/zit_usbl.h>

/**
 * @class MicroRosPublisher
 * @brief 高频状态发布模块
 *
 * 负责发布：
 * - /zit6/state/pos (30Hz) — 世界系位姿 [x,y,z,roll,pitch,yaw]
 * - /zit6/state/vel (50Hz) — 机体系速度 [u,v,w,p,q,r]
 * - /zit6/state/thr (30Hz) — 机体系推力/力矩 [Fx,Fy,Fz,Mroll,Mpitch,Myaw]
 * - /zit6/state/zithbt (1Hz) — 节点心跳
 * - /zit6/state/status (10Hz) — 核心状态汇总
 * - /zit6/state/USBL (事件驱动) — USBL 有效帧
 * - /zit6/log (事件驱动) — ROS2 日志
 */
class MicroRosPublisher {
public:
  MicroRosPublisher(auv::system::AppContext *ctx) : ctx_(ctx) {}

  /**
   * @brief 初始化所有 Publisher
   * @param node rcl 节点指针
   * @return true 全部初始化成功
   */
  bool init(rcl_node_t *node);

  /**
   * @brief 定时发布（由主循环每 ~1ms 调用）
   * @param now_ms 当前系统毫秒时间
   */
  void publish(uint32_t now_ms);

  /**
   * @brief 销毁所有 Publisher
   * @param node rcl 节点指针
   */
  void cleanup(rcl_node_t *node);

private:
  auv::system::AppContext *ctx_;

  // --- 发布器句柄 ---
  rcl_publisher_t pos_pub_{};
  rcl_publisher_t vel_pub_{};
  rcl_publisher_t thr_pub_{};
  rcl_publisher_t zithbt_pub_{};
  rcl_publisher_t status_pub_{};
  rcl_publisher_t usbl_pub_{};
  rcl_publisher_t log_pub_{};

  bool pos_pub_initialized_ = false;
  bool vel_pub_initialized_ = false;
  bool thr_pub_initialized_ = false;
  bool zithbt_pub_initialized_ = false;
  bool status_pub_initialized_ = false;
  bool usbl_pub_initialized_ = false;
  bool log_pub_initialized_ = false;

  // --- 消息缓冲区（栈分配，生命周期与类绑定） ---
  std_msgs__msg__Float32MultiArray pos_fb_msg_, vel_fb_msg_, thr_fb_msg_;
  std_msgs__msg__UInt32 node_heartbeat_msg_;
  zit6_interfaces__msg__ZitStatus status_msg_;
  zit6_interfaces__msg__ZitUsbl usbl_msg_;
  rcl_interfaces__msg__Log log_msg_;

  // --- 原始数据缓冲区（协议为 6 元素 [X,Y,Z,Roll,Pitch,Yaw]） ---
  float pos_buf_[6] = {0};
  float vel_buf_[6] = {0};
  float thr_buf_[6] = {0};

  // --- LOG 内部静态缓冲区 ---
  char log_msg_buf_[128] = {0};

  // --- 节流定时器 ---
  uint32_t last_hbt_pub_tick_ = 0;
  uint32_t last_vel_pub_tick_ = 0;
  uint32_t last_thr_pub_tick_ = 0;
  uint32_t last_pos_pub_tick_ = 0;
  uint32_t last_status_pub_tick_ = 0;
};

#endif // __MICROROS_PUBLISHER_HPP
