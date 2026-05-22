/**
 * @file INS_Driver.hpp
 * @brief NAV-300 集成惯导驱动类
 */

#ifndef __INS_DRIVER_HPP
#define __INS_DRIVER_HPP

#include "CommonConfig.hpp"
#include "SerialPort.hpp"
#include <cstdint>
#include <cstring>

namespace auv {
namespace device {

/**
 * @class INS_Driver
 * @brief 惯导驱动实现类
 */
class INS_Driver {
public:
  /**
   * @brief 构造函数
   * @param rx_uart 接收数据的 UART 句柄 (通常开启 DMA)
   * @param tx_uart 发送指令的 UART 句柄
   */
  INS_Driver(UART_HandleTypeDef *rx_uart, UART_HandleTypeDef *tx_uart,
             uint8_t *ext_rx_buf, uint16_t rx_buf_size)
      : rx_port_(rx_uart, ext_rx_buf, rx_buf_size), tx_uart_(tx_uart) {}

  INS_Driver(UART_HandleTypeDef *rx_uart, UART_HandleTypeDef *tx_uart)
      : rx_port_(rx_uart, 512), tx_uart_(tx_uart) {}

  void init();
  bool update(auv::common::NavState &state);
  auv::common::NavState getNavState() const { return state_; }
  float getManometerZ() const { return manometer_z_; }

  /**
   * @brief 检查惯导数据是否新鲜 (200ms 内有更新)
   */
  bool isDataFresh() const;

  // --- 指令发送接口 ---

  /**
   * @brief 发送控制指令 (带 1 字节 value)
   */
  void sendCommand(uint8_t cmd_id, uint8_t value);

  /**
   * @brief 发送带多字节数据的控制指令
   */
  void sendCommand(uint8_t cmd_id, const uint8_t *data, uint8_t data_len);

  /**
   * @brief 全局位置增量清零 (ID: 0x02)
   */
  void resetPosition();

  /**
   * @brief 控制 DVL 模块电源状态 (ID: 0x03)
   */
  void setDvlPower(bool on);

  /**
   * @brief 触发惯导系统重启 (ID: 0x04)
   */
  void restart();

  /**
   * @brief 惯导初始位置装订 (ID: 0x20)
   * @param lat 纬度 (度)
   * @param lon 经度 (度)
   */
  void setInitialPosition(double lat, double lon);

  // 新增：手动注入“家坐标”，实现软件层面的坐标系偏置
  void setHomeOffset(float x, float y, float z, float yaw) {
    use_offset_ = true;
    offset_x_ = x;
    offset_y_ = y;
    offset_z_ = z;
    // offset_yaw_ = yaw;
    offset_yaw = 0;
  }

  void clearHomeOffset() { use_offset_ = false; }

private:
  porting::SerialPort rx_port_; ///< 串口接收驱动
  UART_HandleTypeDef *tx_uart_; ///< 指令发送串口句柄
  uint32_t rx_total_bytes_ = 0; ///< 累计接收字节数（调试统计）
  uint32_t last_update_ms_ = 0; ///< 上次收到有效包的时间

  static constexpr uint16_t kMaxFrameSize = 256;
  static constexpr uint16_t kMinFrameSize = 133;
  uint8_t packet_buf_[kMaxFrameSize] = {0}; ///< 帧解析临时缓冲区
  uint16_t frame_len_ = 0;                  ///< 当前解析长度

  auv::common::NavState state_{};      ///< 缓存的最新有效位姿状态
  auv::common::NavState prev_state_{}; ///< 上一帧状态，用于速度差分估计
  bool has_prev_state_ = false;

  // 坐标偏置变量
  bool use_offset_ = false;
  float offset_x_ = 0, offset_y_ = 0, offset_z_ = 0, offset_yaw_ = 0;

  // 压力计深度 (m)
  float manometer_z_ = 0.0f;

  // 内部私有方法
  uint8_t checkData(const uint8_t *data, uint8_t size);
  bool parseByte(uint8_t b);
  bool validateFrame();
  void decodePacket(auv::common::NavState &state);
};

} // namespace device
} // namespace auv

#endif
