/**
 * @file INS_Driver.hpp
 * @brief NAV-300 集成惯导驱动类
 */

#ifndef __INS_DRIVER_HPP
#define __INS_DRIVER_HPP

#include "MotionContext.hpp"
#include <cstdint>
#include <cstring>

namespace auv {
namespace peripheral {

/**
 * @struct InsPortOps
 * @brief INS 惯导硬件操作接口（函数指针表）
 */
struct InsPortOps {
  void *ctx;
  bool (*init)(void *ctx);
  uint16_t (*read)(void *ctx, uint8_t *buf, uint16_t max_len);
  bool (*transmit)(void *ctx, const uint8_t *data, uint16_t len);
  void (*getDiagnostics)(void *ctx, struct InsPortDiagnostics *out);
};

/** @brief INS UART/DMA 快照，供调试输出使用。 */
struct InsPortDiagnostics {
  uint32_t read_events = 0;
  uint32_t total_bytes = 0;
  uint32_t valid_frames = 0;
  uint32_t invalid_frames = 0;
  uint16_t write_pos = 0;
  uint16_t dma_remaining = 0;
  bool dma_enabled = false;
  uint32_t uart_isr = 0;
  uint8_t rx_preview[4] = {};
};

/**
 * @class INS_Driver
 * @brief 惯导驱动实现类
 */
class INS_Driver {
public:
  static constexpr uint16_t kFrameSize = 133U;

  INS_Driver(InsPortOps ops) : ops_(ops) {}

  void init();
  bool update(auv::motion::NavState &state);
  auv::motion::NavState getNavState() const { return state_; }
  float getManometerZ() const { return manometer_z_; }
  bool isDataFresh() const;

  /** @brief 拷贝最近一次校验通过的原始帧，供 Debug 输出使用。 */
  uint16_t copyLastFrame(uint8_t *dst, uint16_t max_len) const;

  /** @brief 获取驱动和底层 DMA 的诊断快照。 */
  void getDiagnostics(InsPortDiagnostics &out) const;

  void sendCommand(uint8_t cmd_id, uint8_t value);
  void sendCommand(uint8_t cmd_id, const uint8_t *data, uint8_t data_len);
  void resetPosition();
  void setDvlPower(bool on);
  void restart();
  void setInitialPosition(double lat, double lon);

private:
  struct ProtocolPacket {
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
    float roll_rate = 0.0f;
    float pitch_rate = 0.0f;
    float yaw_rate = 0.0f;
    float velocity_x = 0.0f;
    float velocity_y = 0.0f;
    float velocity_z = 0.0f;
    int32_t latitude_e7 = 0;
    int32_t longitude_e7 = 0;
    float combined_depth = 0.0f;
    float north_offset = 0.0f;
    float east_offset = 0.0f;
    float pressure_depth = 0.0f;
    float dvl_altitude = 0.0f;
    uint8_t sensor_flags = 0U;
    uint8_t navigation_mode = 0U;
  };

  /** Private NAV-300 parser and command codec owned by this INS driver. */
  class ProtocolParser {
  public:
    static constexpr uint16_t commandFrameSize() { return 14U; }

    void reset();
    int pushByte(uint8_t byte);
    bool decode(ProtocolPacket &packet) const;
    uint16_t copyLastFrame(uint8_t *dst, uint16_t max_len) const;

    static uint16_t encodeCommand(uint8_t command_id, const uint8_t *data,
                                  uint8_t data_len, uint8_t *output,
                                  uint16_t output_size);
    static void encodeInitialPosition(double latitude, double longitude,
                                      uint8_t output[8]);

  private:
    bool validateWorkingFrame() const;
    void resynchronizeAfterInvalidFrame();

    static constexpr uint16_t kFrameSize = 133U;
    static constexpr uint16_t kMaxFrameSize = 256U;
    __attribute__((aligned(4))) uint8_t working_frame_[kMaxFrameSize] = {};
    uint8_t last_frame_[kFrameSize] = {};
    uint16_t working_length_ = 0U;
    bool has_last_frame_ = false;
  };

  InsPortOps ops_;
  uint32_t rx_total_bytes_ = 0; ///< 累计接收字节数（调试统计）
  uint32_t last_update_ms_ = 0; ///< 上次收到有效包的时间

  ProtocolParser protocol_parser_;

  auv::motion::NavState state_{};      ///< 缓存的最新有效位姿状态
  auv::motion::NavState prev_state_{}; ///< 上一帧状态，用于速度差分估计
  bool has_prev_state_ = false;
  uint32_t valid_frames_ = 0;
  uint32_t invalid_frames_ = 0;

  // 压力计深度 (m)
  float manometer_z_ = 0.0f;

  // 内部私有方法
  void decodePacket(const ProtocolPacket &packet, auv::motion::NavState &state);
};

} // namespace peripheral
} // namespace auv

#endif
