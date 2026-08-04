#pragma once

#include <cstdint>

namespace auv {
namespace protocol {
namespace ins {

constexpr uint16_t kFrameSize = 133;
constexpr uint16_t kMaxFrameSize = 256;
constexpr uint8_t kHeader1 = 0xFA;
constexpr uint8_t kHeader2 = 0xAF;
constexpr uint8_t kTail1 = 0xFB;
constexpr uint8_t kTail2 = 0xBF;

/** Decoded fields from one valid NAV-300 navigation frame. */
struct NavigationPacket {
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

/**
 * Streaming parser and codec for the fixed-size NAV-300 wire protocol.
 *
 * This class has no UART/HAL or application-state dependency. It only
 * recognizes, validates, stores, and decodes protocol frames.
 */
class Parser {
public:
  Parser() = default;

  void reset();

  /** Feed one byte: 1=valid frame, -1=completed invalid frame, 0=ongoing. */
  int pushByte(uint8_t byte);

  /** Decode the most recently validated frame. */
  bool decode(NavigationPacket &packet) const;

  /** Copy the most recently validated raw frame for diagnostics. */
  uint16_t copyLastFrame(uint8_t *dst, uint16_t max_len) const;

  static constexpr uint16_t commandFrameSize() { return 14U; }

  /** Encode an INS command frame; returns 14 on success, otherwise 0. */
  static uint16_t encodeCommand(uint8_t command_id, const uint8_t *data,
                                uint8_t data_len, uint8_t *output,
                                uint16_t output_size);

  /** Encode the big-endian fixed-point latitude/longitude command payload. */
  static void encodeInitialPosition(double latitude, double longitude,
                                    uint8_t output[8]);

private:
  bool validateWorkingFrame() const;
  void resynchronizeAfterInvalidFrame();

  __attribute__((aligned(4))) uint8_t working_frame_[kMaxFrameSize] = {};
  uint8_t last_frame_[kFrameSize] = {};
  uint16_t working_length_ = 0U;
  bool has_last_frame_ = false;
};

} // namespace ins
} // namespace protocol
} // namespace auv
