#ifndef PUSHROD_BACKEND_HPP
#define PUSHROD_BACKEND_HPP

#include <stdint.h>

namespace auv {
namespace peripheral {

/** Application-level pushrod command. Wire encoding belongs to the backend. */
struct PushrodTask {
  uint32_t task_id = 0U;
  int16_t power_x1000 = 0;
  uint32_t duration_ms = 0U;
};

/** Application-level pushrod acknowledgement. Wire decoding belongs to the backend. */
struct PushrodAck {
  uint32_t task_id = 0U;
  uint8_t result = 0U;
  uint8_t queue_count = 0U;
  uint8_t ready = 0U;
};

namespace pushrod {
enum Result : uint8_t {
  kOk = 0x00U,
  kInvalidFrame = 0x01U,
  kCrcError = 0x02U,
  kPowerOutOfRange = 0x03U,
  kDurationInvalid = 0x04U,
  kIdOutOfOrder = 0x05U,
  kIdConflict = 0x06U,
  kQueueFull = 0x07U,
  kNotInitialized = 0x08U,
};
} // namespace pushrod

class PushrodBackend {
public:
  virtual ~PushrodBackend() = default;

  virtual bool init() = 0;
  virtual void start() = 0;
  virtual bool sendTask(const PushrodTask &task) = 0;
  virtual bool readAck(PushrodAck *ack) = 0;
  virtual bool isSupported() const = 0;
};

} // namespace peripheral
} // namespace auv

#endif
