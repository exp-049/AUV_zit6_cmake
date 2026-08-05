#ifndef PUSHROD_DRIVER_HPP
#define PUSHROD_DRIVER_HPP

#include "Pushrod_Backend.hpp"

namespace auv {
namespace peripheral {

class Pushrod_Driver {
public:
  explicit Pushrod_Driver(PushrodBackend *backend);
  ~Pushrod_Driver();

  bool Init();
  void start();
  bool sendTask(const PushrodTask &task);
  bool readAck(PushrodAck *ack);
  bool isSupported() const;

private:
  PushrodBackend *backend_;
};

} // namespace peripheral
} // namespace auv

#endif
