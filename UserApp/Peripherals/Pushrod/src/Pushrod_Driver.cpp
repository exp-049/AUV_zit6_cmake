#include "Pushrod_Driver.hpp"

namespace auv {
namespace peripheral {

Pushrod_Driver::Pushrod_Driver(PushrodBackend *backend) : backend_(backend) {}

Pushrod_Driver::~Pushrod_Driver() {}

bool Pushrod_Driver::Init() {
  return backend_ != nullptr && backend_->init();
}

void Pushrod_Driver::start() {
  if (backend_ != nullptr) {
    backend_->start();
  }
}

void Pushrod_Driver::poll(uint32_t now_ms) {
  if (backend_ != nullptr) {
    backend_->poll(now_ms);
  }
}

void Pushrod_Driver::stop() {
  if (backend_ != nullptr) {
    backend_->stop();
  }
}

bool Pushrod_Driver::sendTask(const PushrodTask &task) {
  return backend_ != nullptr && backend_->sendTask(task);
}

bool Pushrod_Driver::readAck(PushrodAck *ack) {
  return backend_ != nullptr && backend_->readAck(ack);
}

bool Pushrod_Driver::isSupported() const {
  return backend_ != nullptr && backend_->isSupported();
}

} // namespace peripheral
} // namespace auv
