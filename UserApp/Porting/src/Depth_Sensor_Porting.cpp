#include "Depth_Sensor_Porting.hpp"

#include "SerialHandles.hpp"

#if DEPTH_SENSOR_BACKEND == DEPTH_BACKEND_SELF_UART
#include "Pushrod_Porting.hpp"
#include "UART_MS5837Backend.hpp"
#elif DEPTH_SENSOR_BACKEND == DEPTH_BACKEND_COMMERCIAL_UART
#include "DepthCalcBoard_Porting.hpp"
#include "UART_DepthBackend.hpp"
#elif DEPTH_SENSOR_BACKEND == DEPTH_BACKEND_I2C
#include "I2C_DepthBackend.hpp"
#include "MS5837_Porting.hpp"
#include "i2c.h"
#endif

namespace auv {
namespace porting {
namespace {

class UnsupportedPushrodBackend final
    : public auv::peripheral::PushrodBackend {
public:
  bool init() override { return false; }
  void start() override {}
  bool sendTask(const auv::peripheral::PushrodTask &) override { return false; }
  bool readAck(auv::peripheral::PushrodAck *) override { return false; }
  bool isSupported() const override { return false; }
};

#if DEPTH_SENSOR_BACKEND == DEPTH_BACKEND_SELF_UART
struct SelectedDrivers {
  Pushrod_Porting port;
  auv::peripheral::Self_CalcBoard_Link link;
  UnsupportedPushrodBackend unsupported_pushrod;
  auv::peripheral::Depth_Sensor_Driver depth;
  auv::peripheral::Pushrod_Driver pushrod;

  SelectedDrivers()
      : port(&AUV_UART_DEPTH_CAL),
        link(auv::peripheral::UART_MS5837PortOps{
            .ctx = &port,
            .transmit = &Pushrod_Porting::transmitPort,
            .poll = &Pushrod_Porting::pollPort,
            .startRx = &Pushrod_Porting::startRxPort,
            .getTickMs = &Pushrod_Porting::getTickPort,
            .getRxRecoveryCount =
                &Pushrod_Porting::getRxRecoveryCountPort,
            .getRxErrorCount = &Pushrod_Porting::getRxErrorCountPort,
            .getLastRxError = &Pushrod_Porting::getLastRxErrorPort,
            .getLastRxRecoveryReason =
                &Pushrod_Porting::getLastRxRecoveryReasonPort,
            .getRxEventCount = &Pushrod_Porting::getRxEventCountPort,
            .getDmaWritePos = &Pushrod_Porting::getDmaWritePosPort,
        }),
        depth(&link, &link), pushrod(&link) {
    port.setBackend(&link);
  }
};
#elif DEPTH_SENSOR_BACKEND == DEPTH_BACKEND_COMMERCIAL_UART
struct SelectedDrivers {
  DepthCalcBoard_Porting port;
  auv::peripheral::UART_DepthBackend depth_backend;
  UnsupportedPushrodBackend unsupported_pushrod;
  auv::peripheral::Depth_Sensor_Driver depth;
  auv::peripheral::Pushrod_Driver pushrod;

  SelectedDrivers()
      : port(&AUV_UART_DEPTH_CAL),
        depth_backend(auv::peripheral::UartPortOps{
            .ctx = &port,
            .transmit = &DepthCalcBoard_Porting::transmitPort,
            .poll = &DepthCalcBoard_Porting::pollPort,
            .startRx = &DepthCalcBoard_Porting::startRxPort,
        }),
        depth(&depth_backend, &unsupported_pushrod),
        pushrod(&unsupported_pushrod) {
    port.setBackend(&depth_backend);
  }
};
#elif DEPTH_SENSOR_BACKEND == DEPTH_BACKEND_I2C
struct SelectedDrivers {
  auv::porting::MS5837_Porting port;
  auv::peripheral::I2C_DepthBackend depth_backend;
  UnsupportedPushrodBackend unsupported_pushrod;
  auv::peripheral::Depth_Sensor_Driver depth;
  auv::peripheral::Pushrod_Driver pushrod;

  SelectedDrivers()
      : port(&hi2c1, 0x76 << 1),
        depth_backend(auv::peripheral::DepthPortOps{
            .ctx = &port,
            .writeByte = &auv::porting::MS5837_Porting::writePort,
            .readByte = &auv::porting::MS5837_Porting::readPortByte,
            .read = &auv::porting::MS5837_Porting::readPort,
            .delay = &auv::porting::MS5837_Porting::delayPort,
            .start = &auv::porting::MS5837_Porting::startPort,
        },
                      0x76 << 1),
        depth(&depth_backend, &unsupported_pushrod),
        pushrod(&unsupported_pushrod) {
    port.setBackend(&depth_backend);
  }
};
#else
#error "Unsupported DEPTH_SENSOR_BACKEND value"
#endif

SelectedDrivers &selectedDrivers() {
  static SelectedDrivers drivers;
  return drivers;
}

} // namespace

auv::peripheral::Depth_Sensor_Driver *getDepthSensorDriver() {
  return &selectedDrivers().depth;
}

auv::peripheral::Pushrod_Driver *getPushrodDriver() {
  return &selectedDrivers().pushrod;
}

} // namespace porting
} // namespace auv
