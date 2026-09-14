#include "Peripheral_Preset.hpp"
#include "Depth_Sensor_Porting.hpp"

#include "Pushrod_Porting_Config.h"
#include "SerialHandles.hpp"

#if PUSHROD_BACKEND == PUSHROD_BACKEND_GPIO
#include "Pushrod_GPIO_Backend.hpp"
#include "Pushrod_GPIO_Porting.hpp"
#endif

#if DEPTH_SENSOR_BACKEND == DEPTH_BACKEND_SELF_UART
#include "Self_CalcBoard_Porting.hpp"
#include "UART_MS5837Backend.hpp"
#elif DEPTH_SENSOR_BACKEND == DEPTH_BACKEND_COMMERCIAL_UART
#include "DepthCalcBoard_Porting.hpp"
#include "UART_DepthBackend.hpp"
#elif DEPTH_SENSOR_BACKEND == DEPTH_BACKEND_M14_UART
#include "DepthCalcBoard_Porting.hpp"
#include "M14_UART_Backend.hpp"
#elif DEPTH_SENSOR_BACKEND == DEPTH_BACKEND_I2C
#include "I2C_DepthBackend.hpp"
#include "MS5837_Porting.hpp"
#include "i2c.h"
#endif

#if PUSHROD_BACKEND != PUSHROD_BACKEND_SELF_UART &&                         \
    PUSHROD_BACKEND != PUSHROD_BACKEND_GPIO
#error "Unsupported PUSHROD_BACKEND value"
#endif

/* PB7/PB8 are I2C1_SDA/I2C1_SCL in the current board design. */
#if DEPTH_SENSOR_BACKEND == DEPTH_BACKEND_I2C &&                            \
    PUSHROD_BACKEND == PUSHROD_BACKEND_GPIO
#error "GPIO pushrod backend conflicts with I2C depth backend on PB7/PB8"
#endif

namespace auv {
namespace porting {
namespace {

class UnsupportedPushrodBackend final
    : public auv::peripheral::PushrodBackend {
public:
  bool init() override { return false; }
  void start() override {}
  void poll(uint32_t now_ms) override { (void)now_ms; }
  void stop() override {}
  bool sendTask(const auv::peripheral::PushrodTask &) override { return false; }
  bool readAck(auv::peripheral::PushrodAck *) override { return false; }
  bool isSupported() const override { return false; }
};

#if DEPTH_SENSOR_BACKEND == DEPTH_BACKEND_SELF_UART
struct SelectedDrivers {
  Self_CalcBoard_Porting depth_port;
  auv::peripheral::Self_CalcBoard_Link depth_link;
  auv::peripheral::Depth_Sensor_Driver depth;

#if PUSHROD_BACKEND == PUSHROD_BACKEND_GPIO
  Pushrod_GPIO_Porting pushrod_port;
  auv::peripheral::Pushrod_GPIO_Backend pushrod_backend;
  auv::peripheral::Pushrod_Driver pushrod;

  SelectedDrivers()
      : depth_port(&AUV_UART_DEPTH_CAL),
        depth_link(auv::peripheral::UART_MS5837PortOps{
            .ctx = &depth_port,
            .transmit = &Self_CalcBoard_Porting::transmitPort,
            .poll = &Self_CalcBoard_Porting::pollPort,
            .startRx = &Self_CalcBoard_Porting::startRxPort,
            .getTickMs = &Self_CalcBoard_Porting::getTickPort,
            .getRxRecoveryCount =
                &Self_CalcBoard_Porting::getRxRecoveryCountPort,
            .getRxErrorCount = &Self_CalcBoard_Porting::getRxErrorCountPort,
            .getLastRxError = &Self_CalcBoard_Porting::getLastRxErrorPort,
            .getLastRxRecoveryReason =
                &Self_CalcBoard_Porting::getLastRxRecoveryReasonPort,
            .getRxEventCount = &Self_CalcBoard_Porting::getRxEventCountPort,
            .getDmaWritePos = &Self_CalcBoard_Porting::getDmaWritePosPort,
        }),
        depth(&depth_link),
        pushrod_backend(auv::peripheral::PushrodGpioPortOps{
            .ctx = &pushrod_port,
            .init = &Pushrod_GPIO_Porting::initPort,
            .setOutputs = &Pushrod_GPIO_Porting::setOutputsPort,
            .getTickMs = &Pushrod_GPIO_Porting::getTickPort,
        }),
        pushrod(&pushrod_backend) {
    depth_port.setBackend(&depth_link);
  }
#else
  auv::peripheral::Pushrod_Driver pushrod;

  SelectedDrivers()
      : depth_port(&AUV_UART_DEPTH_CAL),
        depth_link(auv::peripheral::UART_MS5837PortOps{
            .ctx = &depth_port,
            .transmit = &Self_CalcBoard_Porting::transmitPort,
            .poll = &Self_CalcBoard_Porting::pollPort,
            .startRx = &Self_CalcBoard_Porting::startRxPort,
            .getTickMs = &Self_CalcBoard_Porting::getTickPort,
            .getRxRecoveryCount =
                &Self_CalcBoard_Porting::getRxRecoveryCountPort,
            .getRxErrorCount = &Self_CalcBoard_Porting::getRxErrorCountPort,
            .getLastRxError = &Self_CalcBoard_Porting::getLastRxErrorPort,
            .getLastRxRecoveryReason =
                &Self_CalcBoard_Porting::getLastRxRecoveryReasonPort,
            .getRxEventCount = &Self_CalcBoard_Porting::getRxEventCountPort,
            .getDmaWritePos = &Self_CalcBoard_Porting::getDmaWritePosPort,
        }),
        depth(&depth_link), pushrod(&depth_link) {
    depth_port.setBackend(&depth_link);
  }
#endif
};

#elif DEPTH_SENSOR_BACKEND == DEPTH_BACKEND_COMMERCIAL_UART
struct SelectedDrivers {
  DepthCalcBoard_Porting depth_port;
  auv::peripheral::UART_DepthBackend depth_backend;
  auv::peripheral::Depth_Sensor_Driver depth;

#if PUSHROD_BACKEND == PUSHROD_BACKEND_GPIO
  Pushrod_GPIO_Porting pushrod_port;
  auv::peripheral::Pushrod_GPIO_Backend pushrod_backend;
  auv::peripheral::Pushrod_Driver pushrod;

  SelectedDrivers()
      : depth_port(&AUV_UART_DEPTH_CAL),
        depth_backend(auv::peripheral::UartPortOps{
            .ctx = &depth_port,
            .transmit = &DepthCalcBoard_Porting::transmitPort,
            .poll = &DepthCalcBoard_Porting::pollPort,
            .startRx = &DepthCalcBoard_Porting::startRxPort,
        }),
        depth(&depth_backend),
        pushrod_backend(auv::peripheral::PushrodGpioPortOps{
            .ctx = &pushrod_port,
            .init = &Pushrod_GPIO_Porting::initPort,
            .setOutputs = &Pushrod_GPIO_Porting::setOutputsPort,
            .getTickMs = &Pushrod_GPIO_Porting::getTickPort,
        }),
        pushrod(&pushrod_backend) {
    depth_port.setBackend(&depth_backend);
  }
#else
  UnsupportedPushrodBackend unsupported_pushrod;
  auv::peripheral::Pushrod_Driver pushrod;

  SelectedDrivers()
      : depth_port(&AUV_UART_DEPTH_CAL),
        depth_backend(auv::peripheral::UartPortOps{
            .ctx = &depth_port,
            .transmit = &DepthCalcBoard_Porting::transmitPort,
            .poll = &DepthCalcBoard_Porting::pollPort,
            .startRx = &DepthCalcBoard_Porting::startRxPort,
        }),
        depth(&depth_backend),
        pushrod(&unsupported_pushrod) {
    depth_port.setBackend(&depth_backend);
  }
#endif
};

#elif DEPTH_SENSOR_BACKEND == DEPTH_BACKEND_M14_UART
struct SelectedDrivers {
  DepthCalcBoard_Porting depth_port;
  auv::peripheral::M14_UART_Backend depth_backend;
  auv::peripheral::Depth_Sensor_Driver depth;

#if PUSHROD_BACKEND == PUSHROD_BACKEND_GPIO
  Pushrod_GPIO_Porting pushrod_port;
  auv::peripheral::Pushrod_GPIO_Backend pushrod_backend;
  auv::peripheral::Pushrod_Driver pushrod;

  SelectedDrivers()
      : depth_port(&AUV_UART_DEPTH_CAL),
        depth_backend(auv::peripheral::UartPortOps{
            .ctx = &depth_port,
            .transmit = &DepthCalcBoard_Porting::transmitPort,
            .poll = &DepthCalcBoard_Porting::pollPort,
            .startRx = &DepthCalcBoard_Porting::startRxPort,
        }),
        depth(&depth_backend),
        pushrod_backend(auv::peripheral::PushrodGpioPortOps{
            .ctx = &pushrod_port,
            .init = &Pushrod_GPIO_Porting::initPort,
            .setOutputs = &Pushrod_GPIO_Porting::setOutputsPort,
            .getTickMs = &Pushrod_GPIO_Porting::getTickPort,
        }),
        pushrod(&pushrod_backend) {
    depth_port.setBackend(&depth_backend);
  }
#else
  UnsupportedPushrodBackend unsupported_pushrod;
  auv::peripheral::Pushrod_Driver pushrod;

  SelectedDrivers()
      : depth_port(&AUV_UART_DEPTH_CAL),
        depth_backend(auv::peripheral::UartPortOps{
            .ctx = &depth_port,
            .transmit = &DepthCalcBoard_Porting::transmitPort,
            .poll = &DepthCalcBoard_Porting::pollPort,
            .startRx = &DepthCalcBoard_Porting::startRxPort,
        }),
        depth(&depth_backend),
        pushrod(&unsupported_pushrod) {
    depth_port.setBackend(&depth_backend);
  }
#endif
};

#elif DEPTH_SENSOR_BACKEND == DEPTH_BACKEND_I2C
struct SelectedDrivers {
  auv::porting::MS5837_Porting depth_port;
  auv::peripheral::I2C_DepthBackend depth_backend;
  UnsupportedPushrodBackend unsupported_pushrod;
  auv::peripheral::Depth_Sensor_Driver depth;
  auv::peripheral::Pushrod_Driver pushrod;

  SelectedDrivers()
      : depth_port(&hi2c1, 0x76 << 1),
        depth_backend(auv::peripheral::DepthPortOps{
            .ctx = &depth_port,
            .writeByte = &auv::porting::MS5837_Porting::writePort,
            .readByte = &auv::porting::MS5837_Porting::readPortByte,
            .read = &auv::porting::MS5837_Porting::readPort,
            .delay = &auv::porting::MS5837_Porting::delayPort,
            .start = &auv::porting::MS5837_Porting::startPort,
        },
                      0x76 << 1),
        depth(&depth_backend),
        pushrod(&unsupported_pushrod) {
    depth_port.setBackend(&depth_backend);
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
