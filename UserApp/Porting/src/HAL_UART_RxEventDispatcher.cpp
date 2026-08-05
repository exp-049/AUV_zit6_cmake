#include "Pushrod_Porting.hpp"
#include "USBL_Porting.hpp"

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                             uint16_t size) {
  auv::porting::USBL_Porting::handleHalRxEvent(huart, size);
  auv::porting::Pushrod_Porting::handleHalRxEvent(huart, size);
}

extern "C" void UserApp_UART4_ErrorHook(UART_HandleTypeDef *huart) {
  auv::porting::Pushrod_Porting::handleHalError(huart);
}
