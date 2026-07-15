#pragma once

// Central UART ownership map. Keep peripheral selection in one place so
// application and porting code do not embed HAL handle names.
#include "usart.h"

#define AUV_UART_USBL huart3
#define AUV_UART_INS huart1
#define AUV_UART_DEPTH_CAL huart7
#define AUV_UART_MOTOR huart6
#define AUV_UART_MICROROS huart2
