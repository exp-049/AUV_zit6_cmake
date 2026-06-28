/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AppMain.hpp"
#include "usart.h"
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <uxr/client/transport.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
/* USER CODE END Variables */
/* Definitions for micro_ros_task */
osThreadId_t micro_ros_taskHandle;
const osThreadAttr_t micro_ros_task_attributes = {
  .name = "micro_ros_task",
  .stack_size = 5000 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal7,
};
/* Definitions for hardware_bridge */
osThreadId_t hardware_bridgeHandle;
const osThreadAttr_t hardware_bridge_attributes = {
  .name = "hardware_bridge",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for iic_task */
osThreadId_t iic_taskHandle;
const osThreadAttr_t iic_task_attributes = {
  .name = "iic_task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for monitor_task */
osThreadId_t monitor_taskHandle;
const osThreadAttr_t monitor_task_attributes = {
  .name = "monitor_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for imu_queue */
osMessageQueueId_t imu_queueHandle;
const osMessageQueueAttr_t imu_queue_attributes = {
  .name = "imu_queue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport,
                              const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
                             size_t len, int timeout, uint8_t *err);

void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
                             void *state);
/* USER CODE END FunctionPrototypes */

void Entry_MicroRosTask(void *argument);
void Entry_ControlTask(void *argument);
void Entry_IICTask(void *argument);
void Entry_MonitorTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName) {
  /* Run time stack overflow checking is performed if
  configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
  called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of imu_queue */
  imu_queueHandle = osMessageQueueNew (16, sizeof(uint32_t), &imu_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of micro_ros_task */
  micro_ros_taskHandle = osThreadNew(Entry_MicroRosTask, NULL, &micro_ros_task_attributes);

  /* creation of hardware_bridge */
  hardware_bridgeHandle = osThreadNew(Entry_ControlTask, NULL, &hardware_bridge_attributes);

  /* creation of iic_task */
  iic_taskHandle = osThreadNew(Entry_IICTask, NULL, &iic_task_attributes);

  /* creation of monitor_task */
  monitor_taskHandle = osThreadNew(Entry_MonitorTask, NULL, &monitor_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Entry_MicroRosTask */
/**
 * @brief  Function implementing the micro_ros_task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_Entry_MicroRosTask */
void Entry_MicroRosTask(void *argument)
{
  /* USER CODE BEGIN Entry_MicroRosTask */
  UserApp_MicroRosTask(argument);
  /* USER CODE END Entry_MicroRosTask */
}

/* USER CODE BEGIN Header_Entry_ControlTask */
/**
 * @brief Function implementing the hardware_bridge thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Entry_ControlTask */
void Entry_ControlTask(void *argument)
{
  /* USER CODE BEGIN Entry_ControlTask */
  UserApp_ControlTask(argument);
  /* USER CODE END Entry_ControlTask */
}

/* USER CODE BEGIN Header_Entry_IICTask */
/**
 * @brief Function implementing the iic_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Entry_IICTask */
void Entry_IICTask(void *argument)
{
  /* USER CODE BEGIN Entry_IICTask */
  UserApp_IICTask(argument);
  /* USER CODE END Entry_IICTask */
}

/* USER CODE BEGIN Header_Entry_MonitorTask */
/**
 * @brief Function implementing the monitor_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Entry_MonitorTask */
void Entry_MonitorTask(void *argument)
{
  /* USER CODE BEGIN Entry_MonitorTask */
  UserApp_MonitorTask(argument);
  /* USER CODE END Entry_MonitorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

