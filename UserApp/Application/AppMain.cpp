/**
 * @file AppMain.cpp
 * @brief This file is now modularized. Logic has been moved to:
 *        - ControlTask.cpp
 *        - MicroRosTask.cpp
 *        - IICTask.cpp
 *        - MotionContext.cpp
 *        - SystemContext.cpp
 */
#include "AppMain.hpp"
#include "DebugApp.hpp"
#include "cmsis_os2.h"
#include "main.h"

namespace {

// Keep the NORMAL task configuration identical to the former CubeMX-generated
// definitions. The realtime bootstrap task creates all three before yielding,
// so no application task can run against a partially initialized task set.
constexpr osThreadAttr_t kMicroRosTaskAttributes = {
    .name = "micro_ros_task",
    .stack_size = 5000U * 4U,
    .priority = osPriorityAboveNormal7,
};

constexpr osThreadAttr_t kControlTaskAttributes = {
    .name = "controll_task",
    .stack_size = 2048U * 4U,
    .priority = osPriorityHigh,
};

constexpr osThreadAttr_t kMonitorTaskAttributes = {
    .name = "monitor_task",
    .stack_size = 512U * 4U,
    .priority = osPriorityNormal,
};

constexpr osThreadAttr_t kMs5837CalDebugTaskAttributes = {
    .name = "ms5837_cal_debug",
    .stack_size = 1024U * 4U,
    .priority = osPriorityNormal,
};

constexpr osThreadAttr_t kUsblDebugTaskAttributes = {
    .name = "usbl_debug",
    .stack_size = 2048U * 4U,
    .priority = osPriorityNormal,
};

constexpr osThreadAttr_t kInsDebugTaskAttributes = {
    .name = "ins_debug",
    .stack_size = 2048U * 4U,
    .priority = osPriorityNormal,
};

constexpr osThreadAttr_t kPushrodDebugTaskAttributes = {
    .name = "pushrod_debug",
    .stack_size = 2048U * 4U,
    .priority = osPriorityNormal,
};

constexpr osThreadAttr_t kMotionDebugTaskAttributes = {
    .name = "motion_debug",
    .stack_size = 2048U * 4U,
    .priority = osPriorityNormal,
};

} // namespace

extern "C" void UserApp_Start(void) {
#if AUV_APP_MODE == 0
  // Preserve the original CubeMX creation order.
  const osThreadId_t micro_ros_task =
      osThreadNew(UserApp_MicroRosTask, nullptr, &kMicroRosTaskAttributes);
  const osThreadId_t control_task =
      osThreadNew(UserApp_ControlTask, nullptr, &kControlTaskAttributes);
  const osThreadId_t monitor_task =
      osThreadNew(UserApp_MonitorTask, nullptr, &kMonitorTaskAttributes);

  if (micro_ros_task == nullptr || control_task == nullptr ||
      monitor_task == nullptr) {
    // A partially started control system is unsafe. Error_Handler stops here;
    // the already-running IWDG will subsequently reset the MCU.
    Error_Handler();
  }

  // The bootstrap task has no further responsibility after all NORMAL tasks
  // exist. Releasing it also returns its 2 KiB stack to the FreeRTOS heap.
  osThreadExit();
#elif AUV_APP_MODE == 1
  if (osThreadNew(UserApp_Ms5837CalDebugTask, nullptr,
                  &kMs5837CalDebugTaskAttributes) == nullptr) {
    Error_Handler();
  }
  osThreadExit();
#elif AUV_APP_MODE == 2
  if (osThreadNew(UserApp_UsblDebugTask, nullptr,
                  &kUsblDebugTaskAttributes) == nullptr) {
    Error_Handler();
  }
  osThreadExit();
#elif AUV_APP_MODE == 3
  if (osThreadNew(UserApp_InsDebugTask, nullptr,
                  &kInsDebugTaskAttributes) == nullptr) {
    Error_Handler();
  }
  osThreadExit();
#elif AUV_APP_MODE == 5
  if (osThreadNew(UserApp_PushrodDebugTask, nullptr,
                  &kPushrodDebugTaskAttributes) == nullptr) {
    Error_Handler();
  }
  osThreadExit();
#elif AUV_APP_MODE == 4
  if (osThreadNew(UserApp_MotionDebugTask, nullptr,
                  &kMotionDebugTaskAttributes) == nullptr) {
    Error_Handler();
  }
  osThreadExit();
#else
#error "Unknown AUV_APP_MODE"
#endif
}
