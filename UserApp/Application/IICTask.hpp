#ifndef __IIC_TASK_HPP
#define __IIC_TASK_HPP

#include "AppContext.hpp"
#include "FreeRTOS.h"

class IICTask {
public:
  IICTask(auv::system::AppContext *ctx) : ctx_(ctx) {}
  void run();

private:
  auv::system::AppContext *ctx_;
  TickType_t last_wake_time_ = 0;
};

#ifdef __cplusplus
extern "C" {
#endif

void UserApp_IICTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif
