#pragma once

// FreeRTOS 桩头文件 — 用于主机端单元测试
// 将临界区和任务 API 替换为空操作

#include <cstdint>

using TickType_t = unsigned long;
using BaseType_t = long;
using UBaseType_t = unsigned long;

using TaskHandle_t = void *;

// 关键区 — 测试中为空操作
#define taskENTER_CRITICAL()  do {} while(0)
#define taskEXIT_CRITICAL()   do {} while(0)

#define portTICK_PERIOD_MS 1

// 队列/信号量等 — 测试中不需要
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))

#define configMAX_PRIORITIES 56
#define configTICK_RATE_HZ   1000

inline TickType_t xTaskGetTickCount() { return 0; }
inline void vTaskDelayUntil(const TickType_t *pxPreviousWakeTime,
                            TickType_t xTimeIncrement) {}
inline void vTaskDelay(TickType_t xTicksToDelay) {}
