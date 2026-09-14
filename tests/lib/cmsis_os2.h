#pragma once

// CMSIS-RTOS2 桩 — 用于主机端测试
// 只定义测试需要的类型

#include <cstdint>
#include <cstddef>

enum osPriority_t : int32_t {
    osPriorityNone          =  0,
    osPriorityIdle          =  1,
    osPriorityLow           =  8,
    osPriorityBelowNormal   = 16,
    osPriorityNormal        = 24,
    osPriorityAboveNormal   = 32,
    osPriorityAboveNormal7  = 39,
    osPriorityHigh          = 40,
    osPriorityRealtime      = 48,
    osPriorityISR           = 56,
    osPriorityError         = -1
};

using osThreadId_t = void *;

struct osThreadAttr_t {
    const char* name;
    uint32_t attr_bits;
    void* cb_mem;
    uint32_t cb_size;
    void* stack_mem;
    uint32_t stack_size;
    osPriority_t priority;
    uint32_t reserved_flags;
};

inline osThreadId_t osThreadNew(void (*func)(void *), void *argument,
                                const osThreadAttr_t *attr) { return nullptr; }
