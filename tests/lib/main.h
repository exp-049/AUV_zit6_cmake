#pragma once

// HAL 桩 — 用于主机端测试
// HAL_GetTick 由测试代码控制，通过 setHALTick() 设置
// s_hal_tick 定义在 hal_tick_stub.cpp 中，确保所有编译单元共享同一份

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t g_hal_tick;

inline uint32_t HAL_GetTick() { return g_hal_tick; }

inline void setHALTick(uint32_t tick) { g_hal_tick = tick; }
inline void advanceHALTick(uint32_t ms) { g_hal_tick += ms; }

#ifdef __cplusplus
}
#endif

#define assert_param(expr) ((void)0)
