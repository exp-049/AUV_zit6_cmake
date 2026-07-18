#pragma once

#include "RosLogger.hpp"

// MS5837 日志总开关：
//   0 - 正式运行默认关闭，避免每帧/每次轮询产生 ROS 日志
//   1 - 打开 MS5837 后端、驱动和主程序诊断日志
#ifndef MS5837_LOG_ENABLE
#define MS5837_LOG_ENABLE 0
#endif

// 主程序低频诊断：默认每秒一条，便于现场确认 z 数据链路；不包含每帧日志。
#ifndef MS5837_MAIN_DIAG_ENABLE
#define MS5837_MAIN_DIAG_ENABLE 0
#endif

#if MS5837_LOG_ENABLE
#define MS5837_LOG_DEBUG(fmt, ...) ROS_LOG_DEBUG(fmt, ##__VA_ARGS__)
#define MS5837_LOG_WARN(fmt, ...) ROS_LOG_WARN(fmt, ##__VA_ARGS__)
#else
#define MS5837_LOG_DEBUG(fmt, ...) ((void)0)
#define MS5837_LOG_WARN(fmt, ...) ((void)0)
#endif

#if MS5837_MAIN_DIAG_ENABLE
#define MS5837_LOG_DIAG(fmt, ...) ROS_LOG_WARN(fmt, ##__VA_ARGS__)
#else
#define MS5837_LOG_DIAG(fmt, ...) ((void)0)
#endif
