#ifndef __CONTROL_TASK_HPP
#define __CONTROL_TASK_HPP

#include "MotionContext.hpp"
#include "SafetyMonitor.hpp"
#include "SystemContext.hpp"
#include "FreeRTOS.h"
#include <stdint.h>

class ControlTask {
public:
	ControlTask() = default;
	void run();

private:
	static constexpr uint32_t kLoopPeriodMs = 10;

	TickType_t last_wake_time_ = 0;
	uint32_t last_tick_ = 0;

	auv::system::SafetyMonitor safety_monitor_;

	void init();
	void refreshHardwareWatchdogIfNeeded();
	auv::motion::NavState updateNavigation();
	void computeAndPublish();
};

#endif // __CONTROL_TASK_HPP
