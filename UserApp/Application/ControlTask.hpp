#ifndef __CONTROL_TASK_HPP
#define __CONTROL_TASK_HPP

#include "MotionContext.hpp"
#include "SystemContext.hpp"
#include "FreeRTOS.h"
#include <stdint.h>

class ControlTask {
public:
	ControlTask() = default;
	void run();

private:

	static constexpr uint32_t kLoopPeriodMs = 10;
	static constexpr uint32_t kArmedHeartbeatTimeoutMs = 500; // 恢复：短阈值，快速 disarm
	static constexpr uint32_t kDisarmedHeartbeatTimeoutMs = 1000;
	static constexpr uint32_t kArmMinDurationMs = 1000;
	static constexpr uint32_t kArmMinHeartbeatCount = 10;
	static constexpr uint32_t kRemoteModeHeartbeatData = 3;

	TickType_t last_wake_time_ = 0;
	uint32_t last_tick_ = 0;

	void init();
	void refreshHardwareWatchdogIfNeeded();
	auv::motion::NavState updateNavigation();
	void setControlLevelNone();
	void forceDisarmWithNeutralLevel();
	void handleArmState(uint32_t now);
	void computeAndPublish();
};

#endif // __CONTROL_TASK_HPP
