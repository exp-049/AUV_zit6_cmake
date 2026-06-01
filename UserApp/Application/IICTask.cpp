#include "IICTask.hpp"
#include "MotionContext.hpp"
#include "SystemContext.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "SoftWatchdog.hpp"
#include <cmath>

void UserApp_IICTask(void *argument) {
    auv::device::depth_sensor.Init();

    // Validation state
    float last_valid_depth = 0.0f;
    int   bad_count = 0;
    const int kMaxBadCount = 5;

    for (;;) {
        TickType_t last_wake = xTaskGetTickCount();
        if (auv::device::depth_sensor.is_connected) {
            int r = auv::device::depth_sensor.Read();
            if (r > 0) {
                float d = 0.0f;
                auv::device::depth_sensor.Depth(&d);

                bool valid = true;
                if (!std::isfinite(d)) valid = false;

                // If reading is exactly zero while we previously had a sensible depth,
                // treat it as invalid (common symptom of I2C read returning zeros).
                if (d == 0.0f && last_valid_depth > 0.5f) valid = false;

                // Range sanity check (adjust bounds if your platform needs different limits)
                if (d < -5.0f || d > 500.0f) valid = false;

                if (valid) {
                    bad_count = 0;
                    auv::device::depth_sensor.setMS5837Z(d);

                    last_valid_depth = d;
                    auv::device::SoftWatchdog::getInstance().feed(auv::device::SoftWatchdog::Component::DEPTH);
                } else {
                    bad_count++;
                    if (bad_count >= kMaxBadCount) {
                        // Try to recover sensor if repeated bad readings
                        auv::device::depth_sensor.Init();
                        bad_count = 0;
                    }
                }
            } else if (r < 0) {
                // I2C error
                bad_count++;
                if (bad_count >= kMaxBadCount) {
                    auv::device::depth_sensor.Init();
                    bad_count = 0;
                }
            } else {
                // r == 0 -> conversion in progress, nothing to do this cycle
            }
        } else {
            // Not connected, try to init
            auv::device::depth_sensor.Init();
        }
        // Aim for ~60Hz sampling calls to the non-blocking Read()
        vTaskDelay(pdMS_TO_TICKS(8)); // ~125Hz loop; with 2-step conversion yields ~62.5Hz samples
    }
}
