#ifndef __APP_MAIN_HPP
#define __APP_MAIN_HPP

#ifdef __cplusplus
extern "C" {
#endif

void UserApp_ControlTask(void *argument);
void UserApp_MonitorTask(void *argument);
void UserApp_MicroRosTask(void *argument);
void UserApp_IICTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif
