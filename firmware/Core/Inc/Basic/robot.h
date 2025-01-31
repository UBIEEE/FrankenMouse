#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void Robot_Init(void);
void Robot_Update(void);
void Robot_OnConnect(void);
void Robot_DashboardAppReady(void);
void Robot_OnDisconnect(void);
void Robot_SendFeedback(void);

void RobotControl_RunTask(uint8_t task, uint8_t start_location);
void RobotControl_ResetMaze(void);

void ErrorManager_ReportError(void);

void Buzzer_PlaySong(uint8_t song);

void Drive_UpdatePIDConstants(uint8_t* buf);

void Vision_Calibrate(uint8_t reset_or_calibrate);

#ifdef __cplusplus
}
#endif
