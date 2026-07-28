#ifndef HARDWARE_BUTTON_H
#define HARDWARE_BUTTON_H

#include <stdbool.h>
#include <stdint.h>

void Button_Init(void);
void Button_Update(void);
bool Button_IsRunning(void);
void Button_NotifyRightAngle(void);
void Button_Stop(void);
uint8_t Button_GetTargetLaps(void);
uint8_t Button_GetCurrentLaps(void);
uint8_t Button_GetCornerCount(void);
bool Button_IsLapComplete(void);

#endif
