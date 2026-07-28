#ifndef HARDWARE_TIMER_H
#define HARDWARE_TIMER_H

#include <stdint.h>

void TIMER_Init(void);
void TIMER_Start(void);
void TIMER_Stop(void);
uint32_t TIMER_GetTick(void);
void TIMER_HandleInterrupt(void);
/* The callback runs from TIMER_0_INST_IRQHandler, not from the main loop. */
void TIMER_RegisterCallback(void (*callback)(void));

#endif
