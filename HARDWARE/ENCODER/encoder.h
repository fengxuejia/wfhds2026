#ifndef HARDWARE_ENCODER_H
#define HARDWARE_ENCODER_H

#include <stdint.h>

void Encoder_Init(void);
/* Return forward A-phase pulses accumulated since the previous read. */
int32_t Encoder_ReadLeft(void);
int32_t Encoder_ReadRight(void);
void Encoder_Reset(void);

#endif
