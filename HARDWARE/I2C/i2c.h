#ifndef HARDWARE_I2C_H
#define HARDWARE_I2C_H

#include <stdbool.h>
#include <stdint.h>

void I2C_Init(void);
bool I2C_IsControllerBusy(void);
bool I2C_ReadLineSensors(uint8_t sensors[12]);

#endif
