#ifndef APP_PID_H
#define APP_PID_H

#include <stdint.h>

/* Target and measured speeds are mm/s. Outputs are forward PWM commands. */
void VelocityPid_Init(void);
void VelocityPid_Compute(float target_left, float target_right,
                         float measured_left, float measured_right,
                         int16_t *pwm_left, int16_t *pwm_right);
void VelocityPid_Reset(void);

#endif
