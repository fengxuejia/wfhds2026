#ifndef HARDWARE_MOTOR_H
#define HARDWARE_MOTOR_H

#include <stdint.h>

/* Must match MOTOR_PWM.timerCount in empty.syscfg. */
#define MOTOR_PWM_PERIOD 2000U
/* Conservative 60% duty cap for software-counted 500 ppr encoders. */
#define MOTOR_PWM_LIMIT  1200

#if MOTOR_PWM_LIMIT > MOTOR_PWM_PERIOD
#error "MOTOR_PWM_LIMIT must not exceed MOTOR_PWM_PERIOD"
#endif

void Motor_Init(void);
/* Signed PWM commands: magnitude sets duty and sign selects direction. */
void Motor_SetPwm(int16_t left, int16_t right);
void Motor_Stop(void);

#endif
