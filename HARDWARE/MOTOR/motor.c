#include "motor.h"
#include "ti_msp_dl_config.h"

static int16_t limit_pwm(int16_t value)
{
    if (value > MOTOR_PWM_LIMIT) {
        return MOTOR_PWM_LIMIT;
    }
    if (value < -MOTOR_PWM_LIMIT) {
        return -MOTOR_PWM_LIMIT;
    }
    return value;
}

static void set_direction(GPIO_Regs *port, uint32_t pin, bool forward)
{
    if (forward) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
}

static void set_pwm(uint32_t index, int16_t value)
{
    uint16_t duty = (uint16_t)((value < 0) ? -value : value);
    /* Down-count PWM is high from LOAD to CC, so compare is inverse to duty. */
    uint16_t compare = (uint16_t)(MOTOR_PWM_PERIOD - duty);
    DL_TimerA_setCaptureCompareValue(MOTOR_PWM_INST, compare, index);
}

void Motor_Init(void)
{
    Motor_Stop();
    DL_TimerA_startCounter(MOTOR_PWM_INST);
}

void Motor_SetPwm(int16_t left, int16_t right)
{
    left = limit_pwm(left);
    right = limit_pwm(right);

    /* The installed motor wiring reverses the logical forward polarity. */
    set_direction(MOTOR_GPIO_AIN1_PORT, MOTOR_GPIO_AIN1_PIN, left < 0);
    set_direction(MOTOR_GPIO_AIN2_PORT, MOTOR_GPIO_AIN2_PIN, left >= 0);
    set_direction(MOTOR_GPIO_BIN1_PORT, MOTOR_GPIO_BIN1_PIN, right < 0);
    set_direction(MOTOR_GPIO_BIN2_PORT, MOTOR_GPIO_BIN2_PIN, right >= 0);

    set_pwm(DL_TIMER_CC_0_INDEX, left);  /* PWMA: left motor */
    set_pwm(DL_TIMER_CC_1_INDEX, right); /* PWMB: right motor */
}

void Motor_Stop(void)
{
    set_pwm(DL_TIMER_CC_0_INDEX, 0);
    set_pwm(DL_TIMER_CC_1_INDEX, 0);
    DL_GPIO_clearPins(MOTOR_GPIO_AIN1_PORT, MOTOR_GPIO_AIN1_PIN);
    DL_GPIO_clearPins(MOTOR_GPIO_AIN2_PORT, MOTOR_GPIO_AIN2_PIN);
    DL_GPIO_clearPins(MOTOR_GPIO_BIN1_PORT, MOTOR_GPIO_BIN1_PIN);
    DL_GPIO_clearPins(MOTOR_GPIO_BIN2_PORT, MOTOR_GPIO_BIN2_PIN);
}
