#include "pid.h"

/* Keep this no greater than MOTOR_PWM_LIMIT in HARDWARE/MOTOR/motor.h. */
#define PID_OUTPUT_LIMIT 1200.0f

/* PWM per mm/s. This feedforward supplies most of the steady-state output. */
#define PID_SPEED_FEEDFORWARD 2.0f
#define PID_SPEED_KP 0.5f
/* Integral gain is per 2.5 ms control tick, not per second. */
#define PID_SPEED_KI 0.0025f
#define PID_SPEED_INTEGRAL_LIMIT 500.0f
#define PID_OUTPUT_RISE_STEP 20.0f

static float left_output;
static float right_output;
static float left_integral;
static float right_integral;

static float limit_integral(float value)
{
    if (value > PID_SPEED_INTEGRAL_LIMIT) {
        return PID_SPEED_INTEGRAL_LIMIT;
    }
    if (value < -PID_SPEED_INTEGRAL_LIMIT) {
        return -PID_SPEED_INTEGRAL_LIMIT;
    }
    return value;
}

static float update_wheel_pid(float target, float measured,
                              float previous_output, float *integral)
{
    /* All line-follow targets are forward speeds in mm/s. Clearing the
     * integral at zero target prevents a stopped wheel from creeping. */
    if (target <= 0.0f) {
        *integral = 0.0f;
        return 0.0f;
    }

    float error = target - measured;
    float candidate_integral = limit_integral(
        *integral + PID_SPEED_KI * error);

    float desired_output = PID_SPEED_FEEDFORWARD * target +
                           PID_SPEED_KP * error + candidate_integral;
    float maximum_output = previous_output + PID_OUTPUT_RISE_STEP;
    if (maximum_output > PID_OUTPUT_LIMIT) {
        maximum_output = PID_OUTPUT_LIMIT;
    }

    /* Block integration only when it would push farther into a limit. A
     * correction in the opposite direction is retained so the PI can unwind. */
    if (desired_output > maximum_output) {
        if (error < 0.0f) {
            *integral = candidate_integral;
        }
        return maximum_output;
    }
    if (desired_output < 0.0f) {
        if (error > 0.0f) {
            *integral = candidate_integral;
        }
        return 0.0f;
    }

    *integral = candidate_integral;
    return desired_output;
}

void VelocityPid_Init(void)
{
    VelocityPid_Reset();
}

void VelocityPid_Reset(void)
{
    left_output = 0.0f;
    right_output = 0.0f;
    left_integral = 0.0f;
    right_integral = 0.0f;
}

void VelocityPid_Compute(float target_left, float target_right,
                         float measured_left_value,
                         float measured_right_value,
                         int16_t *pwm_left, int16_t *pwm_right)
{
    /* Each encoder closes its own loop. This corrects motor mismatch while
     * preserving the differential targets selected by the line follower. */
    left_output = update_wheel_pid(target_left, measured_left_value,
                                   left_output, &left_integral);
    right_output = update_wheel_pid(target_right, measured_right_value,
                                    right_output, &right_integral);

    *pwm_left = (int16_t)left_output;
    *pwm_right = (int16_t)right_output;
}
