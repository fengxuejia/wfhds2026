#include "ti_msp_dl_config.h"
#include "HARDWARE/BUTTON/button.h"
#include "HARDWARE/ENCODER/encoder.h"
#include "HARDWARE/I2C/i2c.h"
#include "HARDWARE/OLED/oled.h"
#include "HARDWARE/LED/led.h"
#include "HARDWARE/MOTOR/motor.h"
#include "HARDWARE/TIMER/timer.h"
#include "APP/LINE_FOLLOW/line_follow.h"
#include "APP/PID/pid.h"

/* Sensor frames are produced in main() and consumed by the 400 Hz ISR. */
static volatile uint8_t latest_frame[LINE_SENSOR_COUNT];
static volatile bool frame_pending;
static volatile bool frame_valid;
static volatile bool sensor_comm_ok;
static volatile uint32_t last_frame_tick;
static volatile int32_t display_target_left;
static volatile int32_t display_target_right;
static volatile int32_t display_measured_left;
static volatile int32_t display_measured_right;

#define CONTROL_FREQUENCY_HZ 400.0f /* Must match TIMER_0's 2.5 ms period. */
#define OLED_UPDATE_PERIOD_TICKS 40U
#define SENSOR_TIMEOUT_TICKS 100U  /* 250 ms at 400 Hz. */

/* Rear driven wheels are 65 mm; the front caster is not used for odometry. */
#define PI_F 3.14159265f
#define WHEEL_DIAMETER_MM 65.0f
#define WHEEL_CIRCUMFERENCE_MM (PI_F * WHEEL_DIAMETER_MM)
/* MG513 GMR encoder, P30 gearbox, A-phase rising-edge counting (x1). */
#define ENCODER_PULSES_PER_MOTOR_REV 500.0f
#define MOTOR_GEAR_RATIO 30.0f

static float encoder_counts_to_mmps(int32_t counts)
{
    const float counts_per_wheel_revolution =
        ENCODER_PULSES_PER_MOTOR_REV * MOTOR_GEAR_RATIO;

    return (float)counts * CONTROL_FREQUENCY_HZ * WHEEL_CIRCUMFERENCE_MM /
           counts_per_wheel_revolution;
}

static void oled_format_speed(char *text, char motor, char type,
                              int32_t speed)
{
    uint32_t value;

    text[0] = 'M';
    text[1] = motor;
    text[2] = ' ';
    text[3] = type;
    text[4] = ':';
    text[5] = speed < 0 ? '-' : '+';
    value = (uint32_t)(speed < 0 ? -speed : speed);
    text[6] = (char)('0' + (value / 1000U) % 10U);
    text[7] = (char)('0' + (value / 100U) % 10U);
    text[8] = (char)('0' + (value / 10U) % 10U);
    text[9] = (char)('0' + value % 10U);
    text[10] = '\0';
}

static void oled_update(void)
{
    char text[11];
    char sensor_text[21];

    oled_format_speed(text, '1', 'T', display_target_left);
    OLED_ShowString(0U, 0U, (u8 *)text, 12U);
    oled_format_speed(text, '1', 'A', display_measured_left);
    OLED_ShowString(0U, 12U, (u8 *)text, 12U);
    oled_format_speed(text, '2', 'T', display_target_right);
    OLED_ShowString(0U, 24U, (u8 *)text, 12U);
    oled_format_speed(text, '2', 'A', display_measured_right);
    OLED_ShowString(0U, 36U, (u8 *)text, 12U);

    sensor_text[0] = 'S';
    sensor_text[1] = ':';
    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++) {
        sensor_text[i + 2U] = (latest_frame[i] & 1U) ? '1' : '0';
    }
    sensor_text[14] = ' ';
    sensor_text[15] = 'C';
    sensor_text[16] = ':';
    if (sensor_comm_ok) {
        sensor_text[17] = 'O';
        sensor_text[18] = 'K';
        sensor_text[19] = '1';
        sensor_text[20] = '\0';
    } else {
        sensor_text[17] = 'E';
        sensor_text[18] = 'R';
        sensor_text[19] = 'R';
        sensor_text[20] = '\0';
    }
    OLED_ShowString(0U, 48U, (u8 *)sensor_text, 12U);
    OLED_Refresh();
}

/* Runs in the 2.5 ms timer ISR; keep blocking peripheral work in main(). */
static void control_tick(void)
{
    uint8_t frame[LINE_SENSOR_COUNT];
    bool new_frame = false;
    uint32_t tick = TIMER_GetTick();

    __disable_irq();
    if (frame_pending) {
        for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++) {
            frame[i] = latest_frame[i];
        }
        frame_pending = false;
        new_frame = true;
    }
    __enable_irq();

    if (new_frame) {
        LineFollow_Update(frame);
        if (LineFollow_ConsumeRightAngleEvent()) {
            Button_NotifyRightAngle();
            if (Button_IsLapComplete() &&
                Button_GetCurrentLaps() >= Button_GetTargetLaps()) {
                Button_Stop();
                Motor_Stop();
                VelocityPid_Reset();
                return;
            }
        }
        frame_valid = true;
    }

    /* Keep sampling the encoders while stopped so manual wheel checks are
     * visible on the OLED and do not accumulate into the next run. */
    int32_t left_counts = Encoder_ReadLeft();
    int32_t right_counts = Encoder_ReadRight();
    float measured_left = encoder_counts_to_mmps(left_counts);
    float measured_right = encoder_counts_to_mmps(right_counts);
    display_measured_left = (int32_t)measured_left;
    display_measured_right = (int32_t)measured_right;

    if (!frame_valid || !Button_IsRunning() ||
        (tick - last_frame_tick > SENSOR_TIMEOUT_TICKS)) {
        if (tick - last_frame_tick > SENSOR_TIMEOUT_TICKS) {
            sensor_comm_ok = false;
        }
        display_target_left = 0;
        display_target_right = 0;
        Motor_Stop();
        VelocityPid_Reset();
        return;
    }

    /* Single-channel counting measures speed magnitude. This is sufficient
     * because the three-wheel line follower commands both drive wheels forward. */
    float turn_diff = LineFollow_GetTurnDiff();
    float base_speed = LineFollow_GetBaseSpeed();
    /* Positive turn_diff means a left turn: slow the left wheel and speed up
     * the right wheel. All four speed values use mm/s. */
    float target_left = base_speed - turn_diff;
    float target_right = base_speed + turn_diff;
    display_target_left = (int32_t)target_left;
    display_target_right = (int32_t)target_right;
    int16_t left_pwm;
    int16_t right_pwm;
    VelocityPid_Compute(target_left, target_right,
                        measured_left, measured_right,
                        &left_pwm, &right_pwm);

    Motor_SetPwm(left_pwm, right_pwm);
}

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    LED_Init();
    I2C_Init();
    Motor_Init();
    Button_Init();
    Encoder_Init();
    LineFollow_Init();
    VelocityPid_Init();
    TIMER_Init();
    TIMER_RegisterCallback(control_tick);
    TIMER_Start();
    oled_update();

    while (1) {
        static uint32_t last_oled_tick;
        static uint32_t last_sensor_tick;
        uint32_t tick = TIMER_GetTick();

        Button_Update();
        if (tick != last_sensor_tick) {
            uint8_t frame[LINE_SENSOR_COUNT];

            last_sensor_tick = tick;
            bool sensor_frame_ready = I2C_ReadLineSensors(frame);
            if (sensor_frame_ready) {
                sensor_comm_ok = true;
                __disable_irq();
                for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++) {
                    latest_frame[i] = frame[i];
                }
                last_frame_tick = tick;
                frame_pending = true;
                __enable_irq();
            }
        }
        if (!Button_IsRunning()) {
            Motor_Stop();
            VelocityPid_Reset();
        }
        if (tick - last_oled_tick >= OLED_UPDATE_PERIOD_TICKS) {
            last_oled_tick = tick;
            oled_update();
        }
    }
}
