#include "button.h"
#include "ti_msp_dl_config.h"
#include "HARDWARE/TIMER/timer.h"

#define SHORT_PRESS_TICKS      160U /* 400 ms at the 2.5 ms system tick. */

static volatile bool running;
static bool press_active;
static uint32_t press_start_tick;
static volatile uint8_t target_laps;
static volatile uint8_t current_laps;
static volatile uint8_t corner_count;
static volatile bool lap_complete;

void Button_Init(void)
{
    running = false;
    press_active = false;
    press_start_tick = 0U;
    target_laps = 1U;
    current_laps = 0U;
    corner_count = 0U;
    lap_complete = false;
}

void Button_Update(void)
{
    bool pressed = (DL_GPIO_readPins(BUTTON_PORT, BUTTON_START_PIN) == 0U);
    uint32_t tick = TIMER_GetTick();

    /* The STM32 reference accepts button commands only while stopped. */
    if (running) {
        press_active = false;
        return;
    }

    if (pressed && !press_active) {
        press_active = true;
        press_start_tick = tick;
    } else if (!pressed && press_active) {
        uint32_t duration = tick - press_start_tick;

        press_active = false;
        if (duration < SHORT_PRESS_TICKS) {
            target_laps++;
            if (target_laps > 5U) {
                target_laps = 1U;
            }
        } else {
            running = true;
            current_laps = 0U;
            corner_count = 0U;
            lap_complete = false;
        }
    }
}

bool Button_IsRunning(void)
{
    return running;
}

void Button_NotifyRightAngle(void)
{
    if (!running) {
        return;
    }

    corner_count++;
    if (corner_count >= 4U) {
        corner_count = 0U;
        current_laps++;
        lap_complete = true;
    }
}

void Button_Stop(void)
{
    running = false;
    press_active = false;
}

uint8_t Button_GetTargetLaps(void)
{
    return target_laps;
}

uint8_t Button_GetCurrentLaps(void)
{
    return current_laps;
}

uint8_t Button_GetCornerCount(void)
{
    return corner_count;
}

bool Button_IsLapComplete(void)
{
    return lap_complete;
}
