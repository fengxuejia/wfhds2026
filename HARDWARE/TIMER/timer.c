#include "timer.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t timer_tick;
static void (*timer_callback)(void);

void TIMER_Init(void)
{
    timer_tick = 0U;
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

void TIMER_RegisterCallback(void (*callback)(void))
{
    timer_callback = callback;
}

void TIMER_Start(void)
{
    DL_TimerA_startCounter(TIMER_0_INST);
}

void TIMER_Stop(void)
{
    DL_TimerA_stopCounter(TIMER_0_INST);
}

uint32_t TIMER_GetTick(void)
{
    uint32_t tick;

    __disable_irq();
    tick = timer_tick;
    __enable_irq();
    return tick;
}

void TIMER_HandleInterrupt(void)
{
    uint32_t status = DL_TimerA_getPendingInterrupt(TIMER_0_INST);

    if (status == DL_TIMERA_IIDX_ZERO)
    {
        DL_TimerA_clearInterruptStatus(
            TIMER_0_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
        timer_tick++;
        if (timer_callback != (void *)0)
        {
            /* Registered callbacks execute in timer interrupt context. */
            timer_callback();
        }
    }
}

void TIMER_0_INST_IRQHandler(void)
{
    TIMER_HandleInterrupt();
}
