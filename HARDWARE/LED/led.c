#include "led.h"
#include "ti_msp_dl_config.h"

void LED_Init(void)
{
    /* Pin direction and initial state are owned by SysConfig. */
}

void LED_On(void)
{
    DL_GPIO_setPins(LED_PORT, LED_LED0_PIN);
}

void LED_Off(void)
{
    DL_GPIO_clearPins(LED_PORT, LED_LED0_PIN);
}

void LED_Toggle(void)
{
    DL_GPIO_togglePins(LED_PORT, LED_LED0_PIN);
}
