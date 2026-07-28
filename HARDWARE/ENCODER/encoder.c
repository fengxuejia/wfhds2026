#include "encoder.h"
#include "ti_msp_dl_config.h"

static volatile int32_t left_count;
static volatile int32_t right_count;

void Encoder_Init(void)
{
    left_count = 0;
    right_count = 0;

    NVIC_ClearPendingIRQ(ENCODER_GPIO_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_GPIO_INT_IRQN);
}

int32_t Encoder_ReadLeft(void)
{
    int32_t count;
    __disable_irq();
    /* Reading also starts the next fixed-period measurement window. */
    count = left_count;
    left_count = 0;
    __enable_irq();
    return count;
}

int32_t Encoder_ReadRight(void)
{
    int32_t count;
    __disable_irq();
    count = right_count;
    right_count = 0;
    __enable_irq();
    return count;
}

void Encoder_Reset(void)
{
    __disable_irq();
    left_count = 0;
    right_count = 0;
    __enable_irq();
}

void GROUP1_IRQHandler(void)
{
    /* The car only commands forward motion. Counting one rising edge per A
     * pulse cuts the GMR interrupt rate to one quarter of quadrature x4. */
    uint32_t gpio_a = DL_GPIO_getEnabledInterruptStatus(
        GPIOA, ENCODER_GPIO_ENCODER_A1_PIN |
              ENCODER_GPIO_ENCODER_A2_PIN);

    if ((gpio_a & ENCODER_GPIO_ENCODER_A1_PIN) != 0U) {
        left_count++;
    }
    if ((gpio_a & ENCODER_GPIO_ENCODER_A2_PIN) != 0U) {
        right_count++;
    }
    if (gpio_a != 0U) {
        DL_GPIO_clearInterruptStatus(GPIOA, gpio_a);
    }
}
