#include "gpio.h"

/*
 * LED on board: PB-2
 */
void led_onboard_init(void)
{
    GPIO_InitTypeDef IO_Init;

    __HAL_RCC_GPIOB_CLK_ENABLE();

    IO_Init.Pin   = GPIO_PIN_2;
    IO_Init.Mode  = GPIO_MODE_OUTPUT_PP;
    IO_Init.Speed = GPIO_SPEED_LOW;

    HAL_GPIO_Init(GPIOB, &IO_Init);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
}