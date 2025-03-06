#include "stm32f1xx_hal.h"

static CRC_HandleTypeDef hcrc;

void crc_init(void)
{
    __HAL_RCC_CRC_CLK_ENABLE();

    hcrc.Instance = CRC;
    HAL_CRC_Init(&hcrc);
}


static uint32_t revbit(uint32_t data)
{
    asm("rbit r0, r0" :"=r"(data));
    return data;
};

uint32_t crc32_calculate(uint32_t *data, int length)
{
    uint32_t crc;
    int i;

    hcrc.State = HAL_CRC_STATE_BUSY;
    __HAL_CRC_DR_RESET(&hcrc);

    for (i = 0; i < length; i++)
        hcrc.Instance->DR = revbit(data[i]);

    crc = hcrc.Instance->DR;
    hcrc.State = HAL_CRC_STATE_READY;

    return crc;
}