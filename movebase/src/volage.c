#include <stm32f1xx_hal.h>
#include <rtthread.h>
#include <qfplib-m3.h>

static ADC_HandleTypeDef hadc1;

/**
 * adc for measuring voltage
 * PA5 for ADC, and connect the GND
 */
void voltage_adc_init(void)
{
    GPIO_InitTypeDef IO_Init;
    ADC_ChannelConfTypeDef AC_Init;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    // PA5
    IO_Init.Pin = GPIO_PIN_5;
    IO_Init.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &IO_Init);

    // 12MHz
    __HAL_RCC_ADC_CONFIG(RCC_ADCPCLK2_DIV6);

    // adc
    hadc1.Instance = ADC1;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    HAL_ADC_Init(&hadc1);

    // channel: PA5 - channel5
    AC_Init.Channel = ADC_CHANNEL_5;
    AC_Init.Rank = ADC_REGULAR_RANK_1;
    AC_Init.SamplingTime = ADC_SAMPLETIME_239CYCLES_5; // 采样时间 239.5 周期
    HAL_ADC_ConfigChannel(&hadc1, &AC_Init);

    // 校准
    HAL_ADCEx_Calibration_Start(&hadc1);
}

static uint16_t read_adc(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    HAL_ADC_Stop(&hadc1);

    return HAL_ADC_GetValue(&hadc1);
}

/**
 * 一阶 IIR 滤波
 *    y[n] = α·x[n] + (1−α)·y[n−1]
 *    α 决定对新数据的信任度, 越大相应越快,但平滑效果减弱
 *
 * volt = (volt_num / 4096 ) * 3.3 V * 11(分压比例)
 */
float read_voltage(void)
{
    static float filtered_volt;
    float volt;
    const float alpha = 0.3;

    volt = read_adc() * 0.0088623;
    filtered_volt = alpha * volt + (1-alpha) * filtered_volt;

    return filtered_volt;
}