#include <stm32f1xx_hal.h>
#include <rtthread.h>
#include <qfplib-m3.h>
#include "gpio.h"
#include "include/multi.h"

/***************************************************************************************
 *                       TIM1/2 Init for Reading Hall-Encoder                          *
 *                                                                                     *
 *         Phase-A         Phase-B                                                     *
 * TIM1:  PA8   PA9    >   Motor_A                                                     *
 * TIM2:  PA1   PA0    >   Motor_B                                                     *
 *! Warn: Channel1/2 of TIM2 is relevant to phase A/B of encoder                       *
 *!       motor-B rotates in the opposite direction to motor-A                         *
 ***************************************************************************************/

static TIM_HandleTypeDef htim1, htim2;

static void __encoder_tim1_init(GPIO_InitTypeDef *IO_Init, TIM_Encoder_InitTypeDef *EC_Init)
{
    __HAL_RCC_TIM1_CLK_ENABLE();

    IO_Init->Pin = GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOA, IO_Init);

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 65535; // 最大计数值
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Encoder_Init(&htim1, EC_Init);

    __HAL_TIM_SET_COUNTER(&htim1, 0);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
}

static void __encoder_tim2_init(GPIO_InitTypeDef *IO_Init, TIM_Encoder_InitTypeDef *EC_Init)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    IO_Init->Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, IO_Init);

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65535;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Encoder_Init(&htim2, EC_Init);

    // 重置计数器
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

void encoder_init(void)
{
    TIM_Encoder_InitTypeDef EC_Init;
    GPIO_InitTypeDef IO_Init;

    __HAL_RCC_GPIOA_CLK_ENABLE();

    IO_Init.Mode = GPIO_MODE_INPUT;
    // TI1 TI2 正交编码 + 上升沿触发
    EC_Init.EncoderMode = TIM_ENCODERMODE_TI12;
    EC_Init.IC1Polarity = TIM_ICPOLARITY_RISING;
    EC_Init.IC2Polarity = TIM_ICPOLARITY_RISING;
    EC_Init.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    EC_Init.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    EC_Init.IC1Filter = 10;
    EC_Init.IC2Filter = 10;

    __encoder_tim1_init(&IO_Init, &EC_Init);
    __encoder_tim2_init(&IO_Init, &EC_Init);
}


/***************************************************************************************
 *                              Motor Control Relevant                                 *
 *  a: left     b: right                                                               *
 *  mutex_speed: lock of read/update speed                                             *
 ***************************************************************************************/

static float speed_mota, speed_motb;
static rt_mutex_t mutex_speed;

static int hal_gettick_us(void)
{
    int ms, us;

    ms = HAL_GetTick();
    us = TIM4->CNT;

    return ms*1000 + us;
}

static int motor_get_direction(TIM_TypeDef *timx)
{
    return ((timx->CR1 & ((0x1UL << (4U)))) == ((0x1UL << (4U))));
}

static int __encoder_get_num_of_pulses(TIM_TypeDef *timx)
{
    int cnt;

    cnt = (int)(int16_t)timx->CNT;
    timx->CNT = 0;

    return cnt;
}

/**
 * @param: delta_t (ms)
 * m / s
 * 
 * 时间秒 = delta_t / 1000
 * 码盘线数(精度) = 13
 * 减速比 = 30
 * 倍频 = 4
 * 轮胎周长 = 0.26m
 * 速度 = [采集的脉冲数 * 轮胎周长 / (码盘线数 x 减速比 x 倍频)] / 时间秒
 *      = (float)pulse * 1000 * 0.26 / 13 / 30 / 4 / delta_t;
 */
static float caculate_speed(TIM_TypeDef *timx, int delta_t)
{
    int pulse = __encoder_get_num_of_pulses(timx);

    return qfp_fdiv(pulse * 0.16666667, delta_t);
}

/**
 * for updating motor speed
 * reverse motor-a to ensure dual motor the same direction
 */
void motor_speed_update()
{
    static int last_tick;
    int current = HAL_GetTick();
    int dt =  current - last_tick;

    rt_mutex_take(mutex_speed, RT_WAITING_FOREVER);

    speed_mota = -caculate_speed(TIM1, dt);
    speed_motb =  caculate_speed(TIM2, dt);

    rt_mutex_release(mutex_speed);
    last_tick = current;
}

/**
 * read speed value of motor
 */
float motor_speed_read(char motor)
{
    float speed;
    rt_mutex_take(mutex_speed, RT_WAITING_FOREVER);

    if (motor == 'a')
        speed = speed_mota;
    else if (motor == 'b')
        speed = speed_motb;
    else
        speed = 0;

    rt_mutex_release(mutex_speed);
    return speed;
}

/**
 * 电机速度读写锁
 */
void lock_speed_init(void)
{
    mutex_speed = rt_mutex_create(
        "speed mutex",
        RT_IPC_FLAG_FIFO
    );

    if (mutex_speed == RT_NULL) {
        rt_kprintf("lock: Failed to init mutex of speed");
    }
}
