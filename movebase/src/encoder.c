#include <stm32f1xx_hal.h>
#include <rtthread.h>
#include <qfplib-m3.h>
#include "gpio.h"
#include "include/multi.h"

/*******************************
 ** 编码器数据读取初始化 (TIM2 TIM1)
 *       A相   B相
 * TIM1: PA8   PA9    > Motor_A
 * TIM2: PA1   PA0    > Motor_B
 *! 注意: TIM2的1,2通道 与 编码器B,A相 对应
 *!      motor_B 转动方向与 motor_A 相反
 */
static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim1;

static void encoder_tim1_init(GPIO_InitTypeDef *IO_Init,
                              TIM_Encoder_InitTypeDef *EC_Init)
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

static void encoder_tim2_init(GPIO_InitTypeDef *IO_Init,
                              TIM_Encoder_InitTypeDef *EC_Init)
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
    GPIO_InitTypeDef IO_Init;
    TIM_Encoder_InitTypeDef EC_Init;

    __HAL_RCC_GPIOA_CLK_ENABLE();

// gpio
    IO_Init.Mode = GPIO_MODE_INPUT;
// encoder
    EC_Init.EncoderMode = TIM_ENCODERMODE_TI12; // TI1 + TI2 正交编码
    EC_Init.IC1Polarity = TIM_ICPOLARITY_RISING; // 上升沿触发
    EC_Init.IC2Polarity = TIM_ICPOLARITY_RISING;
    EC_Init.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    EC_Init.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    EC_Init.IC1Filter = 10; // 滤波
    EC_Init.IC2Filter = 10;
// init
    encoder_tim1_init(&IO_Init, &EC_Init);
    encoder_tim2_init(&IO_Init, &EC_Init);
}


/***************************
 ** 电机控制相关 
 ** motorx_speed: 电机x的速度
 ** a: left     b: right
 ** speed_mux: 速度读写锁
 */
static float speed_mota;
static float speed_motb;
static rt_mutex_t speed_mux;

int getick_us(void)
{
    int ms, us;

    ms = HAL_GetTick();
    us = TIM4->CNT;

    return ms*1000 + us;
}

/**
 * 电机运动方向
 */
int motor_get_direction(TIM_TypeDef *timx)
{
    return ((timx->CR1 & ((0x1UL << (4U)))) == ((0x1UL << (4U))));
}

/**
 * 编码器模式下 CNT 录到的脉冲数
 */
static int get_pulse(TIM_TypeDef *timx)
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
    int pulse;
    float speed;

    pulse = get_pulse(timx);
    speed = qfp_fdiv(pulse * 0.16666667, delta_t);

    return speed;
}

/*
 * individual thread for updating motor speed
 */
void motor_speed_update()
{
    rt_mutex_take(speed_mux, RT_WAITING_FOREVER);

    speed_mota = -caculate_speed(TIM1, PWM_ENCODER_PERIOD);
    speed_motb =  caculate_speed(TIM2, PWM_ENCODER_PERIOD);

    rt_mutex_release(speed_mux);
}

/**
 * read value of motor speed
 */
float motor_speed_read(char motor)
{
    float speed;
    rt_mutex_take(speed_mux, RT_WAITING_FOREVER);

    if (motor == 'A')
        speed = speed_mota;
    else if (motor == 'B')
        speed = speed_motb;
    else
        speed = 0;

    rt_mutex_release(speed_mux);
    return speed;
}

/**
 * 电机速度读写锁
 */
void lock_speed_init(void)
{
    speed_mux = rt_mutex_create("speed mutex", RT_IPC_FLAG_FIFO);

    if (speed_mux == RT_NULL) {
        rt_kprintf("Failed in init mutex of speed...");
    }
}
