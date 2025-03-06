#include <stm32f1xx_hal.h>
#include <qfplib-m3.h>
#include "gpio.h"
#include "include/multi.h"

static TIM_HandleTypeDef htim3;

/**
 * gpio of A/B_IN init
 * AIN1: PB10   BIN1: PA7
 * AIN2: PB11   BIN2: PA6
 * @usage: 用于电机正反转控制的 GPIO
 */
void gpio_ain_bin_init(void)
{
    GPIO_InitTypeDef IO_Init = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    IO_Init.Pin   = GPIO_PIN_11 | GPIO_PIN_10;
    IO_Init.Mode  = GPIO_MODE_OUTPUT_PP;
    IO_Init.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &IO_Init);

    IO_Init.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &IO_Init);
}

/**
 * PWM_A and PWM_B init
 * PWM_A: PB1 ( TIM3_CH4 )
 * PWM_B: PB0 ( TIM3_CH3 )
 * @usage: 用于电机速度控制的 PWM
 */
void pwm_generator_init(void)
{
    GPIO_InitTypeDef   IO_Init;
    TIM_OC_InitTypeDef OC_Init;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_AFIO_REMAP_TIM3_PARTIAL();
    __HAL_RCC_TIM3_CLK_ENABLE();
// gpio
    IO_Init.Pin = GPIO_PIN_1 | GPIO_PIN_0;
    IO_Init.Mode = GPIO_MODE_AF_PP;
    IO_Init.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &IO_Init);
// timer
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP; // 向上计数
    htim3.Init.Period = 7199; // RELOAD
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 不分频
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim3);
// pwm channel
    OC_Init.OCMode = TIM_OCMODE_PWM1;
    OC_Init.Pulse = 0; // 初始占空比
    OC_Init.OCPolarity = TIM_OCPOLARITY_HIGH;
    OC_Init.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &OC_Init, TIM_CHANNEL_4);
    HAL_TIM_PWM_ConfigChannel(&htim3, &OC_Init, TIM_CHANNEL_3);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

void motor_drive_pwm_set(int pwm_l, int pwm_r)
{
    // Motor Left
    if (pwm_l > 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
        pwm_l = -pwm_l;
    }
    // Motor Right
    if (pwm_r > 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
        pwm_r = -pwm_r;
    }
    // 设置比较寄存器值
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_l);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_r);
}


/***************************************************************************************
 *                                 PID Controller                                      *
 *                                                                                     *
 * error[1]: err_prev                                                                  *
 * error[2]: err_int                                                                   *
 *                                                                                     *
 * {Kp, Ki, Kd} = {0.55,  0,  0.7 }                                                    *
 *                {0.55,  0,  0.35}                                                    *
 *                {0.2 ,  0,  0.15}                                                    *
 *                {0.3 ,  0,  0.25}                                                    *
 ***************************************************************************************/
#define DIV_7199_1DOT48  4864.18919

float Kp = 0.2;
float Ki = 0;
float Kd = 0.15;

static float pid_core_old(float target, float actual, float error[3])
{
    float out;
    int err;

    // error and intergration
    err = target - actual;
    error[0] = err;
    error[1] += err;

    // 积分限幅
    if (error[1] >  0.2) error[1] =  0.2;
    if (error[1] < -0.2) error[1] = -0.2;

    // delta_out
    out = Kp * err + Ki * error[1] + Kd * (err - error[0]);

    // 输出限幅
    if (out >  1.48) out =  1.48;
    if (out < -1.48) out = -1.48;

    return out;
}

static float pid_core_oold(float target, float actual, float error[3])
{
    float out;

    // error and intergration
    error[1] = error[0];
    error[0] = target - actual;
    error[2] += error[0];

    // 积分限幅
    if (error[2] > 0.2)  error[2] = 0.2;
    if (error[2] < -0.2) error[2] = -0.2;

    // delta_out
    out = Kp * error[0] + Ki * error[2] + Kd * (error[0] - error[1]);

    // 输出限幅
    if (out > 1.48)  out = 1.48;
    if (out < -1.48) out = -1.48;

    return out;
}

int pid_corrector_old(float target, float actual, float error[3])
{
    return pid_core_old(target, actual, error) * DIV_7199_1DOT48;
}


static float pid_core(float error, struct pid x)
{
    float out;

    // error and intergration
    x.error_prev = x.error;
    x.error = error;
    x.error_int += x.error;

    // 积分限幅
    if (x.error_int >  0.2) x.error_int =  0.2;
    if (x.error_int < -0.2) x.error_int = -0.2;

    // delta_out
    out = Kp * x.error + Ki * x.error_int + Kd * (x.error - x.error_prev);

    // 输出限幅
    if (out > 1.48)  out = 1.48;
    if (out < -1.48) out = -1.48;

    return out;
}

int pid_corrector(float error, struct pid x)
{
    return pid_core(error, x) * DIV_7199_1DOT48;
}
