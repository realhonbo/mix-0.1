/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/*                        外设寄存器去 src 对应的文件内找                         */
/******************************************************************************/
#include "main.h"
#include "stm32f1xx_it.h"

extern TIM_HandleTypeDef htim4;

/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
    while (1);
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
    while (1);
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
    while (1);
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
    while (1);
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void);

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void);

/**
 * tim4 used as timebase of hal_gettick
 * and systick for rtthread
 */
void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim4);
}