#include "main.h"
#include "stm32f1xx_it.h"

extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma1_uart1_tx;
extern DMA_HandleTypeDef hdma1_uart2_tx;

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
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
void SVC_Handler(void)
{

}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{

}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/


void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim4);
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

void DMA1_Channel4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma1_uart1_tx);
}

void DMA1_Channel7_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma1_uart2_tx);
}