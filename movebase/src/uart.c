#include <stm32f1xx_hal.h>
#include <rtthread.h>
#include "include/multi.h"

static UART_HandleTypeDef huart1, huart2;
static DMA_HandleTypeDef  hdma1_uart1_tx;
static DMA_HandleTypeDef  hdma1_uart2_rx;

/***************************************************************************************
 *                              UART, Interrupt and DMA Init                           *
 *                                                                                     *
 * uart1 for kernel debug console                                                      *
 *      use dma-tx to release cpu's presure                                            *
 *                                                                                     *
 * uart2 for message transmit to ros                                                   *
 *      use idle + dma-rx to achieve a frame receice                                   *
 ***************************************************************************************/
void kdbg_uart_init(void)
{
    GPIO_InitTypeDef IO_Init;

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_AFIO_REMAP_USART1_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    // pin
    IO_Init.Pin = GPIO_PIN_6;
    IO_Init.Mode = GPIO_MODE_AF_PP;
    IO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &IO_Init);
    IO_Init.Pin = GPIO_PIN_7;
    IO_Init.Mode = GPIO_MODE_INPUT;
    IO_Init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &IO_Init);
    // hal
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
    // dma
    hdma1_uart1_tx.Instance = DMA1_Channel4;
    hdma1_uart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma1_uart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma1_uart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma1_uart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma1_uart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma1_uart1_tx.Init.Mode = DMA_NORMAL;
    hdma1_uart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma1_uart1_tx);
    __HAL_LINKDMA(&huart1, hdmatx, hdma1_uart1_tx);
    // interrupt
    HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

void ros_msg_uart_init(void)
{
    GPIO_InitTypeDef IO_Init;

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    // pin
    IO_Init.Pin = GPIO_PIN_2;
    IO_Init.Mode = GPIO_MODE_AF_PP;
    IO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &IO_Init);
    IO_Init.Pin = GPIO_PIN_3;
    IO_Init.Mode = GPIO_MODE_INPUT;
    IO_Init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &IO_Init);
    // hal
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
    // dma
    hdma1_uart2_rx.Instance = DMA1_Channel6;
    hdma1_uart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma1_uart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma1_uart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma1_uart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma1_uart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma1_uart2_rx.Init.Mode = DMA_NORMAL;
    hdma1_uart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma1_uart2_rx);
    __HAL_LINKDMA(&huart2, hdmarx, hdma1_uart2_rx);
    // interrupt
    HAL_NVIC_SetPriority(USART2_IRQn, 3, 1);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 1);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}


/***************************************************************************************
 *                                Printf Re-Direction                                  *
 * use semaphore to lock write stream, to ensure safety-thread                         *
 *                                                                                     *
 * use uart-dma to debug can improve about 70 percent peformance in this project       *
 *  or you can choose whether using dma in printf by USE_PRINTF_DMA                    *
 *  rt_kprintf is responsible for logging, do not suggest to use dma                   *
 ***************************************************************************************/

static rt_sem_t uart_sem;
static char dma_mem[128];

void lock_uart_init(void)
{
    uart_sem = rt_sem_create(
        "uart_sem", 
        1, 
        RT_IPC_FLAG_FIFO
    );

    if (uart_sem == RT_NULL) {
        rt_kprintf("lock: Failed to init sem of printf");
    }
}

int _write(int file, char *ptr, int len)
{
#ifdef USE_PRINTF_DMA
    rt_sem_take(uart_sem, RT_WAITING_FOREVER);
    rt_memcpy(dma_mem, ptr, len);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)dma_mem, len);
#else
    (void)file;
    rt_sem_take(uart_sem, RT_WAITING_FOREVER);
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 0xff);
    rt_sem_release(uart_sem);
#endif
    return len;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
        rt_sem_release(uart_sem);
}

void rt_hw_console_output(const char *str)
{
    int size;
    int i;

    __HAL_UNLOCK(&huart1);
    size = rt_strlen(str);

    HAL_UART_Transmit(&huart1, (uint8_t *)str, size, 0xff);
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 0xff);
}


/***************************************************************************************
 *                         Params Debug: Transmit and Receive API                      *
 * kdb_raw_data_transmit: 
 * vofa_receive_it / HAL_UART_RxCpltCallback: for debug PID params and so on with vofa *
 *  frame format: 0xFA  0xFB  0x01  [ Kp-hex ]                                         *
 ***************************************************************************************/

extern float Kp, Ki, Kd;
static uint8_t param_buffer[7];

void kdb_raw_data_transmit(uint8_t *data, int size)
{
    HAL_UART_Transmit(&huart1, data, size, 0xff);
}

void vofa_receive_it(void)
{
    HAL_UART_Receive_IT(&huart1, param_buffer, sizeof(param_buffer));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
    // debug serial
        if (*(uint16_t *)param_buffer == 0xFBFA) {
            switch (param_buffer[2]) {
            case 0x01: // Kp
                Kp = *((float *)(param_buffer+3)); break;
            case 0x02: // Ki
                Ki = *((float *)(param_buffer+3)); break;
            case 0x04: // Kd
                Kd = *((float *)(param_buffer+3)); break;
            }
        }
    }
}


/***************************************************************************************
 *                                Connect ROS                                          *
 * receive ros velocity message by uart2                                               *
 ***************************************************************************************/

void ros_message_transmit(const uint8_t *buffer, uint16_t size)
{
    HAL_UART_Transmit(&huart2, buffer, size, 0xff);
}

void ros_message_receive_it(uint8_t *buffer, uint16_t size)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, buffer, size);
}


/***************************************************************************************
 *                              UART and DMA IRQ-Handler                               *
 ***************************************************************************************/
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

void USART2_IRQHandler(void)
{
    extern struct rx_pack rx_ros;

    HAL_UART_IRQHandler(&huart2);

    if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        /*HAL_UART_DMAStop(&huart2); + Data Process, but we don't need*/
        HAL_UART_Receive_DMA(&huart2, (uint8_t *)&rx_ros, 14);
    }
}

void DMA1_Channel4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma1_uart1_tx);
}

void DMA1_Channel6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma1_uart2_rx);
}

