/* Includes ------------------------------------------------------------------*/

#include "stm32f1xx_hal.h"

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef console_handle;

/* Exported functions --------------------------------------------------------*/


/* Console init function */
void console_init(void)
{
  console_handle.Instance = USART1;
  console_handle.Init.BaudRate = 115200;
  console_handle.Init.WordLength = UART_WORDLENGTH_8B;
  console_handle.Init.StopBits = UART_STOPBITS_1;
  console_handle.Init.Parity = UART_PARITY_NONE;
  console_handle.Init.Mode = UART_MODE_TX_RX;
  console_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  console_handle.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&console_handle) != HAL_OK)
  {
    Error_Handler();
  }
}

