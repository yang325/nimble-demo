/* Includes ------------------------------------------------------------------*/

#include "stm32f1xx_hal.h"

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hci_handle;

/* Exported functions --------------------------------------------------------*/


/* SPI3 init function */
void hci_remap_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PF2 PF3 PF4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* SPI3 parameter configuration*/
  hci_handle.Instance = SPI3;
  hci_handle.Init.Mode = SPI_MODE_MASTER;
  hci_handle.Init.Direction = SPI_DIRECTION_2LINES;
  hci_handle.Init.DataSize = SPI_DATASIZE_8BIT;
  hci_handle.Init.CLKPolarity = SPI_POLARITY_LOW;
  hci_handle.Init.CLKPhase = SPI_PHASE_1EDGE;
  hci_handle.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hci_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hci_handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hci_handle.Init.TIMode = SPI_TIMODE_DISABLE;
  hci_handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hci_handle.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hci_handle) != HAL_OK)
  {
    Error_Handler();
  }
}

