/* Includes ------------------------------------------------------------------*/

#include "stm32f1xx_hal.h"

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hci_handle;

/* Exported functions --------------------------------------------------------*/


/* SPI3 init function */
void hci_remap_init(void)
{
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

