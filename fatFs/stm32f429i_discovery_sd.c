#include "stm32f429i_discovery_sd.h"

SD_HandleTypeDef gSdHandle;
DMA_HandleTypeDef gDmaRxHandle;
DMA_HandleTypeDef gDmaTxHandle;

void SDIO_IRQHandler() { HAL_SD_IRQHandler (&gSdHandle); }
void DMA2_Stream3_IRQHandler() { HAL_DMA_IRQHandler (gSdHandle.hdmarx); }
void DMA2_Stream6_IRQHandler() { HAL_DMA_IRQHandler (gSdHandle.hdmatx); }

//{{{
uint8_t BSP_SD_Init() {

  uint8_t sd_state = MSD_OK;

  /* PLLSAI is dedicated to LCD periph. Do not use it to get 48MHz*/

  /* uSD device interface configuration */
  gSdHandle.Instance = SDIO;
  gSdHandle.Init.ClockEdge           = SDIO_CLOCK_EDGE_RISING;
  gSdHandle.Init.ClockBypass         = SDIO_CLOCK_BYPASS_DISABLE;
  gSdHandle.Init.ClockPowerSave      = SDIO_CLOCK_POWER_SAVE_DISABLE;
  gSdHandle.Init.BusWide             = SDIO_BUS_WIDE_1B;
  gSdHandle.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
  gSdHandle.Init.ClockDiv            = SDIO_TRANSFER_CLK_DIV;

  if (BSP_SD_IsDetected() != SD_PRESENT)
    return MSD_ERROR_SD_NOT_PRESENT;

  __HAL_RCC_SDIO_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  //{{{  gpio init
  // sdPresent init - PC6
  GPIO_InitTypeDef gpio_init_structure;
  gpio_init_structure.Pin = GPIO_PIN_6;
  gpio_init_structure.Mode = GPIO_MODE_INPUT;
  gpio_init_structure.Pull = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init (GPIOC, &gpio_init_structure);

  // SDIO D0..D3 - PC8..PC11
  gpio_init_structure.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Alternate = GPIO_AF12_SDIO;
  HAL_GPIO_Init (GPIOC, &gpio_init_structure);

  // SDIO CMD - PD2
  gpio_init_structure.Pin = GPIO_PIN_2;
  HAL_GPIO_Init (GPIOD, &gpio_init_structure);

  // SDIO CLK - PC12
  gpio_init_structure.Pin = GPIO_PIN_12;
  //gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init (GPIOC, &gpio_init_structure);
  //}}}

  //{{{  DMA rx parameters
  gDmaRxHandle.Instance                 = DMA2_Stream3;
  gDmaRxHandle.Init.Channel             = DMA_CHANNEL_4;
  gDmaRxHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  gDmaRxHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  gDmaRxHandle.Init.MemInc              = DMA_MINC_ENABLE;
  gDmaRxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  gDmaRxHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  gDmaRxHandle.Init.Mode                = DMA_PFCTRL;
  gDmaRxHandle.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
  gDmaRxHandle.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
  gDmaRxHandle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  gDmaRxHandle.Init.MemBurst            = DMA_MBURST_INC4;
  gDmaRxHandle.Init.PeriphBurst         = DMA_PBURST_INC4;

  __HAL_LINKDMA (&gSdHandle, hdmarx, gDmaRxHandle);
  HAL_DMA_DeInit (&gDmaRxHandle);
  HAL_DMA_Init (&gDmaRxHandle);
  //}}}
  //{{{  DMA tx parameters
  gDmaTxHandle.Instance                 = DMA2_Stream6;
  gDmaTxHandle.Init.Channel             = DMA_CHANNEL_4;
  gDmaTxHandle.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  gDmaTxHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  gDmaTxHandle.Init.MemInc              = DMA_MINC_ENABLE;
  gDmaTxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  gDmaTxHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  gDmaTxHandle.Init.Mode                = DMA_PFCTRL;
  gDmaTxHandle.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
  gDmaTxHandle.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
  gDmaTxHandle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  gDmaTxHandle.Init.MemBurst            = DMA_MBURST_INC4;
  gDmaTxHandle.Init.PeriphBurst         = DMA_PBURST_INC4;

  __HAL_LINKDMA (&gSdHandle, hdmatx, gDmaTxHandle);
  HAL_DMA_DeInit (&gDmaTxHandle);
  HAL_DMA_Init (&gDmaTxHandle);
  //}}}

  HAL_NVIC_SetPriority (DMA2_Stream3_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ (DMA2_Stream3_IRQn);
  HAL_NVIC_SetPriority (DMA2_Stream6_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ (DMA2_Stream6_IRQn);
  HAL_NVIC_SetPriority (SDIO_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ (SDIO_IRQn);

  /* HAL SD initialization */
  if (HAL_SD_Init(&gSdHandle) != HAL_OK)
    sd_state = MSD_ERROR;

  /* Configure SD Bus width */
  if (sd_state == MSD_OK) {
    /* Enable wide operation */
    if(HAL_SD_ConfigWideBusOperation (&gSdHandle, SDIO_BUS_WIDE_4B) != HAL_OK)
      sd_state = MSD_ERROR;
    else
      sd_state = MSD_OK;
    }

  return  sd_state;
  }
//}}}
//{{{
uint8_t BSP_SD_DeInit()
{
  uint8_t sd_state = MSD_OK;
  return  sd_state;
}
//}}}

//{{{
uint8_t BSP_SD_IsDetected() {
  return SD_PRESENT;
  }
//}}}

//{{{
uint8_t BSP_SD_ReadBlocks (uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout) {

  if (HAL_SD_ReadBlocks(&gSdHandle, (uint8_t *)pData, ReadAddr, NumOfBlocks, Timeout) != HAL_OK)
    return MSD_ERROR;
  else
    return MSD_OK;
  }
//}}}
//{{{
uint8_t BSP_SD_WriteBlocks (uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout) {

  if(HAL_SD_WriteBlocks(&gSdHandle, (uint8_t *)pData, WriteAddr, NumOfBlocks, Timeout) != HAL_OK)
    return MSD_ERROR;
  else
    return MSD_OK;
  }
//}}}
//{{{
uint8_t BSP_SD_ReadBlocks_DMA (uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks) {

  /* Read block(s) in DMA transfer mode */
  if (HAL_SD_ReadBlocks_DMA(&gSdHandle, (uint8_t *)pData, ReadAddr, NumOfBlocks) != HAL_OK)
    return MSD_ERROR;
  else
    return MSD_OK;
  }
//}}}
//{{{
uint8_t BSP_SD_WriteBlocks_DMA (uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks) {

  /* Write block(s) in DMA transfer mode */
  if(HAL_SD_WriteBlocks_DMA(&gSdHandle, (uint8_t *)pData, WriteAddr, NumOfBlocks) != HAL_OK)
    return MSD_ERROR;
  else
    return MSD_OK;
  }
//}}}

//{{{
/**
  * @brief  Gets the current SD card data status.
  * @retval Data transfer state.
  *          This value can be one of the following values:
  *            @arg  SD_TRANSFER_OK: No data transfer is acting
  *            @arg  SD_TRANSFER_BUSY: Data transfer is acting
  */
uint8_t BSP_SD_GetCardState()
{
  return((HAL_SD_GetCardState(&gSdHandle) == HAL_SD_CARD_TRANSFER ) ? SD_TRANSFER_OK : SD_TRANSFER_BUSY);
}

//}}}
void BSP_SD_GetCardInfo (HAL_SD_CardInfoTypeDef *CardInfo) { HAL_SD_GetCardInfo(&gSdHandle, CardInfo); }

void HAL_SD_AbortCallback (SD_HandleTypeDef *hsd) { BSP_SD_AbortCallback(); }
void HAL_SD_TxCpltCallback (SD_HandleTypeDef *hsd) { BSP_SD_WriteCpltCallback(); }
void HAL_SD_RxCpltCallback (SD_HandleTypeDef *hsd) { BSP_SD_ReadCpltCallback(); }

__weak void BSP_SD_AbortCallback() {}
__weak void BSP_SD_WriteCpltCallback() { }
__weak void BSP_SD_ReadCpltCallback() {}
