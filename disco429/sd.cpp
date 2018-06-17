// sd.cpp
#include "sd.h"
#include "cLcd.h"

SD_HandleTypeDef gSdHandle;
DMA_HandleTypeDef gDmaRxHandle;
DMA_HandleTypeDef gDmaTxHandle;

#define SD_TIMEOUT 1000
#define SD_DEFAULT_BLOCK_SIZE 512

static volatile DSTATUS gStat = STA_NOINIT;
static volatile UINT gWriteStatus = 0;
static volatile UINT gReadStatus = 0;

//{{{
void HAL_SD_TxCpltCallback (SD_HandleTypeDef* hsd) {
  gWriteStatus = 1;
  }
//}}}
//{{{
void HAL_SD_RxCpltCallback (SD_HandleTypeDef* hsd) {
  gReadStatus = 1;
  }
//}}}

extern "C" { void SDIO_IRQHandler() { HAL_SD_IRQHandler (&gSdHandle); } }
extern "C" { void DMA2_Stream3_IRQHandler() { HAL_DMA_IRQHandler (gSdHandle.hdmarx); } }
extern "C" { void DMA2_Stream6_IRQHandler() { HAL_DMA_IRQHandler (gSdHandle.hdmatx); } }

//{{{
uint8_t isDetected() {
  return SD_PRESENT;
  }
//}}}
//{{{
uint8_t getCardState() {                
  return HAL_SD_GetCardState (&gSdHandle) == HAL_SD_CARD_TRANSFER ? SD_TRANSFER_OK : SD_TRANSFER_BUSY;
  }
//}}}
//{{{
void getCardInfo (HAL_SD_CardInfoTypeDef *CardInfo) {
  HAL_SD_GetCardInfo (&gSdHandle, CardInfo);
  }
//}}}
//{{{
DSTATUS checkStatus (BYTE lun) {

  gStat = STA_NOINIT;

  if (getCardState() == MSD_OK)
    gStat &= ~STA_NOINIT;

  return gStat;
  }
//}}}

//{{{
DSTATUS SD_initialize (BYTE lun) {

  gStat = STA_NOINIT;

  /* PLLSAI is dedicated to LCD periph. Do not use it to get 48MHz*/

  /* uSD device interface configuration */
  gSdHandle.Instance = SDIO;
  gSdHandle.Init.ClockEdge           = SDIO_CLOCK_EDGE_RISING;
  gSdHandle.Init.ClockBypass         = SDIO_CLOCK_BYPASS_DISABLE;
  gSdHandle.Init.ClockPowerSave      = SDIO_CLOCK_POWER_SAVE_DISABLE;
  gSdHandle.Init.BusWide             = SDIO_BUS_WIDE_1B;
  gSdHandle.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
  gSdHandle.Init.ClockDiv            = SDIO_TRANSFER_CLK_DIV;

  if (isDetected() != SD_PRESENT)
    return MSD_ERROR_SD_NOT_PRESENT;

  __HAL_RCC_SDIO_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  //{{{  gpio init
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

  if (HAL_SD_Init (&gSdHandle) != HAL_OK)
    cLcd::mLcd->debug (COL_RED, "HAL_SD_Init failed");
  else if (HAL_SD_ConfigWideBusOperation (&gSdHandle, SDIO_BUS_WIDE_4B) != HAL_OK)
    cLcd::mLcd->debug (COL_RED, "HAL_SD_ConfigWideBusOperation failed");

  gStat = checkStatus (lun);

  return gStat;
  }
//}}}
//{{{
DSTATUS SD_status (BYTE lun) {
  return checkStatus (lun);
  }
//}}}
//{{{
DRESULT SD_read (BYTE lun, BYTE* buff, DWORD sector, UINT count) {

  cLcd::mLcd->debug (COL_GREEN, "readBlocks " + hex (uint32_t(buff)) + " " + dec (sector) + " " + dec(count));

  gReadStatus = 0;
  if (HAL_SD_ReadBlocks_DMA (&gSdHandle, buff, sector, count) == HAL_OK) {
    int wait = 0;
    while (gReadStatus == 0)
      wait++;
    while (getCardState() != MSD_OK)
      wait++;

    gReadStatus = 0;
    return RES_OK;
    }

  gReadStatus = 0;
  return RES_ERROR;
  }
//}}}
//{{{
DRESULT SD_write (BYTE lun, const BYTE* buff, DWORD sector, UINT count) {

  gWriteStatus = 0;

  if (HAL_SD_WriteBlocks_DMA (&gSdHandle, (BYTE*)buff, sector, count) == HAL_OK) {
    int wait = 0;
    while (gWriteStatus == 0)
      wait++;
    while (getCardState() != MSD_OK)
      wait++;

    gWriteStatus = 0;
    return RES_OK;
    }

  gWriteStatus = 0;
  return RES_ERROR;
  }
//}}}
//{{{
DRESULT SD_ioctl (BYTE lun, BYTE cmd, void* buff) {

  if (gStat & STA_NOINIT)
    return RES_NOTRDY;

  BSP_SD_CardInfo CardInfo;
  switch (cmd) {
    /* Make sure that no pending write process */
    case CTRL_SYNC :
      return RES_OK;

    /* Get number of sectors on the disk (DWORD) */
    case GET_SECTOR_COUNT :
      getCardInfo(&CardInfo);
      *(DWORD*)buff = CardInfo.LogBlockNbr;
      return RES_OK;

    /* Get R/W sector size (WORD) */
    case GET_SECTOR_SIZE :
      getCardInfo(&CardInfo);
      *(WORD*)buff = CardInfo.LogBlockSize;
      return RES_OK;

    /* Get erase block size in unit of sector (DWORD) */
    case GET_BLOCK_SIZE :
      getCardInfo(&CardInfo);
      *(DWORD*)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
      return RES_OK;

    default:
      return RES_PARERR;
    }
  }
//}}}

//{{{
const Diskio_drvTypeDef SD_Driver = {
  SD_initialize,
  SD_status,
  SD_read,
  SD_write,
  SD_ioctl,
  };
//}}}
