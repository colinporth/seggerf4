#include "ff_gen_drv.h"
#include "sd_diskio.h"

#define SD_TIMEOUT 1000
#define SD_DEFAULT_BLOCK_SIZE 512

static volatile DSTATUS Stat = STA_NOINIT;
static volatile UINT WriteStatus = 0;
static volatile UINT ReadStatus = 0;

static DSTATUS SD_CheckStatus (BYTE lun);

DSTATUS SD_initialize (BYTE);
DSTATUS SD_status (BYTE);

DRESULT SD_read (BYTE, BYTE*, DWORD, UINT);
DRESULT SD_write (BYTE, const BYTE*, DWORD, UINT);
DRESULT SD_ioctl (BYTE, BYTE, void*);

//{{{
const Diskio_drvTypeDef SD_Driver = {
  SD_initialize,
  SD_status,
  SD_read,
  SD_write,
  SD_ioctl,
  };
//}}}

//{{{
static DSTATUS SD_CheckStatus (BYTE lun)
{
  Stat = STA_NOINIT;

  if(BSP_SD_GetCardState() == MSD_OK)
  {
    Stat &= ~STA_NOINIT;
  }

  return Stat;
}
//}}}

//{{{
/**
  * @brief  Initializes a Drive
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_initialize (BYTE lun)
{
  Stat = STA_NOINIT;
#if !defined(DISABLE_SD_INIT)

  if(BSP_SD_Init() == MSD_OK)
    Stat = SD_CheckStatus(lun);

#else
  Stat = SD_CheckStatus(lun);
#endif

  return Stat;
}
//}}}
//{{{
/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_status (BYTE lun)
{
  return SD_CheckStatus(lun);
}
//}}}

//{{{
DRESULT SD_read (BYTE lun, BYTE* buff, DWORD sector, UINT count) {

  ReadStatus = 0;
  if (BSP_SD_ReadBlocks_DMA ((uint32_t*)buff, (uint32_t) (sector), count) == MSD_OK) {
    int wait = 0;
    while (ReadStatus == 0)
      wait++;
    while (BSP_SD_GetCardState() != MSD_OK)
      wait++;

    ReadStatus = 0;
    return RES_OK;
    }

  ReadStatus = 0;
  return RES_ERROR;
  }
//}}}
//{{{
DRESULT SD_write (BYTE lun, const BYTE* buff, DWORD sector, UINT count) {

  WriteStatus = 0;
  if (BSP_SD_WriteBlocks_DMA ((uint32_t*)buff, (uint32_t)(sector), count) == MSD_OK) {
    int wait = 0;
    while (WriteStatus == 0)
      wait++;
    while (BSP_SD_GetCardState() != MSD_OK)
      wait++;

    WriteStatus = 0;
    return RES_OK;
    }

  WriteStatus = 0;
  return RES_ERROR;
  }
//}}}
//{{{
DRESULT SD_ioctl (BYTE lun, BYTE cmd, void* buff) {

  if (Stat & STA_NOINIT)
    return RES_NOTRDY;

  BSP_SD_CardInfo CardInfo;
  switch (cmd) {
    /* Make sure that no pending write process */
    case CTRL_SYNC :
      return RES_OK;

    /* Get number of sectors on the disk (DWORD) */
    case GET_SECTOR_COUNT :
      BSP_SD_GetCardInfo(&CardInfo);
      *(DWORD*)buff = CardInfo.LogBlockNbr;
      return RES_OK;

    /* Get R/W sector size (WORD) */
    case GET_SECTOR_SIZE :
      BSP_SD_GetCardInfo(&CardInfo);
      *(WORD*)buff = CardInfo.LogBlockSize;
      return RES_OK;

    /* Get erase block size in unit of sector (DWORD) */
    case GET_BLOCK_SIZE :
      BSP_SD_GetCardInfo(&CardInfo);
      *(DWORD*)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
      return RES_OK;

    default:
      return RES_PARERR;
    }
  }
//}}}

//{{{
void HAL_SD_TxCpltCallback (SD_HandleTypeDef* hsd) {
  WriteStatus = 1;
  }
//}}}
//{{{
void HAL_SD_RxCpltCallback (SD_HandleTypeDef* hsd) {
  ReadStatus = 1;
  }
//}}}
