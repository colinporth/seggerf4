#include "ff_gen_drv.h"
#include "sd_diskio.h"

#define SD_TIMEOUT 1000
#define SD_DEFAULT_BLOCK_SIZE 512

static volatile DSTATUS gStat = STA_NOINIT;
static volatile UINT gWriteStatus = 0;
static volatile UINT gReadStatus = 0;

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
static DSTATUS SD_CheckStatus (BYTE lun) {

  gStat = STA_NOINIT;

  if (BSP_SD_GetCardState() == MSD_OK)
    gStat &= ~STA_NOINIT;

  return gStat;
  }
//}}}

//{{{
DSTATUS SD_initialize (BYTE lun) {

  gStat = STA_NOINIT;

  if (BSP_SD_Init() == MSD_OK)
    gStat = SD_CheckStatus (lun);

  gStat = SD_CheckStatus (lun);
  return gStat;
  }
//}}}
//{{{
DSTATUS SD_status (BYTE lun) {
  return SD_CheckStatus (lun);
  }
//}}}

//{{{
DRESULT SD_read (BYTE lun, BYTE* buff, DWORD sector, UINT count) {

  gReadStatus = 0;
  if (BSP_SD_ReadBlocks ((uint32_t*)buff, (uint32_t) (sector), count) == MSD_OK) {
    int wait = 0;
    while (gReadStatus == 0)
      wait++;
    while (BSP_SD_GetCardState() != MSD_OK)
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
  if (BSP_SD_WriteBlocks ((uint32_t*)buff, (uint32_t)(sector), count) == MSD_OK) {
    int wait = 0;
    while (gWriteStatus == 0)
      wait++;
    while (BSP_SD_GetCardState() != MSD_OK)
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
  gWriteStatus = 1;
  }
//}}}
//{{{
void HAL_SD_RxCpltCallback (SD_HandleTypeDef* hsd) {
  gReadStatus = 1;
  }
//}}}
