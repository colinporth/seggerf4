#include "ff_gen_drv.h"
#include "sd_diskio.h"

#define SD_TIMEOUT 1000

#define SD_DEFAULT_BLOCK_SIZE 512

static volatile DSTATUS Stat = STA_NOINIT;

static DSTATUS SD_CheckStatus(BYTE lun);

DSTATUS SD_initialize (BYTE);
DSTATUS SD_status (BYTE);

DRESULT SD_read (BYTE, BYTE*, DWORD, UINT);
DRESULT SD_write (BYTE, const BYTE*, DWORD, UINT);
DRESULT SD_ioctl (BYTE, BYTE, void*);

//{{{
const Diskio_drvTypeDef  SD_Driver =
{
  SD_initialize,
  SD_status,
  SD_read,
#if  _USE_WRITE == 1
  SD_write,
#endif /* _USE_WRITE == 1 */

#if  _USE_IOCTL == 1
  SD_ioctl,
#endif /* _USE_IOCTL == 1 */
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
/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT SD_read (BYTE lun, BYTE *buff, DWORD sector, UINT count) {

  DRESULT res = RES_ERROR;

  if (BSP_SD_ReadBlocks_DMA ((uint32_t*)buff, (uint32_t) (sector), count) == MSD_OK) {
    /* wait until the read operation is finished */
    int wait = 0;
    while (BSP_SD_GetCardState() != MSD_OK) 
      wait++;
      
    res = RES_OK;
    }

  return res;
  }
//}}}
//{{{
DRESULT SD_write (BYTE lun, const BYTE *buff, DWORD sector, UINT count) {

  DRESULT res = RES_ERROR;
  if (BSP_SD_WriteBlocks_DMA((uint32_t*)buff, (uint32_t)(sector), count) == MSD_OK) {
    int wait = 0;
    while(BSP_SD_GetCardState() != MSD_OK)
      wait++;

    res = RES_OK;
    }

  return res;
  }
//}}}
//{{{
DRESULT SD_ioctl (BYTE lun, BYTE cmd, void *buff)
{
  DRESULT res = RES_ERROR;
  BSP_SD_CardInfo CardInfo;

  if (Stat & STA_NOINIT) return RES_NOTRDY;

  switch (cmd)
  {
  /* Make sure that no pending write process */
  case CTRL_SYNC :
    res = RES_OK;
    break;

  /* Get number of sectors on the disk (DWORD) */
  case GET_SECTOR_COUNT :
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockNbr;
    res = RES_OK;
    break;

  /* Get R/W sector size (WORD) */
  case GET_SECTOR_SIZE :
    BSP_SD_GetCardInfo(&CardInfo);
    *(WORD*)buff = CardInfo.LogBlockSize;
    res = RES_OK;
    break;

  /* Get erase block size in unit of sector (DWORD) */
  case GET_BLOCK_SIZE :
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
  res = RES_OK;
    break;

  default:
    res = RES_PARERR;
  }

  return res;
}
//}}}
