#ifndef __FF_GEN_DRV_H
#define __FF_GEN_DRV_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "diskio.h"
#include "ff.h"
#include <stdint.h>

typedef struct {
  DSTATUS (*disk_initialize) (BYTE);
  DSTATUS (*disk_status)     (BYTE);
  DRESULT (*disk_read)       (BYTE, BYTE*, DWORD, UINT);
#if _USE_WRITE == 1
  DRESULT (*disk_write)      (BYTE, const BYTE*, DWORD, UINT);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT (*disk_ioctl)      (BYTE, BYTE, void*);
#endif /* _USE_IOCTL == 1 */
  } Diskio_drvTypeDef;

typedef struct {
  uint8_t                 is_initialized[_VOLUMES];
  const Diskio_drvTypeDef *drv[_VOLUMES];
  uint8_t                 lun[_VOLUMES];
  volatile uint8_t        nbr;
  } Disk_drvTypeDef;

uint8_t FATFS_LinkDriver (const Diskio_drvTypeDef *drv, char *path);
uint8_t FATFS_UnLinkDriver (char *path);
uint8_t FATFS_LinkDriverEx (const Diskio_drvTypeDef *drv, char *path, BYTE lun);
uint8_t FATFS_UnLinkDriverEx (char *path, BYTE lun);
uint8_t FATFS_GetAttachedDriversNbr();

#ifdef __cplusplus
}
#endif

#endif /* __FF_GEN_DRV_H */
