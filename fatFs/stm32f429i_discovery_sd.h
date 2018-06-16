#ifndef __STM32469I_DISCOVERY_SD_H
#define __STM32469I_DISCOVERY_SD_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32F429i_discovery.h"

#define BSP_SD_CardInfo HAL_SD_CardInfoTypeDef

#define MSD_OK                        ((uint8_t)0x00)
#define MSD_ERROR                     ((uint8_t)0x01)
#define MSD_ERROR_SD_NOT_PRESENT      ((uint8_t)0x02)

#define SD_TRANSFER_OK                ((uint8_t)0x00)
#define SD_TRANSFER_BUSY              ((uint8_t)0x01)

#define SD_PRESENT               ((uint8_t)0x01)
#define SD_NOT_PRESENT           ((uint8_t)0x00)

uint8_t BSP_SD_Init();

uint8_t BSP_SD_GetCardState();
void    BSP_SD_GetCardInfo(HAL_SD_CardInfoTypeDef *CardInfo);
uint8_t BSP_SD_IsDetected();

uint8_t BSP_SD_ReadBlocks_DMA (uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks);
uint8_t BSP_SD_WriteBlocks_DMA (uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks);

void    BSP_SD_AbortCallback();
void    BSP_SD_WriteCpltCallback();
void    BSP_SD_ReadCpltCallback();

#ifdef __cplusplus
}
#endif

#endif /* __STM32469I_DISCOVERY_SD_H */
