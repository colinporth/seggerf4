#pragma once
//{{{
#ifdef __cplusplus
 extern "C" {
#endif
//}}}

#include "stm32f4_discovery.h"

#include "lis302dl.h"
typedef enum { ACCELERO_OK = 0, ACCELERO_ERROR = 1, ACCELERO_TIMEOUT = 2 }ACCELERO_StatusTypeDef;

uint8_t BSP_ACCELERO_Init();
uint8_t BSP_ACCELERO_ReadID();
void    BSP_ACCELERO_Reset();
void    BSP_ACCELERO_Click_ITConfig();
void    BSP_ACCELERO_Click_ITClear();
void    BSP_ACCELERO_GetXYZ (int16_t *pDataXYZ);

//{{{
#ifdef __cplusplus
}
#endif
//}}}
