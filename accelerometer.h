#pragma once
//{{{
#ifdef __cplusplus
 extern "C" {
#endif
//}}}

#include "stm32f4_discovery.h"

void BSP_ACCELERO_Init();
uint8_t BSP_ACCELERO_ReadID();
void BSP_ACCELERO_Reset();
void BSP_ACCELERO_GetXYZ (int8_t* pDataXYZ);

void BSP_ACCELERO_Click_ITConfig();
void BSP_ACCELERO_Click_ITClear();

//{{{
#ifdef __cplusplus
}
#endif
//}}}
