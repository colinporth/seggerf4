#pragma once
//{{{
#ifdef __cplusplus
 extern "C" {
#endif
//}}}
#include "stm32f429i_discovery.h"

typedef enum {
  GYRO_OK = 0,
  GYRO_ERROR = 1,
  GYRO_TIMEOUT = 2
  } GYRO_StatusTypeDef;

// GYRO Interrupt struct
typedef struct {
  uint8_t Latch_Request;          /* Latch interrupt request into CLICK_SRC register */
  uint8_t Interrupt_Axes;         /* X, Y, Z Axes Interrupts */
  uint8_t Interrupt_ActiveEdge;   /* Interrupt Active edge */
  } GYRO_InterruptConfigTypeDef;

uint8_t BSP_GYRO_Init();
void    BSP_GYRO_Reset();
uint8_t BSP_GYRO_ReadID();
void    BSP_GYRO_ITConfig (GYRO_InterruptConfigTypeDef *pIntConfigStruct);
void    BSP_GYRO_EnableIT (uint8_t IntPin);
void    BSP_GYRO_DisableIT (uint8_t IntPin);
void    BSP_GYRO_GetXYZ (float* pfData);

//{{{
#ifdef __cplusplus
}
#endif
//}}}
