#pragma once
#include "stm32f429i_discovery.h"
//{{{
#ifdef __cplusplus
 extern "C" {
#endif
//}}}

// GYRO Interrupt struct
typedef struct {
  uint8_t Latch_Request;          /* Latch interrupt request into CLICK_SRC register */
  uint8_t Interrupt_Axes;         /* X, Y, Z Axes Interrupts */
  uint8_t Interrupt_ActiveEdge;   /* Interrupt Active edge */
  } GYRO_InterruptConfigTypeDef;

uint8_t gyroInit();
void gyroReset();
uint8_t gyroGetStatus();
uint8_t gyroGetFifoSrc();
void gyroGetXYZ (int16_t* xyz);

void gyroITConfig (GYRO_InterruptConfigTypeDef* pIntConfigStruct);
void gyroEnableIT (uint8_t IntPin);
void gyroDisableIT (uint8_t IntPin);

//{{{
#ifdef __cplusplus
}
#endif
//}}}
