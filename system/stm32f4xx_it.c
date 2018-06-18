#include "cmsis_os.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

void NMI_Handler() { }

void MemManage_Handler() { while (1) { } }
void BusFault_Handler() { while (1) { } }
void UsageFault_Handler() { while (1) { } }

//void SVC_Handler() { }
void DebugMon_Handler() { }
//void PendSV_Handler() { }

void SysTick_Handler() { 
  osSystickHandler(); 
  HAL_IncTick(); 
  }
