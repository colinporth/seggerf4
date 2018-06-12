#pragma once
#include "stm32f4xx_hal.h"
//{{{
#ifdef __cplusplus
 extern "C" {
#endif
//}}}

typedef enum { LED3 = 0, LED4 = 1 } Led_TypeDef;
typedef enum { BUTTON_KEY = 0, } Button_TypeDef;
typedef enum { BUTTON_MODE_GPIO = 0, BUTTON_MODE_EXTI = 1 } ButtonMode_TypeDef;

#define USE_STM32F429I_DISCO

#define LEDn     2
#define BUTTONn  1

void BSP_LED_Init (Led_TypeDef Led);
void BSP_LED_On (Led_TypeDef Led);
void BSP_LED_Off (Led_TypeDef Led);
void BSP_LED_Toggle (Led_TypeDef Led);

void BSP_PB_Init (Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
uint32_t BSP_PB_GetState (Button_TypeDef Button);

//{{{
#ifdef __cplusplus
}
#endif
//}}}
