#pragma once
//{{{
#ifdef __cplusplus
 extern "C" {
#endif
//}}}

#include "stm32f4xx_hal.h"

#define LEDn     4
typedef enum { LED4 = 0, LED3 = 1, LED5 = 2, LED6 = 3 } Led_TypeDef;

#define BUTTONn  1
typedef enum { BUTTON_KEY = 0, } Button_TypeDef;
typedef enum { BUTTON_MODE_GPIO = 0, BUTTON_MODE_EXTI = 1 } ButtonMode_TypeDef;

void BSP_LED_Init (Led_TypeDef Led);
void BSP_LED_On (Led_TypeDef Led);
void BSP_LED_Off (Led_TypeDef Led);
void BSP_LED_Toggle (Led_TypeDef Led);

void BSP_PB_Init (Button_TypeDef Button, ButtonMode_TypeDef Mode);
uint32_t BSP_PB_GetState (Button_TypeDef Button);
//{{{
#ifdef __cplusplus
}
#endif
//}}}
