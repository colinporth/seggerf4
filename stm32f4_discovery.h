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

//{{{  audio defines
#define AUDIO_RESET_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOD_CLK_ENABLE()
#define AUDIO_RESET_PIN                       GPIO_PIN_4
#define AUDIO_RESET_GPIO                      GPIOD

#define AUDIO_I2C_ADDRESS                     0x94
//}}}

void BSP_LED_Init (Led_TypeDef Led);
void BSP_LED_On (Led_TypeDef Led);
void BSP_LED_Off (Led_TypeDef Led);
void BSP_LED_Toggle (Led_TypeDef Led);

void BSP_PB_Init (Button_TypeDef Button, ButtonMode_TypeDef Mode);
uint32_t BSP_PB_GetState (Button_TypeDef Button);

void ACCELERO_IO_Init();
void ACCELERO_IO_ITConfig();
void ACCELERO_IO_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void ACCELERO_IO_Read (uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);

//{{{
#ifdef __cplusplus
}
#endif
//}}}
