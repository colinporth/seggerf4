// Copyright (c) 2014 Rowley Associates Limited.                             *

.macro ISR_HANDLER name=
  .section .vectors, "ax"
  .word \name
  .section .init, "ax"
  .thumb_func
  .weak \name
\name:
1: b 1b // endless loop
.endm

.macro ISR_RESERVED
  .section .vectors, "ax"
  .word 0
.endm

  .syntax unified
  .global reset_handler

  .section .vectors, "ax"
  .code 16
  .global _vectors

.macro DEFAULT_ISR_HANDLER name=
  .thumb_func
  .weak \name
\name:
1: b 1b // endless loop

_vectors:
  .word __stack_end__
  .word reset_handler

ISR_HANDLER NMI_Handler
ISR_HANDLER HardFault_Handler
ISR_RESERVED // Populate if using MemManage (MPU)
ISR_RESERVED // Populate if using Bus fault
ISR_RESERVED // Populate if using Usage fault
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_HANDLER SVC_Handler
ISR_RESERVED // Populate if using a debug monitor
ISR_RESERVED
ISR_HANDLER PendSV_Handler
ISR_HANDLER SysTick_Handler

ISR_HANDLER  WWDG_IRQHandler
ISR_HANDLER  PVD_IRQHandler
ISR_HANDLER  TAMP_STAMP_IRQHandler
ISR_HANDLER  RTC_WKUP_IRQHandler
ISR_HANDLER  FLASH_IRQHandler
ISR_HANDLER  RCC_IRQHandler
ISR_HANDLER  EXTI0_IRQHandler
ISR_HANDLER  EXTI1_IRQHandler
ISR_HANDLER  EXTI2_IRQHandler
ISR_HANDLER  EXTI3_IRQHandler
ISR_HANDLER  EXTI4_IRQHandler
ISR_HANDLER  DMA1_Stream0_IRQHandler
ISR_HANDLER  DMA1_Stream1_IRQHandler
ISR_HANDLER  DMA1_Stream2_IRQHandler
ISR_HANDLER  DMA1_Stream3_IRQHandler
ISR_HANDLER  DMA1_Stream4_IRQHandler
ISR_HANDLER  DMA1_Stream5_IRQHandler
ISR_HANDLER  DMA1_Stream6_IRQHandler
ISR_HANDLER  ADC_IRQHandler
ISR_HANDLER  CAN1_TX_IRQHandler
ISR_HANDLER  CAN1_RX0_IRQHandler
ISR_HANDLER  CAN1_RX1_IRQHandler
ISR_HANDLER  CAN1_SCE_IRQHandler
ISR_HANDLER  EXTI9_5_IRQHandler
ISR_HANDLER  TIM1_BRK_TIM9_IRQHandler
ISR_HANDLER  TIM1_UP_TIM10_IRQHandler
ISR_HANDLER  TIM1_TRG_COM_TIM11_IRQHandler
ISR_HANDLER  TIM1_CC_IRQHandler
ISR_HANDLER  TIM2_IRQHandler
ISR_HANDLER  TIM3_IRQHandler
ISR_HANDLER  TIM4_IRQHandler
ISR_HANDLER  I2C1_EV_IRQHandler
ISR_HANDLER  I2C1_ER_IRQHandler
ISR_HANDLER  I2C2_EV_IRQHandler
ISR_HANDLER  I2C2_ER_IRQHandler
ISR_HANDLER  SPI1_IRQHandler
ISR_HANDLER  SPI2_IRQHandler
ISR_HANDLER  USART1_IRQHandler
ISR_HANDLER  USART2_IRQHandler
ISR_HANDLER  USART3_IRQHandler
ISR_HANDLER  EXTI15_10_IRQHandler
ISR_HANDLER  RTC_Alarm_IRQHandler
ISR_HANDLER  OTG_FS_WKUP_IRQHandler
ISR_HANDLER  TIM8_BRK_TIM12_IRQHandler
ISR_HANDLER  TIM8_UP_TIM13_IRQHandler
ISR_HANDLER  TIM8_TRG_COM_TIM14_IRQHandler
ISR_HANDLER  TIM8_CC_IRQHandler
ISR_HANDLER  DMA1_Stream7_IRQHandler
ISR_HANDLER  FMC_IRQHandler
ISR_HANDLER  SDIO_IRQHandler
ISR_HANDLER  TIM5_IRQHandler
ISR_HANDLER  SPI3_IRQHandler
ISR_HANDLER  UART4_IRQHandler
ISR_HANDLER  UART5_IRQHandler
ISR_HANDLER  TIM6_DAC_IRQHandler
ISR_HANDLER  TIM7_IRQHandler
ISR_HANDLER  DMA2_Stream0_IRQHandler
ISR_HANDLER  DMA2_Stream1_IRQHandler
ISR_HANDLER  DMA2_Stream2_IRQHandler
ISR_HANDLER  DMA2_Stream3_IRQHandler
ISR_HANDLER  DMA2_Stream4_IRQHandler
ISR_HANDLER  ETH_IRQHandler
ISR_HANDLER  ETH_WKUP_IRQHandler
ISR_HANDLER  CAN2_TX_IRQHandler
ISR_HANDLER  CAN2_RX0_IRQHandler
ISR_HANDLER  CAN2_RX1_IRQHandler
ISR_HANDLER  CAN2_SCE_IRQHandler
ISR_HANDLER  OTG_FS_IRQHandler
ISR_HANDLER  DMA2_Stream5_IRQHandler
ISR_HANDLER  DMA2_Stream6_IRQHandler
ISR_HANDLER  DMA2_Stream7_IRQHandler
ISR_HANDLER  USART6_IRQHandler
ISR_HANDLER  I2C3_EV_IRQHandler
ISR_HANDLER  I2C3_ER_IRQHandler
ISR_HANDLER  OTG_HS_EP1_OUT_IRQHandler
ISR_HANDLER  OTG_HS_EP1_IN_IRQHandler
ISR_HANDLER  OTG_HS_WKUP_IRQHandler
ISR_HANDLER  OTG_HS_IRQHandler
ISR_HANDLER  DCMI_IRQHandler
ISR_RESERVED
ISR_HANDLER  HASH_RNG_IRQHandler
ISR_HANDLER  FPU_IRQHandler
  .section .vectors, "ax"
_vectors_end:

  .section .init, "ax"
  .thumb_func
reset_handler:
  ldr r0, =__SRAM_segment_end__
  mov sp, r0
  bl SystemInit
  b _start
