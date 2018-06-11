//{{{
/*  [..] The real-time clock (RTC), the RTC backup registers, and the backup
       SRAM (BKP SRAM) can be powered from the VBAT voltage when the main
       VDD supply is powered off.
       To retain the content of the RTC backup registers, backup SRAM, and supply
       the RTC when VDD is turned off, VBAT pin can be connected to an optional
       standby voltage supplied by a battery or by another source.

  [..] To allow the RTC operating even when the main digital supply (VDD) is turned
       off, the VBAT pin powers the following blocks:
    (#) The RTC
    (#) The LSE oscillator
    (#) The backup SRAM when the low power backup regulator is enabled
    (#) PC13 to PC15 I/Os, plus PI8 I/O (when available)

  [..] When the backup domain is supplied by VDD (analog switch connected to VDD),
       the following pins are available:
    (#) PC14 and PC15 can be used as either GPIO or LSE pins
    (#) PC13 can be used as a GPIO or as the RTC_AF1 pin
    (#) PI8 can be used as a GPIO or as the RTC_AF2 pin

  [..] When the backup domain is supplied by VBAT (analog switch connected to VBAT
       because VDD is not present), the following pins are available:
    (#) PC14 and PC15 can be used as LSE pins only
    (#) PC13 can be used as the RTC_AF1 pin
    (#) PI8 can be used as the RTC_AF2 pin

                   ##### Backup Domain Reset #####
  ==================================================================
  [..] The backup domain reset sets all RTC registers and the RCC_BDCR register
       to their reset values. The BKPSRAM is not affected by this reset. The only
       way to reset the BKPSRAM is through the Flash interface by requesting
       a protection level change from 1 to 0.
  [..] A backup domain reset is generated when one of the following events occurs:
    (#) Software reset, triggered by setting the BDRST bit in the
        RCC Backup domain control register (RCC_BDCR).
    (#) VDD or VBAT power on, if both supplies have previously been powered off.

                   ##### Backup Domain Access #####
  ==================================================================
  [..] After reset, the backup domain (RTC registers, RTC backup data
       registers and backup SRAM) is protected against possible unwanted write
       accesses.
  [..] To enable access to the RTC Domain and RTC registers, proceed as follows:
    (+) Enable the Power Controller (PWR) APB1 interface clock using the
        __HAL_RCC_PWR_CLK_ENABLE() function.
    (+) Enable access to RTC domain using the HAL_PWR_EnableBkUpAccess() function.
    (+) Select the RTC clock source using the __HAL_RCC_RTC_CONFIG() function.
    (+) Enable RTC Clock using the __HAL_RCC_RTC_ENABLE() function.


                  ##### How to use this driver #####
  ==================================================================
  [..]
    (+) Enable the RTC domain access (see description in the section above).
    (+) Configure the RTC Prescaler (Asynchronous and Synchronous) and RTC hour
        format using the HAL_RTC_Init() function.

  *** Time and Date configuration ***
  ===================================
  [..]
    (+) To configure the RTC Calendar (Time and Date) use the HAL_RTC_SetTime()
        and HAL_RTC_SetDate() functions.
    (+) To read the RTC Calendar, use the HAL_RTC_GetTime() and HAL_RTC_GetDate() functions.

  *** Alarm configuration ***
  ===========================
  [..]
    (+) To configure the RTC Alarm use the HAL_RTC_SetAlarm() function.
        You can also configure the RTC Alarm with interrupt mode using the HAL_RTC_SetAlarm_IT() function.
    (+) To read the RTC Alarm, use the HAL_RTC_GetAlarm() function.

                  ##### RTC and low power modes #####
  ==================================================================
  [..] The MCU can be woken up from a low power mode by an RTC alternate
       function.
  [..] The RTC alternate functions are the RTC alarms (Alarm A and Alarm B),
       RTC wake-up, RTC tamper event detection and RTC time stamp event detection.
       These RTC alternate functions can wake up the system from the Stop and
       Standby low power modes.
  [..] The system can also wake up from low power modes without depending
       on an external interrupt (Auto-wake-up mode), by using the RTC alarm
       or the RTC wake-up events.
  [..] The RTC provides a programmable time base for waking up from the
       Stop or Standby mode at regular intervals.
       Wake-up from STOP and STANDBY modes is possible only when the RTC clock source
       is LSE or LSI.
  */
//}}}
#include "stm32f4xx_hal.h"
//{{{
/*
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
   [..] This section provides functions allowing to initialize and configure the
         RTC Prescaler (Synchronous and Asynchronous), RTC Hour format, disable
         RTC registers Write protection, enter and exit the RTC initialization mode,
         RTC registers synchronization check and reference clock detection enable.
         (#) The RTC Prescaler is programmed to generate the RTC 1Hz time base.
             It is split into 2 programmable prescalers to minimize power consumption.
             (++) A 7-bit asynchronous prescaler and a 13-bit synchronous prescaler.
             (++) When both prescalers are used, it is recommended to configure the
                 asynchronous prescaler to a high value to minimize power consumption.
         (#) All RTC registers are Write protected. Writing to the RTC registers
             is enabled by writing a key into the Write Protection register, RTC_WPR.
         (#) To configure the RTC Calendar, user application should enter
             initialization mode. In this mode, the calendar counter is stopped
             and its value can be updated. When the initialization sequence is
             complete, the calendar restarts counting after 4 RTCCLK cycles.
         (#) To read the calendar through the shadow registers after Calendar
             initialization, calendar update or after wake-up from low power modes
             the software must first clear the RSF flag. The software must then
             wait until it is set again before reading the calendar, which means
             that the calendar registers have been correctly copied into the
             RTC_TR and RTC_DR shadow registers.The HAL_RTC_WaitForSynchro() function
             implements the above software sequence (RSF clear and RSF check).
  */
//}}}

//{{{
static uint8_t byteToBcd2 (uint8_t Value)
{
  uint32_t bcdhigh = 0U;

  while(Value >= 10U)
  {
    bcdhigh++;
    Value -= 10U;
  }

  return  ((uint8_t)(bcdhigh << 4U) | Value);
}

//}}}
//{{{
static uint8_t bcd2ToByte (uint8_t Value)
{
  uint32_t tmp = 0U;
  tmp = ((uint8_t)(Value & (uint8_t)0xF0) >> (uint8_t)0x4) * 10;
  return (tmp + (Value & (uint8_t)0x0F));
}
//}}}

__weak void HAL_RTC_AlarmAEventCallback (RTC_HandleTypeDef* hrtc) { UNUSED(hrtc); }

//{{{
HAL_StatusTypeDef HAL_RTC_Init (RTC_HandleTypeDef* hrtc) {

  // Disable the write protection for RTC registers
  __HAL_RTC_WRITEPROTECTION_DISABLE (hrtc);

  // Set Initialization mode
  if (RTC_EnterInitMode (hrtc) != HAL_OK) {
    __HAL_RTC_WRITEPROTECTION_ENABLE (hrtc);
    return HAL_ERROR;
    }

  else {
    RTC->CR = hrtc->Init.HourFormat;
    RTC->PRER = (uint32_t)(hrtc->Init.SynchPrediv);
    RTC->PRER |= (uint32_t)(hrtc->Init.AsynchPrediv << 16U);

    // Exit Initialization mode
    RTC->ISR &= (uint32_t)~RTC_ISR_INIT;

    // If CR_BYPSHAD bit = 0, wait for synchro else this check is not needed
    if ((RTC->CR & RTC_CR_BYPSHAD) == RESET) {
      if (HAL_RTC_WaitForSynchro(hrtc) != HAL_OK) {
        // Enable the write protection for RTC registers
        __HAL_RTC_WRITEPROTECTION_ENABLE (hrtc);
        return HAL_ERROR;
        }
      }

    RTC->TAFCR &= (uint32_t)~RTC_TAFCR_ALARMOUTTYPE;
    RTC->TAFCR |= (uint32_t)(hrtc->Init.OutPutType);

    __HAL_RTC_WRITEPROTECTION_ENABLE (hrtc);
    return HAL_OK;
    }
  }
//}}}
//{{{
HAL_StatusTypeDef RTC_EnterInitMode (RTC_HandleTypeDef* hrtc) {

  // Check if the Initialization mode is set
  if ((RTC->ISR & RTC_ISR_INITF) == (uint32_t)RESET) {
    // Set the Initialization mode
    RTC->ISR = (uint32_t)RTC_INIT_MASK;

    /* Get tick */
    uint32_t tickstart = HAL_GetTick();

    // Wait till RTC is in INIT state and if Time out is reached exit
    while ((RTC->ISR & RTC_ISR_INITF) == (uint32_t)RESET)
      if ((HAL_GetTick() - tickstart ) > RTC_TIMEOUT_VALUE)
        return HAL_TIMEOUT;
    }

  return HAL_OK;
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_RTC_WaitForSynchro (RTC_HandleTypeDef* hrtc) {

  // Clear RSF flag
  RTC->ISR &= (uint32_t)RTC_RSF_MASK;

  uint32_t tickstart = HAL_GetTick();

  // Wait the registers to be synchronised
  while ((RTC->ISR & RTC_ISR_RSF) == (uint32_t)RESET)
    if ((HAL_GetTick() - tickstart ) > RTC_TIMEOUT_VALUE)
      return HAL_TIMEOUT;

  return HAL_OK;
  }
//}}}

//{{{
HAL_StatusTypeDef HAL_RTC_GetTime (RTC_HandleTypeDef* hrtc, RTC_TimeTypeDef* sTime, uint32_t Format) {

  sTime->SubSeconds = RTC->SSR;
  sTime->SecondFraction = RTC->PRER & RTC_PRER_PREDIV_S;

  uint32_t tr = RTC->TR;
  sTime->TimeFormat = (tr & (RTC_TR_PM)) >> 16U;
  sTime->Hours = bcd2ToByte ((tr & (RTC_TR_HT | RTC_TR_HU)) >> 16U);
  sTime->Minutes = bcd2ToByte ((tr & (RTC_TR_MNT | RTC_TR_MNU)) >> 8U);
  sTime->Seconds = bcd2ToByte (tr & (RTC_TR_ST | RTC_TR_SU));

  return HAL_OK;
}
//}}}
//{{{
HAL_StatusTypeDef HAL_RTC_GetDate (RTC_HandleTypeDef* hrtc, RTC_DateTypeDef* sDate, uint32_t Format) {

  /* Fill the structure fields with the read parameters */
  uint32_t dr = RTC->DR;
  sDate->Year = bcd2ToByte ((dr & (RTC_DR_YT | RTC_DR_YU)) >> 16U);
  sDate->WeekDay = (dr & (RTC_DR_WDU)) >> 13U;
  sDate->Month = bcd2ToByte ((dr & (RTC_DR_MT | RTC_DR_MU)) >> 8U);
  sDate->Date = bcd2ToByte (dr & (RTC_DR_DT | RTC_DR_DU));

  return HAL_OK;
  }
//}}}

//{{{
HAL_StatusTypeDef HAL_RTC_SetTime (RTC_HandleTypeDef* hrtc, RTC_TimeTypeDef* sTime, uint32_t Format) {

  if ((RTC->CR & RTC_CR_FMT) == (uint32_t)RESET)
    sTime->TimeFormat = 0x00U;

  uint32_t tmpreg = (uint32_t)(((uint32_t)byteToBcd2(sTime->Hours) << 16U) | \
                   ((uint32_t)byteToBcd2(sTime->Minutes) << 8U) | \
                   ((uint32_t)byteToBcd2(sTime->Seconds)) | \
                    (((uint32_t)sTime->TimeFormat) << 16U));
  __HAL_RTC_WRITEPROTECTION_DISABLE (hrtc);

  /* Set Initialization mode */
  if (RTC_EnterInitMode (hrtc) != HAL_OK) {
    /* Enable the write protection for RTC registers */
    __HAL_RTC_WRITEPROTECTION_ENABLE (hrtc);
    return HAL_ERROR;
    }

  else {
    /* Set the RTC_TR register */
    RTC->TR = (uint32_t)(tmpreg & RTC_TR_RESERVED_MASK);

    /* Clear the bits to be configured */
    RTC->CR &= (uint32_t)~RTC_CR_BCK;

    /* Configure the RTC_CR register */
    RTC->CR |= (uint32_t)(sTime->DayLightSaving | sTime->StoreOperation);

    /* Exit Initialization mode */
    RTC->ISR &= (uint32_t)~RTC_ISR_INIT;

    /* If CR_BYPSHAD bit = 0, wait for synchro else this check is not needed */
    if((RTC->CR & RTC_CR_BYPSHAD) == RESET) {
      if(HAL_RTC_WaitForSynchro(hrtc) != HAL_OK) {
        /* Enable the write protection for RTC registers */
        __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
        return HAL_ERROR;
        }
      }

    /* Enable the write protection for RTC registers */
    __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
    return HAL_OK;
    }
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_RTC_SetDate (RTC_HandleTypeDef* hrtc, RTC_DateTypeDef* sDate, uint32_t Format) {


  if ((sDate->Month & 0x10U) == 0x10U)
    sDate->Month = (uint8_t)((sDate->Month & (uint8_t)~(0x10U)) + (uint8_t)0x0AU);

  uint32_t datetmpreg = ((uint32_t)byteToBcd2(sDate->Year) << 16U) | ((uint32_t)byteToBcd2(sDate->Month) << 8U) |
                        ((uint32_t)byteToBcd2(sDate->Date))        | ((uint32_t)sDate->WeekDay << 13U);

  /* Disable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* Set Initialization mode */
  if(RTC_EnterInitMode(hrtc) != HAL_OK) {
    /* Enable the write protection for RTC registers */
    __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
    return HAL_ERROR;
    }

  else {
    /* Set the RTC_DR register */
    RTC->DR = (uint32_t)(datetmpreg & RTC_DR_RESERVED_MASK);

    /* Exit Initialization mode */
    RTC->ISR &= (uint32_t)~RTC_ISR_INIT;

    /* If  CR_BYPSHAD bit = 0, wait for synchro else this check is not needed */
    if((RTC->CR & RTC_CR_BYPSHAD) == RESET) {
      if(HAL_RTC_WaitForSynchro(hrtc) != HAL_OK) {
        /* Enable the write protection for RTC registers */
        __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
        return HAL_ERROR;
        }
      }

    /* Enable the write protection for RTC registers */
    __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
    return HAL_OK;
    }
  }
//}}}

//{{{
HAL_StatusTypeDef HAL_RTC_SetAlarm (RTC_HandleTypeDef* hrtc, RTC_AlarmTypeDef* sAlarm, uint32_t Format) {

  uint32_t tickstart = 0U;
  uint32_t tmpreg = 0U, subsecondtmpreg = 0U;

  if(Format == RTC_FORMAT_BIN) {
    if((RTC->CR & RTC_CR_FMT) != (uint32_t)RESET) {
      }
    else
      sAlarm->AlarmTime.TimeFormat = 0x00U;

    tmpreg = (((uint32_t)byteToBcd2(sAlarm->AlarmTime.Hours) << 16U) | \
              ((uint32_t)byteToBcd2(sAlarm->AlarmTime.Minutes) << 8U) | \
              ((uint32_t)byteToBcd2(sAlarm->AlarmTime.Seconds)) | \
              ((uint32_t)(sAlarm->AlarmTime.TimeFormat) << 16U) | \
              ((uint32_t)byteToBcd2(sAlarm->AlarmDateWeekDay) << 24U) | \
              ((uint32_t)sAlarm->AlarmDateWeekDaySel) | \
              ((uint32_t)sAlarm->AlarmMask));
    }
  else {
    if((RTC->CR & RTC_CR_FMT) != (uint32_t)RESET)
      tmpreg = bcd2ToByte(sAlarm->AlarmTime.Hours);
    else
      sAlarm->AlarmTime.TimeFormat = 0x00U;

    if(sAlarm->AlarmDateWeekDaySel == RTC_ALARMDATEWEEKDAYSEL_DATE)
      tmpreg = bcd2ToByte(sAlarm->AlarmDateWeekDay);
    else
      tmpreg = bcd2ToByte(sAlarm->AlarmDateWeekDay);

    tmpreg = (((uint32_t)(sAlarm->AlarmTime.Hours) << 16U) | ((uint32_t)(sAlarm->AlarmTime.Minutes) << 8U) | \
              ((uint32_t) sAlarm->AlarmTime.Seconds) | ((uint32_t)(sAlarm->AlarmTime.TimeFormat) << 16U) | \
              ((uint32_t)(sAlarm->AlarmDateWeekDay) << 24U) | ((uint32_t)sAlarm->AlarmDateWeekDaySel) | \
              ((uint32_t)sAlarm->AlarmMask));
    }

  /* Configure the Alarm A or Alarm B Sub Second registers */
  subsecondtmpreg = (uint32_t)((uint32_t)(sAlarm->AlarmTime.SubSeconds) | (uint32_t)(sAlarm->AlarmSubSecondMask));

  /* Disable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* Configure the Alarm register */
  if(sAlarm->Alarm == RTC_ALARM_A) {
    /* Disable the Alarm A interrupt */
    __HAL_RTC_ALARMA_DISABLE(hrtc);

    /* In case of interrupt mode is used, the interrupt source must disabled */
    __HAL_RTC_ALARM_DISABLE_IT(hrtc, RTC_IT_ALRA);

    /* Get tick */
    tickstart = HAL_GetTick();

    /* Wait till RTC ALRAWF flag is set and if Time out is reached exit */
    while(__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRAWF) == RESET) {
      if((HAL_GetTick() - tickstart ) > RTC_TIMEOUT_VALUE) {
        /* Enable the write protection for RTC registers */
        __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
        return HAL_TIMEOUT;
      }
    }

    RTC->ALRMAR = (uint32_t)tmpreg;
    /* Configure the Alarm A Sub Second register */
    RTC->ALRMASSR = subsecondtmpreg;
    /* Configure the Alarm state: Enable Alarm */
    __HAL_RTC_ALARMA_ENABLE(hrtc);
    }
  else {
    /* Disable the Alarm B interrupt */
    __HAL_RTC_ALARMB_DISABLE(hrtc);

    /* In case of interrupt mode is used, the interrupt source must disabled */
    __HAL_RTC_ALARM_DISABLE_IT(hrtc, RTC_IT_ALRB);

    /* Get tick */
    tickstart = HAL_GetTick();

    /* Wait till RTC ALRBWF flag is set and if Time out is reached exit */
    while(__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRBWF) == RESET) {
      if((HAL_GetTick() - tickstart ) > RTC_TIMEOUT_VALUE) {
        /* Enable the write protection for RTC registers */
        __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
        return HAL_TIMEOUT;
        }
      }

    RTC->ALRMBR = (uint32_t)tmpreg;
    /* Configure the Alarm B Sub Second register */
    RTC->ALRMBSSR = subsecondtmpreg;
    /* Configure the Alarm state: Enable Alarm */
    __HAL_RTC_ALARMB_ENABLE(hrtc);
    }

  /* Enable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  return HAL_OK;
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_RTC_SetAlarm_IT (RTC_HandleTypeDef* hrtc, RTC_AlarmTypeDef* sAlarm, uint32_t Format) {

  uint32_t tmpreg = 0U, subsecondtmpreg = 0U;
  __IO uint32_t count = RTC_TIMEOUT_VALUE  * (SystemCoreClock / 32U / 1000U) ;

  if (Format == RTC_FORMAT_BIN) {
    if ((RTC->CR & RTC_CR_FMT) != (uint32_t)RESET) {
      }
    else
      sAlarm->AlarmTime.TimeFormat = 0x00U;

    tmpreg = (((uint32_t)byteToBcd2(sAlarm->AlarmTime.Hours) << 16U) | \
              ((uint32_t)byteToBcd2(sAlarm->AlarmTime.Minutes) << 8U) | \
              ((uint32_t)byteToBcd2(sAlarm->AlarmTime.Seconds)) | \
              ((uint32_t)(sAlarm->AlarmTime.TimeFormat) << 16U) | \
              ((uint32_t)byteToBcd2(sAlarm->AlarmDateWeekDay) << 24U) | \
              ((uint32_t)sAlarm->AlarmDateWeekDaySel) | \
              ((uint32_t)sAlarm->AlarmMask));
    }
  else {
    if((RTC->CR & RTC_CR_FMT) != (uint32_t)RESET)
      tmpreg = bcd2ToByte(sAlarm->AlarmTime.Hours);
    else
      sAlarm->AlarmTime.TimeFormat = 0x00U;

    if(sAlarm->AlarmDateWeekDaySel == RTC_ALARMDATEWEEKDAYSEL_DATE)
      tmpreg = bcd2ToByte(sAlarm->AlarmDateWeekDay);
    else
      tmpreg = bcd2ToByte(sAlarm->AlarmDateWeekDay);
    tmpreg = (((uint32_t)(sAlarm->AlarmTime.Hours) << 16U) | \
              ((uint32_t)(sAlarm->AlarmTime.Minutes) << 8U) | \
              ((uint32_t) sAlarm->AlarmTime.Seconds) | \
              ((uint32_t)(sAlarm->AlarmTime.TimeFormat) << 16U) | \
              ((uint32_t)(sAlarm->AlarmDateWeekDay) << 24U) | \
              ((uint32_t)sAlarm->AlarmDateWeekDaySel) | \
              ((uint32_t)sAlarm->AlarmMask));
    }
  /* Configure the Alarm A or Alarm B Sub Second registers */
  subsecondtmpreg = (uint32_t)((uint32_t)(sAlarm->AlarmTime.SubSeconds) | (uint32_t)(sAlarm->AlarmSubSecondMask));

  /* Disable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* Configure the Alarm register */
  if(sAlarm->Alarm == RTC_ALARM_A) {
    /* Disable the Alarm A interrupt */
    __HAL_RTC_ALARMA_DISABLE(hrtc);

    /* Clear flag alarm A */
    __HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_FLAG_ALRAF);

    /* Wait till RTC ALRAWF flag is set and if Time out is reached exit */
    do {
      if (count-- == 0U) {
        /* Enable the write protection for RTC registers */
        __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
        return HAL_TIMEOUT;
        }
      }
    while (__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRAWF) == RESET);

    RTC->ALRMAR = (uint32_t)tmpreg;
    /* Configure the Alarm A Sub Second register */
    RTC->ALRMASSR = subsecondtmpreg;
    /* Configure the Alarm state: Enable Alarm */
    __HAL_RTC_ALARMA_ENABLE(hrtc);
    /* Configure the Alarm interrupt */
    __HAL_RTC_ALARM_ENABLE_IT(hrtc,RTC_IT_ALRA);
    }
  else {
    //{{{  Disable the Alarm B interrupt */
    __HAL_RTC_ALARMB_DISABLE(hrtc);

    /* Clear flag alarm B */
    __HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_FLAG_ALRBF);

    /* Wait till RTC ALRBWF flag is set and if Time out is reached exit */
    do {
      if (count-- == 0U) {
        /* Enable the write protection for RTC registers */
        __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
        return HAL_TIMEOUT;
        }
      }
    while (__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRBWF) == RESET);

    RTC->ALRMBR = (uint32_t)tmpreg;
    /* Configure the Alarm B Sub Second register */
    RTC->ALRMBSSR = subsecondtmpreg;
    /* Configure the Alarm state: Enable Alarm */
    __HAL_RTC_ALARMB_ENABLE(hrtc);
    /* Configure the Alarm interrupt */
    __HAL_RTC_ALARM_ENABLE_IT(hrtc, RTC_IT_ALRB);
    }
    //}}}

  /* RTC Alarm Interrupt Configuration: EXTI configuration */
  __HAL_RTC_ALARM_EXTI_ENABLE_IT();

  EXTI->RTSR |= RTC_EXTI_LINE_ALARM_EVENT;

  /* Enable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  return HAL_OK;
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_RTC_DeactivateAlarm (RTC_HandleTypeDef* hrtc, uint32_t Alarm)
{
  uint32_t tickstart = 0U;

  /* Disable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  if(Alarm == RTC_ALARM_A) {
    /* AlarmA */
    __HAL_RTC_ALARMA_DISABLE(hrtc);

    /* In case of interrupt mode is used, the interrupt source must disabled */
    __HAL_RTC_ALARM_DISABLE_IT(hrtc, RTC_IT_ALRA);

    /* Get tick */
    tickstart = HAL_GetTick();

    /* Wait till RTC ALRxWF flag is set and if Time out is reached exit */
    while(__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRAWF) == RESET) {
      if((HAL_GetTick() - tickstart ) > RTC_TIMEOUT_VALUE) {
        /* Enable the write protection for RTC registers */
        __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
        return HAL_TIMEOUT;
        }
      }
    }
  else {
    /* AlarmB */
    __HAL_RTC_ALARMB_DISABLE(hrtc);

    /* In case of interrupt mode is used, the interrupt source must disabled */
    __HAL_RTC_ALARM_DISABLE_IT(hrtc,RTC_IT_ALRB);

    /* Get tick */
    tickstart = HAL_GetTick();

    /* Wait till RTC ALRxWF flag is set and if Time out is reached exit */
    while(__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRBWF) == RESET) {
      if((HAL_GetTick() - tickstart ) > RTC_TIMEOUT_VALUE) {
        /* Enable the write protection for RTC registers */
        __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
        return HAL_TIMEOUT;
        }
      }
    }

  /* Enable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  return HAL_OK;
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_RTC_GetAlarm (RTC_HandleTypeDef* hrtc, RTC_AlarmTypeDef* sAlarm, uint32_t Alarm, uint32_t Format)
{
  uint32_t tmpreg = 0U, subsecondtmpreg = 0U;

  if(Alarm == RTC_ALARM_A) {
    /* AlarmA */
    sAlarm->Alarm = RTC_ALARM_A;
    tmpreg = (uint32_t)(RTC->ALRMAR);
    subsecondtmpreg = (uint32_t)((RTC->ALRMASSR ) & RTC_ALRMASSR_SS);
    }
  else {
    sAlarm->Alarm = RTC_ALARM_B;
    tmpreg = (uint32_t)(RTC->ALRMBR);
    subsecondtmpreg = (uint32_t)((RTC->ALRMBSSR) & RTC_ALRMBSSR_SS);
    }

  /* Fill the structure with the read parameters */
  sAlarm->AlarmTime.Hours = (uint32_t)((tmpreg & (RTC_ALRMAR_HT | RTC_ALRMAR_HU)) >> 16U);
  sAlarm->AlarmTime.Minutes = (uint32_t)((tmpreg & (RTC_ALRMAR_MNT | RTC_ALRMAR_MNU)) >> 8U);
  sAlarm->AlarmTime.Seconds = (uint32_t)(tmpreg & (RTC_ALRMAR_ST | RTC_ALRMAR_SU));
  sAlarm->AlarmTime.TimeFormat = (uint32_t)((tmpreg & RTC_ALRMAR_PM) >> 16U);
  sAlarm->AlarmTime.SubSeconds = (uint32_t) subsecondtmpreg;
  sAlarm->AlarmDateWeekDay = (uint32_t)((tmpreg & (RTC_ALRMAR_DT | RTC_ALRMAR_DU)) >> 24U);
  sAlarm->AlarmDateWeekDaySel = (uint32_t)(tmpreg & RTC_ALRMAR_WDSEL);
  sAlarm->AlarmMask = (uint32_t)(tmpreg & RTC_ALARMMASK_ALL);

  if (Format == RTC_FORMAT_BIN) {
    sAlarm->AlarmTime.Hours = bcd2ToByte(sAlarm->AlarmTime.Hours);
    sAlarm->AlarmTime.Minutes = bcd2ToByte(sAlarm->AlarmTime.Minutes);
    sAlarm->AlarmTime.Seconds = bcd2ToByte(sAlarm->AlarmTime.Seconds);
    sAlarm->AlarmDateWeekDay = bcd2ToByte(sAlarm->AlarmDateWeekDay);
    }

  return HAL_OK;
  }
//}}}

//{{{
void HAL_RTC_AlarmIRQHandler (RTC_HandleTypeDef* hrtc)
{
  if (__HAL_RTC_ALARM_GET_IT(hrtc, RTC_IT_ALRA)) {
    /* Get the status of the Interrupt */
    if ((uint32_t)(RTC->CR & RTC_IT_ALRA) != (uint32_t)RESET) {
      /* AlarmA callback */
      HAL_RTC_AlarmAEventCallback (hrtc);

      /* Clear the Alarm interrupt pending bit */
      __HAL_RTC_ALARM_CLEAR_FLAG(hrtc,RTC_FLAG_ALRAF);
      }
    }

  if(__HAL_RTC_ALARM_GET_IT(hrtc, RTC_IT_ALRB)) {
    /* Get the status of the Interrupt */
    if ((uint32_t)(RTC->CR & RTC_IT_ALRB) != (uint32_t)RESET) { /* AlarmB callback */
      HAL_RTCEx_AlarmBEventCallback(hrtc);

      /* Clear the Alarm interrupt pending bit */
      __HAL_RTC_ALARM_CLEAR_FLAG(hrtc,RTC_FLAG_ALRBF);
      }
    }

  /* Clear the EXTI's line Flag for RTC Alarm */
  __HAL_RTC_ALARM_EXTI_CLEAR_FLAG();
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_RTC_PollForAlarmAEvent (RTC_HandleTypeDef* hrtc, uint32_t Timeout) {

  uint32_t tickstart = 0U;

  tickstart = HAL_GetTick();

  while (__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRAF) == RESET)
    if (Timeout != HAL_MAX_DELAY)
      if ((Timeout == 0U)||((HAL_GetTick() - tickstart ) > Timeout))
        return HAL_TIMEOUT;

  /* Clear the Alarm interrupt pending bit */
  __HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_FLAG_ALRAF);
  return HAL_OK;
  }
//}}}
