//{{{
/**
  [..]
  (#) 12-bit, 10-bit, 8-bit or 6-bit configurable resolution.
  (#) Interrupt generation at the end of conversion, end of injected conversion,
      and in case of analog watchdog or overrun events
  (#) Single and continuous conversion modes.
  (#) Scan mode for automatic conversion of channel 0 to channel x.
  (#) Data alignment with in-built data coherency.
  (#) Channel-wise programmable sampling time.
  (#) External trigger option with configurable polarity for both regular and
      injected conversion.
  (#) Dual/Triple mode (on devices with 2 ADCs or more).
  (#) Configurable DMA data storage in Dual/Triple ADC mode.
  (#) Configurable delay between conversions in Dual/Triple interleaved mode.
  (#) ADC conversion type (refer to the datasheets).
  (#) ADC supply requirements: 2.4 V to 3.6 V at full speed and down to 1.8 V at
      slower speed.
  (#) ADC input range: VREF(minus) = VIN = VREF(plus).
  (#) DMA request generation during regular channel conversion.

  (#)Initialize the ADC low level resources by implementing the HAL_ADC_MspInit():
       (##) Enable the ADC interface clock using __HAL_RCC_ADC_CLK_ENABLE()
       (##) ADC pins configuration
             (+++) Enable the clock for the ADC GPIOs using the following function:
                   __HAL_RCC_GPIOx_CLK_ENABLE()
             (+++) Configure these ADC pins in analog mode using HAL_GPIO_Init()
       (##) In case of using interrupts (e.g. HAL_ADC_Start_IT())
             (+++) Configure the ADC interrupt priority using HAL_NVIC_SetPriority()
             (+++) Enable the ADC IRQ handler using HAL_NVIC_EnableIRQ()
             (+++) In ADC IRQ handler, call HAL_ADC_IRQHandler()
       (##) In case of using DMA to control data transfer (e.g. HAL_ADC_Start_DMA())
             (+++) Enable the DMAx interface clock using __HAL_RCC_DMAx_CLK_ENABLE()
             (+++) Configure and enable two DMA streams stream for managing data
                 transfer from peripheral to memory (output stream)
             (+++) Associate the initialized DMA handle to the CRYP DMA handle
                 using  __HAL_LINKDMA()
             (+++) Configure the priority and enable the NVIC for the transfer complete
                 interrupt on the two DMA Streams. The output stream should have higher
                 priority than the input stream.

    *** Configuration of ADC, groups regular/injected, channels parameters ***
  (#) Configure the ADC parameters (resolution, data alignment, ...)
      and regular group parameters (conversion trigger, sequencer, ...)
      using function HAL_ADC_Init().

  (#) Configure the channels for regular group parameters (channel number,
      channel rank into sequencer, ..., into regular group)
      using function HAL_ADC_ConfigChannel().

  (#) Optionally, configure the injected group parameters (conversion trigger,
      sequencer, ..., of injected group)
      and the channels for injected group parameters (channel number,
      channel rank into sequencer, ..., into injected group)
      using function HAL_ADCEx_InjectedConfigChannel().

  (#) Optionally, configure the analog watchdog parameters (channels
      monitored, thresholds, ...) using function HAL_ADC_AnalogWDGConfig().

  (#) Optionally, for devices with several ADC instances: configure the
      multimode parameters using function HAL_ADCEx_MultiModeConfigChannel().

  (#) ADC driver can be used among three modes: polling, interruption,
      transfer by DMA.

     *** Polling mode IO operation ***
     =================================
     [..]
       (+) Start the ADC peripheral using HAL_ADC_Start()
       (+) Wait for end of conversion using HAL_ADC_PollForConversion(), at this stage
           user can specify the value of timeout according to his end application
       (+) To read the ADC converted values, use the HAL_ADC_GetValue() function.
       (+) Stop the ADC peripheral using HAL_ADC_Stop()

     *** Interrupt mode IO operation ***
     ===================================
     [..]
       (+) Start the ADC peripheral using HAL_ADC_Start_IT()
       (+) Use HAL_ADC_IRQHandler() called under ADC_IRQHandler() Interrupt subroutine
       (+) At ADC end of conversion HAL_ADC_ConvCpltCallback() function is executed and user can
           add his own code by customization of function pointer HAL_ADC_ConvCpltCallback
       (+) In case of ADC Error, HAL_ADC_ErrorCallback() function is executed and user can
           add his own code by customization of function pointer HAL_ADC_ErrorCallback
       (+) Stop the ADC peripheral using HAL_ADC_Stop_IT()

     *** DMA mode IO operation ***
     ==============================
     [..]
       (+) Start the ADC peripheral using HAL_ADC_Start_DMA(), at this stage the user specify the length
           of data to be transferred at each end of conversion
       (+) At The end of data transfer by HAL_ADC_ConvCpltCallback() function is executed and user can
           add his own code by customization of function pointer HAL_ADC_ConvCpltCallback
       (+) In case of transfer Error, HAL_ADC_ErrorCallback() function is executed and user can
           add his own code by customization of function pointer HAL_ADC_ErrorCallback
       (+) Stop the ADC peripheral using HAL_ADC_Stop_DMA()

     *** ADC HAL driver macros list ***
     =============================================
     [..]
       Below the list of most used macros in ADC HAL driver.

      (+) __HAL_ADC_ENABLE : Enable the ADC peripheral
      (+) __HAL_ADC_DISABLE : Disable the ADC peripheral
      (+) __HAL_ADC_ENABLE_IT: Enable the ADC end of conversion interrupt
      (+) __HAL_ADC_DISABLE_IT: Disable the ADC end of conversion interrupt
      (+) __HAL_ADC_GET_IT_SOURCE: Check if the specified ADC interrupt source is enabled or disabled
      (+) __HAL_ADC_CLEAR_FLAG: Clear the ADC's pending flags
      (+) __HAL_ADC_GET_FLAG: Get the selected ADC's flag status
      (+) ADC_GET_RESOLUTION: Return resolution bits in CR1 register

     [..]
       (@) You can refer to the ADC HAL driver header file for more useful macros

  (#) Disable the ADC interface
     (++) ADC clock can be hard reset and disabled at RCC top level.
     (++) Hard reset of ADC peripherals
          using macro __HAL_RCC_ADC_FORCE_RESET(), __HAL_RCC_ADC_RELEASE_RESET().
     (++) ADC clock disable using the equivalent macro/functions as configuration step.
               (+++) Example:
                   Into HAL_ADC_MspDeInit() (recommended code location) or with
                   other device clock parameters configuration:
               (+++) HAL_RCC_GetOscConfig(&RCC_OscInitStructure);
               (+++) RCC_OscInitStructure.OscillatorType = RCC_OSCILLATORTYPE_HSI;
               (+++) RCC_OscInitStructure.HSIState = RCC_HSI_OFF; (if not used for system clock)
               (+++) HAL_RCC_OscConfig(&RCC_OscInitStructure);

  (#) ADC pins configuration
     (++) Disable the clock for the ADC GPIOs using macro __HAL_RCC_GPIOx_CLK_DISABLE()

  (#) Optionally, in case of usage of ADC with interruptions:
     (++) Disable the NVIC for ADC using function HAL_NVIC_DisableIRQ(ADCx_IRQn)

  (#) Optionally, in case of usage of DMA:
        (++) Deinitialize the DMA using function HAL_DMA_DeInit().
        (++) Disable the NVIC for DMA using function HAL_NVIC_DisableIRQ(DMAx_Channelx_IRQn)

     *** Polling mode IO operation ***
     =================================
     [..]
       (+) Start the ADC peripheral using HAL_ADCEx_InjectedStart()
       (+) Wait for end of conversion using HAL_ADC_PollForConversion(), at this stage
           user can specify the value of timeout according to his end application
       (+) To read the ADC converted values, use the HAL_ADCEx_InjectedGetValue() function.
       (+) Stop the ADC peripheral using HAL_ADCEx_InjectedStop()

     *** Interrupt mode IO operation ***
     ===================================
     [..]
       (+) Start the ADC peripheral using HAL_ADCEx_InjectedStart_IT()
       (+) Use HAL_ADC_IRQHandler() called under ADC_IRQHandler() Interrupt subroutine
       (+) At ADC end of conversion HAL_ADCEx_InjectedConvCpltCallback() function is executed and user can
            add his own code by customization of function pointer HAL_ADCEx_InjectedConvCpltCallback
       (+) In case of ADC Error, HAL_ADCEx_InjectedErrorCallback() function is executed and user can
            add his own code by customization of function pointer HAL_ADCEx_InjectedErrorCallback
       (+) Stop the ADC peripheral using HAL_ADCEx_InjectedStop_IT()


     *** DMA mode IO operation ***
     ==============================
     [..]
       (+) Start the ADC peripheral using HAL_ADCEx_InjectedStart_DMA(), at this stage the user specify the length
           of data to be transferred at each end of conversion
       (+) At The end of data transfer ba HAL_ADCEx_InjectedConvCpltCallback() function is executed and user can
            add his own code by customization of function pointer HAL_ADCEx_InjectedConvCpltCallback
       (+) In case of transfer Error, HAL_ADCEx_InjectedErrorCallback() function is executed and user can
            add his own code by customization of function pointer HAL_ADCEx_InjectedErrorCallback
        (+) Stop the ADC peripheral using HAL_ADCEx_InjectedStop_DMA()

     *** Multi mode ADCs Regular channels configuration ***
     ======================================================
     [..]
       (+) Select the Multi mode ADC regular channels features (dual or triple mode)
          and configure the DMA mode using HAL_ADCEx_MultiModeConfigChannel() functions.
       (+) Start the ADC peripheral using HAL_ADCEx_MultiModeStart_DMA(), at this stage the user specify the length
           of data to be transferred at each end of conversion
       (+) Read the ADCs converted values using the HAL_ADCEx_MultiModeGetValue() function.
  */
//}}}
#include "stm32f4xx_hal.h"

__weak void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc) { UNUSED(hadc); }
__weak void HAL_ADC_ConvHalfCpltCallback (ADC_HandleTypeDef* hadc) { UNUSED(hadc); }
__weak void HAL_ADC_LevelOutOfWindowCallback (ADC_HandleTypeDef* hadc) { UNUSED(hadc); }
__weak void HAL_ADC_ErrorCallback (ADC_HandleTypeDef *hadc) { UNUSED(hadc); }
__weak void HAL_ADCEx_InjectedConvCpltCallback (ADC_HandleTypeDef* hadc) { UNUSED(hadc); }

//{{{
static void ADC_DMAConvCplt (DMA_HandleTypeDef *hdma)
{
  /* Retrieve ADC handle corresponding to current DMA handle */
  ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  /* Update state machine on conversion status if not in error state */
  if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL | HAL_ADC_STATE_ERROR_DMA))
  {
    /* Update ADC state machine */
    SET_BIT(hadc->State, HAL_ADC_STATE_REG_EOC);

    /* Determine whether any further conversion upcoming on group regular   */
    /* by external trigger, continuous mode or scan sequence on going.      */
    /* Note: On STM32F4, there is no independent flag of end of sequence.   */
    /*       The test of scan sequence on going is done either with scan    */
    /*       sequence disabled or with end of conversion flag set to        */
    /*       of end of sequence.                                            */
    if(ADC_IS_SOFTWARE_START_REGULAR(hadc)                   &&
       (hadc->Init.ContinuousConvMode == DISABLE)            &&
       (HAL_IS_BIT_CLR(hadc->Instance->SQR1, ADC_SQR1_L) ||
        HAL_IS_BIT_CLR(hadc->Instance->CR2, ADC_CR2_EOCS)  )   )
    {
      /* Disable ADC end of single conversion interrupt on group regular */
      /* Note: Overrun interrupt was enabled with EOC interrupt in          */
      /* HAL_ADC_Start_IT(), but is not disabled here because can be used   */
      /* by overrun IRQ process below.                                      */
      __HAL_ADC_DISABLE_IT(hadc, ADC_IT_EOC);

      /* Set ADC state */
      CLEAR_BIT(hadc->State, HAL_ADC_STATE_REG_BUSY);

      if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_INJ_BUSY))
      {
        SET_BIT(hadc->State, HAL_ADC_STATE_READY);
      }
    }

    /* Conversion complete callback */
    HAL_ADC_ConvCpltCallback(hadc);
  }
  else
  {
    /* Call DMA error callback */
    hadc->DMA_Handle->XferErrorCallback(hdma);
  }
}
//}}}
//{{{
static void ADC_DMAHalfConvCplt (DMA_HandleTypeDef *hdma)
{
  ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  /* Conversion complete callback */
  HAL_ADC_ConvHalfCpltCallback(hadc);
}
//}}}
//{{{
static void ADC_DMAError (DMA_HandleTypeDef *hdma)
{
  ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  hadc->State= HAL_ADC_STATE_ERROR_DMA;
  /* Set ADC error code to DMA error */
  hadc->ErrorCode |= HAL_ADC_ERROR_DMA;
  HAL_ADC_ErrorCallback(hadc);
}
//}}}
//{{{
static void ADC_MultiModeDMAConvCplt (DMA_HandleTypeDef *hdma)
{
  /* Retrieve ADC handle corresponding to current DMA handle */
  ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  /* Update state machine on conversion status if not in error state */
  if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL | HAL_ADC_STATE_ERROR_DMA))
  {
    /* Update ADC state machine */
    SET_BIT(hadc->State, HAL_ADC_STATE_REG_EOC);

    /* Determine whether any further conversion upcoming on group regular   */
    /* by external trigger, continuous mode or scan sequence on going.      */
    /* Note: On STM32F4, there is no independent flag of end of sequence.   */
    /*       The test of scan sequence on going is done either with scan    */
    /*       sequence disabled or with end of conversion flag set to        */
    /*       of end of sequence.                                            */
    if(ADC_IS_SOFTWARE_START_REGULAR(hadc)                   &&
       (hadc->Init.ContinuousConvMode == DISABLE)            &&
       (HAL_IS_BIT_CLR(hadc->Instance->SQR1, ADC_SQR1_L) ||
        HAL_IS_BIT_CLR(hadc->Instance->CR2, ADC_CR2_EOCS)  )   )
    {
      /* Disable ADC end of single conversion interrupt on group regular */
      /* Note: Overrun interrupt was enabled with EOC interrupt in          */
      /* HAL_ADC_Start_IT(), but is not disabled here because can be used   */
      /* by overrun IRQ process below.                                      */
      __HAL_ADC_DISABLE_IT(hadc, ADC_IT_EOC);

      /* Set ADC state */
      CLEAR_BIT(hadc->State, HAL_ADC_STATE_REG_BUSY);

      if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_INJ_BUSY))
      {
        SET_BIT(hadc->State, HAL_ADC_STATE_READY);
      }
    }

    /* Conversion complete callback */
    HAL_ADC_ConvCpltCallback(hadc);
  }
  else
  {
    /* Call DMA error callback */
    hadc->DMA_Handle->XferErrorCallback(hdma);
  }
}
//}}}
//{{{
static void ADC_MultiModeDMAHalfConvCplt (DMA_HandleTypeDef *hdma)
{
    ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
    /* Conversion complete callback */
    HAL_ADC_ConvHalfCpltCallback(hadc);
}
//}}}
//{{{
static void ADC_MultiModeDMAError (DMA_HandleTypeDef *hdma)
{
    ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
    hadc->State= HAL_ADC_STATE_ERROR_DMA;
    /* Set ADC error code to DMA error */
    hadc->ErrorCode |= HAL_ADC_ERROR_DMA;
    HAL_ADC_ErrorCallback(hadc);
}
//}}}

//{{{
HAL_StatusTypeDef HAL_ADC_Init (ADC_HandleTypeDef* hadc) {

  // Set ADC state
  ADC_STATE_CLR_SET (hadc->State, HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY, HAL_ADC_STATE_BUSY_INTERNAL);

  // Set ADC parameters
  ADC_Common_TypeDef* tmpADC_Common = ADC_COMMON_REGISTER (hadc);

  // Set the ADC clock prescaler
  tmpADC_Common->CCR &= ~(ADC_CCR_ADCPRE);
  tmpADC_Common->CCR |=  hadc->Init.ClockPrescaler;

  // Set ADC scan mode
  hadc->Instance->CR1 &= ~(ADC_CR1_SCAN);
  hadc->Instance->CR1 |=  ADC_CR1_SCANCONV (hadc->Init.ScanConvMode);

  // Set ADC resolution
  hadc->Instance->CR1 &= ~(ADC_CR1_RES);
  hadc->Instance->CR1 |=  hadc->Init.Resolution;

  // Set ADC data alignment
  hadc->Instance->CR2 &= ~(ADC_CR2_ALIGN);
  hadc->Instance->CR2 |= hadc->Init.DataAlign;

  if (hadc->Init.ExternalTrigConv != ADC_SOFTWARE_START) {
    //{{{  select external trigger to start conversion
    hadc->Instance->CR2 &= ~(ADC_CR2_EXTSEL);
    hadc->Instance->CR2 |= hadc->Init.ExternalTrigConv;

    /* Select external trigger polarity */
    hadc->Instance->CR2 &= ~(ADC_CR2_EXTEN);
    hadc->Instance->CR2 |= hadc->Init.ExternalTrigConvEdge;
    }
    //}}}
  else {
    //{{{  reset external trigger
    hadc->Instance->CR2 &= ~(ADC_CR2_EXTSEL);
    hadc->Instance->CR2 &= ~(ADC_CR2_EXTEN);
    }
    //}}}

  // enable/disable ADC continuous conversion mode
  hadc->Instance->CR2 &= ~(ADC_CR2_CONT);
  hadc->Instance->CR2 |= ADC_CR2_CONTINUOUS(hadc->Init.ContinuousConvMode);

  if (hadc->Init.DiscontinuousConvMode != DISABLE) {
    //{{{  enable the selected ADC regular discontinuous mode
    hadc->Instance->CR1 |= (uint32_t)ADC_CR1_DISCEN;

    /* Set the number of channels to be converted in discontinuous mode */
    hadc->Instance->CR1 &= ~(ADC_CR1_DISCNUM);
    hadc->Instance->CR1 |=  ADC_CR1_DISCONTINUOUS(hadc->Init.NbrOfDiscConversion);
    }
    //}}}
  else
    //{{{  disable the selected ADC regular discontinuous mode
    hadc->Instance->CR1 &= ~(ADC_CR1_DISCEN);
    //}}}

  // set ADC number of conversion
  hadc->Instance->SQR1 &= ~(ADC_SQR1_L);
  hadc->Instance->SQR1 |=  ADC_SQR1 (hadc->Init.NbrOfConversion);

  // enable/disable ADC DMA continuous request
  hadc->Instance->CR2 &= ~(ADC_CR2_DDS);
  hadc->Instance->CR2 |= ADC_CR2_DMAContReq(hadc->Init.DMAContinuousRequests);

  // enable/disable ADC end of conversion selection
  hadc->Instance->CR2 &= ~(ADC_CR2_EOCS);
  hadc->Instance->CR2 |= ADC_CR2_EOCSelection(hadc->Init.EOCSelection);

  // Set ADC error code to none
  ADC_CLEAR_ERRORCODE (hadc);

  // Set the ADC state
  ADC_STATE_CLR_SET (hadc->State, HAL_ADC_STATE_BUSY_INTERNAL, HAL_ADC_STATE_READY);

  return HAL_OK;
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_ADC_ConfigChannel (ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig) {

  if (sConfig->Channel > ADC_CHANNEL_9) {
    //{{{  ADC_Channel_10 ... ADC_Channel_18 is selected
    /* Clear the old sample time */
    hadc->Instance->SMPR1 &= ~ADC_SMPR1 (ADC_SMPR1_SMP10, sConfig->Channel);

    /* Set the new sample time */
    hadc->Instance->SMPR1 |= ADC_SMPR1 (sConfig->SamplingTime, sConfig->Channel);
    }
    //}}}
  else {
    //{{{  ADC_Channel include in ADC_Channel_[0..9]
    /* Clear the old sample time */
    hadc->Instance->SMPR2 &= ~ADC_SMPR2 (ADC_SMPR2_SMP0, sConfig->Channel);

    /* Set the new sample time */
    hadc->Instance->SMPR2 |= ADC_SMPR2 (sConfig->SamplingTime, sConfig->Channel);
    }
    //}}}

  if (sConfig->Rank < 7U) {
    //{{{  Rank 1 to 6
    /* Clear the old SQx bits for the selected rank */
    hadc->Instance->SQR3 &= ~ADC_SQR3_RK (ADC_SQR3_SQ1, sConfig->Rank);

    /* Set the SQx bits for the selected rank */
    hadc->Instance->SQR3 |= ADC_SQR3_RK (sConfig->Channel, sConfig->Rank);
    }
    //}}}
  else if (sConfig->Rank < 13U) {
    //{{{  Rank 7 to 12
    /* Clear the old SQx bits for the selected rank */
    hadc->Instance->SQR2 &= ~ADC_SQR2_RK (ADC_SQR2_SQ7, sConfig->Rank);

    /* Set the SQx bits for the selected rank */
    hadc->Instance->SQR2 |= ADC_SQR2_RK (sConfig->Channel, sConfig->Rank);
    }
    //}}}
  else {
    //{{{  Rank 13 to 16
    /* Clear the old SQx bits for the selected rank */
    hadc->Instance->SQR1 &= ~ADC_SQR1_RK (ADC_SQR1_SQ13, sConfig->Rank);

    /* Set the SQx bits for the selected rank */
    hadc->Instance->SQR1 |= ADC_SQR1_RK (sConfig->Channel, sConfig->Rank);
    }
    //}}}

  ADC_Common_TypeDef* tmpADC_Common = ADC_COMMON_REGISTER (hadc);
  if ((hadc->Instance == ADC1) && (sConfig->Channel == ADC_CHANNEL_VBAT))
    // if ADC1 Channel_18 is selected enable VBAT Channel
    tmpADC_Common->CCR |= ADC_CCR_VBATE;

  if ((hadc->Instance == ADC1) &&
      ((sConfig->Channel == ADC_CHANNEL_TEMPSENSOR) || (sConfig->Channel == ADC_CHANNEL_VREFINT))) {
    // if ADC1 Channel_16 or Channel_17 is selected enable TSVREFE Channel(Temperature sensor and VREFINT)
    // Enable the TSVREFE channel*/
    tmpADC_Common->CCR |= ADC_CCR_TSVREFE;
    if ((sConfig->Channel == ADC_CHANNEL_TEMPSENSOR)) {
      // Delay for temperature sensor stabilization time Compute number of CPU cycles to wait for
      uint32_t counter = (ADC_TEMPSENSOR_DELAY_US * (SystemCoreClock / 1000000U));
      while (counter != 0U)
        counter--;
      }
    }

  return HAL_OK;
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_ADC_AnalogWDGConfig (ADC_HandleTypeDef* hadc, ADC_AnalogWDGConfTypeDef* AnalogWDGConfig) {

  if(AnalogWDGConfig->ITMode == ENABLE)
    __HAL_ADC_ENABLE_IT(hadc, ADC_IT_AWD);
  else
    __HAL_ADC_DISABLE_IT(hadc, ADC_IT_AWD);

  /* Clear AWDEN, JAWDEN and AWDSGL bits */
  hadc->Instance->CR1 &=  ~(ADC_CR1_AWDSGL | ADC_CR1_JAWDEN | ADC_CR1_AWDEN);

  /* Set the analog watchdog enable mode */
  hadc->Instance->CR1 |= AnalogWDGConfig->WatchdogMode;

  /* Set the high threshold */
  hadc->Instance->HTR = AnalogWDGConfig->HighThreshold;

  /* Set the low threshold */
  hadc->Instance->LTR = AnalogWDGConfig->LowThreshold;

  /* Clear the Analog watchdog channel select bits */
  hadc->Instance->CR1 &= ~ADC_CR1_AWDCH;

  /* Set the Analog watchdog channel */
  hadc->Instance->CR1 |= (uint32_t)((uint16_t)(AnalogWDGConfig->Channel));

  return HAL_OK;
  }
//}}}

//{{{
HAL_StatusTypeDef HAL_ADC_Start (ADC_HandleTypeDef* hadc) {

  /* Check if ADC peripheral is disabled in order to enable it and wait during Tstab time the ADC's stabilization */
  if ((hadc->Instance->CR2 & ADC_CR2_ADON) != ADC_CR2_ADON) {
    /* Enable the Peripheral */
    __HAL_ADC_ENABLE (hadc);

    /* Delay for ADC stabilization time Compute number of CPU cycles to wait for */
    uint32_t counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
    while (counter != 0U)
      counter--;
    }

  /* Start conversion if ADC is effectively enabled */
  if (HAL_IS_BIT_SET (hadc->Instance->CR2, ADC_CR2_ADON)) {
    // - Clear state bitfield related to regular group conversion results
    // - Set state bitfield related to regular group operation
    ADC_STATE_CLR_SET (hadc->State,
                       HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR, HAL_ADC_STATE_REG_BUSY);

    // If conversions on group regular are also triggering group injected  update ADC state.                                                      */
    if (READ_BIT(hadc->Instance->CR1, ADC_CR1_JAUTO) != RESET)
      ADC_STATE_CLR_SET (hadc->State, HAL_ADC_STATE_INJ_EOC, HAL_ADC_STATE_INJ_BUSY);

    // State machine update: Check if an injected conversion is ongoing
    if (HAL_IS_BIT_SET (hadc->State, HAL_ADC_STATE_INJ_BUSY))
      // Reset ADC error code fields related to conversions on group regular
      CLEAR_BIT (hadc->ErrorCode, (HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA));
    else
      // Reset ADC all error code fields
      ADC_CLEAR_ERRORCODE (hadc);

    // Pointer to the common control register to which is belonging hadc
    // (Depending on STM32F4 product, there may be up to 3 ADCs and 1 common control register)                                                    */
    ADC_Common_TypeDef* tmpADC_Common = ADC_COMMON_REGISTER (hadc);

    /* Clear regular group conversion flag and overrun flag */
    /* (To ensure of no unknown state from potential previous ADC operations) */
    __HAL_ADC_CLEAR_FLAG (hadc, ADC_FLAG_EOC | ADC_FLAG_OVR);

    /* Check if Multimode enabled */
    if (HAL_IS_BIT_CLR(tmpADC_Common->CCR, ADC_CCR_MULTI)) {
      /* if no external trigger present enable software conversion of regular channels */
      if ((hadc->Instance->CR2 & ADC_CR2_EXTEN) == RESET)
        /* Enable the selected ADC software conversion for regular group */
        hadc->Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
      }
    else {
      // if ADC1 and no external trigger present enable software conversion of regular channels
      if ((hadc->Instance == ADC1) && ((hadc->Instance->CR2 & ADC_CR2_EXTEN) == RESET))
        /* Enable the selected ADC software conversion for regular group */
        hadc->Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
      }
    }

  return HAL_OK;
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_ADC_Stop (ADC_HandleTypeDef* hadc) {

  /* Stop potential conversion on going, on regular and injected groups  Disable ADC peripheral */
  __HAL_ADC_DISABLE(hadc);

  /* Check if ADC is effectively disabled */
  if(HAL_IS_BIT_CLR(hadc->Instance->CR2, ADC_CR2_ADON))
    /* Set ADC state */
    ADC_STATE_CLR_SET(hadc->State, HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY, HAL_ADC_STATE_READY);

  return HAL_OK;
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_ADC_PollForConversion (ADC_HandleTypeDef* hadc, uint32_t Timeout) {
// Verification that ADC configuration is compliant with polling for  each conversion:
// Particular case is ADC configured in DMA mode and ADC sequencer with
// several ranks and polling for end of each conversion.
// For code simplicity sake, this particular case is generalized to
// ADC configured in DMA mode and polling for end of each conversion.

  if (HAL_IS_BIT_SET (hadc->Instance->CR2, ADC_CR2_EOCS) &&
      HAL_IS_BIT_SET (hadc->Instance->CR2, ADC_CR2_DMA)) {
    // Update ADC state machine to error
    SET_BIT (hadc->State, HAL_ADC_STATE_ERROR_CONFIG);
    return HAL_ERROR;
    }

  // Check End of conversion flag
  uint32_t tickstart = HAL_GetTick();
  while (!(__HAL_ADC_GET_FLAG (hadc, ADC_FLAG_EOC))) {
    // Check if timeout is disabled (set to infinite wait)
    if (Timeout != HAL_MAX_DELAY) {
      if ((Timeout == 0U) || ((HAL_GetTick() - tickstart ) > Timeout)) {
        // Update ADC state machine to timeout
        SET_BIT (hadc->State, HAL_ADC_STATE_TIMEOUT);
        return HAL_TIMEOUT;
        }
      }
    }

  // Clear regular group conversion flag
  __HAL_ADC_CLEAR_FLAG (hadc, ADC_FLAG_STRT | ADC_FLAG_EOC);

  // Update ADC state machine
  SET_BIT (hadc->State, HAL_ADC_STATE_REG_EOC);

  // Determine whether any further conversion upcoming on group regular
  // by external trigger, continuous mode or scan sequence on going.
  // On STM32F4, there is no independent flag of end of sequence.
  // The test of scan sequence on going is done either with scan
  // sequence disabled or with end of conversion flag set to of end of sequence                                                */
  if (ADC_IS_SOFTWARE_START_REGULAR (hadc) &&
      (hadc->Init.ContinuousConvMode == DISABLE) &&
      (HAL_IS_BIT_CLR (hadc->Instance->SQR1, ADC_SQR1_L) ||
       HAL_IS_BIT_CLR (hadc->Instance->CR2, ADC_CR2_EOCS))) {
    // Set ADC state */
    CLEAR_BIT (hadc->State, HAL_ADC_STATE_REG_BUSY);
    if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_INJ_BUSY))
      SET_BIT (hadc->State, HAL_ADC_STATE_READY);
    }

  // Return ADC state
  return HAL_OK;
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_ADC_PollForEvent (ADC_HandleTypeDef* hadc, uint32_t EventType, uint32_t Timeout) {

  /* Check selected event flag */
  uint32_t tickstart = HAL_GetTick();
  while (!(__HAL_ADC_GET_FLAG(hadc,EventType))) {
    // Check for the Timeout
    if (Timeout != HAL_MAX_DELAY) {
      if ((Timeout == 0U) || ((HAL_GetTick() - tickstart ) > Timeout)) {
        // Update ADC state machine to timeout
        SET_BIT(hadc->State, HAL_ADC_STATE_TIMEOUT);
        return HAL_TIMEOUT;
        }
      }
    }

  /* Analog watchdog (level out of window) event */
  if (EventType == ADC_AWD_EVENT) {
    /* Set ADC state */
    SET_BIT(hadc->State, HAL_ADC_STATE_AWD1);
    /* Clear ADC analog watchdog flag */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_AWD);
    }
  /* Overrun event */
  else {
    /* Set ADC state */
    SET_BIT(hadc->State, HAL_ADC_STATE_REG_OVR);
    /* Set ADC error code to overrun */
    SET_BIT(hadc->ErrorCode, HAL_ADC_ERROR_OVR);

    /* Clear ADC overrun flag */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_OVR);
    }

  /* Return ADC state */
  return HAL_OK;
  }
//}}}
uint32_t HAL_ADC_GetValue (ADC_HandleTypeDef* hadc) { return hadc->Instance->DR; }

//{{{
HAL_StatusTypeDef HAL_ADC_Start_IT (ADC_HandleTypeDef* hadc) {

  // Check if ADC peripheral is disabled in order to enable it and wait during Tstab time the ADC's stabilization */
  if((hadc->Instance->CR2 & ADC_CR2_ADON) != ADC_CR2_ADON) {
    __HAL_ADC_ENABLE (hadc);
    uint32_t counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
    while (counter != 0U)
      counter--;
    }

  /* Start conversion if ADC is effectively enabled */
  if(HAL_IS_BIT_SET(hadc->Instance->CR2, ADC_CR2_ADON)) {
    /* - Clear state bitfield related to regular group conversion results     */
    /* - Set state bitfield related to regular group operation                */
    ADC_STATE_CLR_SET(hadc->State,
                      HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR, HAL_ADC_STATE_REG_BUSY);

    /* If conversions on group regular are also triggering group injected,    */
    /* update ADC state.                                                      */
    if (READ_BIT(hadc->Instance->CR1, ADC_CR1_JAUTO) != RESET)
      ADC_STATE_CLR_SET(hadc->State, HAL_ADC_STATE_INJ_EOC, HAL_ADC_STATE_INJ_BUSY);

    /* State machine update: Check if an injected conversion is ongoing */
    if (HAL_IS_BIT_SET(hadc->State, HAL_ADC_STATE_INJ_BUSY))
      /* Reset ADC error code fields related to conversions on group regular */
      CLEAR_BIT(hadc->ErrorCode, (HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA));
    else
      /* Reset ADC all error code fields */
      ADC_CLEAR_ERRORCODE(hadc);

    /* Pointer to the common control register to which is belonging hadc    */
    /* (Depending on STM32F4 product, there may be up to 3 ADCs and 1 common */
    /* control register)                                                    */
    ADC_Common_TypeDef* tmpADC_Common = ADC_COMMON_REGISTER(hadc);

    /* Clear regular group conversion flag and overrun flag */
    /* (To ensure of no unknown state from potential previous ADC operations) */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_EOC | ADC_FLAG_OVR);

    /* Enable end of conversion interrupt for regular group */
    __HAL_ADC_ENABLE_IT (hadc, (ADC_IT_EOC | ADC_IT_OVR));

    /* Check if Multimode enabled */
    if (HAL_IS_BIT_CLR (tmpADC_Common->CCR, ADC_CCR_MULTI)) {
      /* if no external trigger present enable software conversion of regular channels */
      if ((hadc->Instance->CR2 & ADC_CR2_EXTEN) == RESET)
        /* Enable the selected ADC software conversion for regular group */
        hadc->Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
      }
    else {
      /* if instance of handle correspond to ADC1 and  no external trigger present enable software conversion of regular channels */
      if ((hadc->Instance == ADC1) && ((hadc->Instance->CR2 & ADC_CR2_EXTEN) == RESET))
        /* Enable the selected ADC software conversion for regular group */
          hadc->Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
      }
    }

  return HAL_OK;
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_ADC_Stop_IT (ADC_HandleTypeDef* hadc) {

  /* Stop potential conversion on going, on regular and injected groups */
  /* Disable ADC peripheral */
  __HAL_ADC_DISABLE(hadc);

  /* Check if ADC is effectively disabled */
  if (HAL_IS_BIT_CLR(hadc->Instance->CR2, ADC_CR2_ADON)) {
    /* Disable ADC end of conversion interrupt for regular group */
    __HAL_ADC_DISABLE_IT(hadc, (ADC_IT_EOC | ADC_IT_OVR));

    /* Set ADC state */
    ADC_STATE_CLR_SET(hadc->State,
                      HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY, HAL_ADC_STATE_READY);
    }

  return HAL_OK;
  }
//}}}
//{{{
void HAL_ADC_IRQHandler (ADC_HandleTypeDef* hadc) {

  // Check End of conversion flag for regular channels
  uint32_t tmp1 = __HAL_ADC_GET_FLAG (hadc, ADC_FLAG_EOC);
  uint32_t tmp2 = __HAL_ADC_GET_IT_SOURCE (hadc, ADC_IT_EOC);
  if (tmp1 && tmp2) {
    /* Update state machine on conversion status if not in error state */
    if (HAL_IS_BIT_CLR (hadc->State, HAL_ADC_STATE_ERROR_INTERNAL))
      /* Set ADC state */
      SET_BIT (hadc->State, HAL_ADC_STATE_REG_EOC);

    /* Determine whether any further conversion upcoming on group regular   */
    /* by external trigger, continuous mode or scan sequence on going.      */
    /* Note: On STM32F4, there is no independent flag of end of sequence.   */
    /*       The test of scan sequence on going is done either with scan    */
    /*       sequence disabled or with end of conversion flag set to        */
    /*       of end of sequence.                                            */
    if (ADC_IS_SOFTWARE_START_REGULAR(hadc) &&
        (hadc->Init.ContinuousConvMode == DISABLE) &&
        (HAL_IS_BIT_CLR(hadc->Instance->SQR1, ADC_SQR1_L) ||
         HAL_IS_BIT_CLR(hadc->Instance->CR2, ADC_CR2_EOCS))) {
      /* Disable ADC end of single conversion interrupt on group regular */
      /* Note: Overrun interrupt was enabled with EOC interrupt in          */
      /* HAL_ADC_Start_IT(), but is not disabled here because can be used   */
      /* by overrun IRQ process below.                                      */
      __HAL_ADC_DISABLE_IT(hadc, ADC_IT_EOC);

      /* Set ADC state */
      CLEAR_BIT (hadc->State, HAL_ADC_STATE_REG_BUSY);
      if (HAL_IS_BIT_CLR (hadc->State, HAL_ADC_STATE_INJ_BUSY))
        SET_BIT (hadc->State, HAL_ADC_STATE_READY);
      }

    /* Conversion complete callback */
    HAL_ADC_ConvCpltCallback (hadc);

    /* Clear regular group conversion flag */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_STRT | ADC_FLAG_EOC);
    }

  /* Check End of conversion flag for injected channels */
  tmp1 = __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOC);
  tmp2 = __HAL_ADC_GET_IT_SOURCE(hadc, ADC_IT_JEOC);
  if (tmp1 && tmp2) {
    /* Update state machine on conversion status if not in error state */
    if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL))
      /* Set ADC state */
      SET_BIT (hadc->State, HAL_ADC_STATE_INJ_EOC);

    /* Determine whether any further conversion upcoming on group injected  */
    /* by external trigger, scan sequence on going or by automatic injected */
    /* conversion from group regular (same conditions as group regular      */
    /* interruption disabling above).                                       */
    if (ADC_IS_SOFTWARE_START_INJECTED(hadc) &&
        (HAL_IS_BIT_CLR (hadc->Instance->JSQR, ADC_JSQR_JL) ||
         HAL_IS_BIT_CLR (hadc->Instance->CR2, ADC_CR2_EOCS)) &&
        (HAL_IS_BIT_CLR (hadc->Instance->CR1, ADC_CR1_JAUTO) &&
         (ADC_IS_SOFTWARE_START_REGULAR (hadc) &&
         (hadc->Init.ContinuousConvMode == DISABLE)))) {
      /* Disable ADC end of single conversion interrupt on group injected */
      __HAL_ADC_DISABLE_IT (hadc, ADC_IT_JEOC);
      /* Set ADC state */
      CLEAR_BIT (hadc->State, HAL_ADC_STATE_INJ_BUSY);
      if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_REG_BUSY))
        SET_BIT (hadc->State, HAL_ADC_STATE_READY);
      }

    /* Conversion complete callback */
    HAL_ADCEx_InjectedConvCpltCallback(hadc);
    /* Clear injected group conversion flag */
    __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_JSTRT | ADC_FLAG_JEOC));
    }

  /* Check Analog watchdog flag */
  tmp1 = __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_AWD);
  tmp2 = __HAL_ADC_GET_IT_SOURCE(hadc, ADC_IT_AWD);
  if (tmp1 && tmp2) {
    if (__HAL_ADC_GET_FLAG (hadc, ADC_FLAG_AWD)) {
      /* Set ADC state */
      SET_BIT (hadc->State, HAL_ADC_STATE_AWD1);
      /* Level out of window callback */
      HAL_ADC_LevelOutOfWindowCallback (hadc);
      /* Clear the ADC analog watchdog flag */
      __HAL_ADC_CLEAR_FLAG (hadc, ADC_FLAG_AWD);
      }
    }

  /* Check Overrun flag */
  tmp1 = __HAL_ADC_GET_FLAG (hadc, ADC_FLAG_OVR);
  tmp2 = __HAL_ADC_GET_IT_SOURCE (hadc, ADC_IT_OVR);
  if( tmp1 && tmp2) {
    /* Note: On STM32F4, ADC overrun can be set through other parameters    */
    /*       refer to description of parameter "EOCSelection" for more      */
    /*       details.                                                       */
    /* Set ADC error code to overrun */
    SET_BIT (hadc->ErrorCode, HAL_ADC_ERROR_OVR);
    /* Clear ADC overrun flag */
    __HAL_ADC_CLEAR_FLAG (hadc, ADC_FLAG_OVR);
    /* Error callback */
    HAL_ADC_ErrorCallback (hadc);
    /* Clear the Overrun flag */
    __HAL_ADC_CLEAR_FLAG (hadc, ADC_FLAG_OVR);
    }
  }
//}}}

//{{{
HAL_StatusTypeDef HAL_ADC_Start_DMA (ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length) {

  ADC_Common_TypeDef *tmpADC_Common;

  /* Enable the ADC peripheral */
  /* Check if ADC peripheral is disabled in order to enable it and wait during
  Tstab time the ADC's stabilization */
  if((hadc->Instance->CR2 & ADC_CR2_ADON) != ADC_CR2_ADON) {
    /* Enable the Peripheral */
    __HAL_ADC_ENABLE(hadc);

    /* Delay for ADC stabilization time */
    uint32_t counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
    while (counter != 0U)
      counter--;
    }

  /* Start conversion if ADC is effectively enabled */
  if(HAL_IS_BIT_SET(hadc->Instance->CR2, ADC_CR2_ADON)) {
    /* Set ADC state                                                          */
    /* - Clear state bitfield related to regular group conversion results     */
    /* - Set state bitfield related to regular group operation                */
    ADC_STATE_CLR_SET(hadc->State,
                      HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR, HAL_ADC_STATE_REG_BUSY);

    /* If conversions on group regular are also triggering group injected,    */
    /* update ADC state.                                                      */
    if (READ_BIT(hadc->Instance->CR1, ADC_CR1_JAUTO) != RESET)
      ADC_STATE_CLR_SET(hadc->State, HAL_ADC_STATE_INJ_EOC, HAL_ADC_STATE_INJ_BUSY);

    /* State machine update: Check if an injected conversion is ongoing */
    if (HAL_IS_BIT_SET(hadc->State, HAL_ADC_STATE_INJ_BUSY))
      /* Reset ADC error code fields related to conversions on group regular */
      CLEAR_BIT(hadc->ErrorCode, (HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA));
    else
      /* Reset ADC all error code fields */
      ADC_CLEAR_ERRORCODE (hadc);

    tmpADC_Common = ADC_COMMON_REGISTER (hadc);

    /* Set the DMA transfer complete callback */
    hadc->DMA_Handle->XferCpltCallback = ADC_DMAConvCplt;

    /* Set the DMA half transfer complete callback */
    hadc->DMA_Handle->XferHalfCpltCallback = ADC_DMAHalfConvCplt;

    /* Set the DMA error callback */
    hadc->DMA_Handle->XferErrorCallback = ADC_DMAError;

    /* Manage ADC and DMA start: ADC overrun interruption, DMA start, ADC     */
    /* start (in case of SW start):                                           */
    /* Clear regular group conversion flag and overrun flag */
    /* (To ensure of no unknown state from potential previous ADC operations) */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_EOC | ADC_FLAG_OVR);

    /* Enable ADC overrun interrupt */
    __HAL_ADC_ENABLE_IT(hadc, ADC_IT_OVR);

    /* Enable ADC DMA mode */
    hadc->Instance->CR2 |= ADC_CR2_DMA;

    /* Start the DMA channel */
    HAL_DMA_Start_IT(hadc->DMA_Handle, (uint32_t)&hadc->Instance->DR, (uint32_t)pData, Length);

    /* Check if Multimode enabled */
    if(HAL_IS_BIT_CLR(tmpADC_Common->CCR, ADC_CCR_MULTI)) {
      /* if no external trigger present enable software conversion of regular channels */
      if((hadc->Instance->CR2 & ADC_CR2_EXTEN) == RESET)
        /* Enable the selected ADC software conversion for regular group */
        hadc->Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
      }
    else {
      /* if instance of handle correspond to ADC1 and  no external trigger present enable software conversion of regular channels */
      if((hadc->Instance == ADC1) && ((hadc->Instance->CR2 & ADC_CR2_EXTEN) == RESET))
        /* Enable the selected ADC software conversion for regular group */
          hadc->Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
      }
    }

  return HAL_OK;
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_ADC_Stop_DMA (ADC_HandleTypeDef* hadc)
{
  HAL_StatusTypeDef tmp_hal_status = HAL_OK;

  /* Stop potential conversion on going, on regular and injected groups */
  /* Disable ADC peripheral */
  __HAL_ADC_DISABLE(hadc);

  /* Check if ADC is effectively disabled */
  if(HAL_IS_BIT_CLR(hadc->Instance->CR2, ADC_CR2_ADON))
  {
    /* Disable the selected ADC DMA mode */
    hadc->Instance->CR2 &= ~ADC_CR2_DMA;

    /* Disable the DMA channel (in case of DMA in circular mode or stop while */
    /* DMA transfer is on going)                                              */
    tmp_hal_status = HAL_DMA_Abort(hadc->DMA_Handle);

    /* Disable ADC overrun interrupt */
    __HAL_ADC_DISABLE_IT(hadc, ADC_IT_OVR);

    /* Set ADC state */
    ADC_STATE_CLR_SET(hadc->State,
                      HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,
                      HAL_ADC_STATE_READY);
  }

  return tmp_hal_status;
}
//}}}

//{{{
HAL_StatusTypeDef HAL_ADCEx_InjectedStart (ADC_HandleTypeDef* hadc) {

  uint32_t tmp1 = 0U, tmp2 = 0U;
  ADC_Common_TypeDef *tmpADC_Common;

  /* Check if ADC peripheral is disabled in order to enable it and wait during
     Tstab time the ADC's stabilization */
  if((hadc->Instance->CR2 & ADC_CR2_ADON) != ADC_CR2_ADON) {
    /* Enable the Peripheral */
    __HAL_ADC_ENABLE(hadc);

    uint32_t counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
    while (counter != 0U)
      counter--;
    }

  /* Start conversion if ADC is effectively enabled */
  if(HAL_IS_BIT_SET(hadc->Instance->CR2, ADC_CR2_ADON)) {
    /* Set ADC state                                                          */
    /* - Clear state bitfield related to injected group conversion results    */
    /* - Set state bitfield related to injected operation                     */
    ADC_STATE_CLR_SET(hadc->State,
                      HAL_ADC_STATE_READY | HAL_ADC_STATE_INJ_EOC,
                      HAL_ADC_STATE_INJ_BUSY);
    /* Check if a regular conversion is ongoing */
    /* Note: On this device, there is no ADC error code fields related to     */
    /*       conversions on group injected only. In case of conversion on     */
    /*       going on group regular, no error code is reset.                  */
    if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_REG_BUSY))
      /* Reset ADC all error code fields */
      ADC_CLEAR_ERRORCODE(hadc);

    /* Clear injected group conversion flag */
    /* (To ensure of no unknown state from potential previous ADC operations) */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JEOC);

    /* Pointer to the common control register to which is belonging hadc    */
    /* (Depending on STM32F4 product, there may be up to 3 ADC and 1 common */
    /* control register)                                                    */
    tmpADC_Common = ADC_COMMON_REGISTER(hadc);

    /* Check if Multimode enabled */
    if (HAL_IS_BIT_CLR(tmpADC_Common->CCR, ADC_CCR_MULTI)) {
      tmp1 = HAL_IS_BIT_CLR(hadc->Instance->CR2, ADC_CR2_JEXTEN);
      tmp2 = HAL_IS_BIT_CLR(hadc->Instance->CR1, ADC_CR1_JAUTO);
      if(tmp1 && tmp2)
        /* Enable the selected ADC software conversion for injected group */
        hadc->Instance->CR2 |= ADC_CR2_JSWSTART;
      }
    else {
      tmp1 = HAL_IS_BIT_CLR(hadc->Instance->CR2, ADC_CR2_JEXTEN);
      tmp2 = HAL_IS_BIT_CLR(hadc->Instance->CR1, ADC_CR1_JAUTO);
      if ((hadc->Instance == ADC1) && tmp1 && tmp2)
        /* Enable the selected ADC software conversion for injected group */
        hadc->Instance->CR2 |= ADC_CR2_JSWSTART;
      }
    }

  return HAL_OK;
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT (ADC_HandleTypeDef* hadc) {

  uint32_t tmp1 = 0U, tmp2 = 0U;
  ADC_Common_TypeDef *tmpADC_Common;

  /* Check if ADC peripheral is disabled in order to enable it and wait during
     Tstab time the ADC's stabilization */
  if((hadc->Instance->CR2 & ADC_CR2_ADON) != ADC_CR2_ADON) {
    /* Enable the Peripheral */
    __HAL_ADC_ENABLE(hadc);

    uint32_t counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
    while (counter != 0U)
      counter--;
    }

  /* Start conversion if ADC is effectively enabled */
  if(HAL_IS_BIT_SET(hadc->Instance->CR2, ADC_CR2_ADON)) {
    /* Set ADC state                                                          */
    /* - Clear state bitfield related to injected group conversion results    */
    /* - Set state bitfield related to injected operation                     */
    ADC_STATE_CLR_SET(hadc->State,
                      HAL_ADC_STATE_READY | HAL_ADC_STATE_INJ_EOC, HAL_ADC_STATE_INJ_BUSY);

    /* Check if a regular conversion is ongoing */
    /* Note: On this device, there is no ADC error code fields related to     */
    /*       conversions on group injected only. In case of conversion on     */
    /*       going on group regular, no error code is reset.                  */
    if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_REG_BUSY))
      ADC_CLEAR_ERRORCODE(hadc);

    /* Clear injected group conversion flag */
    /* (To ensure of no unknown state from potential previous ADC operations) */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JEOC);

    /* Enable end of conversion interrupt for injected channels */
    __HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOC);

    /* Pointer to the common control register to which is belonging hadc    */
    /* (Depending on STM32F4 product, there may be up to 3 ADC and 1 common */
    /* control register)                                                    */
    tmpADC_Common = ADC_COMMON_REGISTER(hadc);

    /* Check if Multimode enabled */
    if(HAL_IS_BIT_CLR(tmpADC_Common->CCR, ADC_CCR_MULTI)) {
      tmp1 = HAL_IS_BIT_CLR(hadc->Instance->CR2, ADC_CR2_JEXTEN);
      tmp2 = HAL_IS_BIT_CLR(hadc->Instance->CR1, ADC_CR1_JAUTO);
      if(tmp1 && tmp2)
        /* Enable the selected ADC software conversion for injected group */
        hadc->Instance->CR2 |= ADC_CR2_JSWSTART;
      }
    else {
      tmp1 = HAL_IS_BIT_CLR(hadc->Instance->CR2, ADC_CR2_JEXTEN);
      tmp2 = HAL_IS_BIT_CLR(hadc->Instance->CR1, ADC_CR1_JAUTO);
      if((hadc->Instance == ADC1) && tmp1 && tmp2)
        /* Enable the selected ADC software conversion for injected group */
        hadc->Instance->CR2 |= ADC_CR2_JSWSTART;
      }
    }

  return HAL_OK;
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_ADCEx_InjectedStop (ADC_HandleTypeDef* hadc)
{
  HAL_StatusTypeDef tmp_hal_status = HAL_OK;

  /* Stop potential conversion and disable ADC peripheral                     */
  /* Conditioned to:                                                          */
  /* - No conversion on the other group (regular group) is intended to        */
  /*   continue (injected and regular groups stop conversion and ADC disable  */
  /*   are common)                                                            */
  /* - In case of auto-injection mode, HAL_ADC_Stop must be used.             */
  if(((hadc->State & HAL_ADC_STATE_REG_BUSY) == RESET)  &&
     HAL_IS_BIT_CLR(hadc->Instance->CR1, ADC_CR1_JAUTO)   )
  {
    /* Stop potential conversion on going, on regular and injected groups */
    /* Disable ADC peripheral */
    __HAL_ADC_DISABLE(hadc);

    /* Check if ADC is effectively disabled */
    if(HAL_IS_BIT_CLR(hadc->Instance->CR2, ADC_CR2_ADON))
    {
      /* Set ADC state */
      ADC_STATE_CLR_SET(hadc->State,
                        HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,
                        HAL_ADC_STATE_READY);
    }
  }
  else
  {
    /* Update ADC state machine to error */
    SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);

    tmp_hal_status = HAL_ERROR;
  }

  return tmp_hal_status;
}
//}}}
//{{{
HAL_StatusTypeDef HAL_ADCEx_InjectedPollForConversion (ADC_HandleTypeDef* hadc, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  /* Get tick */
  tickstart = HAL_GetTick();

  /* Check End of conversion flag */
  while(!(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOC)))
  {
    /* Check for the Timeout */
    if(Timeout != HAL_MAX_DELAY)
    {
      if((Timeout == 0U)||((HAL_GetTick() - tickstart ) > Timeout))
      {
        hadc->State= HAL_ADC_STATE_TIMEOUT;
        return HAL_TIMEOUT;
      }
    }
  }

  /* Clear injected group conversion flag */
  __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JSTRT | ADC_FLAG_JEOC);

  /* Update ADC state machine */
  SET_BIT(hadc->State, HAL_ADC_STATE_INJ_EOC);

  /* Determine whether any further conversion upcoming on group injected      */
  /* by external trigger, continuous mode or scan sequence on going.          */
  /* Note: On STM32F4, there is no independent flag of end of sequence.       */
  /*       The test of scan sequence on going is done either with scan        */
  /*       sequence disabled or with end of conversion flag set to            */
  /*       of end of sequence.                                                */
  if(ADC_IS_SOFTWARE_START_INJECTED(hadc)                    &&
     (HAL_IS_BIT_CLR(hadc->Instance->JSQR, ADC_JSQR_JL)  ||
      HAL_IS_BIT_CLR(hadc->Instance->CR2, ADC_CR2_EOCS)    ) &&
     (HAL_IS_BIT_CLR(hadc->Instance->CR1, ADC_CR1_JAUTO) &&
      (ADC_IS_SOFTWARE_START_REGULAR(hadc)       &&
      (hadc->Init.ContinuousConvMode == DISABLE)   )       )   )
  {
    /* Set ADC state */
    CLEAR_BIT(hadc->State, HAL_ADC_STATE_INJ_BUSY);

    if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_REG_BUSY))
    {
      SET_BIT(hadc->State, HAL_ADC_STATE_READY);
    }
  }

  /* Return ADC state */
  return HAL_OK;
}
//}}}
//{{{
HAL_StatusTypeDef HAL_ADCEx_InjectedStop_IT (ADC_HandleTypeDef* hadc)
{
  HAL_StatusTypeDef tmp_hal_status = HAL_OK;

  /* Stop potential conversion and disable ADC peripheral                     */
  /* Conditioned to:                                                          */
  /* - No conversion on the other group (regular group) is intended to        */
  /*   continue (injected and regular groups stop conversion and ADC disable  */
  /*   are common)                                                            */
  /* - In case of auto-injection mode, HAL_ADC_Stop must be used.             */
  if(((hadc->State & HAL_ADC_STATE_REG_BUSY) == RESET)  &&
     HAL_IS_BIT_CLR(hadc->Instance->CR1, ADC_CR1_JAUTO)   )
  {
    /* Stop potential conversion on going, on regular and injected groups */
    /* Disable ADC peripheral */
    __HAL_ADC_DISABLE(hadc);

    /* Check if ADC is effectively disabled */
    if(HAL_IS_BIT_CLR(hadc->Instance->CR2, ADC_CR2_ADON))
    {
      /* Disable ADC end of conversion interrupt for injected channels */
      __HAL_ADC_DISABLE_IT(hadc, ADC_IT_JEOC);

      /* Set ADC state */
      ADC_STATE_CLR_SET(hadc->State,
                        HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,
                        HAL_ADC_STATE_READY);
    }
  }
  else
  {
    /* Update ADC state machine to error */
    SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);

    tmp_hal_status = HAL_ERROR;
  }

  /* Return function status */
  return tmp_hal_status;
}
//}}}
//{{{
uint32_t HAL_ADCEx_InjectedGetValue (ADC_HandleTypeDef* hadc, uint32_t InjectedRank)
{
  __IO uint32_t tmp = 0U;

  /* Clear injected group conversion flag to have similar behaviour as        */
  /* regular group: reading data register also clears end of conversion flag. */
  __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_JEOC);

  /* Return the selected ADC converted value */
  switch(InjectedRank)
  {
    case ADC_INJECTED_RANK_4:
    {
      tmp =  hadc->Instance->JDR4;
    }
    break;
    case ADC_INJECTED_RANK_3:
    {
      tmp =  hadc->Instance->JDR3;
    }
    break;
    case ADC_INJECTED_RANK_2:
    {
      tmp =  hadc->Instance->JDR2;
    }
    break;
    case ADC_INJECTED_RANK_1:
    {
      tmp =  hadc->Instance->JDR1;
    }
    break;
    default:
    break;
  }
  return tmp;
}
//}}}
//{{{
HAL_StatusTypeDef HAL_ADCEx_InjectedConfigChannel (ADC_HandleTypeDef* hadc, ADC_InjectionConfTypeDef* sConfigInjected)
{

  ADC_Common_TypeDef *tmpADC_Common;

  /* if ADC_Channel_10 ... ADC_Channel_18 is selected */
  if (sConfigInjected->InjectedChannel > ADC_CHANNEL_9)
  {
    /* Clear the old sample time */
    hadc->Instance->SMPR1 &= ~ADC_SMPR1(ADC_SMPR1_SMP10, sConfigInjected->InjectedChannel);

    /* Set the new sample time */
    hadc->Instance->SMPR1 |= ADC_SMPR1(sConfigInjected->InjectedSamplingTime, sConfigInjected->InjectedChannel);
  }
  else /* ADC_Channel include in ADC_Channel_[0..9] */
  {
    /* Clear the old sample time */
    hadc->Instance->SMPR2 &= ~ADC_SMPR2(ADC_SMPR2_SMP0, sConfigInjected->InjectedChannel);

    /* Set the new sample time */
    hadc->Instance->SMPR2 |= ADC_SMPR2(sConfigInjected->InjectedSamplingTime, sConfigInjected->InjectedChannel);
  }

  /*---------------------------- ADCx JSQR Configuration -----------------*/
  hadc->Instance->JSQR &= ~(ADC_JSQR_JL);
  hadc->Instance->JSQR |=  ADC_SQR1(sConfigInjected->InjectedNbrOfConversion);

  /* Rank configuration */

  /* Clear the old SQx bits for the selected rank */
  hadc->Instance->JSQR &= ~ADC_JSQR(ADC_JSQR_JSQ1, sConfigInjected->InjectedRank,sConfigInjected->InjectedNbrOfConversion);

  /* Set the SQx bits for the selected rank */
  hadc->Instance->JSQR |= ADC_JSQR(sConfigInjected->InjectedChannel, sConfigInjected->InjectedRank,sConfigInjected->InjectedNbrOfConversion);

  /* Enable external trigger if trigger selection is different of software  */
  /* start.                                                                 */
  /* Note: This configuration keeps the hardware feature of parameter       */
  /*       ExternalTrigConvEdge "trigger edge none" equivalent to           */
  /*       software start.                                                  */
  if(sConfigInjected->ExternalTrigInjecConv != ADC_INJECTED_SOFTWARE_START)
  {
    /* Select external trigger to start conversion */
    hadc->Instance->CR2 &= ~(ADC_CR2_JEXTSEL);
    hadc->Instance->CR2 |=  sConfigInjected->ExternalTrigInjecConv;

    /* Select external trigger polarity */
    hadc->Instance->CR2 &= ~(ADC_CR2_JEXTEN);
    hadc->Instance->CR2 |= sConfigInjected->ExternalTrigInjecConvEdge;
  }
  else
  {
    /* Reset the external trigger */
    hadc->Instance->CR2 &= ~(ADC_CR2_JEXTSEL);
    hadc->Instance->CR2 &= ~(ADC_CR2_JEXTEN);
  }

  if (sConfigInjected->AutoInjectedConv != DISABLE)
  {
    /* Enable the selected ADC automatic injected group conversion */
    hadc->Instance->CR1 |= ADC_CR1_JAUTO;
  }
  else
  {
    /* Disable the selected ADC automatic injected group conversion */
    hadc->Instance->CR1 &= ~(ADC_CR1_JAUTO);
  }

  if (sConfigInjected->InjectedDiscontinuousConvMode != DISABLE)
  {
    /* Enable the selected ADC injected discontinuous mode */
    hadc->Instance->CR1 |= ADC_CR1_JDISCEN;
  }
  else
  {
    /* Disable the selected ADC injected discontinuous mode */
    hadc->Instance->CR1 &= ~(ADC_CR1_JDISCEN);
  }

  switch(sConfigInjected->InjectedRank)
  {
    case 1U:
      /* Set injected channel 1 offset */
      hadc->Instance->JOFR1 &= ~(ADC_JOFR1_JOFFSET1);
      hadc->Instance->JOFR1 |= sConfigInjected->InjectedOffset;
      break;
    case 2U:
      /* Set injected channel 2 offset */
      hadc->Instance->JOFR2 &= ~(ADC_JOFR2_JOFFSET2);
      hadc->Instance->JOFR2 |= sConfigInjected->InjectedOffset;
      break;
    case 3U:
      /* Set injected channel 3 offset */
      hadc->Instance->JOFR3 &= ~(ADC_JOFR3_JOFFSET3);
      hadc->Instance->JOFR3 |= sConfigInjected->InjectedOffset;
      break;
    default:
      /* Set injected channel 4 offset */
      hadc->Instance->JOFR4 &= ~(ADC_JOFR4_JOFFSET4);
      hadc->Instance->JOFR4 |= sConfigInjected->InjectedOffset;
      break;
  }

  /* Pointer to the common control register to which is belonging hadc    */
  /* (Depending on STM32F4 product, there may be up to 3 ADC and 1 common */
  /* control register)                                                    */
    tmpADC_Common = ADC_COMMON_REGISTER(hadc);

  /* if ADC1 Channel_18 is selected enable VBAT Channel */
  if ((hadc->Instance == ADC1) && (sConfigInjected->InjectedChannel == ADC_CHANNEL_VBAT))
  {
    /* Enable the VBAT channel*/
    tmpADC_Common->CCR |= ADC_CCR_VBATE;
  }

  /* if ADC1 Channel_16 or Channel_17 is selected enable TSVREFE Channel(Temperature sensor and VREFINT) */
  if ((hadc->Instance == ADC1) && ((sConfigInjected->InjectedChannel == ADC_CHANNEL_TEMPSENSOR) || (sConfigInjected->InjectedChannel == ADC_CHANNEL_VREFINT)))
  {
    /* Enable the TSVREFE channel*/
    tmpADC_Common->CCR |= ADC_CCR_TSVREFE;
  }

  return HAL_OK;
}
//}}}

//{{{
HAL_StatusTypeDef HAL_ADCEx_MultiModeStart_DMA (ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length) {

  /* Check if ADC peripheral is disabled in order to enable it and wait during Tstab time the ADC's stabilization */
  if ((hadc->Instance->CR2 & ADC_CR2_ADON) != ADC_CR2_ADON) {
    /* Enable the Peripheral */
    __HAL_ADC_ENABLE(hadc);

    uint32_t counter  = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000U));
    while (counter != 0U)
      counter--;
    }

  /* Start conversion if ADC is effectively enabled */
  if (HAL_IS_BIT_SET(hadc->Instance->CR2, ADC_CR2_ADON)) {
    /* Set ADC state                                                          */
    /* - Clear state bitfield related to regular group conversion results     */
    /* - Set state bitfield related to regular group operation                */
    ADC_STATE_CLR_SET(hadc->State,
                      HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR, HAL_ADC_STATE_REG_BUSY);

    /* If conversions on group regular are also triggering group injected,  update ADC state */
    if (READ_BIT(hadc->Instance->CR1, ADC_CR1_JAUTO) != RESET)
      ADC_STATE_CLR_SET (hadc->State, HAL_ADC_STATE_INJ_EOC, HAL_ADC_STATE_INJ_BUSY);

    /* State machine update: Check if an injected conversion is ongoing */
    if (HAL_IS_BIT_SET(hadc->State, HAL_ADC_STATE_INJ_BUSY))
      /* Reset ADC error code fields related to conversions on group regular */
      CLEAR_BIT(hadc->ErrorCode, (HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA));
    else
      /* Reset ADC all error code fields */
      ADC_CLEAR_ERRORCODE (hadc);

    /* Set the DMA transfer complete callback */
    hadc->DMA_Handle->XferCpltCallback = ADC_MultiModeDMAConvCplt;

    /* Set the DMA half transfer complete callback */
    hadc->DMA_Handle->XferHalfCpltCallback = ADC_MultiModeDMAHalfConvCplt;

    /* Set the DMA error callback */
    hadc->DMA_Handle->XferErrorCallback = ADC_MultiModeDMAError ;

    /* Manage ADC and DMA start: ADC overrun interruption, DMA start, ADC     */
    /* start (in case of SW start):                                           */
    /* Clear regular group conversion flag and overrun flag */
    /* (To ensure of no unknown state from potential previous ADC operations) */
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_EOC);

    /* Enable ADC overrun interrupt */
    __HAL_ADC_ENABLE_IT(hadc, ADC_IT_OVR);

    ADC_Common_TypeDef* tmpADC_Common = ADC_COMMON_REGISTER(hadc);
    if (hadc->Init.DMAContinuousRequests != DISABLE)
      /* Enable the selected ADC DMA request after last transfer */
      tmpADC_Common->CCR |= ADC_CCR_DDS;
    else
      /* Disable the selected ADC EOC rising on each regular channel conversion */
      tmpADC_Common->CCR &= ~ADC_CCR_DDS;

    /* Enable the DMA Stream */
    HAL_DMA_Start_IT (hadc->DMA_Handle, (uint32_t)&tmpADC_Common->CDR, (uint32_t)pData, Length);

    /* if no external trigger present enable software conversion of regular channels */
    if ((hadc->Instance->CR2 & ADC_CR2_EXTEN) == RESET)
      /* Enable the selected ADC software conversion for regular group */
      hadc->Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
    }

  return HAL_OK;
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_ADCEx_MultiModeStop_DMA (ADC_HandleTypeDef* hadc) {

  HAL_StatusTypeDef tmp_hal_status = HAL_OK;

  /* Stop potential conversion on going, on regular and injected groups */
  /* Disable ADC peripheral */
  __HAL_ADC_DISABLE(hadc);

  // Pointer to the common control register to which is belonging hadc    */
  // (Depending on STM32F4 product, there may be up to 3 ADC and 1 common control register
  ADC_Common_TypeDef* tmpADC_Common = ADC_COMMON_REGISTER(hadc);

  /* Check if ADC is effectively disabled */
  if (HAL_IS_BIT_CLR(hadc->Instance->CR2, ADC_CR2_ADON)) {
    /* Disable the selected ADC DMA mode for multimode */
    tmpADC_Common->CCR &= ~ADC_CCR_DDS;

    /* Disable the DMA channel (in case of DMA in circular mode or stop while */
    /* DMA transfer is on going)                                              */
    tmp_hal_status = HAL_DMA_Abort(hadc->DMA_Handle);

    /* Disable ADC overrun interrupt */
    __HAL_ADC_DISABLE_IT(hadc, ADC_IT_OVR);

    /* Set ADC state */
    ADC_STATE_CLR_SET(hadc->State,
                      HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY, HAL_ADC_STATE_READY);
    }

  return tmp_hal_status;
  }
//}}}
//{{{
uint32_t HAL_ADCEx_MultiModeGetValue (ADC_HandleTypeDef* hadc) {

  ADC_Common_TypeDef* tmpADC_Common = ADC_COMMON_REGISTER (hadc);
  return tmpADC_Common->CDR;
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_ADCEx_MultiModeConfigChannel (ADC_HandleTypeDef* hadc, ADC_MultiModeTypeDef* multimode) {

  ADC_Common_TypeDef* tmpADC_Common = ADC_COMMON_REGISTER(hadc);

  /* Set ADC mode */
  tmpADC_Common->CCR &= ~(ADC_CCR_MULTI);
  tmpADC_Common->CCR |= multimode->Mode;

  /* Set the ADC DMA access mode */
  tmpADC_Common->CCR &= ~(ADC_CCR_DMA);
  tmpADC_Common->CCR |= multimode->DMAAccessMode;

  /* Set delay between two sampling phases */
  tmpADC_Common->CCR &= ~(ADC_CCR_DELAY);
  tmpADC_Common->CCR |= multimode->TwoSamplingDelay;

  return HAL_OK;
  }
//}}}
