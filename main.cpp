// main.cpp - sharp lcd testbed
//{{{  includes
#include "common/utils.h"
#include "common/cPointRect.h"
#include "common/font.h"

#include "stm32f4xx.h"
//}}}
//{{{  defines
//    xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//    x  GND   EXTMODE   5v   VCOM   MOSI   3.3v  x
//    x  GND     5v     DISP   CS    SCLK    GND  x
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

#define SCK_PIN     GPIO_PIN_13  //  SPI2  PB13  SCK
#define MOSI_PIN    GPIO_PIN_15  //  SPI2  PB15  MOSI
#define CS_PIN      GPIO_PIN_12  //  SPI2  PB12  CS/NSS active hi
#define DISP_PIN    GPIO_PIN_14  //  GPIO  PB14  DISP active hi
#define VCOM_PIN    GPIO_PIN_11  //  GPIO  PB11  VCOM - TIM2 CH4 1Hz flip

#define paddingByte 0x00
#define clearByte   0x20
#define commandByte 0x80
//}}}

SPI_HandleTypeDef SpiHandle;
DMA_HandleTypeDef hdma_tx;
ADC_HandleTypeDef AdcHandle;
DMA_HandleTypeDef  hdma_adc;

//{{{
extern "C" {
  void SysTick_Handler() { HAL_IncTick(); }
  void DMA2_Stream0_IRQHandler() { HAL_DMA_IRQHandler (AdcHandle.DMA_Handle); }
  void DMA1_Stream4_IRQHandler() { HAL_DMA_IRQHandler (SpiHandle.hdmatx); }
  void SPI2_IRQHandler() { HAL_SPI_IRQHandler (&SpiHandle); }
  //{{{
  void HAL_ADC_MspInit (ADC_HandleTypeDef *hadc) {

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    //__HAL_RCC_DMA2_CLK_ENABLE();

    // ADC Channel GPIO pin configuration
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Set the parameters to be configured
    hdma_adc.Instance = DMA2_Stream0;
    hdma_adc.Init.Channel  = DMA_CHANNEL_2;
    hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc.Init.Mode = DMA_CIRCULAR;
    hdma_adc.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_adc.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    hdma_adc.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_adc.Init.PeriphBurst = DMA_PBURST_SINGLE;
    HAL_DMA_Init (&hdma_adc);
    __HAL_LINKDMA (hadc, DMA_Handle, hdma_adc);

    // NVIC configuration for DMA transfer complete interrupt
    HAL_NVIC_SetPriority (DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ (DMA2_Stream0_IRQn);
    }
  //}}}
  }
//}}}
//{{{
class cLcd {
public:
  //{{{
  void init() {

    //  config CS, DISP - GPIOB
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = CS_PIN | DISP_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

    // disable CS, DISP lo
    GPIOB->BSRR = (CS_PIN | DISP_PIN) << 16;

    __HAL_RCC_TIM2_CLK_ENABLE();
    //{{{  config VCOM GPIOB as TIM2 CH4
    GPIO_InitStruct.Pin = VCOM_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;

    HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);
    //}}}
    //{{{  init timer timHandle
    TIM_HandleTypeDef timHandle;

    timHandle.Instance = TIM2;
    timHandle.Init.Period = 10000 - 1;
    uint32_t uwPrescalerValue = (uint32_t) ((SystemCoreClock /2) / 10000) - 1;
    timHandle.Init.Prescaler = uwPrescalerValue;
    timHandle.Init.ClockDivision = 0;
    timHandle.Init.CounterMode = TIM_COUNTERMODE_UP;

    auto result = HAL_TIM_Base_Init (&timHandle);
    //}}}
    //{{{  init timer timOcInit
    TIM_OC_InitTypeDef timOcIn;
    timOcIn.OCMode       = TIM_OCMODE_PWM1;
    timOcIn.OCPolarity   = TIM_OCPOLARITY_HIGH;
    timOcIn.OCFastMode   = TIM_OCFAST_DISABLE;
    timOcIn.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    timOcIn.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    timOcIn.OCIdleState  = TIM_OCIDLESTATE_RESET;
    timOcIn.Pulse = 10000 /2;

    result = HAL_TIM_PWM_ConfigChannel (&timHandle, &timOcIn, TIM_CHANNEL_4);
    //}}}
    result = HAL_TIM_PWM_Start (&timHandle, TIM_CHANNEL_4);

    //{{{  config SPI2 tx
    // config SPI2 GPIOB
    GPIO_InitStruct.Pin = SCK_PIN | MOSI_PIN;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

    __HAL_RCC_SPI2_CLK_ENABLE();

    // set SPI2 master, mode0, 8bit, LSBfirst, NSS pin high, baud rate
    SPI_HandleTypeDef SPI_Handle;
    SpiHandle.Instance = SPI2;
    SpiHandle.Init.Mode = SPI_MODE_MASTER;
    SpiHandle.Init.Direction = SPI_DIRECTION_2LINES;
    //SpiHandle.Init.Direction = SPI_DIRECTION_1LINE;
    SpiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    SpiHandle.Init.CLKPolarity = SPI_POLARITY_LOW; // SPI mode0
    SpiHandle.Init.CLKPhase = SPI_PHASE_1EDGE;     // SPI mode0
    SpiHandle.Init.NSS = SPI_NSS_SOFT;
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; // 168mHz/2 / 8 = 10.5mHz
    SpiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.TIMode = SPI_TIMODE_DISABLE;
    SpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    SpiHandle.Init.CRCPolynomial = 7;
    HAL_SPI_Init (&SpiHandle);

    // config tx dma
    hdma_tx.Instance                 = DMA1_Stream4;
    hdma_tx.Init.Channel             = DMA_CHANNEL_0;
    hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_tx.Init.Mode                = DMA_NORMAL;
    hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_tx.Init.MemBurst            = DMA_MBURST_INC4;
    hdma_tx.Init.PeriphBurst         = DMA_PBURST_INC4;
    HAL_DMA_Init (&hdma_tx);
    __HAL_LINKDMA (&SpiHandle, hdmatx, hdma_tx);

    HAL_NVIC_SetPriority (DMA1_Stream4_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ (DMA1_Stream4_IRQn);

    HAL_NVIC_SetPriority (SPI2_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ (SPI2_IRQn);

    // SPI2 enable
    SPI2->CR1 |= SPI_CR1_SPE;
    //}}}

    memset (mFrameBuf, 0, getPitch() * getHeight());
    for (uint16_t y = 0; y < getHeight(); y++) {
      uint8_t lineByte = y+1;
      // bit reverse
      lineByte = (lineByte & 0xF0) >> 4 | (lineByte & 0x0F) << 4;
      lineByte = (lineByte & 0xCC) >> 2 | (lineByte & 0x33) << 2;
      lineByte = (lineByte & 0xAA) >> 1 | (lineByte & 0x55) << 1;
      mFrameBuf [y*getPitch()] = lineByte;
      }
    present();

    // enable DISP hi
    GPIOB->BSRR = DISP_PIN;
    }
  //}}}

  static const uint16_t getWidth() { return 400; }
  static const uint16_t getWidthBytes() { return 400 / 8; }
  static const uint16_t getHeight() { return 240; }

  int getFrameNum() { return mFrameNum; }
  //{{{
  void drawRect (bool white, cRect rect) {

    if (rect.left < 0)
      rect.left = 0;
    if (rect.right > getWidth())
      rect.right = getWidth();
    if (rect.top < 0)
      rect.top = 0;
    if (rect.bottom > getHeight())
      rect.bottom = getHeight();

    uint8_t xFirstByte = rect.left / 8;
    uint8_t xFirstMask = 0x80 >> (rect.left & 7);

    for (uint16_t y = rect.top; y < rect.bottom; y++) {
      auto framePtr = mFrameBuf + (y * getPitch()) + 1 + xFirstByte;
      uint8_t xmask = xFirstMask;
      for (uint16_t x = rect.left; x < rect.right; x++) {
        if (white)
          *framePtr |= xmask;
        else
          *framePtr &= ~xmask;
        xmask >>= 1;
        if (xmask == 0) {
          xmask = 0x80;
          framePtr++;
          };
        }
      }
    }
  //}}}
  //{{{
  void drawString (bool white, const std::string& str, cRect rect) {

    const font_t* font = &font18;

    for (auto ch : str) {
      if ((ch >= font->firstChar) && (ch <= font->lastChar)) {
        auto fontChar = (fontChar_t*)(font->glyphsBase + font->glyphOffsets[ch - font->firstChar]);
        auto charData = (uint8_t*)fontChar + 5;

        for (int16_t yPix = rect.top + font->height - fontChar->top;
             yPix < rect.top + font->height - fontChar->top + fontChar->height && yPix < getHeight(); yPix++) {
          uint8_t charByte;
          for (int16_t bit = 0; bit < fontChar->width; bit++) {
            if (bit % 8 == 0)
              charByte = *charData++;
            if (charByte & 0x80) {
              int16_t xPix = rect.left + fontChar->left + bit;
              if (xPix >= getWidth())
                break;
              auto framePtr = mFrameBuf + (yPix * getPitch()) + 1 + (xPix/8);
              uint8_t xMask = 0x80 >> (xPix & 7);
              if (white)
                *framePtr |= xMask;
              else
                *framePtr &= ~xMask;
              }
            charByte <<= 1;
            }
          }
        rect.left += fontChar->advance;
        }
      else
        rect.left += font->spaceWidth;
      }
    }
  //}}}

  //{{{
  void clear (bool white) {

    auto framePtr = mFrameBuf + 1;
    for (int y = 0; y < getHeight(); y++) {
      memset (framePtr, white ? 0xFF : 0, getWidthBytes());
      framePtr += getPitch();
      }
    }
  //}}}
  //{{{
  void present() {

    // CS hi
    GPIOB->BSRR = CS_PIN;

    uint8_t byte = commandByte;
   auto result = HAL_SPI_Transmit (&SpiHandle, &byte, 1, 1);
    //while (HAL_SPI_GetState(&SpiHandle) != HAL_SPI_STATE_READY) {}

    //  lines - lineByte | 50 bytes 400 bits | padding 0
    result = HAL_SPI_Transmit (&SpiHandle, mFrameBuf, getHeight() * getPitch(), 100);
    //while (HAL_SPI_GetState(&SpiHandle) != HAL_SPI_STATE_READY) {}

    byte = paddingByte;
    result = HAL_SPI_Transmit (&SpiHandle, &byte, 1, 1);
    //while (HAL_SPI_GetState(&SpiHandle) != HAL_SPI_STATE_READY) {}

    // wait for all sent
    //while (SPI2->SR & SPI_FLAG_BSY);

    // CS lo
    GPIOB->BSRR = CS_PIN << 16;

    mFrameNum++;
    }
  //}}}

private:
  static const uint16_t getPitch() { return (400 / 8) + 2; }

  uint8_t mFrameBuf [((400/8) + 2) * 240];
  bool mCom = false;
  int mFrameNum = 0;
  };
//}}}
//{{{
void adcInit() {

  AdcHandle.Instance                   = ADC1;
  AdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4;
  AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;
  AdcHandle.Init.ScanConvMode          = DISABLE;  // Sequencer disabled - ADC conversion on 1 channel on rank 1
  AdcHandle.Init.ContinuousConvMode    = ENABLE;   // Continuous mode enabled to have continuous conversion
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;  // Parameter discarded because sequencer is disabled
  AdcHandle.Init.NbrOfDiscConversion   = 0;
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;  // Conversion start trigged at each external event
  AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.NbrOfConversion       = 1;
  AdcHandle.Init.DMAContinuousRequests = ENABLE;
  AdcHandle.Init.EOCSelection          = DISABLE;
  HAL_ADC_Init (&AdcHandle);

  ADC_ChannelConfTypeDef sConfig;
  //sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  //sConfig.Channel = ADC_CHANNEL_VBAT;
  //sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Channel      = ADC_CHANNEL_1;
  sConfig.Rank         = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES; // ADC_SAMPLETIME_3CYCLES;
  sConfig.Offset       = 0;

  //Configure ADC Temperature Sensor Channel
  //sConfig.Rank = 1;
  //sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  //sConfig.Offset = 0;

  HAL_ADC_ConfigChannel (&AdcHandle, &sConfig);
  }
//}}}

const int kMaxValues = 400;
const int kMaxLen = 220;
uint32_t values[kMaxValues];

int main() {

  HAL_Init();
  adcInit();
  auto lcd = new cLcd();
  lcd->init();

  HAL_ADC_Start (&AdcHandle);

  float averageVdd = 0;
  uint32_t minValue = 4096;
  uint32_t maxValue = 0;

  auto lastTicks = HAL_GetTick();
  while (1) {
    HAL_ADC_PollForConversion (&AdcHandle, 100);
    auto value = HAL_ADC_GetValue (&AdcHandle);
    values[lcd->getFrameNum() % kMaxValues] = value;
    if (value < minValue)
      minValue = value;
    if (value > maxValue)
      maxValue = value;

    //auto vdd = 1000.f * 1.2f / (value / 4096.f);
    auto vdd = 1000.f * 2.f * 2.93f * value / 4096.f;

    if (averageVdd == 0.f)
      averageVdd = vdd;
    else
      averageVdd = ((averageVdd * 99.f) + vdd) / 100.f;

    auto ticks = HAL_GetTick();

    lcd->clear (true);
    lcd->drawString (false, dec(minValue) + "min " + dec(value)    + " " + dec(maxValue) + "max " +
                            dec(int(averageVdd) / 1000) + "." + dec(int(averageVdd) % 1000, 3) + "v " +
                            dec(ticks - lastTicks),
                     cRect (0, 0, cLcd::getWidth(), 20));
    lastTicks = ticks;

    int valueIndex = lcd->getFrameNum() - kMaxValues;
    for (int i = 0; i < kMaxValues; i++) {
      int16_t len = valueIndex > 0 ? (kMaxLen * (values[valueIndex % kMaxValues] - minValue))  / (maxValue - minValue) : 0;
      lcd->drawRect (false, cRect (i, cLcd:: getHeight()-len, i+1,  cLcd::getHeight()));
      valueIndex++;
      }

    lcd->present();
    }

  return 0;
  }
