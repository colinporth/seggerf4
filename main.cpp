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
DMA_HandleTypeDef hdma_rx;

extern "C" {
  void SysTick_Handler() { HAL_IncTick(); } }
  void DMA1_Stream3_IRQHandler() { HAL_DMA_IRQHandler (SpiHandle.hdmarx); }
  void DMA1_Stream4_IRQHandler() { HAL_DMA_IRQHandler (SpiHandle.hdmatx); }
  void SPI2_IRQHandler() { HAL_SPI_IRQHandler (&SpiHandle);
  }
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

    //{{{  config tx dma
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

    HAL_DMA_Init(&hdma_tx);

    __HAL_LINKDMA (&SpiHandle, hdmatx, hdma_tx);
    //}}}
    //{{{  config rx dma
    hdma_rx.Instance                 = DMA1_Stream3;
    hdma_rx.Init.Channel             = DMA_CHANNEL_0;
    hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_rx.Init.Mode                = DMA_NORMAL;
    hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_rx.Init.MemBurst            = DMA_MBURST_INC4;
    hdma_rx.Init.PeriphBurst         = DMA_PBURST_INC4;
    HAL_DMA_Init (&hdma_rx);
    __HAL_LINKDMA (&SpiHandle, hdmarx, hdma_rx);
    //}}}

    HAL_NVIC_SetPriority (DMA1_Stream4_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ (DMA1_Stream4_IRQn);

    HAL_NVIC_SetPriority (DMA1_Stream3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ (DMA1_Stream3_IRQn);

    HAL_NVIC_SetPriority (SPI2_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ (SPI2_IRQn);

    // SPI2 enable
    SPI2->CR1 |= SPI_CR1_SPE;

    clearScreenBlack();

    // enable DISP hi
    GPIOB->BSRR = DISP_PIN;
    }
  //}}}

  static const uint16_t getWidth() { return 400; }
  static const uint16_t getPitch() { return (getWidth() / 8) + 2; }
  static const uint16_t getHeight() { return 240; }

  //{{{
  void clearScreen (bool white) {

    if (white)
      drawRect (true, cRect (0, 0, getWidth(), getHeight()));
    else
      clearScreenBlack();
    }
  //}}}
  //{{{
  void drawRect (bool white, cRect rect) {

    if (rect.right > getWidth())
      rect.right = getWidth();
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

    drawLines (rect.top, rect.bottom);
    }
  //}}}
  //{{{
  void drawString (bool white, const std::string& str, cRect rect) {

    const font_t* font = &font18;

    for (auto ch : str) {
      if ((ch >= font->firstChar) && (ch <= font->lastChar)) {
        auto fontChar = (fontChar_t*)(font->glyphsBase + font->glyphOffsets[ch - font->firstChar]);
        auto charData = (uint8_t*)fontChar + 5;

        for (int16_t yPix = rect.top + font->height - fontChar->top; yPix < rect.top + font->height - fontChar->top + fontChar->height; yPix++) {
          uint8_t charByte;
          for (int16_t bit = 0; bit < fontChar->width; bit++) {
            if (bit % 8 == 0)
              charByte = *charData++;
            if (charByte & 0x80) {
              int16_t xPix = rect.left + fontChar->left + bit;
              if ((xPix < getWidth()) && (yPix < getHeight())) {
                auto framePtr = mFrameBuf + (yPix * getPitch()) + 1 + (xPix/8);
                uint8_t xMask = 0x80 >> (xPix & 7);
                if (white)
                  *framePtr |= xMask;
                else
                  *framePtr &= ~xMask;
                }
              }
            charByte <<= 1;
            }
          }
        rect.left += fontChar->advance;
        }
      else
        rect.left += font->spaceWidth;
      }

    drawLines (rect.top, rect.bottom);
    }
  //}}}

private:
  //{{{
  void clearScreenBlack() {

    // CS hi
    GPIOB->BSRR = CS_PIN;

    // clearScreen command
    SPI2->DR = clearByte;
    while (!(SPI2->SR & SPI_FLAG_TXE));
    SPI2->DR = paddingByte;
    while (SPI2->SR & SPI_FLAG_BSY);

    // CS lo
    GPIOB->BSRR = CS_PIN << 16;

    // set mFrameBuf, 240 x line  -  lineByte | 50 bytes 400 bits | padding 0
    memset (mFrameBuf, 0, getPitch() * getHeight());
    for (uint16_t y = 0; y < getHeight(); y++) {
      uint8_t lineByte = y+1;
      // bit reverse
      lineByte = (lineByte & 0xF0) >> 4 | (lineByte & 0x0F) << 4;
      lineByte = (lineByte & 0xCC) >> 2 | (lineByte & 0x33) << 2;
      lineByte = (lineByte & 0xAA) >> 1 | (lineByte & 0x55) << 1;
      mFrameBuf [y*getPitch()] = lineByte;
      }
    }
  //}}}
  //{{{
  void drawLines (int16_t top, int16_t bottom) {

    if ((top >= 0) && (bottom <= getHeight())) {
      // CS hi
      GPIOB->BSRR = CS_PIN;

      // command byte
      //SPI2->DR = commandByte;
      //while (!(SPI2->SR & SPI_FLAG_TXE));
      //while (SPI2->SR & SPI_FLAG_BSY);

      uint8_t byte = commandByte;
      auto result = HAL_SPI_Transmit (&SpiHandle, &byte, 1, 1000);

      //  lines - lineByte | 50 bytes 400 bits | padding 0
      auto mFrameBufPtr = mFrameBuf + (top * getPitch());

      //auto result = HAL_SPI_Transmit_DMA (&SpiHandle, mFrameBufPtr, (bottom -top) * getPitch());
      result = HAL_SPI_Transmit (&SpiHandle, mFrameBufPtr, (bottom-top) * getPitch(), 1000);
      //while (HAL_SPI_GetState(&SpiHandle) != HAL_SPI_STATE_READY) {}

      //for (int i = 0; i < (bottom -top) * getPitch(); i++) {
        // wait for empty
      //  while (!(SPI2->SR & SPI_FLAG_TXE));
      //  SPI2->DR = *mFrameBufPtr++;
      //  }

      // wait for empty
      //while (!(SPI2->SR & SPI_FLAG_TXE));

      // terminate by second padding byte
      //SPI2->DR = paddingByte;
      byte = paddingByte;
      result = HAL_SPI_Transmit (&SpiHandle, &byte, 1, 1000);

      // wait for all sent
      while (SPI2->SR & SPI_FLAG_BSY);

      // CS lo
      GPIOB->BSRR = CS_PIN << 16;
      }
    }
  //}}}

  uint8_t mFrameBuf [((400/8) + 2) * 240];
  bool mCom = false;
  };
//}}}


int main() {

  HAL_Init();

  auto lcd = new cLcd();
  lcd->init();

  while (1)
    for (int j = 0; j < 200; j++)
      for (int i = 0; i < cLcd::getHeight(); i += 20) {
        lcd->drawRect (false, cRect (0, i, 400, i + 2));
        lcd->drawRect (true, cRect (0, i + 2, 400, i+20));
        lcd->drawString (false, "helloColin long piece of text " + dec(i),
                         cRect (j+i, i, j+i + 300, i+20));
        }

  return 0;
  }
