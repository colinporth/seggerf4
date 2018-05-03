// main.cpp - sharp lcd testbed
//{{{  includes
#include "common/utils.h"
#include "common/cPointRect.h"
#include "common/font.h"

#include "stm32f4xx.h"
//}}}
//{{{  defines
#define SCK_PIN     GPIO_PIN_13  //  SPI2  PB13  SCK
#define MOSI_PIN    GPIO_PIN_15  //  SPI2  PB15  MOSI
#define CS_PIN      GPIO_PIN_12  //  SPI2  PB12  CS/NSS active hi
#define DISP_PIN    GPIO_PIN_14  //  GPIO  PB14  DISP active hi
#define VCOM_PIN    GPIO_PIN_11  //  GPIO  PB11  VCOM - TIM2 CH4 1Hz flip
//    xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//    x  GND   EXTMODE   5v   VCOM   MOSI   3.3v  x
//    x  GND     5v     DISP   CS    SCLK    GND  x
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
#define paddingByte 0x00
#define commandByte 0x80
#define vcomByte    0x40
#define clearByte   0x20
//}}}

extern "C" { void SysTick_Handler() { HAL_IncTick(); } }

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
    SPI_Handle.Instance = SPI2;
    SPI_Handle.Init.Mode = SPI_MODE_MASTER;
    SPI_Handle.Init.Direction = SPI_DIRECTION_2LINES;
    //SPI_Handle.Init.Direction = SPI_DIRECTION_1LINE;
    SPI_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
    SPI_Handle.Init.CLKPolarity = SPI_POLARITY_LOW; // SPI mode0
    SPI_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;     // SPI mode0
    SPI_Handle.Init.NSS = SPI_NSS_SOFT;
    SPI_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; // 168mHz/2 / 8 = 10.5mHz
    SPI_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    SPI_Handle.Init.TIMode = SPI_TIMODE_DISABLE;
    SPI_Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    SPI_Handle.Init.CRCPolynomial = 7;
    HAL_SPI_Init (&SPI_Handle);

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
  void setPixel (bool white, int16_t x, int16_t y) {

    if ((x < getWidth()) && (y < getHeight())) {
      auto framePtr = mFrameBuf + (y * getPitch()) + 1 + (x/8);
      uint8_t xMask = 0x80 >> (x & 7);
      if (white)
        *framePtr |= xMask;
      else
        *framePtr &= ~xMask;
      }
    }
  //}}}
  //{{{
  void clearScreen (bool white) {

    if (white)
      drawRect (true, 0, 0, getWidth(), getHeight());
    else
      clearScreenBlack();
    }
  //}}}
  //{{{
  void drawRect (bool white, int16_t xorg, int16_t yorg, uint16_t xlen, uint16_t ylen) {

    uint16_t xend = xorg + xlen - 1;
    if (xend >= getWidth())
      xend = getWidth() - 1;

    uint16_t yend = yorg + ylen - 1;
    if (yend >= getHeight())
      yend = getHeight() - 1;

    uint8_t xFirstByte = xorg/8;
    uint8_t xFirstMask = 0x80 >> (xorg & 7);

    for (uint16_t y = yorg; y <= yend; y++) {
      auto framePtr = mFrameBuf + (y * getPitch()) + 1 + xFirstByte;
      uint8_t xmask = xFirstMask;
      for (uint16_t x = xorg; x <= xend; x++) {
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

    drawLines (yorg, yend);
    }
  //}}}
  //{{{
  void drawString (bool white, const std::string& str, int16_t xorg, int16_t yorg, uint16_t xlen, uint16_t ylen) {

    const font_t* font = &font18;
    int16_t x = xorg;
    int16_t y = yorg;

    for (auto ch : str) {
      if ((ch >= font->firstChar) && (ch <= font->lastChar)) {
        auto glyphData = (uint8_t*)(font->glyphsBase + font->glyphOffsets[ch - font->firstChar]);

        uint8_t width = (uint8_t)*glyphData++;
        uint8_t height = (uint8_t)*glyphData++;
        int8_t left = (int8_t)*glyphData++;
        uint8_t top = (uint8_t)*glyphData++;
        uint8_t advance = (uint8_t)*glyphData++;

        for (int16_t j = y + font->height - top; j < y + font->height - top + height; j++) {
          uint8_t glyphByte;
          for (int16_t i = 0; i < width; i++) {
            if (i % 8 == 0)
              glyphByte = *glyphData++;
            if (glyphByte & 0x80)
              setPixel (white, x+left+i, j);
            glyphByte <<= 1;
            }
          }
        x += advance;
        }
      else
        x += font->spaceWidth;
      }

    drawLines (yorg, yorg + ylen - 1);
    }
  //}}}

private:
  //{{{
  void clearScreenBlack() {

    // CS hi
    GPIOB->BSRR = CS_PIN;

    // clearScreen
    SPI2->DR = clearByte;
    while (!(SPI2->SR & SPI_FLAG_TXE));
    SPI2->DR = paddingByte;
    while (SPI2->SR & SPI_FLAG_BSY);

    // CS lo
    GPIOB->BSRR = CS_PIN << 16;

    // clear mFrameBuf, each line has leading lineAddress and training paddingByte
    memset (mFrameBuf, 0, getPitch()*getHeight());
    for (uint16_t y = 0; y < getHeight(); y++) {
      uint8_t lineAddress = y+1;
      lineAddress = (lineAddress & 0xF0) >> 4 | (lineAddress & 0x0F) << 4;
      lineAddress = (lineAddress & 0xCC) >> 2 | (lineAddress & 0x33) << 2;
      lineAddress = (lineAddress & 0xAA) >> 1 | (lineAddress & 0x55) << 1;
      mFrameBuf [y*getPitch()] = lineAddress;
      mFrameBuf [(y*getPitch()) + 1 + (getWidth()/8)] = paddingByte;
      }
    }
  //}}}
  //{{{
  void drawLines (uint16_t yorg, uint16_t yend) {

    if (yorg < getHeight()) {
      GPIOB->BSRR = CS_PIN;

      // leading command byte
      SPI2->DR = commandByte;
      while (!(SPI2->SR & SPI_FLAG_TXE));

      auto mFrameBufPtr = mFrameBuf + (yorg * getPitch());
      for (int i = 0; i < (yend-yorg+1) * getPitch(); i++) {
        SPI2->DR = *mFrameBufPtr++;
        while (!(SPI2->SR & SPI_FLAG_TXE));
        }

      // trainig second padding byte
      SPI2->DR = paddingByte;
      while (SPI2->SR & SPI_FLAG_BSY);

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

  while (1) {
    for (int j = 0; j < 200; j++) {
      for (int i = 0; i < cLcd::getHeight(); i++) {
        lcd->drawRect (true, 0, i*20 + 2, 400, 18);
        lcd->drawRect (false, 0, i*20, 400, 2);
        lcd->drawString (false, "helloColin long piece of text " + dec(i) + " " + dec(i*20), j + i*10, i*20, 300, 20);
        }
      //HAL_Delay (100);
      }
    }

  return 0;
  }
