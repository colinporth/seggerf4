// main.c - sharp lcd testbed
//{{{  includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f4xx.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"

#include "font.h"
//}}}
//{{{  defines
//                                 SPI2 PB13  SCK
//                                 SPI2 PB15  MOSI
#define CS_PIN      GPIO_PIN_12 // SPI2 PB12  CS/NSS active hi
#define DISP_PIN    GPIO_PIN_14 // GPIO PB14  DISP active hi
#define VCOM_PIN    GPIO_PIN_11 // GPIO PB11  VCOM normally flipping
//    xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//    x  GND   EXTMODE   5v   VCOM   MOSI   3.3v  x
//    x  GND     5v     DISP   CS    SCLK    GND  x
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
#define paddingByte 0x00
#define commandByte 0x80
#define vcomByte    0x40
#define clearByte   0x20

#define TFTWIDTH    400 // 96
#define TFTHEIGHT   240 // 96
#define TFTPITCH    ((TFTWIDTH/8)+2)  // line has lineNum, byte packed bit data, padding
//}}}


__IO uint32_t DelayCount;
extern "C" {
  //{{{
  void SysTick_Handler() {
    if (DelayCount != 0x00)
      DelayCount--;
    }
  //}}}
  }
//{{{
void delayMs (uint32_t ms) {
  DelayCount = ms;
  while (DelayCount != 0) {}
  }
//}}}
//{{{
void delayInit() {
  SysTick->VAL = 0;                       // config sysTick
  SysTick->LOAD = SystemCoreClock / 1000; // - countdown value
  SysTick->CTRL = 0x7;                    // - 0x4 = sysTick HCLK, 0x2 = intEnable, 0x1 = enable
  }
//}}}

//{{{
class cScreen {
public:
  //{{{
  void init() {

    // enable clocks
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = CS_PIN | DISP_PIN | VCOM_PIN ;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

    // disable CS, DISP, VCOM lo
    GPIOB->BSRR = (CS_PIN | DISP_PIN | VCOM_PIN) << 16;

    // enable GPIO AF SPI pins
    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

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
  uint16_t getWidth() { return TFTWIDTH; }
  uint16_t getHeight() { return TFTHEIGHT; }

  //{{{
  void setPixel (bool white, int16_t x, int16_t y) {

    if ((x < TFTWIDTH) && (y < TFTHEIGHT)) {
      auto framePtr = mFrameBuf + (y * TFTPITCH) + 1 + (x/8);
      uint8_t xMask = 0x80 >> (x & 7);
      if (white)
        *framePtr |= xMask;
      else
        *framePtr &= ~xMask;
      }
    }
  //}}}
  //{{{
  void drawRect (bool white, int16_t xorg, int16_t yorg, uint16_t xlen, uint16_t ylen) {

    uint16_t xend = xorg + xlen - 1;
    if (xend >= TFTWIDTH)
      xend = TFTWIDTH - 1;

    uint16_t yend = yorg + ylen - 1;
    if (yend >= TFTHEIGHT)
      yend = TFTHEIGHT - 1;

    uint8_t xFirstByte = xorg/8;
    uint8_t xFirstMask = 0x80 >> (xorg & 7);

    for (uint16_t y = yorg; y <= yend; y++) {
      auto framePtr = mFrameBuf + (y * TFTPITCH) + 1 + xFirstByte;
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
  void drawString (bool white, const char* str, int16_t xorg, int16_t yorg, uint16_t xlen, uint16_t ylen) {

    const font_t* font = &font18;
    int16_t x = xorg;
    int16_t y = yorg;

    do {
      if (*str == ' ')
        x += font->spaceWidth;

      else if ((*str >= font->firstChar) && (*str <= font->lastChar)) {
        auto glyphData = (uint8_t*)(font->glyphsBase + font->glyphOffsets[*str - font->firstChar]);

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
      } while (*(++str));

    drawLines (yorg, yorg+ylen-1);
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
  void toggleVcom() {

    if (mCom)
      GPIOB->BSRR = VCOM_PIN << 16;
    else
      GPIOB->BSRR = VCOM_PIN;

    mCom = !mCom;
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
    memset (mFrameBuf, 0, TFTPITCH*TFTHEIGHT);
    for (uint16_t y = 0; y < TFTHEIGHT; y++) {
      uint8_t lineAddress = y+1;
      lineAddress = (lineAddress & 0xF0) >> 4 | (lineAddress & 0x0F) << 4;
      lineAddress = (lineAddress & 0xCC) >> 2 | (lineAddress & 0x33) << 2;
      lineAddress = (lineAddress & 0xAA) >> 1 | (lineAddress & 0x55) << 1;
      mFrameBuf [y*TFTPITCH] = lineAddress;
      mFrameBuf [(y*TFTPITCH) + 1 + (TFTWIDTH/8)] = paddingByte;
      }
    }
  //}}}
  //{{{
  void drawLines (uint16_t yorg, uint16_t yend) {

    if (yorg < TFTHEIGHT) {
      GPIOB->BSRR = CS_PIN;

      // leading command byte
      SPI2->DR = commandByte;
      while (!(SPI2->SR & SPI_FLAG_TXE));

      auto mFrameBufPtr = mFrameBuf + (yorg * TFTPITCH);
      for (int i = 0; i < (yend-yorg+1) * TFTPITCH; i++) {
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

  uint8_t mFrameBuf [TFTPITCH*TFTHEIGHT];
  bool mCom = false;
  };
//}}}

int main() {

  delayInit();

  auto screen = new cScreen();
  screen->init();

  while (1) {
    for (int j = 0; j < 200; j++) {
      for (int i = 0; i < screen->getHeight(); i++) {
        screen->drawRect (true, 0, i*20 + 2, 400, 18);
        screen->drawRect (false, 0, i*20, 400, 2);
        screen->drawString (false, "helloColin long piece of text", j + i*10, i*20, 300, 20);
        }
      screen->toggleVcom();
      }
    }

  return 0;
  }
