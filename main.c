#include <stdio.h>
#include <stdlib.h>

#include "stm32f4xx.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"

#define VCOM_PIN    GPIO_PIN_2 // GPIO PA2 = VCOM normally flipping
#define DISP_PIN    GPIO_PIN_3 // GPIO PA3 = DISP normally hi
#define CS_PIN      GPIO_PIN_4 // GPIO PA4 = CS   normally lo, could be AF SPI NSS
//                                SPI1 PA5 = SCK  AF
//                                SPI1 PA7 = MOSI AF
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

static int vcom;
static uint8_t frameBuf [TFTPITCH*TFTHEIGHT];
//font_t font18;

uint16_t getWidth() { return TFTWIDTH; }
uint16_t getHeight() { return TFTHEIGHT; }
//{{{
uint8_t reverseByte (uint8_t b) {

  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
  }
//}}}
//{{{
void writeByte (uint8_t byte) {

  SPI1->DR = byte;
  while (!(SPI1->SR & SPI_FLAG_TXE));
  }
//}}}

//{{{
void toggleVcom() {
  if (vcom) {
    GPIOA->BSRR = VCOM_PIN << 16;
    vcom = 0;
    }
  else {
    GPIOA->BSRR = VCOM_PIN;
    vcom = 1;
    }
  }
//}}}
//{{{
void clearScreenOff() {
  // CS hi, clearScreen, CS lo
  GPIOA->BSRR = CS_PIN;

  writeByte (clearByte);
  writeByte (paddingByte);

  while (SPI1->SR & SPI_FLAG_BSY);
  GPIOA->BSRR = CS_PIN << 16;
  }
//}}}

//{{{
void setPixel (uint16_t colour, int16_t x, int16_t y) {

  if ((x < TFTWIDTH) && (y < TFTHEIGHT)) {
    uint8_t* framePtr = frameBuf + (y * TFTPITCH) + 1 + (x/8);
    uint8_t xMask = 0x80 >> (x & 7);
    if (colour)
      *framePtr |= xMask;
    else
      *framePtr &= ~xMask;
    }
  }
//}}}
//{{{
void drawLines (uint16_t yorg, uint16_t yend) {

  if (yorg < TFTHEIGHT) {
    GPIOA->BSRR = CS_PIN;

    writeByte (commandByte);

    uint8_t* frameBufPtr = frameBuf + (yorg * TFTPITCH);
    for (int i = 0; i < (yend-yorg+1) * TFTPITCH; i++)
      writeByte (*frameBufPtr++);

    writeByte (paddingByte);
    while (SPI1->SR & SPI_FLAG_BSY);

    GPIOA->BSRR = CS_PIN << 16;
    }

  toggleVcom();
  }
//}}}
//{{{
void drawRect (int black, int16_t xorg, int16_t yorg, uint16_t xlen, uint16_t ylen) {

  uint16_t xend = xorg + xlen - 1;
  if (xend >= TFTWIDTH) 
    xend = TFTWIDTH - 1;

  uint16_t yend = yorg + ylen - 1;
  if (yend >= TFTHEIGHT) 
    yend = TFTHEIGHT - 1;

  uint8_t xFirstByte = xorg/8;
  uint8_t xFirstMask = 0x80 >> (xorg & 7);

  for (uint16_t y = yorg; y <= yend; y++) {
    uint8_t* framePtr = frameBuf + (y * TFTPITCH) + 1 + xFirstByte;
    uint8_t xmask = xFirstMask;
    for (uint16_t x = xorg; x <= xend; x++) {
      if (black)
        *framePtr &= ~xmask;
      else
        *framePtr |= xmask;
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
void clearScreen (uint16_t colour) {

  drawRect (colour, 0, 0, getWidth(), getHeight());
  }
//}}}

//{{{
void displayInit() {

  // enable clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_SPI1_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = VCOM_PIN | DISP_PIN | CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  // VCOM, CS, DISP lo
  GPIOA->BSRR = (VCOM_PIN | DISP_PIN | CS_PIN) << 16;

  // enable GPIO AF SPI pins
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  // set SPI1 master, mode0, 8bit, LSBfirst, NSS pin high, baud rate
  SPI_HandleTypeDef SPI_Handle;
  SPI_Handle.Instance = SPI1;
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

  // SPI1 enable
  SPI1->CR1 |= SPI_CR1_SPE;

  clearScreenOff();

  // DISP hi
  GPIOA->BSRR = DISP_PIN;

  // frameBuf stuffed with lineAddress and line end paddingByte
  for (uint16_t y = 0; y < TFTHEIGHT; y++) {
    frameBuf [y*TFTPITCH] = reverseByte (y+1);
    frameBuf [(y*TFTPITCH) + 1 + (TFTWIDTH/8)] = paddingByte;
    }
  }
//}}}

void main() {

  displayInit();
  clearScreen (0);
  for (int i = 0; i < 8; i++)
    drawRect (1, i*400/8,i*240/8, 400/8, 240/8);
  }
