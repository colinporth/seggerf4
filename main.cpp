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

#define kPi 3.1415926f
#define POLY_X(Z)  ((int32_t)((points + Z)->x))
#define POLY_Y(Z)  ((int32_t)((points + Z)->y))
#define ABS(X)     ((X) > 0 ? (X) : -(X))
//}}}
const char* kMonth[12] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

SPI_HandleTypeDef SpiHandle;
DMA_HandleTypeDef hdma_tx;
ADC_HandleTypeDef AdcHandle;
DMA_HandleTypeDef hdma_adc;

//{{{
extern "C" {
  void SysTick_Handler() { HAL_IncTick(); }
  void DMA2_Stream0_IRQHandler() { HAL_DMA_IRQHandler (AdcHandle.DMA_Handle); }
  void DMA1_Stream4_IRQHandler() { HAL_DMA_IRQHandler (SpiHandle.hdmatx); }
  void SPI2_IRQHandler() { HAL_SPI_IRQHandler (&SpiHandle); }
  //{{{
  void HAL_ADC_MspInit (ADC_HandleTypeDef* hadc) {

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

    mFrameBuf [0] = commandByte;
    memset (mFrameBuf+1, 0, (getPitch() * getHeight()) + 1);
    for (uint16_t y = 0; y < getHeight(); y++) {
      uint8_t lineByte = y+1;
      // bit reverse
      lineByte = (lineByte & 0xF0) >> 4 | (lineByte & 0x0F) << 4;
      lineByte = (lineByte & 0xCC) >> 2 | (lineByte & 0x33) << 2;
      lineByte = (lineByte & 0xAA) >> 1 | (lineByte & 0x55) << 1;
      mFrameBuf [1 + (y*getPitch())] = lineByte;
      }
    present();

    // enable DISP hi
    GPIOB->BSRR = DISP_PIN;
    }
  //}}}

  static const uint16_t getWidth() { return 400; }
  static const uint16_t getWidthBytes() { return getWidth() / 8; }
  static const uint16_t getHeight() { return 240; }
  static cPoint getCentre() { return cPoint (getWidth()/2, getHeight()/2); }

  int getFrameNum() { return mFrameNum; }
  //{{{
  void clear (bool white) {

    auto framePtr = mFrameBuf + 1 + 1;
    for (int y = 0; y < getHeight(); y++) {
      memset (framePtr, white ? 0xFF : 0, getWidthBytes());
      framePtr += getPitch();
      }
    }
  //}}}
  //{{{
  void fillRect (bool white, cRect rect) {

    const uint8_t kFirstMask[8] = { 0xFF, 0x7F, 0x3F, 0x1F, 0x0f, 0x07, 0x03, 0x01 };
    const uint8_t  kLastMask[8] = { 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF };

    // clip
    if (rect.left < 0)
      rect.left = 0;
    else if (rect.right > getWidth())
      rect.right = getWidth();
    if (rect.left >= rect.right)
      return;

    if (rect.top < 0)
      rect.top = 0;
    else if (rect.bottom > getHeight())
      rect.bottom = getHeight();

    uint8_t firstByte = rect.left / 8;
    uint8_t lastByte = (rect.right-1) / 8;

    if (firstByte == lastByte) {
      // simple single x byte case
      uint8_t mask = kFirstMask[rect.left & 7] & kLastMask[(rect.right-1) & 7];
      auto framePtr = mFrameBuf + 1 + (rect.top * getPitch()) + 1 + firstByte;
      for (uint16_t y = rect.top; y < rect.bottom; y++) {
          *framePtr ^= mask;
        framePtr += getPitch();
        }
      }
    else {
      // multiple x bytes
      uint8_t firstMask = kFirstMask[rect.left & 7];
      uint8_t lastMask = kLastMask[(rect.right-1) & 7];

      auto framePtr = mFrameBuf + 1 + (rect.top * getPitch()) + 1 + firstByte;
      for (uint16_t y = rect.top; y < rect.bottom; y++) {
        uint8_t byte = firstByte;
        *framePtr++ ^= firstMask;
        byte++;

        while (byte < lastByte) {
          *framePtr++ ^= 0xFF;
          byte++;
          }

        if (byte == lastByte)
          *framePtr++ ^= lastMask;

        framePtr += getPitch() - (lastByte - firstByte) - 1;
        }
     }
    }
  //}}}
  //{{{
  void fillRect (uint16_t color, uint16_t x, uint16_t y, uint16_t width, uint16_t height) {
    fillRect (color, cRect (x, y, x+width, y+height));
    }
  //}}}
  //{{{
  void drawString (bool white, const std::string& str, cRect rect) {

    const font_t* font = &font18;

    for (auto ch : str) {
      if ((ch >= font->firstChar) && (ch <= font->lastChar)) {
        auto fontChar = (fontChar_t*)(font->glyphsBase + font->glyphOffsets[ch - font->firstChar]);
        auto charBytes = (uint8_t*)fontChar + 5;

        for (int16_t yPix = rect.top + font->height - fontChar->top;
             yPix < rect.top + font->height - fontChar->top + fontChar->height && yPix < getHeight(); yPix++) {

          int16_t x = rect.left + fontChar->left;
          auto framePtr = mFrameBuf + 1 + (yPix * getPitch()) + 1 + (x / 8);

          if (true) {
            int16_t charBits = fontChar->width;
            while (charBits > 0) {
              uint8_t xbit = x & 7;
              if (xbit) {
                *framePtr++ ^= (*charBytes) >> xbit;
                *framePtr ^= (*charBytes) << (8 - xbit);
                }
              else
                *framePtr++ ^= *charBytes;
              charBits -= 8;
              charBytes++;
              }
            }
          else {
            // not yet
            int16_t charBits = fontChar->width;
            uint8_t charByte = 0;
            while (charBits > 0) {
              uint8_t xbit = x & 7;
              *framePtr++ ^= charByte || ((*charBytes) >> xbit);
              charByte = (*charBytes) << (8 - xbit);
              charBytes++;
              charBits -= 8;
              }
            *framePtr++ ^= charByte;
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
  void drawCircle (bool white, cPoint centre, int16_t radius) {

    int32_t decision = 3 - (radius << 1);

    cPoint p = {0, radius};
    while (p.x <= p.y) {
      drawPix (white, centre.x + p.x, centre.y - p.y);
      drawPix (white, centre.x - p.x, centre.y - p.y);
      drawPix (white, centre.x + p.y, centre.y - p.x);
      drawPix (white, centre.x - p.y, centre.y - p.x);
      drawPix (white, centre.x + p.x, centre.y + p.y);
      drawPix (white, centre.x - p.x, centre.y + p.y);
      drawPix (white, centre.x + p.y, centre.y + p.x);
      drawPix (white, centre.x - p.y, centre.y + p.x);

      if (decision < 0)
        decision += (p.x << 2) + 6;
      else {
        decision += ((p.x - p.y) << 2) + 10;
        p.y--;
        }

      p.x++;
      }

    }
  //}}}
  //{{{
  void fillCircle (bool white, cPoint centre, uint16_t radius) {

    int32_t decision = 3 - (radius << 1);

    uint32_t current_x = 0;
    uint32_t current_y = radius;

    while (current_x <= current_y) {
      if (current_y > 0) {
        fillRect (white, centre.x - current_y, centre.y + current_x, 1, 2*current_y);
        fillRect (white, centre.x - current_y, centre.y - current_x, 1, 2*current_y);
        }
      if (current_x > 0) {
        fillRect (white, centre.x - current_x, centre.y - current_y, 1, 2*current_x);
        fillRect (white, centre.x - current_x, centre.y + current_y, 1, 2*current_x);
        }
      if (decision < 0)
        decision += (current_x << 2) + 6;
      else {
        decision += ((current_x - current_y) << 2) + 10;
        current_y--;
        }
      current_x++;
      }

    drawCircle (white, centre, radius);
    }
  //}}}
  //{{{
  void drawEllipse (bool white, cPoint centre, cPoint radius) {

    int x = 0;
    int y = -radius.y;

    int err = 2 - 2 * radius.x;
    float k = (float)radius.y / (float)radius.x;

    do {
      drawPix (white, (centre.x - (int16_t)(x / k)), centre.y + y);
      drawPix (white, (centre.x + (int16_t)(x / k)), centre.y + y);
      drawPix (white, (centre.x + (int16_t)(x / k)), centre.y - y);
      drawPix (white, (centre.x - (int16_t)(x / k)), centre.y - y);

      int e2 = err;
      if (e2 <= x) {
        err += ++x * 2+ 1 ;
        if (-y == x && e2 <= y)
          e2 = 0;
        }
      if (e2 > y)
        err += ++y *2 + 1;
      } while (y <= 0);
    }
  //}}}
  //{{{
  void fillEllipse (bool white, cPoint centre, cPoint radius) {

    int x = 0;
    int y = -radius.y;
    int err = 2 - 2 * radius.x;
    float k = (float)radius.y / (float)radius.x;

    do {
      fillRect (white, (centre.x - (int16_t)(x/k)), (centre.y + y), 1, (2 * (int16_t)(x / k) + 1));
      fillRect (white, (centre.x - (int16_t)(x/k)), (centre.y - y), 1, (2 * (int16_t)(x / k) + 1));

      int e2 = err;
      if (e2 <= x) {
        err += ++x * 2 + 1;
        if (-y == x && e2 <= y)
          e2 = 0;
        }
      if (e2 > y)
        err += ++y*2+1;
      } while (y <= 0);
    }
  //}}}
  //{{{
  void drawPolygon (bool white, cPoint* points, uint16_t numPoints) {

    int16_t x = 0, y = 0;

    if (numPoints < 2)
      return;

    drawLine (white, *points, *(points + numPoints-1));

    while (--numPoints) {
      cPoint point = *points++;
      drawLine (white, point, *points);
      }
    }
  //}}}
  //{{{
  void fillPolygon (bool white, cPoint* points, uint16_t numPoints) {

    cPoint tl = *points;
    cPoint br = *points;

    cPoint pixel;
    for (int16_t counter = 1; counter < numPoints; counter++) {
      pixel.x = POLY_X (counter);
      if (pixel.x < tl.x)
        tl.x = pixel.x;
      if (pixel.x > br.x)
        br.x = pixel.x;

      pixel.y = POLY_Y(counter);
      if (pixel.y < tl.y)
        tl.y = pixel.y;
      if (pixel.y > br.y)
        br.y = pixel.y;
      }

    if (numPoints < 2)
      return;

    cPoint centre = (tl + br) / 2;
    cPoint first = *points;

    cPoint p2;
    while (--numPoints) {
      cPoint p1 = *points;
      points++;
      p2 = *points;

      fillTriangle (white, p1, p2, centre);
      fillTriangle (white, p1, centre, p2);
      fillTriangle (white, centre, p2, p1);
      }

    fillTriangle (white, first, p2, centre);
    fillTriangle (white, first, centre, p2);
    fillTriangle (white, centre, p2, first);
    }
  //}}}
  //{{{
  void drawLine (bool white, cPoint p1, cPoint p2) {

    int16_t xinc1 = 0, xinc2 = 0, yinc1 = 0, yinc2 = 0;
    int16_t den = 0, num = 0, num_add = 0, num_pixels = 0;

    int16_t deltax = ABS(p2.x - p1.x); // The difference between the x's
    int16_t deltay = ABS(p2.y - p1.y); // The difference between the y's
    int16_t x = p1.x;                       // Start x off at the first pixel
    int16_t y = p1.y;                       // Start y off at the first pixel

    if (p2.x >= p1.x) {
      // The x-values are increasing
      xinc1 = 1;
      xinc2 = 1;
      }
    else {
      // The x-values are decreasing
      xinc1 = -1;
      xinc2 = -1;
      }

    if (p2.y >= p1.y) {
      // The y-values are increasing
      yinc1 = 1;
      yinc2 = 1;
      }
    else {
      // The y-values are decreasing
      yinc1 = -1;
      yinc2 = -1;
      }

    if (deltax >= deltay) { // There is at least one x-value for every y-value
      xinc1 = 0;            // Don't change the x when numerator >= denominator
      yinc2 = 0;            // Don't change the y for every iteration
      den = deltax;
      num = deltax / 2;
      num_add = deltay;
      num_pixels = deltax;  // There are more x-values than y-values
      }
    else {                  // There is at least one y-value for every x-value
      xinc2 = 0;            // Don't change the x for every iteration
      yinc1 = 0;            // Don't change the y when numerator >= denominator
      den = deltay;
      num = deltay / 2;
      num_add = deltax;
      num_pixels = deltay;  // There are more y-values than x-values
      }

    for (int16_t curpixel = 0; curpixel <= num_pixels; curpixel++) {
      drawPix (white, x, y);
      num += num_add;     // Increase the numerator by the top of the fraction
      if (num >= den) {   // Check if numerator >= denominator
        num -= den;       // Calculate the new numerator value
        x += xinc1;       // Change the x as appropriate
        y += yinc1;       // Change the y as appropriate
        }

      x += xinc2;         // Change the x as appropriate
      y += yinc2;         // Change the y as appropriate
      }
    }
  //}}}
  //{{{
  void fillTriangle (bool white, cPoint p1, cPoint p2, cPoint p3) {

    cPoint inc1;
    cPoint inc2;

    if (p2.x >= p1.x) {
      //{{{  x increasing
      inc1.x = 1;
      inc2.x = 1;
      }
      //}}}
    else {
      //{{{  x decreasing
      inc1.x = -1;
      inc2.x = -1;
      }
      //}}}

    if (p2.y >= p1.y) {
      //{{{  y increasing
      inc1.y = 1;
      inc2.y = 1;
      }
      //}}}
    else {
      //{{{  y decreasing
      inc1.y = -1;
      inc2.y = -1;
      }
      //}}}

    int16_t den;
    int16_t num;
    int16_t num_add;
    int16_t num_pixels;

    int16_t deltax = ABS (p2.x - p1.x);  // The difference between the x's
    int16_t deltay = ABS (p2.y - p1.y);  // The difference between the y's
    if (deltax >= deltay) {
      //{{{  at least one x-value for every y-value
      inc1.x = 0;           // Don't change the x when numerator >= denominator
      inc2.y = 0;           // Don't change the y for every iteration

      den = deltax;
      num = deltax / 2;
      num_add = deltay;
      num_pixels = deltax;  // There are more x-values than y-values
      }
      //}}}
    else {
      //{{{  at least one y-value for every x-value
      inc2.x = 0;           // Don't change the x for every iteration
      inc1.y = 0;           // Don't change the y when numerator >= denominator

      den = deltay;
      num = deltay / 2;
      num_add = deltax;
      num_pixels = deltay; // There are more y-values than x-values
      }
      //}}}

    cPoint p = p1;
    for (int16_t curpixel = 0; curpixel <= num_pixels; curpixel++) {
      drawLine (white, p, p3);
      num += num_add;     // Increase the numerator by the top of the fraction
      if (num >= den)  {  // Check if numerator >= denominator
        num -= den;       // Calculate the new numerator value
        p += inc1;       // Change the x as appropriate
        }

      p += inc2;         // Change the x as appropriate
      }
    }
  //}}}
  //{{{
  void present() {

    // CS hi
    GPIOB->BSRR = CS_PIN;

    //  lines - lineByte | 50 bytes 400 bits | padding 0
    auto result = HAL_SPI_Transmit (&SpiHandle, mFrameBuf, 1 + getHeight() * getPitch() + 1, 100);
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

  //{{{
  void drawPix (bool white, uint16_t x, uint16_t y) {

    auto framePtr = mFrameBuf + 1 + (y * getPitch()) + 1 + (x / 8);
    uint8_t mask = 0x80 >> (x & 7);
    *framePtr++ ^= mask;
    }
  //}}}

  uint8_t mFrameBuf [1 + (((400/8) + 2) * 240) + 1];
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

  //{{{  rtc config
  // Configue LSE as RTC clock source
  RCC_OscInitTypeDef rccOscInitStruct;
  rccOscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  rccOscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  rccOscInitStruct.LSEState = RCC_LSE_ON;
  rccOscInitStruct.LSIState = RCC_LSI_OFF;
  auto result = HAL_RCC_OscConfig (&rccOscInitStruct);

  RCC_PeriphCLKInitTypeDef periphClkInitStruct;
  periphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  periphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  result = HAL_RCCEx_PeriphCLKConfig (&periphClkInitStruct);
  __HAL_RCC_RTC_ENABLE();

  // Configure LSE RTC prescaler and RTC data registers
  // RTC configured as follow:
  //  - Hour Format    = Format 24
  //  - Asynch Prediv  = Value according to source clock
  //  - Synch Prediv   = Value according to source clock
  //  - OutPut         = Output Disable
  //  - OutPutPolarity = High Polarity
  //  - OutPutType     = Open Drain */
  RTC_HandleTypeDef rtcHandle;
  rtcHandle.Instance = RTC;
  rtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
  rtcHandle.Init.AsynchPrediv = 0x7F;
  rtcHandle.Init.SynchPrediv = 0x00FF;
  rtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
  rtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  rtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  __HAL_RTC_RESET_HANDLE_STATE (&rtcHandle);
  result = HAL_RTC_Init (&rtcHandle);

  std::string time = __TIME__; // hh:mm:ss      - 8
  std::string date = __DATE__; // dd:mmm:yyyy   - 11
  int hour = (time[0]  - 0x30) * 10 + (time[1] -0x30);
  int min = (time[3] - 0x30) * 10 + (time[4] -0x30);
  int sec = (time[6] - 0x30) * 10 + (time[7] -0x30);
  int day = ((date[4] == ' ') ? 0 : date[4] - 0x30) * 10 + (date[5] -0x30);
  int year = (date[9] - 0x30) * 10 + (date[10] -0x30);
  int mon = 0;
  for (int i = 0; i < 12; i++)
    if ((date[0] == *kMonth[i]) && (date[1] == *(kMonth[i]+1)) && (date[2] == *(kMonth[i]+2))) {
      mon = i;
      break;
      }

  RTC_TimeTypeDef rtcTime;
  HAL_RTC_GetTime (&rtcHandle, &rtcTime, RTC_FORMAT_BIN);
  RTC_DateTypeDef rtcDate;
  HAL_RTC_GetDate (&rtcHandle, &rtcDate, RTC_FORMAT_BIN);

  if ((rtcTime.Hours * 3600 + rtcTime.Minutes * 60 + rtcTime.Seconds) < (hour * 3600 + min * 60 + sec)) {
    //{{{  set date time
    rtcDate.Date = day;
    rtcDate.WeekDay = RTC_WEEKDAY_FRIDAY;
    rtcDate.Month = mon;
    rtcDate.Year = year;
    result = HAL_RTC_SetDate (&rtcHandle, &rtcDate, RTC_FORMAT_BIN);

    // set Time 02:00:00
    rtcTime.Hours = hour;
    rtcTime.Minutes = min;
    rtcTime.Seconds = sec;
    rtcTime.TimeFormat = RTC_HOURFORMAT12_AM;
    rtcTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    rtcTime.StoreOperation = RTC_STOREOPERATION_RESET;
    result = HAL_RTC_SetTime (&rtcHandle, &rtcTime, RTC_FORMAT_BIN);

    // Writes a data in a RTC Backup data Register0
    //HAL_RTCEx_BKUPWrite (&rtcHandle, RTC_BKP_DR0, 0x32F2);
    }
    //}}}
  else {
    //{{{  time ok
    //  check reset flags
    // Check if the Power On Reset flag is set
    if (__HAL_RCC_GET_FLAG (RCC_FLAG_PORRST) != RESET) {
      // Power on reset
      }
    if(__HAL_RCC_GET_FLAG (RCC_FLAG_PINRST) != RESET) {
      // Check if Pin Reset flag is set
      }

    // Clear Reset Flag
    __HAL_RCC_CLEAR_RESET_FLAGS();
    }
    //}}}
  // Check if Data stored in BackUp register0: No Need to reconfigure RTC#, Read the BackUp Register 0 Data
  //}}}
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

    RTC_TimeTypeDef rtcTime;
    HAL_RTC_GetTime (&rtcHandle, &rtcTime, RTC_FORMAT_BIN);
    RTC_DateTypeDef rtcDate;
    HAL_RTC_GetDate (&rtcHandle, &rtcDate, RTC_FORMAT_BIN);
    lcd->drawString (false, dec(rtcTime.Hours,2) + ":" + dec(rtcTime.Minutes,2) + ":" + dec(rtcTime.Seconds,2) + " " +
                            kMonth[rtcDate.Month] + " " + dec(rtcDate.Date,2) + " " + dec(2000 + rtcDate.Year,4) + " " +
                            dec(rtcTime.SubSeconds) + " " + dec(rtcTime.SecondFraction),
                     cRect (0, 20, cLcd::getWidth(), 40));
    //lcd->drawString (false, time + " " + date, cRect (0, 40, cLcd::getWidth(), 60));

    cPoint centre = { 400-42, 42 };
    int16_t radius = 40;
    lcd->drawCircle (false, centre, radius);

    float hourRad = radius * 0.7f;
    float hourAng = (1.f - (rtcTime.Hours / 6.f)) * kPi;
    lcd->drawLine (false, centre, centre + cPoint (int16_t(hourRad * sin (hourAng)), int16_t(hourRad * cos (hourAng))));

    float minRad = radius * 0.8f;
    float minAng = (1.f - (rtcTime.Minutes / 30.f)) * kPi;
    lcd->drawLine (false, centre, centre + cPoint (int16_t(minRad * sin (minAng)), int16_t(minRad * cos (minAng))));

    float secRad = radius * 0.9f;
    float secAng = (1.f - (rtcTime.Seconds / 30.f)) * kPi;
    lcd->drawLine (false, centre, centre + cPoint (int16_t(secRad * sin (secAng)), int16_t(secRad * cos (secAng))));

    int valueIndex = lcd->getFrameNum() - kMaxValues;
    for (int i = 0; i < kMaxValues; i++) {
      int16_t len = valueIndex > 0 ? (kMaxLen * (values[valueIndex % kMaxValues] - minValue))  / (maxValue - minValue) : 0;
      lcd->fillRect (false, cRect (i, cLcd:: getHeight()-len, i+1,  cLcd::getHeight()));
      valueIndex++;
      }

    lcd->present();
    }

  return 0;
  }
