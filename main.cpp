// main.cpp - sharp lcd testbed
//{{{  includes
#include "common/utils.h"
#include "common/cPointRect.h"
#include "common/font.h"

#include "stm32f4xx.h"
//}}}

//{{{
class cLcd {
public:
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
  #define clearByte   0x20 // unused
  #define vcomByte    0x40 // unused
  #define commandByte 0x80

  #define POLY_X(Z)  ((int32_t)((points + Z)->x))
  #define POLY_Y(Z)  ((int32_t)((points + Z)->y))
  #define ABS(X)     ((X) > 0 ? (X) : -(X))
  //}}}
  enum eDraw { eInvert, eOff, eOn };
  //{{{
  void init() {

    // config CS, DISP - GPIOB
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // CS, DISP init lo
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = CS_PIN | DISP_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

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
    if (HAL_TIM_Base_Init (&timHandle))
      printf ("HAL_TIM_Base_Init failed\n");
    //}}}
    //{{{  init timer timOcInit
    TIM_OC_InitTypeDef timOcInit;
    timOcInit.OCMode       = TIM_OCMODE_PWM1;
    timOcInit.OCPolarity   = TIM_OCPOLARITY_HIGH;
    timOcInit.OCFastMode   = TIM_OCFAST_DISABLE;
    timOcInit.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    timOcInit.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    timOcInit.OCIdleState  = TIM_OCIDLESTATE_RESET;
    timOcInit.Pulse = 10000 /2;
    if (HAL_TIM_PWM_ConfigChannel (&timHandle, &timOcInit, TIM_CHANNEL_4))
      printf ("HAL_TIM_PWM_ConfigChannel failed\n");
    //}}}
    if (HAL_TIM_PWM_Start (&timHandle, TIM_CHANNEL_4))
      printf ("HAL_TIM_PWM_Start failed\n");

    //{{{  config SPI2 tx
    // config SPI2 GPIOB
    GPIO_InitStruct.Pin = SCK_PIN | MOSI_PIN;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

    __HAL_RCC_SPI2_CLK_ENABLE();

    // set SPI2 master, mode0, 8bit, LSBfirst, NSS pin high, baud rate
    SPI_HandleTypeDef SPI_Handle;
    mSpiHandle.Instance = SPI2;
    mSpiHandle.Init.Mode = SPI_MODE_MASTER;
    mSpiHandle.Init.Direction = SPI_DIRECTION_2LINES;
    mSpiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    mSpiHandle.Init.CLKPolarity = SPI_POLARITY_LOW; // SPI mode0
    mSpiHandle.Init.CLKPhase = SPI_PHASE_1EDGE;     // SPI mode0
    mSpiHandle.Init.NSS = SPI_NSS_SOFT;
    mSpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; // 168mHz/2 / 8 = 10.5mHz
    mSpiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    mSpiHandle.Init.TIMode = SPI_TIMODE_DISABLE;
    mSpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    mSpiHandle.Init.CRCPolynomial = 7;
    HAL_SPI_Init (&mSpiHandle);

    // config tx dma
    mSpiTxDma.Instance                 = DMA1_Stream4;
    mSpiTxDma.Init.Channel             = DMA_CHANNEL_0;
    mSpiTxDma.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    mSpiTxDma.Init.PeriphInc           = DMA_PINC_DISABLE;
    mSpiTxDma.Init.MemInc              = DMA_MINC_ENABLE;
    mSpiTxDma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    mSpiTxDma.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    mSpiTxDma.Init.Mode                = DMA_NORMAL;
    mSpiTxDma.Init.Priority            = DMA_PRIORITY_LOW;
    mSpiTxDma.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    mSpiTxDma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    mSpiTxDma.Init.MemBurst            = DMA_MBURST_INC4;
    mSpiTxDma.Init.PeriphBurst         = DMA_PBURST_INC4;
    HAL_DMA_Init (&mSpiTxDma);
    __HAL_LINKDMA (&mSpiHandle, hdmatx, mSpiTxDma);

    HAL_NVIC_SetPriority (DMA1_Stream4_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ (DMA1_Stream4_IRQn);

    HAL_NVIC_SetPriority (SPI2_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ (SPI2_IRQn);

    // SPI2 enable
    SPI2->CR1 |= SPI_CR1_SPE;
    //}}}

    // init frameBuf command : 240 * (lineByte:15bytes:padding) : padding
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

  const uint16_t getWidth() { return 400; }
  const uint16_t getHeight() { return 240; }
  cPoint getCentre() { return cPoint (getWidth()/2, getHeight()/2); }

  int getFrameNum() { return mFrameNum; }

  //{{{
  void clear (eDraw draw) {

    if (draw == eInvert)
      printf ("cLcd::clear - eInvert clear\n");

    // point to first line pixel bytes
    auto framePtr = mFrameBuf + 2;
    for (int y = 0; y < getHeight(); y++) {
      memset (framePtr, draw == eOn ? 0xFF : 0, getWidth()/8);
      framePtr += getPitch();
      }
    }
  //}}}
  //{{{
  void fillRect (eDraw draw, cRect rect) {

    // clip x
    if (rect.left < 0)
      rect.left = 0;
    else if (rect.right > getWidth())
      rect.right = getWidth();
    if (rect.left >= rect.right)
      return;

    // clip y
    if (rect.top < 0)
      rect.top = 0;
    else if (rect.bottom > getHeight())
      rect.bottom = getHeight();
    if (rect.top >= rect.bottom)
      return;

    uint8_t firstByte = rect.left / 8;
    uint8_t lastByte = (rect.right-1) / 8;

    if (firstByte == lastByte) {
      // simple single x byte case
      uint8_t mask = kFirstMask[rect.left & 7] & kLastMask[(rect.right-1) & 7];
      auto framePtr = getFramePtr (rect.top) + firstByte;
      for (uint16_t y = rect.top; y < rect.bottom; y++) {
        *framePtr ^= mask;
        framePtr += getPitch();
        }
      }
    else {
      // multiple x bytes
      uint8_t firstMask = kFirstMask[rect.left & 7];
      uint8_t lastMask = kLastMask[(rect.right-1) & 7];

      auto framePtr = getFramePtr (rect.top) + firstByte + firstByte;
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
  void fillRect (eDraw draw, uint16_t x, uint16_t y, uint16_t width, uint16_t height) {
    fillRect (draw, cRect (x, y, x+width, y+height));
    }
  //}}}
  //{{{
  void drawString (eDraw draw, const std::string& str, cRect rect) {

    const font_t* font = &font18;

    for (auto ch : str) {
      if ((ch >= font->firstChar) && (ch <= font->lastChar)) {
        auto fontChar = (fontChar_t*)(font->glyphsBase + font->glyphOffsets[ch - font->firstChar]);
        auto charBytes = (uint8_t*)fontChar + 5;

        int16_t xfirst = rect.left + fontChar->left;
        auto framePtr = getFramePtr (rect.top + font->height - fontChar->top) + xfirst/8;

        for (int16_t y = 0; y < fontChar->height; y++) {
          int16_t x = xfirst;
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
          framePtr += getPitch() - (fontChar->width + 7)/8;
          }
        rect.left += fontChar->advance;
        }
      else
        rect.left += font->spaceWidth;
      }
    }
  //}}}
  //{{{
  void drawCircle (eDraw draw, cPoint centre, int16_t radius) {

    int32_t decision = 3 - (radius << 1);

    cPoint p = {0, radius};
    while (p.x <= p.y) {
      drawPix (draw, centre.x + p.x, centre.y - p.y);
      drawPix (draw, centre.x - p.x, centre.y - p.y);
      drawPix (draw, centre.x + p.y, centre.y - p.x);
      drawPix (draw, centre.x - p.y, centre.y - p.x);
      drawPix (draw, centre.x + p.x, centre.y + p.y);
      drawPix (draw, centre.x - p.x, centre.y + p.y);
      drawPix (draw, centre.x + p.y, centre.y + p.x);
      drawPix (draw, centre.x - p.y, centre.y + p.x);

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
  void drawLine (eDraw draw, cPoint p1, cPoint p2) {

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
      drawPix (draw, x, y);
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
  void fillCircle (eDraw draw, cPoint centre, uint16_t radius) {

    int32_t decision = 3 - (radius << 1);

    uint32_t current_x = 0;
    uint32_t current_y = radius;

    while (current_x <= current_y) {
      if (current_y > 0) {
        fillRect (draw, centre.x - current_y, centre.y + current_x, 1, 2*current_y);
        fillRect (draw, centre.x - current_y, centre.y - current_x, 1, 2*current_y);
        }
      if (current_x > 0) {
        fillRect (draw, centre.x - current_x, centre.y - current_y, 1, 2*current_x);
        fillRect (draw, centre.x - current_x, centre.y + current_y, 1, 2*current_x);
        }
      if (decision < 0)
        decision += (current_x << 2) + 6;
      else {
        decision += ((current_x - current_y) << 2) + 10;
        current_y--;
        }
      current_x++;
      }

    drawCircle (draw, centre, radius);
    }
  //}}}
  //{{{
  void drawEllipse (eDraw draw, cPoint centre, cPoint radius) {

    int x = 0;
    int y = -radius.y;

    int err = 2 - 2 * radius.x;
    float k = (float)radius.y / (float)radius.x;

    do {
      drawPix (draw, (centre.x - (int16_t)(x / k)), centre.y + y);
      drawPix (draw, (centre.x + (int16_t)(x / k)), centre.y + y);
      drawPix (draw, (centre.x + (int16_t)(x / k)), centre.y - y);
      drawPix (draw, (centre.x - (int16_t)(x / k)), centre.y - y);

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
  void fillEllipse (eDraw draw, cPoint centre, cPoint radius) {

    int x = 0;
    int y = -radius.y;
    int err = 2 - 2 * radius.x;
    float k = (float)radius.y / (float)radius.x;

    do {
      fillRect (draw, (centre.x - (int16_t)(x/k)), (centre.y + y), 1, (2 * (int16_t)(x / k) + 1));
      fillRect (draw, (centre.x - (int16_t)(x/k)), (centre.y - y), 1, (2 * (int16_t)(x / k) + 1));

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
  void drawPolygon (eDraw draw, cPoint* points, uint16_t numPoints) {

    int16_t x = 0, y = 0;

    if (numPoints < 2)
      return;

    drawLine (draw, *points, *(points + numPoints-1));

    while (--numPoints) {
      cPoint point = *points++;
      drawLine (draw, point, *points);
      }
    }
  //}}}
  //{{{
  void fillPolygon (eDraw draw, cPoint* points, uint16_t numPoints) {

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

      fillTriangle (draw, p1, p2, centre);
      fillTriangle (draw, p1, centre, p2);
      fillTriangle (draw, centre, p2, p1);
      }

    fillTriangle (draw, first, p2, centre);
    fillTriangle (draw, first, centre, p2);
    fillTriangle (draw, centre, p2, first);
    }
  //}}}
  //{{{
  void fillTriangle (eDraw draw, cPoint p1, cPoint p2, cPoint p3) {

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
      drawLine (draw, p, p3);
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
  // commandByte | 240 * (lineByte | 50bytes,400pixels | paddingByte 0) | paddingByte 0

    // CS hi
    GPIOB->BSRR = CS_PIN;

    if (HAL_SPI_Transmit (&mSpiHandle, mFrameBuf, 1 + getHeight() * getPitch() + 1, 100))
      printf ("HAL_SPI_Transmit failed\n");
    //while (HAL_SPI_GetState(&mSpiHandle) != HAL_SPI_STATE_READY) {}
    //while (SPI2->SR & SPI_FLAG_BSY);

    // CS lo
    GPIOB->BSRR = CS_PIN << 16;

    mFrameNum++;
    }
  //}}}

  // for irqs
  SPI_HandleTypeDef* getSpiHandle() { return &mSpiHandle; }

private:
  const uint8_t kFirstMask[8] = { 0xFF, 0x7F, 0x3F, 0x1F, 0x0f, 0x07, 0x03, 0x01 };
  const uint8_t kLastMask[8] = { 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF };

  uint16_t getPitch() { return 2 + getWidth()/8; }
  uint8_t* getFramePtr (int16_t y) { return mFrameBuf + 2 + y*getPitch(); }

  //{{{
  void drawPix (bool white, uint16_t x, uint16_t y) {

    uint8_t mask = 0x80 >> (x & 7);
    auto framePtr = getFramePtr (y) + x/8;
    *framePtr++ ^= mask;
    }
  //}}}

  SPI_HandleTypeDef mSpiHandle;
  DMA_HandleTypeDef mSpiTxDma;

  // commandByte | 240 * (lineAddressByte | 50bytes,400pixels | paddingByte0) | paddingByte0
  uint8_t mFrameBuf [1 + (((400/8) + 2) * 240) + 1];
  int mFrameNum = 0;
  };
//}}}
//{{{
class cAdc {
public:
  //{{{
  void init() {

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    // ADC Channel GPIO pin configuration
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // config ADC dma - right channel,stream?
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
    __HAL_LINKDMA (&mAdcHandle, DMA_Handle, hdma_adc);

    // NVIC configuration for DMA transfer complete interrupt
    HAL_NVIC_SetPriority (DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ (DMA2_Stream0_IRQn);

    // config ADC1
    mAdcHandle.Instance                   = ADC1;
    mAdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4;
    mAdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;
    mAdcHandle.Init.ScanConvMode          = DISABLE;  // Sequencer disabled - ADC conversion on 1 channel on rank 1
    mAdcHandle.Init.ContinuousConvMode    = ENABLE;   // Continuous mode enabled to have continuous conversion
    mAdcHandle.Init.DiscontinuousConvMode = DISABLE;  // Parameter discarded because sequencer is disabled
    mAdcHandle.Init.NbrOfDiscConversion   = 0;
    mAdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;  // Conversion start trigged at each external event
    mAdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
    mAdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    mAdcHandle.Init.NbrOfConversion       = 1;
    mAdcHandle.Init.DMAContinuousRequests = ENABLE;
    mAdcHandle.Init.EOCSelection          = DISABLE;
    HAL_ADC_Init (&mAdcHandle);

    // config ADC channel
    ADC_ChannelConfTypeDef sConfig;
    //sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    //sConfig.Channel = ADC_CHANNEL_VBAT;
    //sConfig.Channel = ADC_CHANNEL_VREFINT;
    sConfig.Channel      = ADC_CHANNEL_1;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES; // ADC_SAMPLETIME_3CYCLES;
    sConfig.Offset       = 0;
    HAL_ADC_ConfigChannel (&mAdcHandle, &sConfig);
    }
  //}}}

  void start() { HAL_ADC_Start (&mAdcHandle); }
  void poll() { HAL_ADC_PollForConversion (&mAdcHandle, 100); }
  uint32_t getValue() { return HAL_ADC_GetValue (&mAdcHandle); }

  ADC_HandleTypeDef mAdcHandle;

private:
  DMA_HandleTypeDef hdma_adc;
  };
//}}}
//{{{
class cRtc {
public:
  //{{{
  void init() {

    // Configue LSE as RTC clock source
    RCC_OscInitTypeDef rccOscInitStruct;
    rccOscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
    rccOscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    rccOscInitStruct.LSEState = RCC_LSE_ON;
    rccOscInitStruct.LSIState = RCC_LSI_OFF;
    if (HAL_RCC_OscConfig (&rccOscInitStruct))
      printf ("HAL_RCC_OscConfig failed\n");

    RCC_PeriphCLKInitTypeDef periphClkInitStruct;
    periphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    periphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig (&periphClkInitStruct))
      printf ("HAL_RCCEx_PeriphCLKConfig failed\n");
    __HAL_RCC_RTC_ENABLE();

    // Configure LSE RTC prescaler and RTC data registers
    mRtcHandle.Instance = RTC;
    mRtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
    mRtcHandle.Init.AsynchPrediv = 0x7F;
    mRtcHandle.Init.SynchPrediv = 0x00FF;
    mRtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
    mRtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    mRtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    __HAL_RTC_RESET_HANDLE_STATE (&mRtcHandle);
    if (HAL_RTC_Init (&mRtcHandle))
      printf ("HAL_RTC_Init failed\n");

    // parse buildTime, buildDate strings
    // time hh:mm:ss      - 8
    // date dd:mmm:yyyy   - 11
    int hours = (mBuildTime[0]  - 0x30) * 10 + (mBuildTime[1] -0x30);
    int minutes = (mBuildTime[3] - 0x30) * 10 + (mBuildTime[4] -0x30);
    int seconds = (mBuildTime[6] - 0x30) * 10 + (mBuildTime[7] -0x30);
    int date = ((mBuildDate[4] == ' ') ? 0 : mBuildDate[4] - 0x30) * 10 + (mBuildDate[5] -0x30);
    int year = (mBuildDate[9] - 0x30) * 10 + (mBuildDate[10] -0x30);
    int month = 0;
    for (int i = 0; i < 12; i++)
      if ((mBuildDate[0] == *kMonth[i]) && (mBuildDate[1] == *(kMonth[i]+1)) && (mBuildDate[2] == *(kMonth[i]+2))) {
        month = i;
        break;
        }

    getDateTime (&mRtcHandle, &mRtcDate, &mRtcTime);
    uint32_t clockDateTime = ((((mRtcDate.Year*12 + mRtcDate.Month)*31 + mRtcDate.Date)*24 +
                                 mRtcTime.Hours)*60 + mRtcTime.Minutes)*60 + mRtcTime.Seconds;
    uint32_t buildDateTime = ((((year*12 + month)*31 + date)*24 + hours)*60 + minutes)*60 + seconds;
    if (clockDateTime < buildDateTime+7) {
      //{{{  set clockDateTime from buildDateTime
      printf ("cRtc::init set clockDateTime < buildDateTime %d < %d\n", clockDateTime, buildDateTime);

      mRtcTime.TimeFormat = RTC_HOURFORMAT12_AM;
      mRtcTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
      mRtcTime.StoreOperation = RTC_STOREOPERATION_RESET;

      buildDateTime += 7;
      mRtcTime.Seconds = buildDateTime % 60;
      buildDateTime /= 60;
      mRtcTime.Minutes = buildDateTime % 60;
      buildDateTime /= 60;
      mRtcTime.Hours = buildDateTime % 24;

      buildDateTime /= 24;
      mRtcDate.Date = buildDateTime % 31;
      buildDateTime /= 31;
      mRtcDate.Month = buildDateTime % 12;
      buildDateTime /= 12;
      mRtcDate.Year = buildDateTime;

      mRtcDate.WeekDay = RTC_WEEKDAY_FRIDAY;  // wrong

      setDateTime (&mRtcHandle, &mRtcDate, &mRtcTime);
      // Writes a data in a RTC Backup data Register0
      //HAL_RTCEx_BKUPWrite (&mRtcHandle, RTC_BKP_DR0, 0x32F2);
      }
      //}}}
    }
  //}}}

  //{{{
  float getClockHourAngle() {

    getDateTime (&mRtcHandle, &mRtcDate, &mRtcTime);
    return (1.f - ((mRtcTime.Hours + (mRtcTime.Minutes / 60.f)) / 6.f)) * kPi;
    }
  //}}}
  //{{{
  float getClockMinuteAngle() {

    getDateTime (&mRtcHandle, &mRtcDate, &mRtcTime);
    return (1.f - ((mRtcTime.Minutes + (mRtcTime.Seconds / 60.f))/ 30.f)) * kPi;
    }
  //}}}
  //{{{
  float getClockSecondAngle() {

    getDateTime (&mRtcHandle, &mRtcDate, &mRtcTime);
    return (1.f - (mRtcTime.Seconds / 30.f)) * kPi;
    }
  //}}}
  //{{{
  std::string getClockTimeString() {

    getDateTime (&mRtcHandle, &mRtcDate, &mRtcTime);

    return dec(mRtcTime.Hours,2) + ":" + dec(mRtcTime.Minutes,2) + ":" + dec(mRtcTime.Seconds,2) + " " +
           kMonth[mRtcDate.Month] + " " + dec(mRtcDate.Date,2) + " " + dec(2000 + mRtcDate.Year,4) + " " +
           dec(mRtcTime.SubSeconds) + " " + dec(mRtcTime.SecondFraction);
    }
  //}}}
  //{{{
  std::string getBuildTimeString() {

    return mBuildTime + " "  + mBuildDate;
    }
  //}}}

private:
  //{{{
  void getDateTime (RTC_HandleTypeDef* hrtc, RTC_DateTypeDef* date, RTC_TimeTypeDef* time) {

    time->SubSeconds = RTC->SSR;
    time->SecondFraction = RTC->PRER & RTC_PRER_PREDIV_S;

    uint32_t tr = RTC->TR;
    time->TimeFormat = (tr & (RTC_TR_PM)) >> 16U;
    time->Hours = bcd2ToByte ((tr & (RTC_TR_HT | RTC_TR_HU)) >> 16U);
    time->Minutes = bcd2ToByte ((tr & (RTC_TR_MNT | RTC_TR_MNU)) >> 8U);
    time->Seconds = bcd2ToByte (tr & (RTC_TR_ST | RTC_TR_SU));

    uint32_t dr = RTC->DR;
    date->Year = bcd2ToByte ((dr & (RTC_DR_YT | RTC_DR_YU)) >> 16U);
    date->WeekDay = (dr & (RTC_DR_WDU)) >> 13U;
    date->Month = bcd2ToByte ((dr & (RTC_DR_MT | RTC_DR_MU)) >> 8U);
    date->Date = bcd2ToByte (dr & (RTC_DR_DT | RTC_DR_DU));
    }
  //}}}
  //{{{
  void setDateTime (RTC_HandleTypeDef* hrtc, RTC_DateTypeDef* date, RTC_TimeTypeDef* time) {

    __HAL_RTC_WRITEPROTECTION_DISABLE (hrtc);

    // Set Initialization mode
    if (RTC_EnterInitMode (hrtc) == HAL_OK) {
      __HAL_RTC_WRITEPROTECTION_DISABLE (hrtc);
      if ((RTC->CR & RTC_CR_FMT) == (uint32_t)RESET)
        time->TimeFormat = 0x00U;

      if ((date->Month & 0x10U) == 0x10U)
        date->Month = (uint8_t)((date->Month & (uint8_t)~(0x10U)) + (uint8_t)0x0AU);

      // Set the RTC_DR register
      uint32_t tmp = ((uint32_t)byteToBcd2(date->Year) << 16U) |
                     ((uint32_t)byteToBcd2(date->Month) << 8U) |
                     ((uint32_t)byteToBcd2(date->Date))        |
                     ((uint32_t)date->WeekDay << 13U);
      RTC->DR = (uint32_t)(tmp & RTC_DR_RESERVED_MASK);

      // Set the RTC_TR register
      tmp = (uint32_t)(((uint32_t)byteToBcd2(time->Hours) << 16U) |
                       ((uint32_t)byteToBcd2(time->Minutes) << 8U) |
                       ((uint32_t)byteToBcd2(time->Seconds)) |
                      (((uint32_t)time->TimeFormat) << 16U));
      RTC->TR = (uint32_t)(tmp & RTC_TR_RESERVED_MASK);

      // Clear the bits to be configured
      RTC->CR &= (uint32_t)~RTC_CR_BCK;

      // Configure the RTC_CR register
      RTC->CR |= (uint32_t)(time->DayLightSaving | time->StoreOperation);

      // Exit Initialization mode
      RTC->ISR &= (uint32_t)~RTC_ISR_INIT;

      // If CR_BYPSHAD bit = 0, wait for synchro else this check is not needed
      if ((RTC->CR & RTC_CR_BYPSHAD) == RESET)
        if (HAL_RTC_WaitForSynchro(hrtc) != HAL_OK){}
      }

    __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
    }
  //}}}
  //{{{
  uint8_t byteToBcd2 (uint8_t value) {

    uint32_t bcdHigh = 0U;
    while (value >= 10U) {
      bcdHigh++;
      value -= 10U;
      }

    return  ((uint8_t)(bcdHigh << 4U) | value);
    }
  //}}}
  //{{{
  uint8_t bcd2ToByte (uint8_t value) {

    uint32_t tmp = ((uint8_t)(value & (uint8_t)0xF0) >> (uint8_t)0x4) * 10;
    return (tmp + (value & (uint8_t)0x0F));
    }
  //}}}

  const float kPi = 3.1415926f;
  const char* kMonth[12] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

  RTC_HandleTypeDef mRtcHandle;
  RTC_TimeTypeDef mRtcTime;
  RTC_DateTypeDef mRtcDate;
  std::string mBuildTime = __TIME__;
  std::string mBuildDate = __DATE__;
  };
//}}}

//{{{
class cApp : public cLcd {
public:
  cApp (bool pinReset, bool powerOnReset) : mPinReset(pinReset), mPowerOnReset(powerOnReset) {}
  //{{{
  void init() {

    cLcd::init();
    mAdc.init();
    mRtc.init();
    }
  //}}}
  //{{{
  void run() {

    auto lastTicks = HAL_GetTick();

    mAdc.start();
    while (true) {
      mAdc.poll();
      auto value = mAdc.getValue();
      mValues[getFrameNum() % kMaxValues] = value;
      if (value < mMinValue)
        mMinValue = value;
      if (value > mMaxValue)
        mMaxValue = value;

      //auto vdd = 1000.f * 1.2f / (value / 4096.f);
      auto vdd = 1000.f * 2.f * 2.93f * value / 4096.f;

      if (mAverageVdd == 0.f)
        mAverageVdd = vdd;
      else
        mAverageVdd = ((mAverageVdd * 99.f) + vdd) / 100.f;

      auto ticks = HAL_GetTick();

      clear (eOn);
      drawString (eInvert, dec(mMinValue) + "min " + dec(value)    + " " + dec(mMaxValue) + "max " +
                           dec(int(mAverageVdd) / 1000) + "." + dec(int(mAverageVdd) % 1000, 3) + "v " +
                           dec(ticks - lastTicks) + " " +
                           (mPowerOnReset ? "pow ": " ") + " " + (mPinReset ? "pin" : ""),
                  cRect (0, 0, getWidth(), 20));
      lastTicks = ticks;

      drawString (eInvert, mRtc.getClockTimeString(), cRect (0, 20, getWidth(), 40));
      drawString (eInvert, mRtc.getBuildTimeString(), cRect (0, 40, getWidth(), 60));

      drawClock();
      drawValues();

      present();

      if (getFrameNum() % 500 == 1)
        printf ("cApp::run loop %d %d\n", ticks, getFrameNum());
      }
    }
  //}}}

  static cApp* mApp;
  cAdc mAdc;

private:
  //{{{
  void drawClock() {

    cPoint centre = { 400-42, 42 };
    int16_t radius = 40;
    drawCircle (eInvert, centre, radius);

    float hourRadius = radius * 0.7f;
    float hourAngle = mRtc.getClockHourAngle();
    drawLine (eInvert, centre, centre + cPoint (int16_t(hourRadius * sin (hourAngle)), int16_t(hourRadius * cos (hourAngle))));

    float minuteRadius = radius * 0.8f;
    float minuteAngle = mRtc.getClockMinuteAngle();
    drawLine (eInvert, centre, centre + cPoint (int16_t(minuteRadius * sin (minuteAngle)), int16_t(minuteRadius * cos (minuteAngle))));

    float secondRadius = radius * 0.9f;
    float secondAngle = mRtc.getClockSecondAngle();
    drawLine (eInvert, centre, centre + cPoint (int16_t(secondRadius * sin (secondAngle)), int16_t(secondRadius * cos (secondAngle))));
    }
  //}}}
  //{{{
  void drawValues() {

    int valueIndex = getFrameNum() - kMaxValues;
    for (int i = 0; i < kMaxValues; i++) {
      int16_t len = valueIndex > 0 ? (kMaxLen * (mValues[valueIndex % kMaxValues] - mMinValue))  / (mMaxValue - mMinValue) : 0;
      fillRect (eInvert, cRect (i, getHeight()-len, i+1, getHeight()));
      valueIndex++;
      }
    }
  //}}}

  static const int kMaxLen = 220;
  static const int kMaxValues = 400;

  cRtc mRtc;
  bool mPowerOnReset;
  bool mPinReset;

  uint32_t mMinValue = 4096;
  uint32_t mMaxValue = 0;
  uint32_t mValues[kMaxValues];
  float mAverageVdd = 0;
  };
//}}}
cApp* cApp::mApp = nullptr;

//{{{
extern "C" {
  // sysTick
  void SysTick_Handler() { HAL_IncTick(); }

  // lcd irq
  void SPI2_IRQHandler() { HAL_SPI_IRQHandler (cApp::mApp->getSpiHandle()); }
  void DMA1_Stream4_IRQHandler() { HAL_DMA_IRQHandler (cApp::mApp->getSpiHandle()->hdmatx); }

  // adc irq
  void DMA2_Stream0_IRQHandler() { HAL_DMA_IRQHandler (cApp::mApp->mAdc.mAdcHandle.DMA_Handle); }
  }
//}}}

int main() {
  printf ("main started\n");

  HAL_Init();
  bool pinReset = __HAL_RCC_GET_FLAG (RCC_FLAG_PINRST) != RESET;
  bool powerOnReset = __HAL_RCC_GET_FLAG (RCC_FLAG_PORRST) != RESET;
  __HAL_RCC_CLEAR_RESET_FLAGS();

  printf ("start cApp pin:%d power:%d\n", pinReset, powerOnReset);
  cApp::mApp = new cApp (pinReset, powerOnReset);
  cApp::mApp->init();
  cApp::mApp->run();

  return 0;
  }
