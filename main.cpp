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
  enum eFont { eSmall, eMedium, eBig, eBigger };
  enum eAlign { eLeft, eCentre, eRight };
  //{{{
  void init() {

    // config CS, DISP
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
    __HAL_RCC_DMA1_CLK_ENABLE();

    // set SPI2 master, mode0, 8bit, LSBfirst, NSS pin high, baud rate
    SPI_HandleTypeDef SPI_Handle;
    mSpiHandle.Instance = SPI2;
    mSpiHandle.Init.Mode              = SPI_MODE_MASTER;
    mSpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    mSpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    mSpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW; // SPI mode0
    mSpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;     // SPI mode0
    mSpiHandle.Init.NSS               = SPI_NSS_SOFT;
    mSpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; // 168mHz/2 / 4 = 20.5mHz
    mSpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    mSpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
    HAL_SPI_Init (&mSpiHandle);

    // config tx dma
    mSpiTxDma.Instance = DMA1_Stream4;
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
    const int frameBufLen = 1 + (((getWidth()/8) + 2) * getHeight()) + 1;
    mFrameBuf = (uint8_t*)malloc (frameBufLen);
    if (mFrameBuf) {
      printf ("cLcd:: init alloc %d\n", frameBufLen);

      memset (mFrameBuf+1, 0, frameBufLen);
      mFrameBuf [0] = commandByte;
      for (uint16_t y = 0; y < getHeight(); y++) {
        uint8_t lineAddressByte = y+1;
        // bit reverse
        lineAddressByte = ((lineAddressByte & 0xF0) >> 4) | ((lineAddressByte & 0x0F) << 4);
        lineAddressByte = ((lineAddressByte & 0xCC) >> 2) | ((lineAddressByte & 0x33) << 2);
        lineAddressByte = ((lineAddressByte & 0xAA) >> 1) | ((lineAddressByte & 0x55) << 1);
        mFrameBuf [1 + (y*getPitch())] = lineAddressByte;
        }
      present();
      }
    else
      printf ("cLcd:: init alloc fail\n");

    // enable DISP hi
    GPIOB->BSRR = DISP_PIN;
    }
  //}}}

  const uint16_t getWidth() { return kWidth; }
  const uint16_t getHeight() { return kHeight; }
  const cPoint getSize() { return cPoint (kWidth, kHeight); }
  const cRect getRect() { return cRect (getSize()); }
  const cPoint getCentre() { return getSize()/2; }

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
        if (draw == eOff)
          *framePtr &= ~mask;
        else if (draw == eInvert)
          *framePtr ^= mask;
        else
          *framePtr |= mask;
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

        if (draw == eOff)
          *framePtr++ &= ~firstMask;
        else if (draw == eInvert)
          *framePtr++ ^= firstMask;
        else
          *framePtr++ |= firstMask;
        byte++;

        while (byte < lastByte) {
          if (draw == eOff)
            *framePtr++ = 0x00;
          else if (draw == eInvert)
            *framePtr++ ^= 0xFF;
          else
            *framePtr++ = 0xFF;
          byte++;
          }

        if (byte == lastByte) {
          if (draw == eOff)
            *framePtr++ &= ~lastMask;
          else if (draw == eInvert)
            *framePtr++ ^= lastMask;
          else
            *framePtr++ |= lastMask;
          }
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
  void drawString (eDraw draw, eFont font, eAlign align, const std::string& str, cPoint p) {
  // simple char clip to width

    const font_t* drawFont;
    if (font == eSmall)
      drawFont = &font18;
    else if (font == eMedium)
      drawFont = &font36;
    else if (font == eBig)
      drawFont = &font72;
    else if (font == eBigger)
      drawFont = &font120;

    switch (align) {
      //{{{
      case eCentre: {
        uint16_t size = 0;
        for (auto ch : str)
          size += getCharWidth (drawFont, ch);
        p.x -= size/2;
        if (p.x < 0)
          p.x = 0;
        else if (p.x >= getWidth())
          p.x = 0;
        }
        break;
      //}}}
      //{{{
      case eRight: {
        uint16_t size = 0;
        for (auto ch : str)
          size += getCharWidth (drawFont, ch);
        p.x -= size;
        if (p.x < 0)
          p.x = 0;
        else if (p.x >= getWidth())
          p.x = 0;
        }
        break;
      //}}}
      }

    for (auto ch : str) {
      if ((ch >= drawFont->firstChar) && (ch <= drawFont->lastChar)) {
        auto fontChar = (fontChar_t*)(drawFont->glyphsBase + drawFont->glyphOffsets[ch - drawFont->firstChar]);
        if (p.x + fontChar->left + fontChar->width < getWidth()) {
          auto charBytes = (uint8_t*)fontChar + 5;
          int16_t xfirst = p.x + fontChar->left;
          auto framePtr = getFramePtr (p.y + drawFont->height - fontChar->top) + xfirst/8;

          for (uint16_t y = 0; y < fontChar->height; y++) {
            int16_t x = xfirst;
            int16_t charBits = fontChar->width;
            uint8_t charByte = 0;
            while (charBits > 0) {
              uint8_t xbit = x & 7;
              if (draw == eInvert)
                *framePtr++ ^= charByte | ((*charBytes) >> xbit);
              else if (draw == eOff)
                *framePtr++ &= ~(charByte | ((*charBytes) >> xbit));
              else
                *framePtr++ |= charByte | ((*charBytes) >> xbit);
              charByte = (*charBytes) << (8 - xbit);
              charBytes++;
              charBits -= 8;
              }
            if (draw == eInvert)
              *framePtr ^= charByte;
            else if (draw == eOff)
              *framePtr &= ~charByte;
            else
              *framePtr |= charByte;
            framePtr += getPitch() - (fontChar->width + 7)/8;
            }
          }
        p.x += fontChar->advance;
        }
      else
        p.x += drawFont->spaceWidth;
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

    // CS hi
    GPIOB->BSRR = CS_PIN;

    if (HAL_SPI_Transmit_DMA (&mSpiHandle, mFrameBuf, 1 + getHeight() * getPitch() + 1))
      printf ("HAL_SPI_Transmit failed\n");
    while (HAL_SPI_GetState (&mSpiHandle) != HAL_SPI_STATE_READY)
      HAL_Delay (1);

    // CS lo
    GPIOB->BSRR = CS_PIN << 16;

    mFrameNum++;
    }
  //}}}

  // for irqs
  SPI_HandleTypeDef* getSpiHandle() { return &mSpiHandle; }

private:
  const uint16_t kWidth = 400;
  const uint16_t kHeight = 240;
  const uint8_t kFirstMask[8] = { 0xFF, 0x7F, 0x3F, 0x1F, 0x0f, 0x07, 0x03, 0x01 };
  const uint8_t kLastMask[8] = { 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF };

  uint16_t getPitch() { return 2 + getWidth()/8; }
  uint8_t* getFramePtr (int16_t y) { return mFrameBuf + 2 + y*getPitch(); }
  //{{{
  uint16_t getCharWidth (const font_t* font, char ascii) {
    if ((ascii >= font->firstChar) && (ascii <= font->lastChar)) {
      auto char8 = (uint8_t*)(font->glyphsBase + font->glyphOffsets[ascii - font->firstChar]);
      return char8[4];
      }
    return font->spaceWidth;
    }
  //}}}

  //{{{
  void drawPix (eDraw draw, uint16_t x, uint16_t y) {

    uint8_t mask = 0x80 >> (x & 7);
    auto framePtr = getFramePtr (y) + x/8;

    if (draw == eInvert)
      *framePtr++ ^= mask;
    else if (draw == eOff)
      *framePtr++ &= ~mask;
    else
      *framePtr++ |= mask;
    }
  //}}}

  // frameBuf - commandByte | 240 * (lineAddressByte | 50bytes,400pixels | paddingByte0) | paddingByte0
  uint8_t* mFrameBuf = nullptr;
  int mFrameNum = 0;

  SPI_HandleTypeDef mSpiHandle;
  DMA_HandleTypeDef mSpiTxDma;
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
    writeProtectDisable();
    if (enterInitMode()) {
      //{{{  init rtc
      RTC->CR = RTC_HOURFORMAT_24;
      RTC->PRER = (uint32_t)(0x00FF);
      RTC->PRER |= (uint32_t)(0x7F << 16U);

      // Exit Initialization mode
      RTC->ISR &= (uint32_t)~RTC_ISR_INIT;

      // If CR_BYPSHAD bit = 0, wait for synchro else this check is not needed
      if ((RTC->CR & RTC_CR_BYPSHAD) == RESET)
        if (!waitForSynchro())
          printf ("timeout waiting for synchro\n");

      RTC->TAFCR &= (uint32_t)~RTC_TAFCR_ALARMOUTTYPE;
      RTC->TAFCR |= (uint32_t)RTC_OUTPUT_TYPE_OPENDRAIN;
      }
      //}}}
    writeProtectEnable();

    loadDateTime();
    uint32_t clockDateTimeValue = mDateTime.getValue();

    cDateTime buildDateTime (mBuildDate, mBuildTime);
    uint32_t buildDateTimeValue = buildDateTime.getValue();
    if (clockDateTimeValue < buildDateTimeValue + kBuildSecs) {
      // set clockDateTime from buildDateTime
      mDateTime.setFromValue (buildDateTimeValue + kBuildSecs);
      printf ("cRtc::init set clock < build %d < %d\n", clockDateTimeValue, buildDateTimeValue);
      saveDateTime();
      mClockSet = true;
      }
    }
  //}}}

  //{{{
  bool getClockSet() {
    return mClockSet;
    }
  //}}}
  //{{{
  void getClockAngles (float& hours, float& minutes, float& seconds) {

    loadDateTime();
    hours = (1.f - ((mDateTime.Hours + (mDateTime.Minutes / 60.f)) / 6.f)) * kPi;
    minutes = (1.f - ((mDateTime.Minutes + (mDateTime.Seconds / 60.f))/ 30.f)) * kPi;
    seconds =  (1.f - (mDateTime.Seconds / 30.f)) * kPi;
    }
  //}}}
  //{{{
  std::string getClockTimeString() {

    return mDateTime.getTimeString();
    }
  //}}}
  //{{{
  std::string getClockTimeDateString() {

    return mDateTime.getTimeDateString();
    }
  //}}}
  //{{{
  std::string getBuildTimeDateString() {

    return mBuildTime + " "  + mBuildDate;
    }
  //}}}

private:
  //{{{
  void loadDateTime() {

    mDateTime.SubSeconds = RTC->SSR;
    mDateTime.SecondFraction = RTC->PRER & RTC_PRER_PREDIV_S;

    uint32_t tr = RTC->TR;
    mDateTime.TimeFormat = (tr & RTC_TR_PM) >> 16U;
    mDateTime.Hours = getByteFromBcd ((tr & (RTC_TR_HT | RTC_TR_HU)) >> 16U);
    mDateTime.Minutes = getByteFromBcd ((tr & (RTC_TR_MNT | RTC_TR_MNU)) >> 8U);
    mDateTime.Seconds = getByteFromBcd (tr & (RTC_TR_ST | RTC_TR_SU));

    uint32_t dr = RTC->DR;
    mDateTime.Year = getByteFromBcd ((dr & (RTC_DR_YT | RTC_DR_YU)) >> 16U);
    mDateTime.WeekDay = (dr & RTC_DR_WDU) >> 13U;
    mDateTime.Month = getByteFromBcd ((dr & (RTC_DR_MT | RTC_DR_MU)) >> 8U);
    mDateTime.Date = getByteFromBcd (dr & (RTC_DR_DT | RTC_DR_DU));
    }
  //}}}
  //{{{
  void saveDateTime() {

    writeProtectDisable();
    if (enterInitMode()) {
      if ((RTC->CR & RTC_CR_FMT) == (uint32_t)RESET)
        mDateTime.TimeFormat = 0x00U;

      if ((mDateTime.Month & 0x10U) == 0x10U)
        mDateTime.Month = (uint8_t)((mDateTime.Month & (uint8_t)~(0x10U)) + (uint8_t)0x0AU);

      // Set the RTC_DR register
      uint32_t tmp = (getBcdFromByte (mDateTime.Year) << 16) |
                     (getBcdFromByte (mDateTime.Month) << 8) |
                      getBcdFromByte (mDateTime.Date) |
                     (mDateTime.WeekDay << 13);
      RTC->DR = (uint32_t)(tmp & RTC_DR_RESERVED_MASK);

      // Set the RTC_TR register
      tmp = ((getBcdFromByte (mDateTime.Hours) << 16) |
             (getBcdFromByte (mDateTime.Minutes) << 8) |
              getBcdFromByte (mDateTime.Seconds) |
            (mDateTime.TimeFormat) << 16);
      RTC->TR = (uint32_t)(tmp & RTC_TR_RESERVED_MASK);

      // Clear the bits to be configured
      RTC->CR &= (uint32_t)~RTC_CR_BCK;

      // Configure the RTC_CR register
      RTC->CR |= mDateTime.DayLightSaving | mDateTime.StoreOperation;

      // Exit Initialization mode
      RTC->ISR &= (uint32_t)~RTC_ISR_INIT;

      if ((RTC->CR & RTC_CR_BYPSHAD) == RESET)
        if (!waitForSynchro())
          printf ("setDateTime - timeout waiting for synchro\n");
      }

    writeProtectEnable();
    }
  //}}}
  //{{{
  uint8_t getBcdFromByte (uint8_t byte) {
    return ((byte / 10) << 4) | (byte % 10);
    }
  //}}}
  //{{{
  uint8_t getByteFromBcd (uint8_t bcd) {
    return (((bcd & 0xF0) >> 4) * 10) + (bcd & 0x0F);
    }
  //}}}

  //{{{
  void writeProtectDisable() {
    RTC->WPR = 0xCAU;
    RTC->WPR = 0x53U;
    }
  //}}}
  //{{{
  bool enterInitMode() {

    // Check if the Initialization mode is set
    if ((RTC->ISR & RTC_ISR_INITF) == (uint32_t)RESET) {
      // Set the Initialization mode
      RTC->ISR = (uint32_t)RTC_INIT_MASK;

      /* Get tick */
      uint32_t tickstart = HAL_GetTick();

      // Wait till RTC is in INIT state and if Time out is reached exit
      while ((RTC->ISR & RTC_ISR_INITF) == (uint32_t)RESET)
        if ((HAL_GetTick() - tickstart ) > RTC_TIMEOUT_VALUE)
          return false;
      }

    return true;
    }
  //}}}
  //{{{
  bool waitForSynchro() {

    // Clear RSF flag
    RTC->ISR &= (uint32_t)RTC_RSF_MASK;

    uint32_t tickstart = HAL_GetTick();

    // Wait the registers to be synchronised
    while ((RTC->ISR & RTC_ISR_RSF) == (uint32_t)RESET)
      if ((HAL_GetTick() - tickstart ) > RTC_TIMEOUT_VALUE)
        return false;

    return true;
    }
  //}}}
  //{{{
  void writeProtectEnable() {
    RTC->WPR = 0xFFU;
    }
  //}}}

  const float kPi = 3.1415926f;
  const int kBuildSecs = 6;
  const std::string mBuildTime = __TIME__;
  const std::string mBuildDate = __DATE__;

  //{{{
  class cDateTime {
  public:
    cDateTime() {}
    //{{{
    cDateTime (const std::string& buildDateStr, const std::string& buildTimeStr) {

      // buildDateStr - dd:mmm:yyyy
      Date = ((buildDateStr[4] == ' ') ? 0 : buildDateStr[4] - 0x30) * 10 + (buildDateStr[5] -0x30);
      Year = (buildDateStr[9] - 0x30) * 10 + (buildDateStr[10] -0x30);

      Month = 0;
      for (int i = 0; i < 12; i++)
        if ((buildDateStr[0] == *kMonth[i]) && (buildDateStr[1] == *(kMonth[i]+1)) && (buildDateStr[2] == *(kMonth[i]+2))) {
          Month = i;
          break;
          }

      // buildTimeStr - hh:mm:ss
      Hours = (buildTimeStr[0]  - 0x30) * 10 + (buildTimeStr[1] -0x30);
      Minutes = (buildTimeStr[3] - 0x30) * 10 + (buildTimeStr[4] -0x30);
      Seconds = (buildTimeStr[6] - 0x30) * 10 + (buildTimeStr[7] -0x30);
      }
    //}}}

    //{{{
    uint32_t getValue() {
      return ((((Year*12 + Month)*31 + Date)*24 + Hours)*60 + Minutes)*60 + Seconds;
      }
    //}}}
    //{{{
    std::string getTimeString() {
      return dec(Hours,2) + ":" + dec(Minutes,2) + ":" + dec(Seconds,2);
             //dec(SubSeconds) + " " + dec(SecondFraction);
      }
    //}}}
    //{{{
    std::string getDateString() {
      return std::string(kMonth[Month]) + " " + dec(Date,2) + " " + dec(2000 + Year,4);
      }
    //}}}
    //{{{
    std::string getTimeDateString() {
      return dec(Hours,2) + ":" + dec(Minutes,2) + ":" + dec(Seconds,2) + " " +
             kMonth[Month] + " " + dec(Date,2) + " " + dec(2000 + Year,4);
             //dec(SubSeconds) + " " + dec(SecondFraction);
      }
    //}}}

    //{{{
    void setFromValue (uint32_t value) {
      TimeFormat = RTC_HOURFORMAT12_AM;
      DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
      StoreOperation = RTC_STOREOPERATION_RESET;

      Seconds = value % 60;
      value /= 60;
      Minutes = value % 60;
      value /= 60;
      Hours = value % 24;
      value /= 24;
      Date = value % 31;
      value /= 31;
      Month = value % 12;
      value /= 12;
      Year = value;

      WeekDay = RTC_WEEKDAY_FRIDAY;  // wrong
      }
    //}}}

    uint8_t Year;
    uint8_t Month;
    uint8_t WeekDay;
    uint8_t Date;
    uint8_t Hours;
    uint8_t Minutes;
    uint8_t Seconds;
    uint8_t TimeFormat;
    uint32_t SubSeconds;
    uint32_t SecondFraction;
    uint32_t DayLightSaving;
    uint32_t StoreOperation;

  private:
    const char* kMonth[12] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    };
  //}}}
  cDateTime mDateTime;
  RTC_HandleTypeDef mRtcHandle;
  bool mClockSet = false;
  };
//}}}

//{{{
class cApp : public cLcd {
public:
  cApp (bool pinReset, bool powerOnReset) : mPinReset(pinReset), mPowerOnReset(powerOnReset) {}

  //{{{
  bool init() {

    cLcd::init();
    mAdc.init();
    mRtc.init();

    mValues = (int16_t*)malloc (getWidth() * 2);

    if (mValues)
      printf ("cApp:: init alloc %d\n", getWidth() * 2);
    else
      printf ("cApp:: init alloc failed\n");

    return mValues;
    }
  //}}}
  //{{{
  void run() {

    auto lastTicks = HAL_GetTick();

    mAdc.start();
    while (true) {
      mAdc.poll();
      auto value = mAdc.getValue();
      printf ("value %d %d\n", value, getFrameNum());

      mValues[getFrameNum() % getWidth()] = value;
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
      drawString (eOff, eSmall, eLeft,
                  dec (mMinValue) + "min " + dec (value)    + " " + dec (mMaxValue) + "max " +
                  dec (int(mAverageVdd) / 1000) + "." + dec (int(mAverageVdd) % 1000, 3) + "v " +
                  dec (ticks - lastTicks) + " " +
                  (mRtc.getClockSet() ? "set ": "") +
                  (mPowerOnReset ? "pow ": "") +
                  (mPinReset ? "pin" : ""),
                  cPoint(0,0));
      lastTicks = ticks;

      drawString (eOff, eSmall, eLeft, mRtc.getBuildTimeDateString(), cPoint(0, 20));
      drawString (eOff, eMedium, eLeft, mRtc.getClockTimeDateString(), cPoint(0, getHeight()-40));

      drawClock (getCentre(), getHeight()/2 - 2);
      //drawClock (cPoint(400-42, 42), 40);
      //drawValues();
      drawTests();

      present();
      }
    }
  //}}}

  static cApp* mApp;
  cAdc mAdc;

private:
  //{{{
  void drawClock (cPoint centre, int16_t radius) {

    drawCircle (eOff, centre, radius);

    float hourAngle;
    float minuteAngle;
    float secondAngle;
    mRtc.getClockAngles (hourAngle, minuteAngle, secondAngle);

    float hourRadius = radius * 0.7f;
    drawLine (eOff, centre, centre + cPoint (int16_t(hourRadius * sin (hourAngle)), int16_t(hourRadius * cos (hourAngle))));

    float minuteRadius = radius * 0.8f;
    drawLine (eOff, centre, centre + cPoint (int16_t(minuteRadius * sin (minuteAngle)), int16_t(minuteRadius * cos (minuteAngle))));

    float secondRadius = radius * 0.9f;
    drawLine (eOff, centre, centre + cPoint (int16_t(secondRadius * sin (secondAngle)), int16_t(secondRadius * cos (secondAngle))));
    }
  //}}}
  //{{{
  void drawValues() {

    int32_t valueIndex = getFrameNum() - getWidth();
    for (int i = 0; i < getWidth(); i++) {
      int16_t len = valueIndex > 0 ? (getHeight() * (mValues[valueIndex % getWidth()] - mMinValue))  / (mMaxValue - mMinValue) : 0;
      fillRect (eOff, cRect (i, getHeight()-len, i+1, getHeight()));
      valueIndex++;
      }
    }
  //}}}
  //{{{
  void drawTests() {

    int16_t iteration = (getFrameNum() / 3) % 100;

    for (int i = 0; i < iteration; i++)
      fillRect (eOff, cRect (i, i, i+1, i+i));

    for (int i = 0; i < iteration; i++)
      fillRect (eOff, cRect (200+i, i, 200+i+i, i+1));
    }
  //}}}

  cRtc mRtc;
  bool mPowerOnReset = false;
  bool mPinReset = false;

  int16_t* mValues = nullptr;
  int16_t mMinValue = 4096;
  int16_t mMaxValue = 0;

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

  // get reset flags
  bool pinReset = __HAL_RCC_GET_FLAG (RCC_FLAG_PINRST);
  bool powerOnReset = __HAL_RCC_GET_FLAG (RCC_FLAG_PORRST);
  __HAL_RCC_CLEAR_RESET_FLAGS();
  printf ("main pin:%d power:%d cApp:%d\n", pinReset, powerOnReset, sizeof (cApp));

  cApp::mApp = new cApp (pinReset, powerOnReset);
  if (cApp::mApp->init())
    cApp::mApp->run();

  return 0;
  }
