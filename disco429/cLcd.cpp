// cLcd.cpp
#include "cLcd.h"
#include "../freetype/FreeSansBold.h"
//{{{  screen resolution defines
#ifdef NEXXY_SCREEN
  // NEXXY 7 inch
  #define LTDC_CLOCK_4      130  // 32.5Mhz
  #define HORIZ_SYNC         64
  #define VERT_SYNC           1
#else
  // ASUS eee 10 inch
  #define LTDC_CLOCK_4      100  // 25Mhz
  #define HORIZ_SYNC        136  // min  136  typ 176   max 216
  #define VERT_SYNC          12  // min   12  typ  25   max  38
#endif
//}}}

extern "C" {
  //{{{
  void LTDC_IRQHandler() {

    // line Interrupt
    if ((LTDC->ISR & LTDC_FLAG_LI) != RESET) {
      LTDC->ICR = LTDC_FLAG_LI;
      if (cLcd::mFrameWait)
        cLcd::mFrameWait = false;
      }

    // register reload Interrupt
    if ((LTDC->ISR & LTDC_FLAG_RR) != RESET) {
      LTDC->ICR = LTDC_FLAG_RR;
      //cLcd::mLcd->debug (LCD_COLOR_YELLOW, "ltdc reload IRQ");
      }
    }
  //}}}
  //{{{
  void LTDC_ER_IRQHandler() {

    // transfer Error Interrupt
    if ((LTDC->ISR &  LTDC_FLAG_TE) != RESET) {
      LTDC->ICR = LTDC_IT_TE;
      //cLcd::mLcd->debug (LCD_COLOR_RED, "ltdc te IRQ");
      }

    // FIFO underrun Interrupt
    if ((LTDC->ISR &  LTDC_FLAG_FU) != RESET) {
      LTDC->ICR = LTDC_FLAG_FU;
      //cLcd::mLcd->debug (LCD_COLOR_RED, "ltdc fifoUnderrun IRQ");
      }
    }
  //}}}
  }

cLcd* cLcd::mLcd = nullptr;
bool cLcd::mFrameWait = false;

//{{{
class cFontChar {
public:
  uint8_t* bitmap;
  int16_t left;
  int16_t top;
  int16_t pitch;
  int16_t rows;
  int16_t advance;
  };
//}}}

//{{{
cLcd::cLcd (uint16_t* buffer0, uint16_t* buffer1)  {

  mBuffer[0] = buffer0;
  mBuffer[1] = buffer1;
  updateNumDrawLines();

  mLcd = this;
  }
//}}}
//{{{
void cLcd::init (std::string title) {

  ltdcInit (mBuffer[mDrawBuffer]);

  // font init
  FT_Init_FreeType (&FTlibrary);
  FT_New_Memory_Face (FTlibrary, (FT_Byte*)freeSansBold, sizeof (freeSansBold), 0, &FTface);
  FTglyphSlot = FTface->glyph;

  // preload fontChars
  for (char ch = 0x20; ch <= 0x7F; ch++)
    loadChar (getFontHeight(), ch);
  //for (char ch = 0x21; ch <= 0x3F; ch++)
  //  loadChar (getBigFontHeight(), ch);
  //for (char ch = 0x21; ch <= 0x3F; ch++)
  //  loadChar (getSmallFontHeight(), ch);

  FT_Done_Face (FTface);
  FT_Done_FreeType (FTlibrary);

  mTitle = title;
  updateNumDrawLines();
  }
//}}}

//{{{
void cLcd::setShowDebug (bool title, bool info, bool footer) {

  mShowTitle = title;
  mShowInfo = info;
  mShowFooter = footer;

  updateNumDrawLines();
  }
//}}}
//{{{
void cLcd::info (uint16_t colour, std::string str) {

  bool tailing = mLastLine == (int)mFirstLine + mNumDrawLines - 1;

  auto line = (mLastLine < mMaxLine-1) ? mLastLine+1 : mLastLine;
  mLines[line].mTime = HAL_GetTick();
  mLines[line].mColour = colour;
  mLines[line].mString = str;
  mLastLine = line;

  if (tailing)
    mFirstLine = mLastLine - mNumDrawLines + 1;
 }
//}}}
//{{{
void cLcd::info (std::string str) {
  info (COL_WHITE, str);
  }
//}}}
//{{{
void cLcd::debug (uint16_t colour, std::string str) {

  info (colour, str);
  render();
  }
//}}}
//{{{
void cLcd::debug (std::string str) {
  debug (COL_WHITE, str);
  }
//}}}

//{{{
void cLcd::pixel (uint16_t colour, int16_t x, int16_t y) {
  *(mBuffer[mDrawBuffer] + y * getWidth() + x) = colour;
  }
//}}}
//{{{
void cLcd::rect (uint16_t colour, int16_t x, int16_t y, uint16_t width, uint16_t height) {
//__IO uint32_t OPFCCR;        /*!< DMA2D Output PFC Control Register,              Address offset: 0x34 */
//__IO uint32_t OCOLR;         /*!< DMA2D Output Color Register,                    Address offset: 0x38 */
//__IO uint32_t OMAR;          /*!< DMA2D Output Memory Address Register,           Address offset: 0x3C */
//__IO uint32_t OOR;           /*!< DMA2D Output Offset Register,                   Address offset: 0x40 */
//__IO uint32_t NLR;           /*!< DMA2D Number of Line Register,                  Address offset: 0x44 */

  ready();
  DMA2D->OPFCCR  = kDstFormat;
  DMA2D->OCOLR = colour;
  DMA2D->OOR = getWidth() - width;
  DMA2D->OMAR = uint32_t(mBuffer[mDrawBuffer] + y * getWidth() + x);
  DMA2D->NLR = (width << 16) | height;

  //uint32_t regs[5];
  //regs[0] = kDstFormat;
  //regs[1] = colour;
  //regs[2] = mBuffer[mDrawBuffer] + y * getWidth()) + x;
  //regs[3] = getWidth() - width;
  //regs[4] = (width << 16) | height;
  //ready();
  //memcpy ((void*)(&DMA2D->OPFCCR), regs, 5*4);

  DMA2D->CR = DMA2D_R2M | DMA2D_CR_TCIE | DMA2D_CR_TEIE | DMA2D_CR_CEIE | DMA2D_CR_START;
  mWait = true;
  }
//}}}
//{{{
void cLcd::stamp (uint16_t colour, uint8_t* src, int16_t x, int16_t y, uint16_t width, uint16_t height) {
//__IO uint32_t FGMAR;         /*!< DMA2D Foreground Memory Address Register,       Address offset: 0x0C */
//__IO uint32_t FGOR;          /*!< DMA2D Foreground Offset Register,               Address offset: 0x10 */
//__IO uint32_t BGMAR;         /*!< DMA2D Background Memory Address Register,       Address offset: 0x14 */
//__IO uint32_t BGOR;          /*!< DMA2D Background Offset Register,               Address offset: 0x18 */
//__IO uint32_t FGPFCCR;       /*!< DMA2D Foreground PFC Control Register,          Address offset: 0x1C */
//__IO uint32_t FGCOLR;        /*!< DMA2D Foreground Color Register,                Address offset: 0x20 */
//__IO uint32_t BGPFCCR;       /*!< DMA2D Background PFC Control Register,          Address offset: 0x24 */
//__IO uint32_t BGCOLR;        /*!< DMA2D Background Color Register,                Address offset: 0x28 */
//__IO uint32_t FGCMAR;        /*!< DMA2D Foreground CLUT Memory Address Register,  Address offset: 0x2C */
//__IO uint32_t BGCMAR;        /*!< DMA2D Background CLUT Memory Address Register,  Address offset: 0x30 */
//__IO uint32_t OPFCCR;        /*!< DMA2D Output PFC Control Register,              Address offset: 0x34 */
//__IO uint32_t OCOLR;         /*!< DMA2D Output Color Register,                    Address offset: 0x38 */
//__IO uint32_t OMAR;          /*!< DMA2D Output Memory Address Register,           Address offset: 0x3C */
//__IO uint32_t OOR;           /*!< DMA2D Output Offset Register,                   Address offset: 0x40 */
//__IO uint32_t NLR;           /*!< DMA2D Number of Line Register,                  Address offset: 0x44 */

  //{{{  alternative code
  //  uint32_t address = mBuffer[mDrawBuffer] + y * getWidth()) + x;
  //  uint32_t col = ((colour & 0xF800) << 8) | ((colour & 0x07E0) << 5) | ((colour & 0x001F) << 3);
  //  uint32_t stride = getWidth() - width;
  //  uint32_t nlr = (width << 16) | height;
  //  ready();
  //  DMA2D->FGMAR   = (uint32_t)src;  // fgnd start address
  //  DMA2D->FGOR    = 0;              // fgnd stride
  //  DMA2D->BGMAR   = address;        // - repeated to bgnd start addres
  //  DMA2D->BGOR    = stride;         // - repeated to bgnd stride
  //  DMA2D->FGPFCCR = DMA2D_INPUT_A8; // fgnd PFC
  //  DMA2D->FGCOLR  = col;
  //  DMA2D->BGPFCCR = kDstFormat;
  //  DMA2D->OPFCCR  = kDstFormat;
  //  DMA2D->OMAR    = address;        // output start address
  //  DMA2D->OOR     = stride;         // output stride
  //  DMA2D->NLR     = nlr;            //  width:height
  //  DMA2D->CR = DMA2D_M2M_BLEND | DMA2D_CR_TCIE | DMA2D_CR_TEIE | DMA2D_CR_CEIE | DMA2D_CR_START;
  //}}}

  uint32_t regs[15];
  regs[0] = (uint32_t)src;
  regs[1] = 0;
  regs[2] = uint32_t(mBuffer[mDrawBuffer] + y * getWidth() + x);
  regs[3] = getWidth() - width;
  regs[4] = DMA2D_INPUT_A8;
  regs[5] = ((colour & 0xF800) << 8) | ((colour & 0x07E0) << 5) | ((colour & 0x001F) << 3);
  regs[6] = kDstFormat;
  regs[7] = 0;
  regs[8] = 0;
  regs[9] = 0;
  regs[10] = kDstFormat;
  regs[11] = 0;
  regs[12] = regs[2];
  regs[13] = regs[3];
  regs[14] = (width << 16) | height;

  ready();
  memcpy ((void*)(&DMA2D->FGMAR), regs, 15*4);
  DMA2D->CR = DMA2D_M2M_BLEND | DMA2D_CR_TCIE | DMA2D_CR_TEIE | DMA2D_CR_CEIE | DMA2D_CR_START;
  mWait = true;
  }
//}}}
//{{{
void cLcd::copy (cTile* srcTile, int16_t x, int16_t y) {

  ready();
  DMA2D->FGPFCCR = srcTile->mFormat;
  DMA2D->FGMAR = (uint32_t)srcTile->mPiccy;
  DMA2D->FGOR = srcTile->mPitch - srcTile->mWidth;
  DMA2D->OPFCCR = kDstFormat;
  DMA2D->OMAR = uint32_t(mBuffer[mDrawBuffer] + y * getWidth() + x);
  DMA2D->OOR = getWidth() - srcTile->mWidth;
  DMA2D->NLR = (srcTile->mWidth << 16) | srcTile->mHeight;
  DMA2D->CR = DMA2D_M2M_PFC | DMA2D_CR_TCIE | DMA2D_CR_TEIE | DMA2D_CR_CEIE | DMA2D_CR_START;
  mWait = true;
  }
//}}}
//{{{
void cLcd::copy90 (cTile* srcTile, int16_t x, int16_t y) {

  uint32_t src = (uint32_t)srcTile->mPiccy;
  uint32_t dst = (uint32_t)mBuffer[mDrawBuffer];

  ready();
  DMA2D->FGPFCCR = srcTile->mFormat;
  DMA2D->FGOR = 0;
  DMA2D->OPFCCR = kDstFormat;
  DMA2D->OOR = getWidth() - 1;
  DMA2D->NLR = 0x10000 | (srcTile->mWidth);

  for (int line = 0; line < srcTile->mHeight; line++) {
    DMA2D->FGMAR = src;
    DMA2D->OMAR = dst;
    DMA2D->CR = DMA2D_M2M_PFC | DMA2D_CR_TCIE | DMA2D_CR_TEIE | DMA2D_CR_CEIE | DMA2D_CR_START;
    src += srcTile->mWidth * srcTile->mComponents;
    dst += kDstComponents;
    wait();
    }
  }
//}}}
//{{{
void cLcd::size (cTile* srcTile, int16_t x, int16_t y, uint16_t width, uint16_t height) {
// 2 passs size with rotates, bilinear blend but broken

  auto tempBuf = (uint32_t)pvPortMalloc (srcTile->mWidth * height * kTempComponents);

  // first pass
  uint32_t srcBase = srcTile->mPiccy + ((srcTile->mY * srcTile->mPitch) + srcTile->mX) * srcTile->mComponents;
  uint32_t blendCoeff = ((srcTile->mHeight-1) << 21) / height;
  uint32_t blendIndex = blendCoeff >> 1;
  uint16_t srcPitch = srcTile->mPitch * srcTile->mComponents;
  uint32_t srcPtr = srcBase + (blendIndex >> 21) * srcPitch;
  uint32_t srcPtr1 = srcPtr + srcPitch;
  uint32_t dstPtr = tempBuf;
  uint32_t fccr = srcTile->mFormat | ((blendIndex >> 13) << 24);
  uint16_t dstPitch = kTempComponents;

  ready();
  DMA2D->FGOR = 0;
  DMA2D->BGPFCCR = 0xff000000 | srcTile->mFormat;
  DMA2D->BGOR = 0;
  DMA2D->OPFCCR = kTempFormat;
  DMA2D->OOR = height - 1;
  DMA2D->NLR = 0x10000 | srcTile->mWidth;
  for (int i = 0; i < height; i++) {
    //{{{  loop lines, src -> temp
    DMA2D->FGPFCCR = fccr;
    DMA2D->FGMAR = srcPtr1;
    DMA2D->BGMAR = srcPtr;
    DMA2D->OMAR = dstPtr;
    DMA2D->CR = DMA2D_M2M_BLEND | DMA2D_CR_TCIE | DMA2D_CR_TEIE | DMA2D_CR_CEIE | DMA2D_CR_START;

    blendIndex += blendCoeff;
    fccr = srcTile->mFormat | ((blendIndex >> 13) << 24);
    srcPtr = srcBase + (blendIndex >> 21) * srcPitch;
    srcPtr1 = srcPtr + srcPitch;
    dstPtr += dstPitch;

    wait();
    }
    //}}}

  // second pass
  srcBase = tempBuf;
  blendCoeff = ((srcTile->mWidth-1) << 21) / width;
  blendIndex = blendCoeff >> 1;
  srcPitch = height * kTempComponents;
  srcPtr = srcBase + (blendIndex >> 21) * srcPitch;
  srcPtr1 = srcPtr + srcPitch;
  dstPtr = uint32_t(mBuffer[mDrawBuffer] + y * getWidth() + x);
  fccr = kTempFormat | ((blendIndex >> 13) << 24);
  dstPitch = kDstComponents;

  DMA2D->BGPFCCR = 0xff000000 | kTempFormat;
  DMA2D->OPFCCR  = kDstFormat;
  DMA2D->OOR = width - 1;
  DMA2D->NLR = 0x10000 | height;
  for (int i = 0; i < width; i++) {
    //{{{  loop columns, temp -> dst
    DMA2D->FGPFCCR = fccr;
    DMA2D->FGMAR = srcPtr1;
    DMA2D->BGMAR = srcPtr;
    DMA2D->OMAR = dstPtr;
    DMA2D->CR = DMA2D_M2M_BLEND | DMA2D_CR_TCIE | DMA2D_CR_TEIE | DMA2D_CR_CEIE | DMA2D_CR_START;

    blendIndex += blendCoeff;
    fccr = kTempFormat | ((blendIndex >> 13) << 24);
    srcPtr = srcBase + (blendIndex >> 21) * srcPitch;
    srcPtr1 = srcPtr + srcPitch;
    dstPtr += dstPitch;

    wait();
    }
    //}}}

  vPortFree ((void*)tempBuf);
  }
//}}}
//{{{
void cLcd::sizeCpu (cTile* srcTile, int16_t x, int16_t y, uint16_t width, uint16_t height) {

  uint32_t xStep16 = ((srcTile->mWidth - 1) << 16) / (width - 1);
  uint32_t yStep16 = ((srcTile->mHeight - 1) << 16) / (height - 1);

  auto dstPtr = mBuffer[mDrawBuffer] + y * getWidth() + x;

  if (srcTile->mComponents == 2) {
    auto srcBase = (uint16_t*)(srcTile->mPiccy) + (srcTile->mY * srcTile->mPitch) + srcTile->mX;
    for (uint32_t y16 = (srcTile->mY << 16); y16 < ((srcTile->mY + height) * yStep16); y16 += yStep16) {
      auto srcy1x1 = srcBase + (y16 >> 16) * srcTile->mPitch;
      for (uint32_t x16 = srcTile->mX << 16; x16 < (srcTile->mX + width) * xStep16; x16 += xStep16)
        *dstPtr++ = *(srcy1x1 + (x16 >> 16));
      dstPtr += getWidth() - width;
      }
    }

  else {
    auto srcBase = (uint8_t*)(srcTile->mPiccy + ((srcTile->mY * srcTile->mPitch) + srcTile->mX) * srcTile->mComponents);
    for (uint32_t y16 = (srcTile->mY << 16); y16 < ((srcTile->mY + height) * yStep16); y16 += yStep16) {
      auto srcy = srcBase + ((y16 >> 16) * srcTile->mPitch) * srcTile->mComponents;
      for (uint32_t x16 = srcTile->mX << 16; x16 < (srcTile->mX + width) * xStep16; x16 += xStep16) {
        auto srcy1x1 = srcy + (x16 >> 16) * srcTile->mComponents;
        *dstPtr++ = ((*srcy1x1++) >> 3) | (((*srcy1x1++) & 0xFC) << 3) | (((*srcy1x1) & 0xF8) << 8);
        }
      dstPtr += getWidth() - width;
      }
    }
  }
//}}}
//{{{
void cLcd::sizeCpuBi (cTile* srcTile, int16_t x, int16_t y, uint16_t width, uint16_t height) {
// only for src->components = 3,4

  uint32_t xStep16 = ((srcTile->mWidth - 1) << 16) / (width - 1);
  uint32_t yStep16 = ((srcTile->mHeight - 1) << 16) / (height - 1);

  uint16_t* dstPtr = mBuffer[mDrawBuffer] + y * getWidth() + x;

  uint32_t ySrcOffset = srcTile->mPitch * srcTile->mComponents;
  for (uint32_t y16 = (srcTile->mY << 16); y16 < (srcTile->mY + height) * yStep16; y16 += yStep16) {
    uint8_t yweight2 = (y16 >> 9) & 0x7F;
    uint8_t yweight1 = 0x80 - yweight2;
    const uint8_t* srcy = (uint8_t*)srcTile->mPiccy + ((y16 >> 16) * ySrcOffset) + (srcTile->mX * srcTile->mComponents);
    for (uint32_t x16 = srcTile->mX << 16; x16 < (srcTile->mX + width) * xStep16; x16 += xStep16) {
      uint8_t xweight2 = (x16 >> 9) & 0x7F;
      uint8_t xweight1 = 0x80 - xweight2;
      const uint8_t* srcy1x1 = srcy + (x16 >> 16) * srcTile->mComponents;
      const uint8_t* srcy1x2 = srcy1x1 + srcTile->mComponents;
      const uint8_t* srcy2x1 = srcy1x1 + ySrcOffset;
      const uint8_t* srcy2x2 = srcy2x1 + srcTile->mComponents;

      *dstPtr++ = ((((*srcy1x1++ * xweight1 + *srcy1x2++ * xweight2) * yweight1) +
                     (*srcy2x1++ * xweight1 + *srcy2x2++ * xweight2) * yweight2) >> 17) |
                  (((((*srcy1x1++ * xweight1 + *srcy1x2++ * xweight2) * yweight1) +
                      (*srcy2x1++ * xweight1 + *srcy2x2++ * xweight2) * yweight2) >> 11) & 0x07E0) |
                  (((((*srcy1x1 * xweight1 + *srcy1x2 * xweight2) * yweight1) +
                      (*srcy2x1 * xweight1 + *srcy2x2 * xweight2) * yweight2) >> 6) & 0xF800);
      }
    }
  }
//}}}

//{{{
void cLcd::clear (uint16_t colour) {
  rect (colour, 0, 0, getWidth(), getHeight());
  }
//}}}
//{{{
void cLcd::pixelClipped (uint16_t colour, int16_t x, int16_t y) {
  rectClipped (colour, x, y, 1, 1);
  }
//}}}
//{{{
void cLcd::stampClipped (uint16_t colour, uint8_t* src, int16_t x, int16_t y, uint16_t width, uint16_t height) {

  if (!width || !height || x < 0)
    return;

  if (y < 0) {
    // top clip
    if (y + height <= 0)
      return;
    height += y;
    src += -y * width;
    y = 0;
    }

  if (y + height > getHeight()) {
    // bottom yclip
    if (y >= getHeight())
      return;
    height = getHeight() - y;
    }

  stamp (colour, src, x, y, width, height);
  }
//}}}
//{{{
void cLcd::rectClipped (uint16_t colour, int16_t x, int16_t y, uint16_t width, uint16_t height) {

  if (x >= getWidth())
    return;
  if (y >= getHeight())
    return;

  int xend = x + width;
  if (xend <= 0)
    return;

  int yend = y + height;
  if (yend <= 0)
    return;

  if (x < 0)
    x = 0;
  if (xend > getWidth())
    xend = getWidth();

  if (y < 0)
    y = 0;
  if (yend > getHeight())
    yend = getHeight();

  if (!width)
    return;
  if (!height)
    return;

  rect (colour, x, y, xend - x, yend - y);
  }
//}}}
//{{{
void cLcd::rectOutline (uint16_t colour, int16_t x, int16_t y, uint16_t width, uint16_t height, uint8_t thickness) {

  rectClipped (colour, x, y, width, thickness);
  rectClipped (colour, x + width-thickness, y, thickness, height);
  rectClipped (colour, x, y + height-thickness, width, thickness);
  rectClipped (colour, x, y, thickness, height);
  }
//}}}
//{{{
void cLcd::ellipse (uint16_t colour, int16_t x, int16_t y, uint16_t xradius, uint16_t yradius) {

  if (!xradius)
    return;
  if (!yradius)
    return;

  int x1 = 0;
  int y1 = -yradius;
  int err = 2 - 2*xradius;
  float k = (float)yradius / xradius;

  do {
    rectClipped (colour, (x-(uint16_t)(x1 / k)), y + y1, (2*(uint16_t)(x1 / k) + 1), 1);
    rectClipped (colour, (x-(uint16_t)(x1 / k)), y - y1, (2*(uint16_t)(x1 / k) + 1), 1);

    int e2 = err;
    if (e2 <= x1) {
      err += ++x1 * 2 + 1;
      if (-y1 == x && e2 <= y1)
        e2 = 0;
      }
    if (e2 > y1)
      err += ++y1*2 + 1;
    } while (y1 <= 0);
  }
//}}}
//{{{
void cLcd::ellipseOutline (uint16_t colour, int16_t x, int16_t y, uint16_t xradius, uint16_t yradius) {

  if (xradius && yradius) {
    int x1 = 0;
    int y1 = -yradius;
    int err = 2 - 2*xradius;
    float k = (float)yradius / xradius;

    do {
      rectClipped (colour, x - (uint16_t)(x1 / k), y + y1, 1, 1);
      rectClipped (colour, x + (uint16_t)(x1 / k), y + y1, 1, 1);
      rectClipped (colour, x + (uint16_t)(x1 / k), y - y1, 1, 1);
      rectClipped (colour, x - (uint16_t)(x1 / k), y - y1, 1, 1);

      int e2 = err;
      if (e2 <= x1) {
        err += ++x1*2 + 1;
        if (-y1 == x1 && e2 <= y1)
          e2 = 0;
        }
      if (e2 > y1)
        err += ++y1*2 + 1;
      } while (y1 <= 0);
    }
  }
//}}}
//{{{
void cLcd::line (uint16_t colour, int16_t x1, int16_t y1, int16_t x2, int16_t y2) {

  int16_t deltax = (x2 - x1) > 0 ? (x2 - x1) : -(x2 - x1);        /* The difference between the x's */
  int16_t deltay = (y2 - y1) > 0 ? (y2 - y1) : -(y2 - y1);        /* The difference between the y's */
  int16_t x = x1;                       /* Start x off at the first pixel */
  int16_t y = y1;                       /* Start y off at the first pixel */

  int16_t xinc1;
  int16_t xinc2;
  if (x2 >= x1) {               /* The x-values are increasing */
    xinc1 = 1;
    xinc2 = 1;
    }
  else {                         /* The x-values are decreasing */
    xinc1 = -1;
    xinc2 = -1;
    }

  int yinc1;
  int yinc2;
  if (y2 >= y1) {                 /* The y-values are increasing */
    yinc1 = 1;
    yinc2 = 1;
    }
  else {                         /* The y-values are decreasing */
    yinc1 = -1;
    yinc2 = -1;
    }

  int den = 0;
  int num = 0;
  int num_add = 0;
  int num_pixels = 0;
  if (deltax >= deltay) {        /* There is at least one x-value for every y-value */
    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc2 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    num_add = deltay;
    num_pixels = deltax;         /* There are more x-values than y-values */
    }
  else {                         /* There is at least one y-value for every x-value */
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    num_add = deltax;
    num_pixels = deltay;         /* There are more y-values than x-values */
  }

  for (int curpixel = 0; curpixel <= num_pixels; curpixel++) {
    rectClipped (colour, x, y, 1, 1);   /* Draw the current pixel */
    num += num_add;                            /* Increase the numerator by the top of the fraction */
    if (num >= den) {                          /* Check if numerator >= denominator */
      num -= den;                             /* Calculate the new numerator value */
      x += xinc1;                             /* Change the x as appropriate */
      y += yinc1;                             /* Change the y as appropriate */
      }
    x += xinc2;                               /* Change the x as appropriate */
    y += yinc2;                               /* Change the y as appropriate */
    }
  }
//}}}
//{{{
int cLcd::text (uint16_t colour, uint16_t fontHeight, std::string str, int16_t x, int16_t y, uint16_t width, uint16_t height) {

  auto xend = x + width;
  for (uint16_t i = 0; i < str.size(); i++) {
    if ((str[i] >= 0x20) && (str[i] <= 0x7F)) {
      auto fontCharIt = mFontCharMap.find (fontHeight<<8 | str[i]);
      if (fontCharIt != mFontCharMap.end()) {
        auto fontChar = fontCharIt->second;
        if (x + fontChar->left + fontChar->pitch >= xend)
          break;
        else if (fontChar->bitmap)
          stampClipped (colour, fontChar->bitmap, x + fontChar->left, y + fontHeight - fontChar->top, fontChar->pitch, fontChar->rows);

        x += fontChar->advance;
        }
      }
    }

  return x;
  }
//}}}

//{{{
void cLcd::rgb888to565cpu (uint8_t* src, uint16_t* dst, uint16_t xsize, uint16_t ysize) {

  ready();
  for (uint16_t x = 0; x < xsize; x++) {
    uint8_t b = (*src++) & 0xF8;
    uint8_t g = (*src++) & 0xFC;
    uint8_t r = (*src++) & 0xF8;
    *dst++ = (r << 8) | (g << 3) | (b >> 3);
    }
  }
//}}}
//{{{
void cLcd::copy565cpu (uint16_t* src, uint16_t xsize, uint16_t ysize) {
  ready();
  memcpy (mBuffer[mDrawBuffer], src, xsize*ysize*2);
  }
//}}}

//{{{
void cLcd::start() {

  mDrawStartTime = HAL_GetTick();
  }
//}}}
//{{{
void cLcd::showInfo (bool force) {

  auto y = 0;

  if ((mShowTitle || force) && !mTitle.empty()) {
    // draw title
    text (COL_YELLOW, getFontHeight(), mTitle, 0, y, getWidth(), getBoxHeight());
    y += getBoxHeight();
    }

  if (mShowInfo || force) {
    // draw info lines
    if (mLastLine >= 0) {
      // draw scroll bar
      auto yorg = getBoxHeight() + ((int)mFirstLine * mNumDrawLines * getBoxHeight() / (mLastLine + 1));
      auto height = mNumDrawLines * mNumDrawLines * getBoxHeight() / (mLastLine + 1);
      rectClipped (COL_YELLOW, 0, yorg, 8, height);
      }

    auto lastLine = (int)mFirstLine + mNumDrawLines - 1;
    if (lastLine > mLastLine)
      lastLine = mLastLine;

    for (auto lineIndex = (int)mFirstLine; lineIndex <= lastLine; lineIndex++) {
      auto x = 0;
      auto xinc = text (COL_GREEN, getFontHeight(),
                        dec ((mLines[lineIndex].mTime-mStartTime) / 1000) + "." +
                        dec ((mLines[lineIndex].mTime-mStartTime) % 1000, 3, '0'),
                        x, y, getWidth(), getBoxHeight());
      x += xinc + 3;

      text (mLines[lineIndex].mColour, getFontHeight(), mLines[lineIndex].mString,
            x, y, getWidth()-x, getHeight());

      y += getBoxHeight();
      }
    }

  if (mShowFooter || force)
    // draw footer
    text (COL_WHITE, getFontHeight(),
          "heap:" + dec (xPortGetFreeHeapSize()) + ":" + dec (xPortGetMinimumEverFreeHeapSize()) +
          " draw:" + dec (mDrawTime) + "ms wait:" + dec (mWaitTime) + "ms ",
          0, -getFontHeight() + getHeight(), getWidth(), getFontHeight());
  }
//}}}
//{{{
void cLcd::present() {

  ready();
  mDrawTime = HAL_GetTick() - mDrawStartTime;

  mWaitStartTime = HAL_GetTick();
  LTDC_Layer1->CFBAR = (uint32_t)mBuffer[mDrawBuffer];
  LTDC->SRCR = LTDC_SRCR_VBR | LTDC_SRCR_IMR;
  mFrameWait = true;
  while (mFrameWait) {
    osDelay (1);
    }
  mWaitTime = HAL_GetTick() - mWaitStartTime;

  // flip
  mDrawBuffer = !mDrawBuffer;
  }
//}}}
//{{{
void cLcd::render() {

  start();
  clear (COL_BLACK);
  showInfo (true);
  present();
  }
//}}}

//{{{
void cLcd::displayOn() {

  // ADJ hi
  GPIOD->BSRR = GPIO_PIN_13;
  }
//}}}
//{{{
void cLcd::displayOff() {

  // ADJ lo
  GPIOD->BSRR = GPIO_PIN_13 << 16;
  }
//}}}
//{{{
void cLcd::press (int pressCount, int16_t x, int16_t y, uint16_t z, int16_t xinc, int16_t yinc) {

  if ((pressCount > 30) && (x <= mStringPos) && (y <= getBoxHeight()))
    reset();
  else if (pressCount == 0) {
    if (x <= mStringPos) {
      // set displayFirstLine
      if (y < 2 * getBoxHeight())
        displayTop();
      else if (y > getHeight() - 2 * getBoxHeight())
        displayTail();
      }
    }
  else {
    // inc firstLine
    float value = mFirstLine - ((2.0f * yinc) / getBoxHeight());

    if (value < 0)
      mFirstLine = 0;
    else if (mLastLine <= (int)mNumDrawLines-1)
      mFirstLine = 0;
    else if (value > mLastLine - mNumDrawLines + 1)
      mFirstLine = mLastLine - mNumDrawLines + 1;
    else
      mFirstLine = value;
    }
  }
//}}}

// private
//{{{
void cLcd::ltdcInit (uint16_t* frameBufferAddress) {

  // PLLSAI_VCO Input  = HSE_VALUE / PLL_M = 1mhz
  // PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN     = 130mhz
  // PLLLCDCLK         = PLLSAI_VCO Output / PLLSAIR    = 130/2 = 65mhz
  // LTDC clock        = PLLLCDCLK / LTDC_PLLSAI_DIVR_2 = 65/2  = 32.5mhz
  RCC_PeriphCLKInitTypeDef rccPeriphClkInit;
  rccPeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  rccPeriphClkInit.PLLSAI.PLLSAIN = LTDC_CLOCK_4;  // hclk = 192mhz, 138/4 = 34.5mhz
  rccPeriphClkInit.PLLSAI.PLLSAIR = 2;
  rccPeriphClkInit.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  HAL_RCCEx_PeriphCLKConfig (&rccPeriphClkInit);
  //{{{  init clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  __HAL_RCC_DMA2D_CLK_ENABLE();
  __HAL_RCC_LTDC_CLK_ENABLE();
  //}}}

  //{{{  int gpio
  //  HS <-> PC.06 - unused
  //  VS <-> PA.04 - unused
  //  R2 <-> PC.10 - unused
  //
  //  CK <-> PG.07
  //  DE <-> PF.10
  //  G2 <-> PA.06   B2 <-> PD.06
  //  R3 <-> PB.00   G3 <-> PG.10   B3 <-> PG.11
  //  R4 <-> PA.11   G4 <-> PB.10   B4 <-> PG.12
  //  R5 <-> PA.12   G5 <-> PB.11   B5 <-> PA.03
  //  R6 <-> PB.01   G6 <-> PC.07   B6 <-> PB.08
  //  R7 <-> PG.06   G7 <-> PD.03   B7 <-> PB.09
  //  ADJ <-> PD.13

  // adj
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pin = GPIO_PIN_13;
  HAL_GPIO_Init (GPIOD, &GPIO_InitStructure);

  // gpioA - AF14
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Alternate = GPIO_AF14_LTDC;
  GPIO_InitStructure.Pin = GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_11 | GPIO_PIN_12; // GPIO_PIN_4
  HAL_GPIO_Init (GPIOA, &GPIO_InitStructure);

  // gpioB
  GPIO_InitStructure.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
  HAL_GPIO_Init (GPIOB, &GPIO_InitStructure);

  // gpioC
  GPIO_InitStructure.Pin = GPIO_PIN_7; // ; | GPIO_PIN_10; // GPIO_PIN_6
  HAL_GPIO_Init (GPIOC, &GPIO_InitStructure);

  // gpioD
  GPIO_InitStructure.Pin = GPIO_PIN_3 | GPIO_PIN_6;
  HAL_GPIO_Init (GPIOD, &GPIO_InitStructure);

  // gpioF
  GPIO_InitStructure.Pin = GPIO_PIN_10;
  HAL_GPIO_Init (GPIOF, &GPIO_InitStructure);

  // gpioG
  GPIO_InitStructure.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_11;
  HAL_GPIO_Init (GPIOG, &GPIO_InitStructure);

  // gpioB - AF9
  GPIO_InitStructure.Alternate = GPIO_AF9_LTDC;
  GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  HAL_GPIO_Init (GPIOB, &GPIO_InitStructure);

  // gpioG - AF9
  GPIO_InitStructure.Pin = GPIO_PIN_10 | GPIO_PIN_12;
  HAL_GPIO_Init (GPIOG, &GPIO_InitStructure);
  //}}}

  LtdcHandler.Instance = LTDC;
  LtdcHandler.Init.HorizontalSync     = HORIZ_SYNC - 1;
  LtdcHandler.Init.AccumulatedHBP     = HORIZ_SYNC - 1;
  LtdcHandler.Init.AccumulatedActiveW = HORIZ_SYNC + LCD_WIDTH - 1;
  LtdcHandler.Init.TotalWidth         = HORIZ_SYNC + LCD_WIDTH - 1;
  LtdcHandler.Init.VerticalSync       = VERT_SYNC - 1;
  LtdcHandler.Init.AccumulatedVBP     = VERT_SYNC - 1;
  LtdcHandler.Init.AccumulatedActiveH = VERT_SYNC + LCD_HEIGHT - 1;
  LtdcHandler.Init.TotalHeigh         = VERT_SYNC + LCD_HEIGHT - 1;
  LtdcHandler.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  LtdcHandler.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  LtdcHandler.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  LtdcHandler.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  LtdcHandler.Init.Backcolor.Red = 0;
  LtdcHandler.Init.Backcolor.Blue = 0;
  LtdcHandler.Init.Backcolor.Green = 0;
  HAL_LTDC_Init (&LtdcHandler);

  LTDC_LayerCfgTypeDef* curLayerCfg = &LtdcHandler.LayerCfg[0];
  curLayerCfg->WindowX0 = 0;
  curLayerCfg->WindowY0 = 0;
  curLayerCfg->WindowX1 = getWidth();
  curLayerCfg->WindowY1 = getHeight();
  curLayerCfg->PixelFormat = kLtdcFormat;
  curLayerCfg->FBStartAdress = (uint32_t)frameBufferAddress;
  curLayerCfg->Alpha = 255;
  curLayerCfg->Alpha0 = 0;
  curLayerCfg->Backcolor.Blue = 0;
  curLayerCfg->Backcolor.Green = 0;
  curLayerCfg->Backcolor.Red = 0;
  curLayerCfg->BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  curLayerCfg->BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  curLayerCfg->ImageWidth = getWidth();
  curLayerCfg->ImageHeight = getHeight();
  HAL_LTDC_ConfigLayer (&LtdcHandler, curLayerCfg, 0);

  //DMA2D->AMTCR = 0x3F01;

  // set line interupt line number
  LTDC->LIPCR = 0;

  // clear interrupts
  LTDC->IER = LTDC_IT_TE | LTDC_IT_FU | LTDC_IT_LI;

  mFrameWait = false;
  HAL_NVIC_SetPriority (LTDC_IRQn, 0xE, 0);
  HAL_NVIC_EnableIRQ (LTDC_IRQn);
  }
//}}}
//{{{
uint32_t cLcd::wait() {

  uint32_t took = 0;
  while (!(DMA2D->ISR & DMA2D_FLAG_TC))
    took++;

  DMA2D->IFCR |= DMA2D_IFSR_CTEIF | DMA2D_IFSR_CTCIF | DMA2D_IFSR_CTWIF |
                 DMA2D_IFSR_CCAEIF | DMA2D_IFSR_CCTCIF | DMA2D_IFSR_CCEIF;

  return took;
  }
//}}}
//{{{
uint32_t cLcd::ready() {

  if (mWait) {
    mWait = false;
    return wait();
    }

  return 0;
  }
//}}}

//{{{
cFontChar* cLcd::loadChar (uint16_t fontHeight, char ch) {

  FT_Set_Pixel_Sizes (FTface, 0, fontHeight);
  FT_Load_Char (FTface, ch, FT_LOAD_RENDER);

  auto fontChar = new cFontChar();
  fontChar->left = FTglyphSlot->bitmap_left;
  fontChar->top = FTglyphSlot->bitmap_top;
  fontChar->pitch = FTglyphSlot->bitmap.pitch;
  fontChar->rows = FTglyphSlot->bitmap.rows;
  fontChar->advance = FTglyphSlot->advance.x / 64;
  fontChar->bitmap = nullptr;

  if (FTglyphSlot->bitmap.buffer) {
    fontChar->bitmap = (uint8_t*)malloc (FTglyphSlot->bitmap.pitch * FTglyphSlot->bitmap.rows);
    memcpy (fontChar->bitmap, FTglyphSlot->bitmap.buffer, FTglyphSlot->bitmap.pitch * FTglyphSlot->bitmap.rows);
    }

  auto insertPair = mFontCharMap.insert (cFontCharMap::value_type (fontHeight<<8 | ch, fontChar));
  auto fontCharIt = insertPair.first;

  return fontCharIt->second;
  }
//}}}

//{{{
void cLcd::reset() {

  for (auto i = 0; i < mMaxLine; i++)
    mLines[i].clear();

  mStartTime = HAL_GetTick();
  mLastLine = -1;
  mFirstLine = 0;
  }
//}}}
//{{{
void cLcd::displayTop() {
  mFirstLine = 0;
  }
//}}}
//{{{
void cLcd::displayTail() {
  mFirstLine = (mLastLine > (int)mNumDrawLines-1) ? mLastLine - mNumDrawLines + 1 : 0;
  }
//}}}
//{{{
void cLcd::updateNumDrawLines() {

  mStringPos = getBoxHeight()*3;

  auto numDrawLines = getHeight() / getBoxHeight();
  if (mShowTitle && !mTitle.empty())
    numDrawLines--;
  if (mShowFooter)
    numDrawLines--;

  mNumDrawLines = numDrawLines;
  }
//}}}
