#pragma once
//{{{  includes
#include "cmsis_os.h"
#include "semphr.h"

#include "utils.h"
#include "cPointRect.h"
#include <map>
#include <vector>

#include "stm32f4xx.h"

#include <ft2build.h>
#include FT_FREETYPE_H
//}}}
//{{{  colour defines
#define COL_BLACK         0x0000
#define COL_GREY          0x7BEF
#define COL_WHITE         0xFFFF

#define COL_BLUE          0x001F
#define COL_GREEN         0x07E0
#define COL_RED           0xF800

#define COL_CYAN          0x07FF
#define COL_MAGENTA       0xF81F
#define COL_YELLOW        0xFFE0
//}}}
//{{{  screen resolution defines
#ifdef NEXXY_SCREEN
  // NEXXY 7 inch
  #define LCD_WIDTH         800
  #define LCD_HEIGHT       1280

  #define BOX_HEIGHT         30
  #define SMALL_FONT_HEIGHT  12
  #define FONT_HEIGHT        26
  #define BIG_FONT_HEIGHT    40

#else
  // ASUS eee 10 inch
  #define LCD_WIDTH        1024  // min 39Mhz typ 45Mhz max 51.42Mhz
  #define LCD_HEIGHT        600

  #define BOX_HEIGHT         20
  #define SMALL_FONT_HEIGHT  10
  #define FONT_HEIGHT        16
  #define BIG_FONT_HEIGHT    32
#endif
//}}}

class cFontChar;

class cLcd {
public:
  enum eDma2dWait { eWaitNone, eWaitDone, eWaitIrq };
  //{{{
  class cTile {
  public:
    cTile() {};
    cTile (uint8_t* piccy, uint16_t components, uint16_t pitch,
           uint16_t x, uint16_t y, uint16_t width, uint16_t height)
       :  mPiccy((uint32_t)piccy), mComponents(components), mPitch(pitch), mX(x), mY(y), mWidth(width), mHeight(height) {
     if (components == 2)
       mFormat = DMA2D_RGB565;
     else if (components == 3)
       mFormat = DMA2D_RGB888;
     else
       mFormat = DMA2D_ARGB8888;
      };

    void free() {
      vPortFree ((void*)mPiccy);
      }

    uint32_t mPiccy;
    uint16_t mComponents;
    uint16_t mPitch;
    uint16_t mX;
    uint16_t mY;
    uint16_t mWidth;
    uint16_t mHeight;
    uint16_t mFormat;
    };
  //}}}
  cLcd (uint16_t* buffer0, uint16_t* buffer1);
  ~cLcd() {}

  void init (std::string title);
  static uint16_t getWidth() { return LCD_WIDTH; }
  static uint16_t getHeight() { return LCD_HEIGHT; }

  static uint16_t getBoxHeight() { return BOX_HEIGHT; }
  static uint16_t getSmallFontHeight() { return SMALL_FONT_HEIGHT; }
  static uint16_t getFontHeight() { return FONT_HEIGHT; }
  static uint16_t getBigFontHeight() { return BIG_FONT_HEIGHT; }

  bool changed();
  void setShowDebug (bool title, bool info, bool footer);
  void info (uint16_t colour, const std::string str);
  void info (const std::string str);
  void debug (uint16_t colour, const std::string str);
  void debug (const std::string str);

  void pixel (uint16_t colour, int16_t x, int16_t y);
  void rect (uint16_t colour, int16_t x, int16_t y, uint16_t width, uint16_t height);
  void stamp (uint16_t colour, uint8_t* src, int16_t x, int16_t y, uint16_t width, uint16_t height);
  void copy (cTile* srcTile, int16_t x, int16_t y);
  void copy90 (cTile* srcTile, int16_t x, int16_t y);
  void size (cTile* srcTile, int16_t x, int16_t y, uint16_t width, uint16_t height);
  void sizeCpu (cTile* srcTile, int16_t x, int16_t y, uint16_t width, uint16_t height);
  void sizeCpuBi (cTile* srcTile, int16_t x, int16_t y, uint16_t width, uint16_t height);

  void clear (uint16_t colour);
  void pixelClipped (uint16_t colour, int16_t x, int16_t y);
  void stampClipped (uint16_t colour, uint8_t* src, int16_t x, int16_t y, uint16_t width, uint16_t height);
  void rectClipped (uint16_t colour, int16_t x, int16_t y, uint16_t width, uint16_t height);
  void rectOutline (uint16_t colour, int16_t x, int16_t y, uint16_t width, uint16_t height, uint8_t thickness);
  void ellipse (uint16_t colour, int16_t x, int16_t y, uint16_t xradius, uint16_t yradius);
  void ellipseOutline (uint16_t colour, int16_t x, int16_t y, uint16_t xradius, uint16_t yradius);
  void line (uint16_t colour, int16_t x1, int16_t y1, int16_t x2, int16_t y2);
  int text (uint16_t colour, uint16_t fontHeight, const std::string str, int16_t x, int16_t y, uint16_t width, uint16_t height);

  void rgb888to565cpu (uint8_t* src, uint16_t* dst, uint16_t xsize, uint16_t ysize);

  void start();
  void showInfo (bool force);
  void present();
  void render();

  void displayOn();
  void displayOff();
  void press (int pressCount, int16_t x, int16_t y, uint16_t z, int16_t xinc, int16_t yinc);

  static cLcd* mLcd;

  static bool mFrameWait;
  static SemaphoreHandle_t mFrameSem;

  static eDma2dWait mDma2dWait;
  static SemaphoreHandle_t mDma2dSem;

private:
  //{{{  const
  const uint32_t kLtdcFormat = LTDC_PIXEL_FORMAT_RGB565;
  const uint32_t kDstFormat = DMA2D_RGB565;
  const uint16_t kDstComponents = 2;

  const uint32_t kTempFormat = DMA2D_RGB565;
  const uint16_t kTempComponents = 2;
  //}}}
  typedef std::map<uint16_t, cFontChar*> cFontCharMap;

  void ltdcInit (uint16_t* frameBufferAddress);
  cFontChar* loadChar (uint16_t fontHeight, char ch);

  void wait();
  void ready();

  void reset();
  void displayTop();
  void displayTail();
  void updateNumDrawLines();

  //{{{  vars
  LTDC_HandleTypeDef LtdcHandler;
  bool mChanged = true;

  int mStartTime = 0;

  float mFirstLine = 0;
  int mNumDrawLines = 0;
  int mStringPos = 0;

  bool mShowTitle = true;
  bool mShowInfo = false;
  bool mShowFooter = true;

  std::string mTitle;

  int mLastLine = -1;
  int mMaxLine = 256;

  //{{{
  class cLine {
  public:
    cLine() {}
    ~cLine() {}

    //{{{
    void clear() {
      mTime = 0;
      mColour = COL_WHITE;
      mString = "";
      }
    //}}}

    int mTime = 0;
    int mColour = COL_WHITE;
    std::string mString;
    };
  //}}}
  cLine mLines[256];

  bool mDrawBuffer = false;
  uint16_t* mBuffer[2] = {nullptr, nullptr};
  uint16_t* mCurFrameBufferAddress = nullptr;

  uint32_t mDrawStartTime = 0;
  uint32_t mDrawTime = 0;
  uint32_t mWaitStartTime = 0;
  uint32_t mWaitTime = 0;

  cFontCharMap mFontCharMap;

  FT_Library FTlibrary;
  FT_Face FTface;
  FT_GlyphSlot FTglyphSlot;
  //}}}
  };
