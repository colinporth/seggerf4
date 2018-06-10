#pragma once
//{{{  includes
#include "utils.h"
#include <map>
#include <vector>

#include "stm32f4xx.h"

#include <ft2build.h>
#include FT_FREETYPE_H
//}}}
//{{{  colour defines
#define COL_WHITE         0xFFFF
//#define COL_LIGHTGREY     0xFFD3D3D3
//#define COL_GREY          0xFF808080
//#define COL_DARKGREY      0xFF404040
//#define COL_DARKERGREY    0xFF202020
#define COL_BLACK         0x0000

#define COL_BLUE          0x001F
//#define COL_LIGHTBLUE     0xFF8080FF
//#define COL_DARKBLUE      0xFF000080

#define COL_GREEN         0x07E0
//#define COL_LIGHTGREEN    0xFF80FF80
//#define COL_DARKGREEN     0xFF008000
//#define COL_DARKERGREEN   0xFF004000

#define COL_RED           0xF800
//#define COL_LIGHTRED      0xFFFF8080
//#define COL_DARKRED       0xFF800000

#define COL_CYAN          0x07FF
//#define COL_LIGHTCYAN     0xFF80FFFF
//#define COL_DARKCYAN      0xFF008080

#define COL_MAGENTA       0xF81F
//#define COL_LIGHTMAGENTA  0xFFFF80FF
//#define COL_DARKMAGENTA   0xFF800080

#define COL_YELLOW        0xFFE0
//#define COL_LIGHTYELLOW   0xFFFFFF80
//#define COL_DARKYELLOW    0xFF808000
//#define COL_DARKERYELLOW  0xFF202000

//#define COL_BROWN         0xFFA52A2A
//#define COL_ORANGE        0xFFFFA500
//#define COL_DARKORANGE    0xFFC07800
//#define COL_DARKERORANGE  0xFF805000
//}}}
//{{{  screen resolution defines
#ifdef NEXXY_SCREEN
  // NEXXY 7 inch
  #define LTDC_CLOCK_4      130  // 32.5Mhz
  #define LCD_WIDTH         800
  #define LCD_HEIGHT       1280
  #define HORIZ_SYNC         64
  #define VERT_SYNC           1

  #define BOX_HEIGHT         30
  #define SMALL_FONT_HEIGHT  12
  #define FONT_HEIGHT        26
  #define BIG_FONT_HEIGHT    40

#else
  // ASUS eee 10 inch
  #define LTDC_CLOCK_4      100  // 25Mhz
  #define LCD_WIDTH        1024  // min 39Mhz typ 45Mhz max 51.42Mhz
  #define LCD_HEIGHT        600
  #define HORIZ_SYNC        136  // min  136  typ 176   max 216
  #define VERT_SYNC          12  // min   12  typ  25   max  38

  #define BOX_HEIGHT         20
  #define SMALL_FONT_HEIGHT  10
  #define FONT_HEIGHT        16
  #define BIG_FONT_HEIGHT    32
#endif
//}}}

class cWidget {
public:
  static uint16_t getBoxHeight() { return BOX_HEIGHT; }
  static uint16_t getSmallFontHeight() { return SMALL_FONT_HEIGHT; }
  static uint16_t getFontHeight() { return FONT_HEIGHT; }
  static uint16_t getBigFontHeight() { return BIG_FONT_HEIGHT; }
  };

class cFontChar;

class cLcd {
public:
  void* operator new (std::size_t size) { return pvPortMalloc (size); }
  void operator delete (void *ptr) { vPortFree (ptr); }

  //{{{
  class cTile {
  public:
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
  cLcd (uint32_t buffer0, uint32_t buffer1);
  ~cLcd() {}
  void init (std::string title);
  uint16_t getWidth() { return LCD_WIDTH; }
  uint16_t getHeight() { return LCD_HEIGHT; }

  void setShowDebug (bool title, bool info, bool lcdStats, bool footer);
  void info (uint16_t colour, std::string str);
  void info (std::string str);
  void debug (std::string str);

  void pixel (uint16_t colour, int16_t x, int16_t y);
  void rect (uint16_t colour, int16_t x, int16_t y, uint16_t width, uint16_t height);
  void stamp (uint16_t colour, uint8_t* src, int16_t x, int16_t y, uint16_t width, uint16_t height);
  void copy (cTile& srcTile, int16_t x, int16_t y);
  void copy90 (cTile& srcTile, int16_t x, int16_t y);
  void size (cTile& srcTile, int16_t x, int16_t y, uint16_t width, uint16_t height);
  void sizeCpu (cTile& srcTile, int16_t x, int16_t y, uint16_t width, uint16_t height);
  void sizeCpuBi (cTile& srcTile, int16_t x, int16_t y, uint16_t width, uint16_t height);

  void clear (uint16_t colour);
  void pixelClipped (uint16_t colour, int16_t x, int16_t y);
  void stampClipped (uint16_t colour, uint8_t* src, int16_t x, int16_t y, uint16_t width, uint16_t height);
  void rectClipped (uint16_t colour, int16_t x, int16_t y, uint16_t width, uint16_t height);
  void rectOutline (uint16_t colour, int16_t x, int16_t y, uint16_t width, uint16_t height, uint8_t thickness);
  void ellipse (uint16_t colour, int16_t x, int16_t y, uint16_t xradius, uint16_t yradius);
  void ellipseOutline (uint16_t colour, int16_t x, int16_t y, uint16_t xradius, uint16_t yradius);
  void line (uint16_t colour, int16_t x1, int16_t y1, int16_t x2, int16_t y2);
  int text (uint16_t colour, uint16_t fontHeight, std::string str, int16_t x, int16_t y, uint16_t width, uint16_t height);

  void startRender();
  void endRender (bool forceInfo);
  void render();
  uint32_t ready();

  void displayOn();
  void displayOff();
  void press (int pressCount, int16_t x, int16_t y, uint16_t z, int16_t xinc, int16_t yinc);

private:
  //{{{  const
  const uint32_t kLtdcFormat = LTDC_PIXEL_FORMAT_RGB565;

  const uint32_t kDstFormat = DMA2D_RGB565;
  const uint16_t kDstComponents = 2;

  const uint32_t kTempFormat = DMA2D_RGB565;
  const uint16_t kTempComponents = 2;
  //}}}
  typedef std::map<uint16_t, cFontChar*> cFontCharMap;

  void ltdcInit (uint32_t frameBufferAddress);
  void layerInit (uint8_t layer, uint32_t frameBufferAddress);
  void setLayer (uint8_t layer, uint32_t frameBufferAddress);
  void showLayer (uint8_t layer, uint32_t frameBufferAddress, uint8_t alpha);
  uint32_t wait();

  cFontChar* loadChar (uint16_t fontHeight, char ch);
  void reset();
  void displayTop();
  void displayTail();
  void updateNumDrawLines();

  //{{{  vars
  LTDC_HandleTypeDef LtdcHandler;

  int mStartTime = 0;

  float mFirstLine = 0;
  int mNumDrawLines = 0;
  int mStringPos = 0;

  bool mShowTitle = true;
  bool mShowInfo = true;
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

  int mDrawStartTime = 0;
  int mDrawTime = 0;
  bool mDrawBuffer = false;
  uint32_t mBuffer[2] = {0,0};
  bool mWait = false;

  uint32_t mCurFrameBufferAddress = 0;
  uint32_t showFrameBufferAddress[2];
  uint8_t showAlpha[2];

  cFontCharMap mFontCharMap;

  FT_Library FTlibrary;
  FT_Face FTface;
  FT_GlyphSlot FTglyphSlot;
  //}}}
  };
