// main.cpp
//{{{  includes
#include "cmsis_os.h"
#include "cpuUsage.h"

#include <vector>
#include "cLcd.h"
#include "cTraceVec.h"

#include "stm32f429i_discovery.h"
#include "gyro.h"

#include "sd.h"
#include "../fatFs/ff.h"
#include "../FatFs/diskio.h"

#include "jpeglib.h"
//}}}
//{{{  sdram defines
#define SDRAM_BANK1_ADDR  ((uint16_t*)0xC0000000)
#define SDRAM_BANK1_LEN    ((uint32_t)0x00800000)

#define SDRAM_BANK2_ADDR  ((uint16_t*)0xD0000000)
#define SDRAM_BANK2_LEN    ((uint32_t)0x01000000)

#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)
//}}}
//{{{  fatfs defines
#define QUEUE_SIZE      10
#define READ_CPLT_MSG   1
#define WRITE_CPLT_MSG  2
//}}}

//{{{  const
const std::string kHello = std::string(__TIME__) + " " + std::string(__DATE__);

const HeapRegion_t kHeapRegions[] = {
  {(uint8_t*)SDRAM_BANK1_ADDR, SDRAM_BANK1_LEN },
  {(uint8_t*)SDRAM_BANK2_ADDR + (LCD_WIDTH*LCD_HEIGHT*4), SDRAM_BANK2_LEN - (LCD_WIDTH*LCD_HEIGHT*4) },
  { nullptr, 0 } };
//}}}
//{{{  var
cLcd* lcd = nullptr;
cTraceVec mTraceVec;

FATFS SDFatFs;
char SDPath[4];

std::vector<std::string> mFileVec;
std::vector<cTile*> mTileVec;
//}}}

//{{{
extern "C" { void EXTI0_IRQHandler() {
  HAL_GPIO_EXTI_IRQHandler (GPIO_PIN_0);
  }
}
//}}}
//{{{
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin) {

  if (GPIO_Pin == GPIO_PIN_0)
    lcd->toggle();
  }
//}}}

//{{{
void SystemClockConfig() {
//  System Clock source            = PLL (HSE)
//    SYSCLK(Hz)                     = 180000000
//    HCLK(Hz)                       = 180000000
//    AHB Prescaler                  = 1
//    APB1 Prescaler                 = 4
//    APB2 Prescaler                 = 2
//    HSE Frequency(Hz)              = 8000000
//    PLL_M                          = 8
//    PLL_N                          = 384
//    PLL_P                          = 2
//    PLL_Q                          = 7
//    VDD(V)                         = 3.3
//    Main regulator output voltage  = Scale1 mode
//    Flash Latency(WS)              = 5

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG (PWR_REGULATOR_VOLTAGE_SCALE1);

  // Enable HSE Oscillator and activate PLL with HSE as source
  RCC_OscInitTypeDef rccOscConfig;
  rccOscConfig.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  rccOscConfig.HSEState = RCC_HSE_ON;
  rccOscConfig.PLL.PLLState = RCC_PLL_ON;
  rccOscConfig.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  rccOscConfig.PLL.PLLM = 8;
  rccOscConfig.PLL.PLLN = 384;               // 192Mhz
  rccOscConfig.PLL.PLLP = RCC_PLLP_DIV2;
  rccOscConfig.PLL.PLLQ = 7;
  HAL_RCC_OscConfig (&rccOscConfig);

  HAL_PWREx_EnableOverDrive();

  //rccPeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDIO | RCC_PERIPHCLK_CK48;
  //rccPeriphClkInit.SdioClockSelection = RCC_SDIOCLKSOURCE_CK48;
  //rccPeriphClkInit.Clk48ClockSelection = RCC_CK48CLKSOURCE_PLLSAIP;
  //rccPeriphClkInit.PLLSAI.PLLSAIN = 384;
  //rccPeriphClkInit.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  //HAL_RCCEx_PeriphCLKConfig (&rccPeriphClkInit);

  // Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers
  RCC_ClkInitTypeDef rccClkConfig;
  rccClkConfig.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                           RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
  rccClkConfig.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  rccClkConfig.AHBCLKDivider = RCC_SYSCLK_DIV1;
  rccClkConfig.APB1CLKDivider = RCC_HCLK_DIV4;
  rccClkConfig.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig (&rccClkConfig, FLASH_LATENCY_5);
  }
//}}}
//{{{
void sdRamInit() {
//{{{  pins
// SDCLK = 90 MHz - HCLK 180MHz/2
//   PG08 -> FMC_SDCLK
//   PC00 -> FMC_SDNWE
// data
//   PD1 4..15 <-> FMC_D00..01
//   PD00..01  <-> FMC_D02..03
//   PE07..15  <-> FMC_D04..12
//   PD08..10  <-> FMC_D13..15
// address
//   PF00..05 -> FMC_A00..05
//   PF12..15 -> FMC_A06..09
//   PG00..01 -> FMC_A10..11
//   PE00 -> FMC_NBL0
//   PE01 -> FMC_NBL1
//   PG15 -> FMC_NCAS
//   PF11 -> FMC_NRAS
// BANK1 address 0xC0000000
//   PC02 -> FMC_SDNE0
//   PC03 -> FMC_SDCKE0
// BANK2 address 0xD0000000
//   PB06 -> FMC_SDNE1
//   PB05 -> FMC_SDCKE1
//}}}

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_FMC_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_Init_Structure;
  GPIO_Init_Structure.Pull = GPIO_NOPULL;
  GPIO_Init_Structure.Mode = GPIO_MODE_AF_PP;
  GPIO_Init_Structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_Init_Structure.Alternate = GPIO_AF12_FMC;
  //{{{  gpioB
  GPIO_Init_Structure.Pin = GPIO_PIN_5 | GPIO_PIN_6;
  HAL_GPIO_Init (GPIOB, &GPIO_Init_Structure);
  //}}}
  //{{{  gpioC
  GPIO_Init_Structure.Pin = GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3;
  HAL_GPIO_Init (GPIOC, &GPIO_Init_Structure);
  //}}}
  //{{{  gpioD
  GPIO_Init_Structure.Pin = GPIO_PIN_0 | GPIO_PIN_1  | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                            GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init (GPIOD, &GPIO_Init_Structure);
  //}}}
  //{{{  gpioE
  GPIO_Init_Structure.Pin = GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_7 | GPIO_PIN_8  | GPIO_PIN_9  |
                            GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init (GPIOE, &GPIO_Init_Structure);
  //}}}
  //{{{  gpioF
  GPIO_Init_Structure.Pin = GPIO_PIN_0  | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3  | GPIO_PIN_4 | GPIO_PIN_5 |
                            GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init (GPIOF, &GPIO_Init_Structure);
  //}}}
  //{{{  gpioG
  GPIO_Init_Structure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_15;
  HAL_GPIO_Init (GPIOG, &GPIO_Init_Structure);
  //}}}

  //{{{  bank command
  // 64m
  const uint32_t kBank1Command =
    FMC_SDRAM_CLOCK_PERIOD_2 |
    FMC_SDRAM_RBURST_ENABLE |
    FMC_SDRAM_RPIPE_DELAY_1 |

    FMC_SDRAM_COLUMN_BITS_NUM_8 |
    FMC_SDRAM_ROW_BITS_NUM_12 |
    FMC_SDRAM_INTERN_BANKS_NUM_4 |
    FMC_SDRAM_MEM_BUS_WIDTH_16  |
    FMC_SDRAM_CAS_LATENCY_3  |
    FMC_SDRAM_WRITE_PROTECTION_DISABLE;

  // 128m
  const uint32_t kBank2Command =
    FMC_SDRAM_COLUMN_BITS_NUM_9 |
    FMC_SDRAM_ROW_BITS_NUM_12 |
    FMC_SDRAM_INTERN_BANKS_NUM_4 |
    FMC_SDRAM_MEM_BUS_WIDTH_16 |
    FMC_SDRAM_CAS_LATENCY_3 |
    FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  //}}}
  FMC_SDRAM_DEVICE->SDCR[FMC_SDRAM_BANK1] = kBank1Command;
  FMC_SDRAM_DEVICE->SDCR[FMC_SDRAM_BANK2] = kBank2Command;

  //{{{  bank timing
  const uint32_t kRowCycleDelay        = 7; // tRC:  min = 63 (6 x 11.90ns)
  const uint32_t kRPDelay              = 2; // tRP:  15ns => 2 x 11.90ns
  const uint32_t kLoadToActiveDelay    = 2; // tMRD: 2 Clock cycles
  const uint32_t kExitSelfRefreshDelay = 7; // tXSR: min = 70ns (6 x 11.90ns)
  const uint32_t kSelfRefreshTime      = 4; // tRAS: min = 42ns (4 x 11.90ns) max=120k (ns)
  const uint32_t kWriteRecoveryTime    = 2; // tWR:  2 Clock cycles
  const uint32_t kRCDDelay             = 2; // tRCD: 15ns => 2 x 11.90ns

  const uint32_t kBank1Timing = (kLoadToActiveDelay-1)          |
                               ((kExitSelfRefreshDelay-1) << 4) |
                               ((kSelfRefreshTime-1) << 8)      |
                               ((kRowCycleDelay-1) << 12)       |
                               ((kWriteRecoveryTime-1) <<16)    |
                               ((kRPDelay-1) << 20)             |
                               ((kRCDDelay-1) << 24);

  const uint32_t kBank2Timing = (kLoadToActiveDelay-1)          |
                               ((kExitSelfRefreshDelay-1) << 4) |
                               ((kSelfRefreshTime-1) << 8)      |
                               ((kWriteRecoveryTime-1) <<16)    |
                               ((kRCDDelay-1) << 24);
  //}}}
  FMC_SDRAM_DEVICE->SDTR[FMC_SDRAM_BANK1] = kBank1Timing;
  FMC_SDRAM_DEVICE->SDTR[FMC_SDRAM_BANK2] = kBank2Timing;

  //{{{  send clockEnable command
  FMC_SDRAM_DEVICE->SDCMR = FMC_SDRAM_CMD_CLK_ENABLE |
                            FMC_SDRAM_CMD_TARGET_BANK1 | FMC_SDRAM_CMD_TARGET_BANK2;
  while (HAL_IS_BIT_SET (FMC_SDRAM_DEVICE->SDSR, FMC_SDSR_BUSY)) {}
  //}}}
  //HAL_Delay (100);

  //{{{  send PALL prechargeAll command
  FMC_SDRAM_DEVICE->SDCMR = FMC_SDRAM_CMD_PALL |
                            FMC_SDRAM_CMD_TARGET_BANK1 | FMC_SDRAM_CMD_TARGET_BANK2;
  while (HAL_IS_BIT_SET (FMC_SDRAM_DEVICE->SDSR, FMC_SDSR_BUSY)) {}
  //}}}
  //{{{  send autoRefresh command
  const uint32_t kAutoRefreshNumber = 4;
  FMC_SDRAM_DEVICE->SDCMR = FMC_SDRAM_CMD_AUTOREFRESH_MODE |
                            FMC_SDRAM_CMD_TARGET_BANK1 | FMC_SDRAM_CMD_TARGET_BANK2 |
                            ((kAutoRefreshNumber-1) << 5);
  while (HAL_IS_BIT_SET (FMC_SDRAM_DEVICE->SDSR, FMC_SDSR_BUSY)) {}
  //}}}
  //{{{  send loadMode command
  FMC_SDRAM_DEVICE->SDCMR =
    FMC_SDRAM_CMD_LOAD_MODE |
    FMC_SDRAM_CMD_TARGET_BANK1 | FMC_SDRAM_CMD_TARGET_BANK2 |
    ((SDRAM_MODEREG_WRITEBURST_MODE_SINGLE | SDRAM_MODEREG_CAS_LATENCY_3 | SDRAM_MODEREG_BURST_LENGTH_2) << 9);
  while (HAL_IS_BIT_SET (FMC_SDRAM_DEVICE->SDSR, FMC_SDSR_BUSY)) {}
  //}}}
  FMC_SDRAM_DEVICE->SDRTR |= 0x0569 << 1;
  }
//}}}
//{{{
void sdRamTest (int iterations, uint16_t* addr, uint32_t len) {

  while (iterations--) {
    // write
    auto writeAddress = addr;
    for (uint32_t i = 0; i < len/2; i++)
      *writeAddress++ = (i+iterations) & 0xFFFF;

    // read
    int32_t readOk = 0;
    int32_t readErr = 0;
    auto readAddress = addr;
    for (uint32_t i = 0; i < len/2; i++) {
      uint16_t read = *readAddress++;
      if (read == ((i+iterations) & 0xFFFF)) {
        //printf ("ok %p read:%x == %x\n", readAddress, read, (i+phase) & 0xFF);
        readOk++;
        }
      else {
        printf ("error %p read:%x != %x\n", readAddress, read, (i+iterations) & 0xFFFF);
        readErr++;
        }

      if ((i & 0xFFFFF) == 0xFFFFF) {
        if (readErr > 0) {
          printf ("%p errors:%x %x\n", readAddress, readErr, readOk);
          //lcd->info ("errors " + dec((uint32_t)readAddress) + " " + dec(readErr));
          //lcd->render();
          }
        else {
          printf ("ok %p %x\n", readAddress, readOk);
          //lcd->info ("ok " + dec((uint32_t)readAddress));
          //lcd->render();
          }
        readOk = 0;
        readErr = 0;
        }
      }
    }
  }
//}}}

//{{{
void findFiles (const std::string& dirPath, const std::string ext) {

  DIR dir;
  if (f_opendir (&dir, dirPath.c_str()) == FR_OK) {
    while (true) {
      FILINFO filinfo;
      if (f_readdir (&dir, &filinfo) != FR_OK || !filinfo.fname[0])
        break;
      if (filinfo.fname[0] == '.')
        continue;

      std::string filePath = dirPath + "/" + filinfo.fname;
      if (filinfo.fattrib & AM_DIR)
        findFiles (filePath, ext);
      else {
        auto found = filePath.find (ext);
        if (found == filePath.size() - 4)
          mFileVec.push_back (filePath);
        }
      }

    f_closedir (&dir);
    }
  }
//}}}
//{{{
cTile* loadFile (const std::string& fileName, int scale) {

  FILINFO filInfo;
  if (f_stat (fileName.c_str(), &filInfo)) {
    lcd->info (COL_RED, fileName + " not found");
    return nullptr;
    }
  lcd->info ("loadFile " + fileName + " bytes:" + dec ((int)(filInfo.fsize)) + " " +
             dec (filInfo.ftime >> 11) + ":" + dec ((filInfo.ftime >> 5) & 63) + " " +
             dec (filInfo.fdate & 31) + ":" + dec ((filInfo.fdate >> 5) & 15) + ":" + dec ((filInfo.fdate >> 9) + 1980)
             );

  FIL gFile;
  if (f_open (&gFile, fileName.c_str(), FA_READ)) {
    lcd->info (COL_RED, fileName + " not opened");
    return nullptr;
    }

  auto buf = (uint8_t*)pvPortMalloc (filInfo.fsize);
  if (!buf)
    lcd->info (COL_RED, "buf fail");

  UINT bytesRead = 0;
  f_read (&gFile, buf, (UINT)filInfo.fsize, &bytesRead);
  f_close (&gFile);

  if (bytesRead > 0) {
    struct jpeg_error_mgr jerr;
    struct jpeg_decompress_struct mCinfo;
    mCinfo.err = jpeg_std_error (&jerr);
    jpeg_create_decompress (&mCinfo);

    jpeg_mem_src (&mCinfo, buf, bytesRead);
    jpeg_read_header (&mCinfo, TRUE);

    mCinfo.dct_method = JDCT_FLOAT;
    mCinfo.out_color_space = JCS_RGB;
    mCinfo.scale_num = 1;
    mCinfo.scale_denom = scale;
    jpeg_start_decompress (&mCinfo);

    auto rgb565pic = (uint16_t*)pvPortMalloc (mCinfo.output_width * mCinfo.output_height * 2);
    auto tile = new cTile ((uint8_t*)rgb565pic, 2, mCinfo.output_width, 0,0, mCinfo.output_width, mCinfo.output_height);

    auto rgbLine = (uint8_t*)malloc (mCinfo.output_width * 3);
    while (mCinfo.output_scanline < mCinfo.output_height) {
      jpeg_read_scanlines (&mCinfo, &rgbLine, 1);
      lcd->rgb888to565 (rgbLine, rgb565pic + ((mCinfo.output_scanline-1) * mCinfo.output_width), mCinfo.output_width);
      }
    free (rgbLine);

    vPortFree (buf);
    jpeg_finish_decompress (&mCinfo);

    lcd->info (COL_YELLOW, "loaded " + dec(mCinfo.image_width) + "x" + dec(mCinfo.image_height) + " " +
                                       dec(mCinfo.output_width) + "x" + dec(mCinfo.output_height));
    jpeg_destroy_decompress (&mCinfo);

    return tile;
    }
  else {
    lcd->info (COL_RED, "loadFile read failed");
    vPortFree (buf);
    return nullptr;
    }
  }
//}}}

//{{{
void displayThread (void* arg) {

  lcd->render();
  lcd->display (50);

  auto lastUsage = osGetCPUUsage();
  while (true) {
    if (lcd->changed() || (osGetCPUUsage() != lastUsage)) {
      lastUsage = osGetCPUUsage();
      lcd->start();
      lcd->clear (COL_BLACK);

      int items = mTileVec.size();
      int rows = int(sqrt (float(items))) + 1;
      int count = 0;
      for (auto tile : mTileVec) {
        lcd->copy (tile, cPoint  (
                    (lcd->getWidth() / rows) * (count % rows) + (lcd->getWidth() / rows - tile->mWidth) / 2,
                    (lcd->getHeight() / rows) * (count / rows) + (lcd->getHeight() / rows - tile->mHeight) / 2));
        count++;
        }

      lcd->drawInfo();
      mTraceVec.draw (lcd, 20, lcd->getHeight()-40);
      lcd->present();
      }
    else
      vTaskDelay (1);
    }
  }
//}}}
//{{{
void loadThread (void* arg) {

  if (FATFS_LinkDriver (&SD_Driver, SDPath) != 0)
    lcd->info (COL_RED, "sdCard - no driver");
  else if (f_mount (&SDFatFs, (TCHAR const*)SDPath, 1) != FR_OK)
    lcd->info (COL_RED, "sdCard - not mounted");
  else {
    // get label
    char label[20] = {0};
    DWORD volumeSerialNumber = 0;
    f_getlabel ("", label, &volumeSerialNumber);
    lcd->info ("sdCard mounted label:" + std::string(label));

    findFiles ("", ".jpg");
    for (auto file : mFileVec) {
      mTileVec.push_back (loadFile (file, 4));
      lcd->change();
      }
    }

  while (true)
    vTaskDelay (1000);
  }
//}}}
//{{{
void gyroThread (void* arg) {

  auto id = gyroInit();
  lcd->info ("gyroId " + dec (id));

  while (true) {
    if (gyroGetFifoSrc() & 0x20)
      vTaskDelay (10);
    else {
      int16_t xyz[3];
      gyroGetXYZ (xyz);
      mTraceVec.addSample (0, xyz[0]);
      mTraceVec.addSample (1, xyz[1]);
      mTraceVec.addSample (2, xyz[2]);
      lcd->change();
      }
    }
  }
//}}}

int main() {

  HAL_Init();
  SystemClockConfig();
  sdRamInit();
  vPortDefineHeapRegions (kHeapRegions);
  BSP_PB_Init (BUTTON_KEY, BUTTON_MODE_EXTI);

  lcd = new cLcd (SDRAM_BANK2_ADDR, SDRAM_BANK2_ADDR+ (LCD_WIDTH*LCD_HEIGHT*2));
  lcd->init (kHello);

  mTraceVec.addTrace (1024, 1, 3);

  TaskHandle_t displayHandle;
  xTaskCreate ((TaskFunction_t)displayThread, "app", 1024, 0, 4, &displayHandle);
  TaskHandle_t loadHandle;
  xTaskCreate ((TaskFunction_t)loadThread, "load", 8192, 0, 2, &loadHandle);
  //TaskHandle_t gyroHandle;
  //xTaskCreate ((TaskFunction_t)gyroThread, "load", 1024, 0, 4, &gyroHandle);

  vTaskStartScheduler();
  }
