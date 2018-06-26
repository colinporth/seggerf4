// main.cpp
#include "stm32f429i_discovery.h"
//{{{  sdRam defines
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)

#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)

#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)

#define SDRAM_BANK1_ADDR  ((uint16_t*)0xC0000000)
#define SDRAM_BANK1_LEN    ((uint32_t)0x01000000)

#define SDRAM_BANK2_ADDR  ((uint16_t*)0xD0000000)
#define SDRAM_BANK2_LEN    ((uint32_t)0x01000000)
//}}}
//{{{
void systemClockConfig() {

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG (PWR_REGULATOR_VOLTAGE_SCALE1);

  // Enable HSE Oscillator and activate PLL with HSE as source
  RCC_OscInitTypeDef rccOscConfig;
  rccOscConfig.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  rccOscConfig.HSEState = RCC_HSE_ON;
  rccOscConfig.PLL.PLLState = RCC_PLL_ON;
  rccOscConfig.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  rccOscConfig.PLL.PLLM = 8;
  rccOscConfig.PLL.PLLN = 200;               // 168mhz
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

    FMC_SDRAM_COLUMN_BITS_NUM_9 |
    FMC_SDRAM_ROW_BITS_NUM_12 |
    FMC_SDRAM_INTERN_BANKS_NUM_4 |
    FMC_SDRAM_MEM_BUS_WIDTH_16  |
    FMC_SDRAM_CAS_LATENCY_3  |
    FMC_SDRAM_WRITE_PROTECTION_DISABLE;

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
  HAL_Delay (2);

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
  //{{{  send autoRefresh command
  FMC_SDRAM_DEVICE->SDCMR = FMC_SDRAM_CMD_AUTOREFRESH_MODE |
                            FMC_SDRAM_CMD_TARGET_BANK1 | FMC_SDRAM_CMD_TARGET_BANK2 |
                            ((kAutoRefreshNumber-1) << 5);
  while (HAL_IS_BIT_SET (FMC_SDRAM_DEVICE->SDSR, FMC_SDSR_BUSY)) {}
  //}}}
  //{{{  send loadMode command
  FMC_SDRAM_DEVICE->SDCMR =
    FMC_SDRAM_CMD_LOAD_MODE |
    FMC_SDRAM_CMD_TARGET_BANK1 | FMC_SDRAM_CMD_TARGET_BANK2 |
    ((SDRAM_MODEREG_WRITEBURST_MODE_SINGLE | SDRAM_MODEREG_CAS_LATENCY_3 | SDRAM_MODEREG_BURST_LENGTH_1) << 9);
  while (HAL_IS_BIT_SET (FMC_SDRAM_DEVICE->SDSR, FMC_SDSR_BUSY)) {}
  //}}}
  FMC_SDRAM_DEVICE->SDRTR |= 0x0569 << 1;
  }
//}}}

void sdRamTest (uint32_t fromValue, uint32_t toValue, uint16_t* addr, uint32_t len) {

  for (uint32_t i = fromValue; i <= toValue; i++) {
    uint16_t data = i;
    auto writeAddress = addr;
    for (uint32_t j = 0; j < len/2; j++)
      *writeAddress++ = data++;

    uint32_t readOk = 0;
    uint32_t readErr = 0;
    auto readAddress = addr;
    for (uint32_t j = 0; j < len / 2; j++) {
      uint16_t readWord1 = *readAddress++;
      if (readWord1 == ((j+i) & 0xFFFF))
        readOk++;
      else {
        if (readErr < 4)
          printf ("- error %p %02x %d - r:%04x != %04x\n", readAddress, i, readErr, readWord1, (j+i) & 0xFFFF);
        readErr++;
        }
      }
    printf ("%p i:%x ok:%x error:%x %d\n", addr, i, readOk, readErr, (readOk * 100) / (len/2));
    }
  }

int main() {

  HAL_Init();
  systemClockConfig();

  sdRamInit();
  while (true)
    sdRamTest (0, 0xff, SDRAM_BANK1_ADDR, 0x01000000);
  }
