// audio.c
#include "audio.h"
#include "stm32f4xx_hal.h"
#include "pdm2pcm_glo.h"

//{{{  i2c defines
#define BSP_I2C_SPEED                            100000

#define DISCOVERY_I2Cx                            I2C1
#define DISCOVERY_I2Cx_CLK_ENABLE()               __HAL_RCC_I2C1_CLK_ENABLE()
#define DISCOVERY_I2Cx_SCL_SDA_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define DISCOVERY_I2Cx_SCL_SDA_AF                 GPIO_AF4_I2C1
#define DISCOVERY_I2Cx_SCL_SDA_GPIO_PORT          GPIOB
#define DISCOVERY_I2Cx_SCL_PIN                    GPIO_PIN_6
#define DISCOVERY_I2Cx_SDA_PIN                    GPIO_PIN_9

#define DISCOVERY_I2Cx_FORCE_RESET()              __HAL_RCC_I2C1_FORCE_RESET()
#define DISCOVERY_I2Cx_RELEASE_RESET()            __HAL_RCC_I2C1_RELEASE_RESET()

/* I2C interrupt requests */
#define DISCOVERY_I2Cx_EV_IRQn                    I2C1_EV_IRQn
#define DISCOVERY_I2Cx_ER_IRQn                    I2C1_ER_IRQn

#define I2Cx_TIMEOUT_MAX    0x1000 /*<! The value of the maximal timeout for BUS waiting loops */
//}}}
//{{{  i2s2 defines
#define I2S2                            SPI2
#define I2S2_CLK_ENABLE()               __HAL_RCC_SPI2_CLK_ENABLE()
#define I2S2_CLK_DISABLE()              __HAL_RCC_SPI2_CLK_DISABLE()
#define I2S2_SCK_PIN                    GPIO_PIN_10
#define I2S2_SCK_GPIO_PORT              GPIOB
#define I2S2_SCK_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2S2_SCK_AF                     GPIO_AF5_SPI2

#define I2S2_MOSI_PIN                   GPIO_PIN_3
#define I2S2_MOSI_GPIO_PORT             GPIOC
#define I2S2_MOSI_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()
#define I2S2_MOSI_AF                    GPIO_AF5_SPI2

// I2S DMA Stream Rx definitions
#define I2S2_DMAx_CLK_ENABLE()          __HAL_RCC_DMA1_CLK_ENABLE()
#define I2S2_DMAx_CLK_DISABLE()         __HAL_RCC_DMA1_CLK_DISABLE()
#define I2S2_DMAx_STREAM                DMA1_Stream3
#define I2S2_DMAx_CHANNEL               DMA_CHANNEL_0
#define I2S2_DMAx_IRQ                   DMA1_Stream3_IRQn
#define I2S2_DMAx_PERIPH_DATA_SIZE      DMA_PDATAALIGN_HALFWORD
#define I2S2_DMAx_MEM_DATA_SIZE         DMA_MDATAALIGN_HALFWORD

#define I2S2_IRQHandler                 DMA1_Stream3_IRQHandler

// Select the interrupt preemption priority and subpriority for the IT/DMA interrupt
#define AUDIO_IN_IRQ_PREPRIO            0x0F   /* Select the preemption priority level(0 is the highest) */
//}}}
//{{{  i2s3 defines
#define I2S3                            SPI3
#define I2S3_CLK_ENABLE()               __HAL_RCC_SPI3_CLK_ENABLE()
#define I2S3_CLK_DISABLE()              __HAL_RCC_SPI3_CLK_DISABLE()
#define I2S3_SCK_SD_WS_AF               GPIO_AF6_SPI3
#define I2S3_SCK_SD_CLK_ENABLE()        __HAL_RCC_GPIOC_CLK_ENABLE()
#define I2S3_MCK_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()
#define I2S3_WS_CLK_ENABLE()            __HAL_RCC_GPIOA_CLK_ENABLE()
#define I2S3_WS_PIN                     GPIO_PIN_4
#define I2S3_SCK_PIN                    GPIO_PIN_10
#define I2S3_SD_PIN                     GPIO_PIN_12
#define I2S3_MCK_PIN                    GPIO_PIN_7
#define I2S3_SCK_SD_GPIO_PORT           GPIOC
#define I2S3_WS_GPIO_PORT               GPIOA
#define I2S3_MCK_GPIO_PORT              GPIOC

// I2S DMA Stream definitions
#define I2S3_DMAx_CLK_ENABLE()          __HAL_RCC_DMA1_CLK_ENABLE()
#define I2S3_DMAx_CLK_DISABLE()         __HAL_RCC_DMA1_CLK_DISABLE()
#define I2S3_DMAx_STREAM                DMA1_Stream7
#define I2S3_DMAx_CHANNEL               DMA_CHANNEL_0
#define I2S3_DMAx_IRQ                   DMA1_Stream7_IRQn
#define I2S3_DMAx_PERIPH_DATA_SIZE      DMA_PDATAALIGN_HALFWORD
#define I2S3_DMAx_MEM_DATA_SIZE         DMA_MDATAALIGN_HALFWORD
#define DMA_MAX_SZE                     0xFFFF

#define I2S3_IRQHandler                 DMA1_Stream7_IRQHandler

// Select the interrupt preemption priority and subpriority for the DMA interrupt */
#define AUDIO_OUT_IRQ_PREPRIO           0x0E   /* Select the preemption priority level(0 is the highest) */
//}}}
//{{{  cs43122 defines
#define CODEC_STANDARD  0x04
#define I2S_STANDARD    I2S_STANDARD_PHILIPS

#define AUDIO_I2C_ADDRESS     0x94

// gpio
#define AUDIO_RESET_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()
#define AUDIO_RESET_PIN       GPIO_PIN_4
#define AUDIO_RESET_GPIO      GPIOD

// Volume Levels values
#define DEFAULT_VOLMIN        0x00
#define DEFAULT_VOLMAX        0xFF
#define DEFAULT_VOLSTEP       0x04

#define AUDIO_PAUSE           0
#define AUDIO_RESUME          1

// MUTE commands
#define AUDIO_MUTE_ON         1
#define AUDIO_MUTE_OFF        0

// AUDIO FREQUENCY
#define AUDIO_FREQUENCY_192K  ((uint32_t)192000)
#define AUDIO_FREQUENCY_96K   ((uint32_t)96000)
#define AUDIO_FREQUENCY_48K   ((uint32_t)48000)
#define AUDIO_FREQUENCY_44K   ((uint32_t)44100)
#define AUDIO_FREQUENCY_32K   ((uint32_t)32000)
#define AUDIO_FREQUENCY_22K   ((uint32_t)22050)
#define AUDIO_FREQUENCY_16K   ((uint32_t)16000)
#define AUDIO_FREQUENCY_11K   ((uint32_t)11025)
#define AUDIO_FREQUENCY_8K    ((uint32_t)8000)

// Chip ID Register: Chip I.D. and Revision Register
// [7:3] CHIPID[4:0]: I.D. code for the CS43L22.
//       Default value: 11100b
// [2:0] REVID[2:0]: CS43L22 revision level.
//       Default value:
//       000 - Rev A0
//       001 - Rev A1
//       010 - Rev B0
//       011 - Rev B1
#define CS43L22_CHIPID_ADDR  0x01
#define  CS43L22_ID             0xE0
#define  CS43L22_ID_MASK        0xF8
#define  CS43L22_REVISION_MASK  0x07

//{{{  registers
#define   CS43L22_REG_ID                  0x01
#define   CS43L22_REG_POWER_CTL1          0x02
#define   CS43L22_REG_POWER_CTL2          0x04
#define   CS43L22_REG_CLOCKING_CTL        0x05
#define   CS43L22_REG_INTERFACE_CTL1      0x06
#define   CS43L22_REG_INTERFACE_CTL2      0x07
#define   CS43L22_REG_PASSTHR_A_SELECT    0x08
#define   CS43L22_REG_PASSTHR_B_SELECT    0x09
#define   CS43L22_REG_ANALOG_ZC_SR_SETT   0x0A
#define   CS43L22_REG_PASSTHR_GANG_CTL    0x0C
#define   CS43L22_REG_PLAYBACK_CTL1       0x0D
#define   CS43L22_REG_MISC_CTL            0x0E
#define   CS43L22_REG_PLAYBACK_CTL2       0x0F
#define   CS43L22_REG_PASSTHR_A_VOL       0x14
#define   CS43L22_REG_PASSTHR_B_VOL       0x15
#define   CS43L22_REG_PCMA_VOL            0x1A
#define   CS43L22_REG_PCMB_VOL            0x1B
#define   CS43L22_REG_BEEP_FREQ_ON_TIME   0x1C
#define   CS43L22_REG_BEEP_VOL_OFF_TIME   0x1D
#define   CS43L22_REG_BEEP_TONE_CFG       0x1E
#define   CS43L22_REG_TONE_CTL            0x1F
#define   CS43L22_REG_MASTER_A_VOL        0x20
#define   CS43L22_REG_MASTER_B_VOL        0x21
#define   CS43L22_REG_HEADPHONE_A_VOL     0x22
#define   CS43L22_REG_HEADPHONE_B_VOL     0x23
#define   CS43L22_REG_SPEAKER_A_VOL       0x24
#define   CS43L22_REG_SPEAKER_B_VOL       0x25
#define   CS43L22_REG_CH_MIXER_SWAP       0x26
#define   CS43L22_REG_LIMIT_CTL1          0x27
#define   CS43L22_REG_LIMIT_CTL2          0x28
#define   CS43L22_REG_LIMIT_ATTACK_RATE   0x29
#define   CS43L22_REG_OVF_CLK_STATUS      0x2E
#define   CS43L22_REG_BATT_COMPENSATION   0x2F
#define   CS43L22_REG_VP_BATTERY_LEVEL    0x30
#define   CS43L22_REG_SPEAKER_STATUS      0x31
#define   CS43L22_REG_TEMPMONITOR_CTL     0x32
#define   CS43L22_REG_THERMAL_FOLDBACK    0x33
#define   CS43L22_REG_CHARGE_PUMP_FREQ    0x34
//}}}
//}}}
//{{{  defines
#define AUDIODATA_SIZE  2   // 16-bits audio data size

#define DMA_MAX(_X_)    (((_X_) <= DMA_MAX_SZE)? (_X_):DMA_MAX_SZE)
#define HTONS(A)        ((((uint16_t)(A) & 0xff00) >> 8) | (((uint16_t)(A) & 0x00ff) << 8))
#define VOLUME_CONVERT(Volume) (((Volume) > 100) ? 255 : ((uint8_t)(((Volume) * 255) / 100)))
//}}}
//{{{  clock const
// These PLL parameters are valid when the f(VCO clock) = 1Mhz
const uint32_t I2SFreq[8] = { 8000, 11025, 16000, 22050, 32000, 44100, 48000, 96000 };
const uint32_t I2SPLLN[8] = { 256, 429, 213, 429, 426, 271, 258, 344 };
const uint32_t I2SPLLR[8] = { 5, 4, 4, 4, 4, 6, 3, 1 };
//}}}

uint32_t I2cxTimeout = I2Cx_TIMEOUT_MAX;
static I2C_HandleTypeDef I2cHandle;

I2S_HandleTypeDef hAudioOutI2s;
volatile uint8_t gOutputDevice = 0;
static uint8_t gOutputStop = 1;

I2S_HandleTypeDef hAudioInI2s;
PDM_Filter_Handler_t PDM_FilterHandler[2];
PDM_Filter_Config_t PDM_FilterConfig[2];
uint16_t AudioInVolume = DEFAULT_AUDIO_IN_VOLUME;

void I2S2_IRQHandler() { HAL_DMA_IRQHandler(hAudioInI2s.hdmarx); }
void I2S3_IRQHandler() { HAL_DMA_IRQHandler (hAudioOutI2s.hdmatx); }

//{{{
static void i2cInit() {

  printf ("i2cInit\n");

  // Enable Reset GPIO Clock
  AUDIO_RESET_GPIO_CLK_ENABLE();

  // Audio reset pin configuration
  GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.Pin = AUDIO_RESET_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(AUDIO_RESET_GPIO, &GPIO_InitStruct);

  // DISCOVERY_I2Cx peripheral configuration */
  I2cHandle.Init.ClockSpeed = BSP_I2C_SPEED;
  I2cHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
  I2cHandle.Init.OwnAddress1 = 0x33;
  I2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle.Instance = DISCOVERY_I2Cx;

  // Enable I2C GPIO clocks
  DISCOVERY_I2Cx_SCL_SDA_GPIO_CLK_ENABLE();

  // DISCOVERY_I2Cx SCL and SDA pins configuration
  GPIO_InitStruct.Pin = DISCOVERY_I2Cx_SCL_PIN | DISCOVERY_I2Cx_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Alternate  = DISCOVERY_I2Cx_SCL_SDA_AF;
  HAL_GPIO_Init(DISCOVERY_I2Cx_SCL_SDA_GPIO_PORT, &GPIO_InitStruct);

  // Enable the DISCOVERY_I2Cx peripheral clock
  DISCOVERY_I2Cx_CLK_ENABLE();

  // Force the I2C peripheral clock reset
  DISCOVERY_I2Cx_FORCE_RESET();

  // Release the I2C peripheral clock reset
  DISCOVERY_I2Cx_RELEASE_RESET();

  // Enable and set I2Cx Interrupt to the highest priority
  HAL_NVIC_SetPriority (DISCOVERY_I2Cx_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ (DISCOVERY_I2Cx_EV_IRQn);

  // Enable and set I2Cx Interrupt to the highest priority
  HAL_NVIC_SetPriority (DISCOVERY_I2Cx_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ (DISCOVERY_I2Cx_ER_IRQn);

  HAL_I2C_Init (&I2cHandle);

  // Power Down the codec
  HAL_GPIO_WritePin (AUDIO_RESET_GPIO, AUDIO_RESET_PIN, GPIO_PIN_RESET);

  // Wait for a delay to insure registers erasing
  HAL_Delay (5);

  // Power on the codec
  HAL_GPIO_WritePin (AUDIO_RESET_GPIO, AUDIO_RESET_PIN, GPIO_PIN_SET);

  // Wait for a delay to insure registers erasing
  HAL_Delay (5);
  }
//}}}
//{{{
static uint8_t i2cRead (uint8_t Addr, uint8_t Reg) {

  uint8_t value = 0;
  if (HAL_I2C_Mem_Read(&I2cHandle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2Cx_TIMEOUT_MAX) != HAL_OK)
    printf ("i2cRead error %x %x %x\n", Addr, Reg, value);
  //else
  //  printf ("i2cRead %x %x %x\n", Addr, Reg, value);

  return value;
  }
//}}}
//{{{
static void i2cWrite (uint8_t Addr, uint8_t Reg, uint8_t Value) {

  if (HAL_I2C_Mem_Write(&I2cHandle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, I2Cx_TIMEOUT_MAX) != HAL_OK)
    printf ("i2cWrite error %x %x %x\n", Addr, Reg, Value);
  //else
  //  printf ("i2cWrite %x %x %x\n", Addr, Reg, Value);
  }
//}}}

//{{{
static void i2sInit (I2S_HandleTypeDef* hi2s, void* Params) {

  static DMA_HandleTypeDef hdma_i2sTx;

  // Enable I2S3 clock
  I2S3_CLK_ENABLE();

  // Enable I2S GPIO clocks
  I2S3_SCK_SD_CLK_ENABLE();
  I2S3_WS_CLK_ENABLE();

  // I2S3 pins configuration: WS, SCK and SD pins
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = I2S3_SCK_PIN | I2S3_SD_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = I2S3_SCK_SD_WS_AF;
  HAL_GPIO_Init (I2S3_SCK_SD_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = I2S3_WS_PIN ;
  HAL_GPIO_Init (I2S3_WS_GPIO_PORT, &GPIO_InitStruct);

  // I2S3 pins configuration: MCK pin
  I2S3_MCK_CLK_ENABLE();
  GPIO_InitStruct.Pin = I2S3_MCK_PIN;
  HAL_GPIO_Init (I2S3_MCK_GPIO_PORT, &GPIO_InitStruct);

  // Enable the I2S DMA clock
  I2S3_DMAx_CLK_ENABLE();
  if (hi2s->Instance == I2S3) {
    hdma_i2sTx.Instance                 = I2S3_DMAx_STREAM;
    hdma_i2sTx.Init.Channel             = I2S3_DMAx_CHANNEL;
    hdma_i2sTx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_i2sTx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_i2sTx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_i2sTx.Init.PeriphDataAlignment = I2S3_DMAx_PERIPH_DATA_SIZE;
    hdma_i2sTx.Init.MemDataAlignment    = I2S3_DMAx_MEM_DATA_SIZE;
    hdma_i2sTx.Init.Mode                = DMA_NORMAL;
    hdma_i2sTx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_i2sTx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma_i2sTx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_i2sTx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_i2sTx.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    __HAL_LINKDMA (hi2s, hdmatx, hdma_i2sTx);
    HAL_DMA_DeInit (&hdma_i2sTx);
    HAL_DMA_Init (&hdma_i2sTx);
    }

  // I2S DMA IRQ Channel configuration
  HAL_NVIC_SetPriority (I2S3_DMAx_IRQ, AUDIO_OUT_IRQ_PREPRIO, 0);
  HAL_NVIC_EnableIRQ (I2S3_DMAx_IRQ);
  }
//}}}
//{{{
static void i2sClockConfig (I2S_HandleTypeDef* hi2s, uint32_t AudioFreq, void* Params) {

  uint8_t index = 0;
  uint8_t freqindex = 0xFF;
  for (index = 0; index < 8; index++)
    if (I2SFreq[index] == AudioFreq)
      freqindex = index;

  // Enable PLLI2S clock */
  RCC_PeriphCLKInitTypeDef rccclkinit;
  HAL_RCCEx_GetPeriphCLKConfig(&rccclkinit);

  // PLLI2S_VCO Input = HSE_VALUE/PLL_M = 1 Mhz
  if ((freqindex & 0x7) == 0) {
    // I2S clock config
    // PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) × (PLLI2SN/PLLM)
    // I2SCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SR
    rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    rccclkinit.PLLI2S.PLLI2SN = I2SPLLN[freqindex];
    rccclkinit.PLLI2S.PLLI2SR = I2SPLLR[freqindex];
    HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
    }
  else {
    // I2S clock config
    // PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) × (PLLI2SN/PLLM)
    // I2SCLK = f(PLLI2S clock output) = f(VCO clock) / PLLI2SR
    rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    rccclkinit.PLLI2S.PLLI2SN = 258;
    rccclkinit.PLLI2S.PLLI2SR = 3;
    HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
    }
  }
//}}}
//{{{
static void i2s2Init (uint32_t AudioFreq) {

  printf ("i2s2Init\n");

  // Initialize the hAudioInI2s Instance parameter
  hAudioInI2s.Instance = I2S2;

  // Disable I2S block
  __HAL_I2S_DISABLE (&hAudioInI2s);

  // I2S2 peripheral configuration
  hAudioInI2s.Init.AudioFreq    = 2 * AudioFreq;
  hAudioInI2s.Init.ClockSource  = I2S_CLOCK_PLL;
  hAudioInI2s.Init.CPOL         = I2S_CPOL_HIGH;
  hAudioInI2s.Init.DataFormat   = I2S_DATAFORMAT_16B;
  hAudioInI2s.Init.MCLKOutput   = I2S_MCLKOUTPUT_DISABLE;
  hAudioInI2s.Init.Mode         = I2S_MODE_MASTER_RX;
  hAudioInI2s.Init.Standard     = I2S_STANDARD_LSB;

  // Initialize the I2S peripheral with the structure above
  if (HAL_I2S_Init(&hAudioInI2s) != HAL_OK)
    printf ("i2s2Init error\n");
  }
//}}}
//{{{
static void i2s3Init (uint32_t AudioFreq) {

  printf ("i2s3Init\n");

  // Initialize the hAudioOutI2s Instance parameter
  hAudioOutI2s.Instance = I2S3;

  // Disable I2S block
  __HAL_I2S_DISABLE (&hAudioOutI2s);

  // I2S3 peripheral configuration
  hAudioOutI2s.Init.AudioFreq   = AudioFreq;
  hAudioOutI2s.Init.ClockSource = I2S_CLOCK_PLL;
  hAudioOutI2s.Init.CPOL        = I2S_CPOL_LOW;
  hAudioOutI2s.Init.DataFormat  = I2S_DATAFORMAT_16B;
  hAudioOutI2s.Init.MCLKOutput  = I2S_MCLKOUTPUT_ENABLE;
  hAudioOutI2s.Init.Mode        = I2S_MODE_MASTER_TX;
  hAudioOutI2s.Init.Standard    = I2S_STANDARD;

  // Initialize the I2S peripheral with the structure above
  if (HAL_I2S_Init (&hAudioOutI2s) != HAL_OK)
    printf ("i2s3Init error\n");
  }
//}}}
//{{{
static void pdmDecoderInit (uint32_t AudioFreq, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut) {

  printf ("pdmDecoderInit\n");

  // Enable CRC peripheral to unlock the PDM library
  __HAL_RCC_CRC_CLK_ENABLE();

  uint32_t index = 0;
  for (index = 0; index < ChnlNbrIn; index++) {
    // Init PDM filters
    PDM_FilterHandler[index].bit_order = PDM_FILTER_BIT_ORDER_LSB;
    PDM_FilterHandler[index].endianness = PDM_FILTER_ENDIANNESS_LE;
    PDM_FilterHandler[index].high_pass_tap = 2122358088;
    PDM_FilterHandler[index].out_ptr_channels = ChnlNbrOut;
    PDM_FilterHandler[index].in_ptr_channels = ChnlNbrIn;
    PDM_Filter_Init ((PDM_Filter_Handler_t *)(&PDM_FilterHandler[index]));

    // PDM lib config phase
    PDM_FilterConfig[index].output_samples_number = AudioFreq / 1000;
    PDM_FilterConfig[index].mic_gain = 24;
    PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_64;
    PDM_Filter_setConfig ((PDM_Filter_Handler_t*)&PDM_FilterHandler[index], &PDM_FilterConfig[index]);
    }
  }
//}}}
//{{{
static void SetVolume (uint16_t deviceAddr, uint8_t volume) {

  uint8_t convertedVolume = VOLUME_CONVERT (volume);
  if (convertedVolume > 0xE6) {
    // Set the Master volume
    i2cWrite (deviceAddr, CS43L22_REG_MASTER_A_VOL, convertedVolume - 0xE7);
    i2cWrite (deviceAddr, CS43L22_REG_MASTER_B_VOL, convertedVolume - 0xE7);
    }
  else {
    // Set the Master volume
    i2cWrite (deviceAddr, CS43L22_REG_MASTER_A_VOL, convertedVolume + 0x19);
    i2cWrite (deviceAddr, CS43L22_REG_MASTER_B_VOL, convertedVolume + 0x19);
    }
  }
//}}}
//{{{
static void SetOutputMode (uint16_t deviceAddr, uint8_t Output) {

  switch (Output) {
    case OUTPUT_DEVICE_SPEAKER:
      gOutputDevice = 0xFA;
      break;

    case OUTPUT_DEVICE_HEADPHONE:
      gOutputDevice = 0xAF;
      break;

    case OUTPUT_DEVICE_BOTH:
      gOutputDevice = 0xAA;
      break;

    default:
      gOutputDevice = 0x05;
      break;
      }

  i2cWrite (deviceAddr, CS43L22_REG_POWER_CTL2, gOutputDevice);
  }
//}}}
//{{{
static void SetMute (uint16_t deviceAddr, uint32_t cmd) {

  // Set the Mute mode
  if (cmd == AUDIO_MUTE_ON) {
    i2cWrite (deviceAddr, CS43L22_REG_POWER_CTL2, 0xFF);
    i2cWrite (deviceAddr, CS43L22_REG_HEADPHONE_A_VOL, 0x01);
    i2cWrite (deviceAddr, CS43L22_REG_HEADPHONE_B_VOL, 0x01);
    }
  else {
    // AUDIO_MUTE_OFF Disable the Mute
    i2cWrite (deviceAddr, CS43L22_REG_HEADPHONE_A_VOL, 0x00);
    i2cWrite (deviceAddr, CS43L22_REG_HEADPHONE_B_VOL, 0x00);
    i2cWrite (deviceAddr, CS43L22_REG_POWER_CTL2, gOutputDevice);
    }
  }
//}}}

//{{{
static void Init (uint16_t deviceAddr, uint16_t gOutputDeviceice, uint8_t Volume, uint32_t AudioFreq) {

  printf ("Init\n");

  // keep Codec powered OFF
  i2cWrite (deviceAddr, CS43L22_REG_POWER_CTL1, 0x01);

  SetOutputMode (deviceAddr, gOutputDeviceice);

  // Clock configuration - autoDetection
  i2cWrite (deviceAddr, CS43L22_REG_CLOCKING_CTL, 0x81);

  // set Slave Mode and audioStandard
  i2cWrite (deviceAddr, CS43L22_REG_INTERFACE_CTL1, CODEC_STANDARD);

  // set master volume
  SetVolume (deviceAddr, Volume);

  // if Speaker is enabled, set Mono mode and volume attenuation level
  if (gOutputDeviceice != OUTPUT_DEVICE_HEADPHONE) {
    // set speaker Mono mode
    i2cWrite (deviceAddr, CS43L22_REG_PLAYBACK_CTL2, 0x06);

    // set speaker attenuation level
    i2cWrite (deviceAddr, CS43L22_REG_SPEAKER_A_VOL, 0x00);
    i2cWrite (deviceAddr, CS43L22_REG_SPEAKER_B_VOL, 0x00);
    }

  // disable analog soft ramp
  i2cWrite (deviceAddr, CS43L22_REG_ANALOG_ZC_SR_SETT, 0x00);

  // disable digital soft ramp
  i2cWrite (deviceAddr, CS43L22_REG_MISC_CTL, 0x04);

  // disable limiter attack level
  i2cWrite (deviceAddr, CS43L22_REG_LIMIT_CTL1, 0x00);

  // adjust Bass,Treble levels
  i2cWrite (deviceAddr, CS43L22_REG_TONE_CTL, 0x0F);

  // adjust PCM volume level
  i2cWrite (deviceAddr, CS43L22_REG_PCMA_VOL, 0x0A);
  i2cWrite (deviceAddr, CS43L22_REG_PCMB_VOL, 0x0A);
  }
//}}}
//{{{
static uint8_t ReadId (uint16_t deviceAddr) {

  i2cInit();

  uint32_t value = i2cRead (deviceAddr, CS43L22_CHIPID_ADDR);
  printf ("ReadID %x id:%x revision:%x\n", value, value & CS43L22_ID_MASK, value & CS43L22_REVISION_MASK);
  return value;
  }
//}}}
//{{{
static void Play (uint16_t deviceAddr, uint16_t* pBuffer, uint16_t Size) {

  if (gOutputStop == 1) {
    // enable digital soft ramp
    i2cWrite (deviceAddr, CS43L22_REG_MISC_CTL, 0x06);

    // enable outputDevice
    SetMute (deviceAddr, AUDIO_MUTE_OFF);

    // power on codec
    i2cWrite (deviceAddr, CS43L22_REG_POWER_CTL1, 0x9E);

    gOutputStop = 0;
    }
  }
//}}}
//{{{
static void Pause (uint16_t deviceAddr) {

  // pause audio file playing, Mute the output first
  SetMute (deviceAddr, AUDIO_MUTE_ON);

  // put odec in Power save mode
  i2cWrite (deviceAddr, CS43L22_REG_POWER_CTL1, 0x01);
  }
//}}}
//{{{
static void Resume (uint16_t deviceAddr) {

  // resumes playing, Unmute the output first
  SetMute (deviceAddr, AUDIO_MUTE_OFF);

  volatile uint32_t index = 0x00;
  for (index = 0x00; index < 0xFF; index++);
  i2cWrite (deviceAddr, CS43L22_REG_POWER_CTL2, gOutputDevice);

  // exit power save mode
  i2cWrite (deviceAddr, CS43L22_REG_POWER_CTL1, 0x9E);
  }
//}}}
//{{{
static void Stop (uint16_t deviceAddr, uint32_t CodecPdwnMode) {

  // mute output first
  SetMute (deviceAddr, AUDIO_MUTE_ON);

  // disable the digital soft ramp
  i2cWrite (deviceAddr, CS43L22_REG_MISC_CTL, 0x04);

  // power down DAC and the speaker (PMDAC and PMSPK bits)
  i2cWrite (deviceAddr, CS43L22_REG_POWER_CTL1, 0x9F);

  gOutputStop = 1;
  }
//}}}

//{{{
void HAL_I2S_TxCpltCallback (I2S_HandleTypeDef* hi2s) {

  if (hi2s->Instance == I2S3)
    BSP_AUDIO_OUT_TransferComplete_CallBack();
  }
//}}}
//{{{
void HAL_I2S_TxHalfCpltCallback (I2S_HandleTypeDef* hi2s) {

  if (hi2s->Instance == I2S3)
    BSP_AUDIO_OUT_HalfTransfer_CallBack();
  }
//}}}
//{{{
void HAL_I2S_RxCpltCallback (I2S_HandleTypeDef* hi2s) {
  BSP_AUDIO_IN_TransferComplete_CallBack();
  }
//}}}
//{{{
void HAL_I2S_RxHalfCpltCallback (I2S_HandleTypeDef* hi2s) {
  BSP_AUDIO_IN_HalfTransfer_CallBack();
  }
//}}}
//{{{
void HAL_I2S_ErrorCallback (I2S_HandleTypeDef* hi2s) {

  if (hi2s->Instance == I2S3)
    BSP_AUDIO_OUT_Error_CallBack();
  if (hi2s->Instance == I2S2)
    BSP_AUDIO_IN_Error_Callback();
  }
//}}}

// audio out
__weak void BSP_AUDIO_OUT_TransferComplete_CallBack() {}
__weak void BSP_AUDIO_OUT_HalfTransfer_CallBack() {}
__weak void BSP_AUDIO_OUT_Error_CallBack() {}

//{{{
void BSP_AUDIO_OUT_Init (uint16_t gOutputDeviceice, uint8_t Volume, uint32_t AudioFreq) {

  printf ("audioId:%x\n", ReadId(AUDIO_I2C_ADDRESS));

  // PLL clock depends on audioFreq, 44.1khz vs 48khz
  i2sClockConfig (&hAudioOutI2s, AudioFreq, NULL);

  // I2S data transfer preparation:
  hAudioOutI2s.Instance = I2S3;
  i2sInit (&hAudioOutI2s, NULL);
  i2s3Init (AudioFreq);
  Init (AUDIO_I2C_ADDRESS, gOutputDeviceice, Volume, AudioFreq);
  }
//}}}
//{{{
void BSP_AUDIO_OUT_Play (uint16_t* pBuffer, uint32_t Size) {

  Play (AUDIO_I2C_ADDRESS, pBuffer, Size);
  HAL_I2S_Transmit_DMA (&hAudioOutI2s, pBuffer, DMA_MAX (Size / AUDIODATA_SIZE));
  }
//}}}
//{{{
void BSP_AUDIO_OUT_ChangeBuffer (uint16_t* pData, uint16_t Size) {

  HAL_I2S_Transmit_DMA (&hAudioOutI2s, pData, Size);
  }
//}}}
//{{{
void BSP_AUDIO_OUT_Pause() {

  Pause (AUDIO_I2C_ADDRESS);
  HAL_I2S_DMAPause (&hAudioOutI2s);
  }
//}}}
//{{{
void BSP_AUDIO_OUT_Resume() {

  Resume (AUDIO_I2C_ADDRESS);
  HAL_I2S_DMAResume (&hAudioOutI2s);
  }
//}}}
//{{{
void BSP_AUDIO_OUT_Stop (uint32_t Option) {

  /* Call DMA Stop to disable DMA stream before stopping codec */
  HAL_I2S_DMAStop (&hAudioOutI2s);

  /* Call Audio Codec Stop function */
  Stop (AUDIO_I2C_ADDRESS, Option);

  if (Option == CODEC_PDWN_HW) {
    /* Wait at least 1ms */
    HAL_Delay(1);

    /* Reset the pin */
    HAL_GPIO_WritePin(AUDIO_RESET_GPIO, AUDIO_RESET_PIN, GPIO_PIN_RESET);
    }
  }
//}}}
//{{{
void BSP_AUDIO_OUT_SetVolume (uint8_t Volume) {
/* Call the codec volume control function with converted volume value */

  SetVolume (AUDIO_I2C_ADDRESS, Volume);
  }
//}}}
//{{{
void BSP_AUDIO_OUT_SetMute (uint32_t Cmd) {

  SetMute(AUDIO_I2C_ADDRESS, Cmd);
  }
//}}}
//{{{
void BSP_AUDIO_OUT_SetOutputMode (uint8_t Output) {

  SetOutputMode (AUDIO_I2C_ADDRESS, Output);
  }
//}}}
//{{{
void BSP_AUDIO_OUT_SetFrequency (uint32_t AudioFreq) {

  // PLL clock is set depending by the AudioFreq (44.1khz vs 48khz groups)
  i2sClockConfig (&hAudioOutI2s, AudioFreq, NULL);

  // Update the I2S audio frequency configuration */
  i2s3Init (AudioFreq);
  }
//}}}

// audio in
//{{{
__weak void BSP_AUDIO_IN_ClockConfig (I2S_HandleTypeDef *hi2s, uint32_t AudioFreq, void *Params)
{
  RCC_PeriphCLKInitTypeDef rccclkinit;

  /*Enable PLLI2S clock*/
  HAL_RCCEx_GetPeriphCLKConfig(&rccclkinit);
  /* PLLI2S_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
  if ((AudioFreq & 0x7) == 0)
  {
    /* Audio frequency multiple of 8 (8/16/32/48/96/192)*/
    /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN = 192 Mhz */
    /* I2SCLK = PLLI2S_VCO Output/PLLI2SR = 192/6 = 32 Mhz */
    rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    rccclkinit.PLLI2S.PLLI2SN = 192;
    rccclkinit.PLLI2S.PLLI2SR = 6;
    HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
  }
  else
  {
    /* Other Frequency (11.025/22.500/44.100) */
    /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN = 290 Mhz */
    /* I2SCLK = PLLI2S_VCO Output/PLLI2SR = 290/2 = 145 Mhz */
    rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    rccclkinit.PLLI2S.PLLI2SN = 290;
    rccclkinit.PLLI2S.PLLI2SR = 2;
    HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
  }
}
//}}}
//{{{
__weak void BSP_AUDIO_IN_MspInit (I2S_HandleTypeDef *hi2s, void *Params) {

  static DMA_HandleTypeDef hdma_i2sRx;
  GPIO_InitTypeDef  GPIO_InitStruct;

  // Enable the I2S2 peripheral clock */
  I2S2_CLK_ENABLE();

  // Enable I2S GPIO clocks */
  I2S2_SCK_GPIO_CLK_ENABLE();
  I2S2_MOSI_GPIO_CLK_ENABLE();

  // I2S2 pins configuration: SCK and MOSI pins
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;

  GPIO_InitStruct.Pin       = I2S2_SCK_PIN;
  GPIO_InitStruct.Alternate = I2S2_SCK_AF;
  HAL_GPIO_Init(I2S2_SCK_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin       = I2S2_MOSI_PIN ;
  GPIO_InitStruct.Alternate = I2S2_MOSI_AF;
  HAL_GPIO_Init(I2S2_MOSI_GPIO_PORT, &GPIO_InitStruct);

  // Enable the DMA clock */
  I2S2_DMAx_CLK_ENABLE();

  if(hi2s->Instance == I2S2) {
    /* Configure the hdma_i2sRx handle parameters */
    hdma_i2sRx.Init.Channel             = I2S2_DMAx_CHANNEL;
    hdma_i2sRx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_i2sRx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_i2sRx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_i2sRx.Init.PeriphDataAlignment = I2S2_DMAx_PERIPH_DATA_SIZE;
    hdma_i2sRx.Init.MemDataAlignment    = I2S2_DMAx_MEM_DATA_SIZE;
    hdma_i2sRx.Init.Mode                = DMA_CIRCULAR;
    hdma_i2sRx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_i2sRx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_i2sRx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_i2sRx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_i2sRx.Init.PeriphBurst         = DMA_MBURST_SINGLE;

    hdma_i2sRx.Instance = I2S2_DMAx_STREAM;

    // Associate the DMA handle */
    __HAL_LINKDMA(hi2s, hdmarx, hdma_i2sRx);

    // Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_i2sRx);

    // Configure the DMA Stream */
    HAL_DMA_Init(&hdma_i2sRx);
    }

  // I2S DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(I2S2_DMAx_IRQ, AUDIO_IN_IRQ_PREPRIO, 0);
  HAL_NVIC_EnableIRQ(I2S2_DMAx_IRQ);
  }
//}}}
//{{{
__weak void BSP_AUDIO_IN_MspDeInit (I2S_HandleTypeDef *hi2s, void *Params)
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* I2S DMA IRQ Channel deactivation */
  HAL_NVIC_DisableIRQ(I2S2_DMAx_IRQ);

  if(hi2s->Instance == I2S2)
  {
    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(hi2s->hdmarx);
  }

 /* Disable I2S block */
  __HAL_I2S_DISABLE(hi2s);

  /* Disable pins: SCK and SD pins */
  gpio_init_structure.Pin = I2S2_SCK_PIN;
  HAL_GPIO_DeInit(I2S2_SCK_GPIO_PORT, gpio_init_structure.Pin);
  gpio_init_structure.Pin = I2S2_MOSI_PIN;
  HAL_GPIO_DeInit(I2S2_MOSI_GPIO_PORT, gpio_init_structure.Pin);

  /* Disable I2S clock */
  I2S2_CLK_DISABLE();

  /* GPIO pins clock and DMA clock can be shut down in the applic
     by surcgarging this __weak function */
}
//}}}
__weak void BSP_AUDIO_IN_TransferComplete_CallBack() {}
__weak void BSP_AUDIO_IN_HalfTransfer_CallBack() {}
__weak void BSP_AUDIO_IN_Error_Callback() {}

//{{{
uint8_t BSP_AUDIO_IN_Init (uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr) {

  // Configure PLL clock */
  BSP_AUDIO_IN_ClockConfig (&hAudioInI2s, AudioFreq, NULL);

  // Configure the PDM library */
  // On STM32F4-Discovery a single microphone is mounted, samples are duplicated to make stereo audio streams */
  pdmDecoderInit (AudioFreq, ChnlNbr, 2);

  // Configure the I2S peripheral */
  hAudioInI2s.Instance = I2S2;
  if (HAL_I2S_GetState (&hAudioInI2s) == HAL_I2S_STATE_RESET)
    // Initialize the I2S Msp: this __weak function can be rewritten by the application */
    BSP_AUDIO_IN_MspInit (&hAudioInI2s, NULL);

  i2s2Init (AudioFreq);

  // Return AUDIO_OK when all operations are correctly done */
  return AUDIO_OK;
  }
//}}}
//{{{
uint8_t BSP_AUDIO_IN_Record (uint16_t* pbuf, uint32_t size) {

  uint32_t ret = AUDIO_ERROR;

  // Start the process receive DMA
  HAL_I2S_Receive_DMA (&hAudioInI2s, pbuf, size);

  // Return AUDIO_OK when all operations are correctly done
  ret = AUDIO_OK;

  return ret;
  }
//}}}
//{{{
uint8_t BSP_AUDIO_IN_Stop() {

  uint32_t ret = AUDIO_ERROR;

  // Call the Media layer pause function
  HAL_I2S_DMAStop (&hAudioInI2s);

  // Return AUDIO_OK when all operations are correctly done
  ret = AUDIO_OK;

  return ret;
  }
//}}}
//{{{
uint8_t BSP_AUDIO_IN_Pause() {

  // Call the Media layer pause function */
  HAL_I2S_DMAPause(&hAudioInI2s);

  // Return AUDIO_OK when all operations are correctly done */
  return AUDIO_OK;
  }
//}}}
//{{{
uint8_t BSP_AUDIO_IN_Resume() {

  // Call the Media layer pause/resume function */
  HAL_I2S_DMAResume(&hAudioInI2s);

  // Return AUDIO_OK when all operations are correctly done */
  return AUDIO_OK;
  }
//}}}
//{{{
uint8_t BSP_AUDIO_IN_SetVolume (uint8_t Volume) {

  // Set the Global variable AudioInVolume */
  AudioInVolume = Volume;

  // Return AUDIO_OK when all operations are correctly done */
  return AUDIO_OK;
  }
//}}}
//{{{
uint8_t BSP_AUDIO_IN_PDMToPCM (uint16_t* PDMBuf, uint16_t* PCMBuf) {

  uint16_t AppPDM[INTERNAL_BUFF_SIZE/2];

  // PDM Demux
  uint32_t index = 0;
  for (index = 0; index<INTERNAL_BUFF_SIZE/2; index++)
    AppPDM[index] = HTONS(PDMBuf[index]);

  for (index = 0; index < DEFAULT_AUDIO_IN_CHANNEL_NBR; index++)
    // PDM to PCM filter
    PDM_Filter((uint8_t*)&AppPDM[index], (uint16_t*)&(PCMBuf[index]), &PDM_FilterHandler[index]);

  // Duplicate samples since a single microphone in mounted on STM32F4-Discovery
  for (index = 0; index < PCM_OUT_SIZE; index++)
    PCMBuf[(index<<1)+1] = PCMBuf[index<<1];

  return AUDIO_OK;
  }
//}}}
