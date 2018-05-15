// audio.c
//{{{  includes
#include "audio.h"
#include "stm32f4xx_hal.h"
#include "pdm2pcm_glo.h"
//}}}
//{{{  i2c defines
#define AUDIO_I2C_ADDRESS              0x94
#define AUDIO_I2C_SPEED                400000

#define I2Cx                            I2C1
#define I2Cx_CLK_ENABLE()               __HAL_RCC_I2C1_CLK_ENABLE()
#define I2Cx_SCL_SDA_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_SDA_AF                 GPIO_AF4_I2C1
#define I2Cx_SCL_SDA_GPIO_PORT          GPIOB
#define I2Cx_SCL_PIN                    GPIO_PIN_6
#define I2Cx_SDA_PIN                    GPIO_PIN_9

#define I2Cx_FORCE_RESET()              __HAL_RCC_I2C1_FORCE_RESET()
#define I2Cx_RELEASE_RESET()            __HAL_RCC_I2C1_RELEASE_RESET()

/* I2C interrupt requests */
#define I2Cx_EV_IRQn                    I2C1_EV_IRQn
#define I2Cx_ER_IRQn                    I2C1_ER_IRQn

#define I2Cx_TIMEOUT_MAX    0x1000 /*<! The value of the maximal timeout for BUS waiting loops */
//}}}
//{{{  cs43122 defines
#define CODEC_STANDARD  0x04
#define I2S_STANDARD    I2S_STANDARD_PHILIPS

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

#define I2S3_IRQHandler                 DMA1_Stream7_IRQHandler

// Select the interrupt preemption priority and subpriority for the DMA interrupt */
#define AUDIO_OUT_IRQ_PREPRIO           0x0E   /* Select the preemption priority level(0 is the highest) */
//}}}
//{{{  defines
#define HTONS(A)                ((((uint16_t)(A) & 0xff00) >> 8) | (((uint16_t)(A) & 0x00ff) << 8))
#define VOLUME_CONVERT(Volume)  (((Volume) > 100) ? 255 : ((uint8_t)(((Volume) * 255) / 100)))
//}}}
//{{{  clock const
// These PLL parameters are valid when the f(VCO clock) = 1Mhz
const uint32_t I2SFreq[8] = { 8000, 11025, 16000, 22050, 32000, 44100, 48000, 96000 };
const uint32_t I2SPLLN[8] = { 256, 429, 213, 429, 426, 271, 258, 344 };
const uint32_t I2SPLLR[8] = { 5, 4, 4, 4, 4, 6, 3, 1 };
//}}}

I2S_HandleTypeDef hAudioOutI2s;
I2S_HandleTypeDef hAudioInI2s;
DMA_HandleTypeDef gI2sTxDma;
DMA_HandleTypeDef gI2sRxDma;

extern "C" {
  void I2S2_IRQHandler() { HAL_DMA_IRQHandler(hAudioInI2s.hdmarx); }
  void I2S3_IRQHandler() { HAL_DMA_IRQHandler (hAudioOutI2s.hdmatx); }
  }

I2C_HandleTypeDef I2cHandle;
uint8_t gOutputDevice = 0;
uint8_t gOutputStop = 1;

PDM_Filter_Handler_t PDM_FilterHandler[2];
PDM_Filter_Config_t PDM_FilterConfig[2];
uint16_t AudioInVolume = DEFAULT_AUDIO_IN_VOLUME;

// i2c
//{{{
void i2cInit() {

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

  // I2Cx peripheral configuration */
  I2cHandle.Init.ClockSpeed = AUDIO_I2C_SPEED;
  I2cHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
  I2cHandle.Init.OwnAddress1 = 0x33;
  I2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle.Instance = I2Cx;

  // Enable I2C GPIO clocks
  I2Cx_SCL_SDA_GPIO_CLK_ENABLE();

  // I2Cx SCL and SDA pins configuration
  GPIO_InitStruct.Pin = I2Cx_SCL_PIN | I2Cx_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Alternate  = I2Cx_SCL_SDA_AF;
  HAL_GPIO_Init (I2Cx_SCL_SDA_GPIO_PORT, &GPIO_InitStruct);

  // Enable the I2Cx peripheral clock
  I2Cx_CLK_ENABLE();

  // Force the I2C peripheral clock reset
  I2Cx_FORCE_RESET();

  // Release the I2C peripheral clock reset
  I2Cx_RELEASE_RESET();

  // Enable and set I2Cx Interrupt to the highest priority
  HAL_NVIC_SetPriority (I2Cx_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ (I2Cx_EV_IRQn);

  // Enable and set I2Cx Interrupt to the highest priority
  HAL_NVIC_SetPriority (I2Cx_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ (I2Cx_ER_IRQn);

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
uint8_t i2cRead (uint8_t addr, uint16_t reg) {

  uint8_t value = 0;
  if (HAL_I2C_Mem_Read (&I2cHandle, addr, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2Cx_TIMEOUT_MAX) != HAL_OK)
    printf ("i2cRead error %x %x %x\n", addr, reg, value);
  //else
  //  printf ("i2cRead %x %x %x\n", addr, reg, value);

  return value;
  }
//}}}
//{{{
void i2cWrite (uint8_t addr, uint16_t reg, uint8_t value) {

  if (HAL_I2C_Mem_Write(&I2cHandle, addr, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2Cx_TIMEOUT_MAX) != HAL_OK)
    printf ("i2cWrite error %x %x %x\n", addr, reg, value);
  //else
  //  printf ("i2cWrite %x %x %x\n", addr, reg, value);
  }
//}}}

// i2s config
//{{{
void i2sClockConfig (I2S_HandleTypeDef* hi2s, uint32_t sampleRate) {

  uint8_t index = 0;
  uint8_t freqindex = 0xFF;
  for (index = 0; index < 8; index++)
    if (I2SFreq[index] == sampleRate)
      freqindex = index;

  // Enable PLLI2S clock
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
void pdmDecoderInit (uint32_t sampleRate, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut) {

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
    PDM_FilterConfig[index].output_samples_number = sampleRate / 1000;
    PDM_FilterConfig[index].mic_gain = 24;
    PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_64;
    PDM_Filter_setConfig ((PDM_Filter_Handler_t*)&PDM_FilterHandler[index], &PDM_FilterConfig[index]);
    }
  }
//}}}

// cs43l22 control
//{{{
void cs43l22setVolume (uint16_t deviceAddr, uint8_t volume) {

  uint8_t convertedVolume = VOLUME_CONVERT (volume);
  if (convertedVolume > 0xE6) {
    i2cWrite (deviceAddr, CS43L22_REG_MASTER_A_VOL, convertedVolume - 0xE7);
    i2cWrite (deviceAddr, CS43L22_REG_MASTER_B_VOL, convertedVolume - 0xE7);
    }
  else {
    i2cWrite (deviceAddr, CS43L22_REG_MASTER_A_VOL, convertedVolume + 0x19);
    i2cWrite (deviceAddr, CS43L22_REG_MASTER_B_VOL, convertedVolume + 0x19);
    }
  }
//}}}
//{{{
void cs43l22setOutputMode (uint16_t deviceAddr, uint8_t output) {

  switch (output) {
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
void cs43l22setMute (uint16_t deviceAddr, uint32_t cmd) {

  // Set the Mute mode
  if (cmd == AUDIO_MUTE_ON) {
    i2cWrite (deviceAddr, CS43L22_REG_POWER_CTL2, 0xFF);
    i2cWrite (deviceAddr, CS43L22_REG_HEADPHONE_A_VOL, 0x01);
    i2cWrite (deviceAddr, CS43L22_REG_HEADPHONE_B_VOL, 0x01);
    }
  else {
    i2cWrite (deviceAddr, CS43L22_REG_HEADPHONE_A_VOL, 0x00);
    i2cWrite (deviceAddr, CS43L22_REG_HEADPHONE_B_VOL, 0x00);
    i2cWrite (deviceAddr, CS43L22_REG_POWER_CTL2, gOutputDevice);
    }
  }
//}}}
//{{{
void cs43l22init (uint16_t deviceAddr, uint16_t gOutputDeviceice, uint8_t Volume, uint32_t sampleRate) {

  printf ("cs43l22init\n");

  // keep Codec powered OFF
  i2cWrite (deviceAddr, CS43L22_REG_POWER_CTL1, 0x01);

  cs43l22setOutputMode (deviceAddr, gOutputDeviceice);

  // Clock configuration - autoDetection
  i2cWrite (deviceAddr, CS43L22_REG_CLOCKING_CTL, 0x81);

  // set Slave Mode and audioStandard
  i2cWrite (deviceAddr, CS43L22_REG_INTERFACE_CTL1, CODEC_STANDARD);

  // set master volume
  cs43l22setVolume (deviceAddr, Volume);

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
uint8_t cs43l22readId (uint16_t deviceAddr) {

  i2cInit();

  uint32_t value = i2cRead (deviceAddr, CS43L22_CHIPID_ADDR);
  printf ("ReadID %x id:%x revision:%x\n", value, value & CS43L22_ID_MASK, value & CS43L22_REVISION_MASK);
  return value;
  }
//}}}
//{{{
void cs43l22play (uint16_t deviceAddr, uint16_t* pBuffer, uint16_t Size) {

  if (gOutputStop == 1) {
    // enable digital soft ramp
    i2cWrite (deviceAddr, CS43L22_REG_MISC_CTL, 0x06);

    // enable outputDevice
    cs43l22setMute (deviceAddr, AUDIO_MUTE_OFF);

    // power on codec
    i2cWrite (deviceAddr, CS43L22_REG_POWER_CTL1, 0x9E);

    gOutputStop = 0;
    }
  }
//}}}
//{{{
void cs43l22pause (uint16_t deviceAddr) {

  // pause audio file playing, Mute the output first
  cs43l22setMute (deviceAddr, AUDIO_MUTE_ON);

  // put codec in Power save mode
  i2cWrite (deviceAddr, CS43L22_REG_POWER_CTL1, 0x01);
  }
//}}}
//{{{
void cs43l22resume (uint16_t deviceAddr) {

  // resumes playing, Unmute the output first
  cs43l22setMute (deviceAddr, AUDIO_MUTE_OFF);

  volatile uint32_t index = 0x00;
  for (index = 0x00; index < 0xFF; index++);

  i2cWrite (deviceAddr, CS43L22_REG_POWER_CTL2, gOutputDevice);

  // exit power save mode
  i2cWrite (deviceAddr, CS43L22_REG_POWER_CTL1, 0x9E);
  }
//}}}
//{{{
void cs43l22stop (uint16_t deviceAddr, uint32_t CodecPdwnMode) {

  // mute output first
  cs43l22setMute (deviceAddr, AUDIO_MUTE_ON);

  // disable the digital soft ramp
  i2cWrite (deviceAddr, CS43L22_REG_MISC_CTL, 0x04);

  // power down DAC and the speaker (PMDAC and PMSPK bits)
  i2cWrite (deviceAddr, CS43L22_REG_POWER_CTL1, 0x9F);

  gOutputStop = 1;
  }
//}}}

// i2s dma callbacks
//{{{
void HAL_I2S_TxCpltCallback (I2S_HandleTypeDef* hi2s) {

  if (hi2s->Instance == I2S3)
    audioTransferComplete_CallBack();
  }
//}}}
//{{{
void HAL_I2S_TxHalfCpltCallback (I2S_HandleTypeDef* hi2s) {

  if (hi2s->Instance == I2S3)
    audioHalfTransfer_CallBack();
  }
//}}}
//{{{
void HAL_I2S_RxCpltCallback (I2S_HandleTypeDef* hi2s) {
  audioInTransferComplete_CallBack();
  }
//}}}
//{{{
void HAL_I2S_RxHalfCpltCallback (I2S_HandleTypeDef* hi2s) {
  audioInHalfTransfer_CallBack();
  }
//}}}
//{{{
void HAL_I2S_ErrorCallback (I2S_HandleTypeDef* hi2s) {

  if (hi2s->Instance == I2S3)
    audioError_CallBack();
  if (hi2s->Instance == I2S2)
    audioInError_Callback();
  }
//}}}

// audio out interface
//{{{
void audioOutInit (uint16_t outputDevice, uint8_t volume, uint32_t sampleRate) {

  printf ("audioInit - id:%x\n", cs43l22readId (AUDIO_I2C_ADDRESS));

  // PLL clock depends on sampleRate, 44.1khz vs 48khz
  i2sClockConfig (&hAudioOutI2s, sampleRate);

  // i2s peripheral configuration
  hAudioOutI2s.Instance = I2S3;
  hAudioOutI2s.Init.AudioFreq   = sampleRate;
  hAudioOutI2s.Init.ClockSource = I2S_CLOCK_PLL;
  hAudioOutI2s.Init.CPOL        = I2S_CPOL_LOW;
  hAudioOutI2s.Init.DataFormat  = I2S_DATAFORMAT_16B;
  hAudioOutI2s.Init.MCLKOutput  = I2S_MCLKOUTPUT_ENABLE;
  hAudioOutI2s.Init.Mode        = I2S_MODE_MASTER_TX;
  hAudioOutI2s.Init.Standard    = I2S_STANDARD;

  // enable i2s3 clock
  I2S3_CLK_ENABLE();

  // enable i2s3 GPIO clocks
  I2S3_SCK_SD_CLK_ENABLE();
  I2S3_WS_CLK_ENABLE();

  // i2s3 pin config WS, SCK and SD pins
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = I2S3_SCK_PIN | I2S3_SD_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = I2S3_SCK_SD_WS_AF;
  HAL_GPIO_Init (I2S3_SCK_SD_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = I2S3_WS_PIN ;
  HAL_GPIO_Init (I2S3_WS_GPIO_PORT, &GPIO_InitStruct);

  // i2s3 config MCK pin
  I2S3_MCK_CLK_ENABLE();
  GPIO_InitStruct.Pin = I2S3_MCK_PIN;
  HAL_GPIO_Init (I2S3_MCK_GPIO_PORT, &GPIO_InitStruct);

  // Enable the i2s3 DMA clock
  I2S3_DMAx_CLK_ENABLE();
  gI2sTxDma.Instance                 = I2S3_DMAx_STREAM;
  gI2sTxDma.Init.Channel             = I2S3_DMAx_CHANNEL;
  gI2sTxDma.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  gI2sTxDma.Init.PeriphInc           = DMA_PINC_DISABLE;
  gI2sTxDma.Init.MemInc              = DMA_MINC_ENABLE;
  gI2sTxDma.Init.PeriphDataAlignment = I2S3_DMAx_PERIPH_DATA_SIZE;
  gI2sTxDma.Init.MemDataAlignment    = I2S3_DMAx_MEM_DATA_SIZE;
  gI2sTxDma.Init.Mode                = DMA_NORMAL;
  gI2sTxDma.Init.Priority            = DMA_PRIORITY_HIGH;
  gI2sTxDma.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
  gI2sTxDma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  gI2sTxDma.Init.MemBurst            = DMA_MBURST_SINGLE;
  gI2sTxDma.Init.PeriphBurst         = DMA_PBURST_SINGLE;
  __HAL_LINKDMA (&hAudioOutI2s, hdmatx, gI2sTxDma);
  HAL_DMA_DeInit (&gI2sTxDma);
  HAL_DMA_Init (&gI2sTxDma);

  // i2s3 DMA IRQ Channel configuration
  HAL_NVIC_SetPriority (I2S3_DMAx_IRQ, AUDIO_OUT_IRQ_PREPRIO, 0);
  HAL_NVIC_EnableIRQ (I2S3_DMAx_IRQ);

  // init i2s3
  __HAL_I2S_DISABLE (&hAudioOutI2s);
  if (HAL_I2S_Init (&hAudioOutI2s) != HAL_OK)
    printf ("i2s3Init error\n");


  cs43l22init (AUDIO_I2C_ADDRESS, outputDevice, volume, sampleRate);
  }
//}}}
//{{{
void audioPlay (uint16_t* buffer, uint32_t size) {

  cs43l22play (AUDIO_I2C_ADDRESS, buffer, size);
  HAL_I2S_Transmit_DMA (&hAudioOutI2s, buffer, size);
  }
//}}}
//{{{
void audioChangeBuffer (uint16_t* data, uint32_t size) {

  HAL_I2S_Transmit_DMA (&hAudioOutI2s, data, size);
  }
//}}}
//{{{
void audioPause() {

  cs43l22pause (AUDIO_I2C_ADDRESS);
  HAL_I2S_DMAPause (&hAudioOutI2s);
  }
//}}}
//{{{
void audioResume() {

  cs43l22resume (AUDIO_I2C_ADDRESS);
  HAL_I2S_DMAResume (&hAudioOutI2s);
  }
//}}}
//{{{
void audioStop (uint32_t Option) {

  // Call DMA Stop to disable DMA stream before stopping codec
  HAL_I2S_DMAStop (&hAudioOutI2s);

  // Call Audio Codec Stop function
  cs43l22stop (AUDIO_I2C_ADDRESS, Option);

  if (Option == CODEC_PDWN_HW) {
    // Wait at least 1ms
    HAL_Delay(1);

    // Reset the pin
    HAL_GPIO_WritePin(AUDIO_RESET_GPIO, AUDIO_RESET_PIN, GPIO_PIN_RESET);
    }
  }
//}}}
//{{{
void audioSetVolume (uint8_t Volume) {
/* Call the codec volume control function with converted volume value */

  cs43l22setVolume (AUDIO_I2C_ADDRESS, Volume);
  }
//}}}
//{{{
void audioSetMute (uint32_t Cmd) {

  cs43l22setMute (AUDIO_I2C_ADDRESS, Cmd);
  }
//}}}

__weak void audioTransferComplete_CallBack() {}
__weak void audioHalfTransfer_CallBack() {}
__weak void audioError_CallBack() {}

// audio in interface
//{{{
uint8_t audioInInit (uint32_t sampleRate, uint32_t BitRes, uint32_t ChnlNbr) {

  // Configure PLL clock
  audioInClockConfig (&hAudioInI2s, sampleRate);

  // Configure the PDM library, single microphone, samples are duplicated to make stereo audio streams
  pdmDecoderInit (sampleRate, ChnlNbr, 2);

  // Configure the I2S peripheral
  hAudioInI2s.Instance = I2S2;
  hAudioInI2s.Init.AudioFreq    = 2 * sampleRate;
  hAudioInI2s.Init.ClockSource  = I2S_CLOCK_PLL;
  hAudioInI2s.Init.CPOL         = I2S_CPOL_HIGH;
  hAudioInI2s.Init.DataFormat   = I2S_DATAFORMAT_16B;
  hAudioInI2s.Init.MCLKOutput   = I2S_MCLKOUTPUT_DISABLE;
  hAudioInI2s.Init.Mode         = I2S_MODE_MASTER_RX;
  hAudioInI2s.Init.Standard     = I2S_STANDARD_LSB;

  // Enable the I2S2 peripheral clock
  I2S2_CLK_ENABLE();

  // Enable I2S GPIO clocks
  I2S2_SCK_GPIO_CLK_ENABLE();
  I2S2_MOSI_GPIO_CLK_ENABLE();

  // I2S2 pins configuration: SCK and MOSI pins
  GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pin       = I2S2_SCK_PIN;
  GPIO_InitStruct.Alternate = I2S2_SCK_AF;
  HAL_GPIO_Init(I2S2_SCK_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin       = I2S2_MOSI_PIN ;
  GPIO_InitStruct.Alternate = I2S2_MOSI_AF;
  HAL_GPIO_Init(I2S2_MOSI_GPIO_PORT, &GPIO_InitStruct);

  // Enable the DMA clock
  I2S2_DMAx_CLK_ENABLE();

  // Configure the gI2sRxDma handle parameters
  gI2sRxDma.Init.Channel             = I2S2_DMAx_CHANNEL;
  gI2sRxDma.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  gI2sRxDma.Init.PeriphInc           = DMA_PINC_DISABLE;
  gI2sRxDma.Init.MemInc              = DMA_MINC_ENABLE;
  gI2sRxDma.Init.PeriphDataAlignment = I2S2_DMAx_PERIPH_DATA_SIZE;
  gI2sRxDma.Init.MemDataAlignment    = I2S2_DMAx_MEM_DATA_SIZE;
  gI2sRxDma.Init.Mode                = DMA_CIRCULAR;
  gI2sRxDma.Init.Priority            = DMA_PRIORITY_HIGH;
  gI2sRxDma.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  gI2sRxDma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  gI2sRxDma.Init.MemBurst            = DMA_MBURST_SINGLE;
  gI2sRxDma.Init.PeriphBurst         = DMA_MBURST_SINGLE;

  gI2sRxDma.Instance = I2S2_DMAx_STREAM;

  // Associate the DMA handle */
  __HAL_LINKDMA (&hAudioInI2s, hdmarx, gI2sRxDma);

  // Deinitialize the Stream for new transfer
  HAL_DMA_DeInit (&gI2sRxDma);

  // Configure the DMA Stream
  HAL_DMA_Init (&gI2sRxDma);

  // I2S DMA IRQ Channel configuration
  HAL_NVIC_SetPriority (I2S2_DMAx_IRQ, AUDIO_IN_IRQ_PREPRIO, 0);
  HAL_NVIC_EnableIRQ (I2S2_DMAx_IRQ);

  // Disable I2S block
  __HAL_I2S_DISABLE (&hAudioInI2s);

  // Initialize the I2S peripheral with the structure above
  if (HAL_I2S_Init (&hAudioInI2s) != HAL_OK)
    printf ("i2s2Init error\n");

  return AUDIO_OK;
  }
//}}}
//{{{
uint8_t audioInRecord (uint16_t* pbuf, uint32_t size) {

  uint32_t ret = AUDIO_ERROR;

  // Start the process receive DMA
  HAL_I2S_Receive_DMA (&hAudioInI2s, pbuf, size);

  // Return AUDIO_OK when all operations are correctly done
  ret = AUDIO_OK;

  return ret;
  }
//}}}
//{{{
uint8_t audioInStop() {

  uint32_t ret = AUDIO_ERROR;

  // Call the Media layer pause function
  HAL_I2S_DMAStop (&hAudioInI2s);

  // Return AUDIO_OK when all operations are correctly done
  ret = AUDIO_OK;

  return ret;
  }
//}}}
//{{{
uint8_t audioInPause() {

  // Call the Media layer pause function */
  HAL_I2S_DMAPause(&hAudioInI2s);

  // Return AUDIO_OK when all operations are correctly done */
  return AUDIO_OK;
  }
//}}}
//{{{
uint8_t audioInResume() {

  // Call the Media layer pause/resume function */
  HAL_I2S_DMAResume(&hAudioInI2s);

  // Return AUDIO_OK when all operations are correctly done */
  return AUDIO_OK;
  }
//}}}
//{{{
uint8_t audioInSetVolume (uint8_t Volume) {

  // Set the Global variable AudioInVolume */
  AudioInVolume = Volume;

  // Return AUDIO_OK when all operations are correctly done */
  return AUDIO_OK;
  }
//}}}
//{{{
uint8_t audioInPDMToPCM (uint16_t* PDMBuf, uint16_t* PCMBuf) {

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

//{{{
__weak void audioInClockConfig (I2S_HandleTypeDef* hi2s, uint32_t sampleRate)
{
  RCC_PeriphCLKInitTypeDef rccclkinit;

  // Enable PLLI2S clock
  HAL_RCCEx_GetPeriphCLKConfig(&rccclkinit);

  // PLLI2S_VCO Input = HSE_VALUE/PLL_M = 1 Mhz
  if ((sampleRate & 0x7) == 0) {
    // Audio frequency multiple of 8 (8/16/32/48/96/192)
    // PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN = 192 Mhz
    // I2SCLK = PLLI2S_VCO Output/PLLI2SR = 192/6 = 32 Mhz
    rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    rccclkinit.PLLI2S.PLLI2SN = 192;
    rccclkinit.PLLI2S.PLLI2SR = 6;
    HAL_RCCEx_PeriphCLKConfig (&rccclkinit);
    }
  else {
    // Other Frequency (11.025/22.500/44.100)
    // PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN = 290 Mhz
    // I2SCLK = PLLI2S_VCO Output/PLLI2SR = 290/2 = 145 Mhz
    rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    rccclkinit.PLLI2S.PLLI2SN = 290;
    rccclkinit.PLLI2S.PLLI2SR = 2;
    HAL_RCCEx_PeriphCLKConfig (&rccclkinit);
    }
  }
//}}}
//{{{
__weak void audioInMspInit (I2S_HandleTypeDef *hi2s) {

  GPIO_InitTypeDef  GPIO_InitStruct;

  // Enable the I2S2 peripheral clock
  I2S2_CLK_ENABLE();

  // Enable I2S GPIO clocks
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

  // Enable the DMA clock
  I2S2_DMAx_CLK_ENABLE();

  // Configure the gI2sRxDma handle parameters
  gI2sRxDma.Init.Channel             = I2S2_DMAx_CHANNEL;
  gI2sRxDma.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  gI2sRxDma.Init.PeriphInc           = DMA_PINC_DISABLE;
  gI2sRxDma.Init.MemInc              = DMA_MINC_ENABLE;
  gI2sRxDma.Init.PeriphDataAlignment = I2S2_DMAx_PERIPH_DATA_SIZE;
  gI2sRxDma.Init.MemDataAlignment    = I2S2_DMAx_MEM_DATA_SIZE;
  gI2sRxDma.Init.Mode                = DMA_CIRCULAR;
  gI2sRxDma.Init.Priority            = DMA_PRIORITY_HIGH;
  gI2sRxDma.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  gI2sRxDma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  gI2sRxDma.Init.MemBurst            = DMA_MBURST_SINGLE;
  gI2sRxDma.Init.PeriphBurst         = DMA_MBURST_SINGLE;

  gI2sRxDma.Instance = I2S2_DMAx_STREAM;

  // Associate the DMA handle */
  __HAL_LINKDMA (hi2s, hdmarx, gI2sRxDma);

  // Deinitialize the Stream for new transfer
  HAL_DMA_DeInit (&gI2sRxDma);

  // Configure the DMA Stream
  HAL_DMA_Init (&gI2sRxDma);

  // I2S DMA IRQ Channel configuration
  HAL_NVIC_SetPriority (I2S2_DMAx_IRQ, AUDIO_IN_IRQ_PREPRIO, 0);
  HAL_NVIC_EnableIRQ (I2S2_DMAx_IRQ);
  }
//}}}
//{{{
__weak void audioInMspDeInit (I2S_HandleTypeDef* hi2s)
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
__weak void audioInTransferComplete_CallBack() {}
__weak void audioInHalfTransfer_CallBack() {}
__weak void audioInError_Callback() {}
