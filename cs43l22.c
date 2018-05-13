#include "stdio.h"
#include "cs43l22.h"
#include "ledsButton.h"

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
#define VOLUME_CONVERT(Volume) (((Volume) > 100) ? 255:((uint8_t)(((Volume) * 255) / 100)))

uint32_t I2cxTimeout = I2Cx_TIMEOUT_MAX;
static I2C_HandleTypeDef I2cHandle;
static uint8_t Is_cs43l22_Stop = 1;
volatile uint8_t OutputDev = 0;

//{{{
static void I2C_WriteData (uint8_t Addr, uint8_t Reg, uint8_t Value) {

  if (HAL_I2C_Mem_Write(&I2cHandle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, I2Cx_TIMEOUT_MAX) != HAL_OK)
    printf ("I2C_WriteData error\n");
  }
//}}}
//{{{
static uint8_t I2C_ReadData (uint8_t Addr, uint8_t Reg) {

  uint8_t value = 0;
  if (HAL_I2C_Mem_Read(&I2cHandle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2Cx_TIMEOUT_MAX) != HAL_OK)
    printf ("I2C_ReadData error\n");

  return value;
  }
//}}}

//{{{
void AUDIO_IO_Init() {

  printf ("AUDIO_IO_Init\n");

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

  // Power on the codec HAL_GPIO_WritePin(AUDIO_RESET_GPIO, AUDIO_RESET_PIN, GPIO_PIN_SET);

  // Wait for a delay to insure registers erasing
  HAL_Delay (5);
  }
//}}}
void AUDIO_IO_DeInit() {}
//{{{
void AUDIO_IO_Write (uint8_t Addr, uint8_t Reg, uint8_t Value) {

  printf ("AUDIO_IO_Write %x %X %x\n", Addr, Reg, Value);
  I2C_WriteData (Addr, Reg, Value);
  }
//}}}
//{{{
uint8_t AUDIO_IO_Read (uint8_t Addr, uint8_t Reg) {

  uint32_t value = I2C_ReadData (Addr, Reg);
  printf ("AUDIO_IO_Read %x %X %x\n", Addr, Reg, value);
  return value;
  }
//}}}

//{{{
static uint8_t CODEC_IO_Write (uint8_t Addr, uint8_t Reg, uint8_t Value) {

  AUDIO_IO_Write (Addr, Reg, Value);
  return 0;
  }
//}}}

//{{{
uint32_t cs43l22_Init (uint16_t DeviceAddr, uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq) {

  printf ("cs43l22_Init\n");

  // Initialize the Control interface of the Audio Codec
  //AUDIO_IO_Init();

  // Keep Codec powered OFF
  uint32_t counter = CODEC_IO_Write (DeviceAddr, CS43L22_REG_POWER_CTL1, 0x01);

  // Save Output device for mute ON/OFF procedure*/
  switch (OutputDevice) {
    case OUTPUT_DEVICE_SPEAKER:
      OutputDev = 0xFA;
      break;

    case OUTPUT_DEVICE_HEADPHONE:
      OutputDev = 0xAF;
      break;

    case OUTPUT_DEVICE_BOTH:
      OutputDev = 0xAA;
      break;

    case OUTPUT_DEVICE_AUTO:
      OutputDev = 0x05;
      break;

    default:
      OutputDev = 0x05;
      break;
    }

  counter += CODEC_IO_Write (DeviceAddr, CS43L22_REG_POWER_CTL2, OutputDev);

  // Clock configuration: Auto detection
  counter += CODEC_IO_Write (DeviceAddr, CS43L22_REG_CLOCKING_CTL, 0x81);

  // Set the Slave Mode and the audio Standard
  counter += CODEC_IO_Write (DeviceAddr, CS43L22_REG_INTERFACE_CTL1, CODEC_STANDARD);

  // Set the Master volume
  counter += cs43l22_SetVolume (DeviceAddr, Volume);

  // If the Speaker is enabled, set the Mono mode and volume attenuation level
  if (OutputDevice != OUTPUT_DEVICE_HEADPHONE) {
    // Set the Speaker Mono mode
    counter += CODEC_IO_Write (DeviceAddr, CS43L22_REG_PLAYBACK_CTL2, 0x06);

    // Set the Speaker attenuation level
    counter += CODEC_IO_Write (DeviceAddr, CS43L22_REG_SPEAKER_A_VOL, 0x00);
    counter += CODEC_IO_Write (DeviceAddr, CS43L22_REG_SPEAKER_B_VOL, 0x00);
    }

  // Additional configuration for the CODEC. These configurations are done to reduce
  // the time needed for the Codec to power off. If these configurations are removed,
  // then a long delay should be added between powering off the Codec and switching
  // off the I2S peripheral MCLK clock (which is the operating clock for Codec).
  // If this delay is not inserted, then the codec will not shut down properly and
  // it results in high noise after shut down

  // Disable the analog soft ramp
  counter += CODEC_IO_Write (DeviceAddr, CS43L22_REG_ANALOG_ZC_SR_SETT, 0x00);
  // Disable the digital soft ramp
  counter += CODEC_IO_Write (DeviceAddr, CS43L22_REG_MISC_CTL, 0x04);
  // Disable the limiter attack level
  counter += CODEC_IO_Write (DeviceAddr, CS43L22_REG_LIMIT_CTL1, 0x00);
  // Adjust Bass and Treble levels
  counter += CODEC_IO_Write (DeviceAddr, CS43L22_REG_TONE_CTL, 0x0F);
  // Adjust PCM volume level
  counter += CODEC_IO_Write (DeviceAddr, CS43L22_REG_PCMA_VOL, 0x0A);
  counter += CODEC_IO_Write (DeviceAddr, CS43L22_REG_PCMB_VOL, 0x0A);

  // Return communication control value
  return counter;
  }
//}}}
//{{{
void cs43l22_DeInit() {
  AUDIO_IO_DeInit();
  }
//}}}

//{{{
uint32_t cs43l22_ReadID (uint16_t DeviceAddr) {

  AUDIO_IO_Init();

  uint32_t value = AUDIO_IO_Read (DeviceAddr, CS43L22_CHIPID_ADDR) & CS43L22_ID_MASK;
  printf ("cs43l22_ReadID %x\n", value);

  return value & CS43L22_ID_MASK;
  }
//}}}

//{{{
uint32_t cs43l22_Play (uint16_t DeviceAddr, uint16_t* pBuffer, uint16_t Size) {

  uint32_t counter = 0;
  if (Is_cs43l22_Stop == 1) {
    // Enable the digital soft ramp
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_MISC_CTL, 0x06);

    // Enable Output device
    counter += cs43l22_SetMute(DeviceAddr, AUDIO_MUTE_OFF);

    // Power on the Codec
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_POWER_CTL1, 0x9E);
    Is_cs43l22_Stop = 0;
    }

  // Return communication control value
  return counter;
  }
//}}}
//{{{
uint32_t cs43l22_Pause (uint16_t DeviceAddr) {

  uint32_t counter = 0;

  // Pause the audio file playing, Mute the output first
  counter += cs43l22_SetMute (DeviceAddr, AUDIO_MUTE_ON);

  // Put the Codec in Power save mode
  counter += CODEC_IO_Write (DeviceAddr, CS43L22_REG_POWER_CTL1, 0x01);

  return counter;
  }
//}}}
//{{{
uint32_t cs43l22_Resume (uint16_t DeviceAddr) {

  uint32_t counter = 0;
  volatile uint32_t index = 0x00;

  // Resumes the audio file playing, Unmute the output first
  counter += cs43l22_SetMute(DeviceAddr, AUDIO_MUTE_OFF);

  for(index = 0x00; index < 0xFF; index++);

  counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_POWER_CTL2, OutputDev);

  // Exit the Power save mode
  counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_POWER_CTL1, 0x9E);
  return counter;
  }
//}}}
//{{{
uint32_t cs43l22_Stop (uint16_t DeviceAddr, uint32_t CodecPdwnMode) {

  uint32_t counter = 0;

  // Mute the output first */
  counter += cs43l22_SetMute(DeviceAddr, AUDIO_MUTE_ON);

  // Disable the digital soft ramp */
  counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_MISC_CTL, 0x04);

  // Power down the DAC and the speaker (PMDAC and PMSPK bits)*/
  counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_POWER_CTL1, 0x9F);

  Is_cs43l22_Stop = 1;
  return counter;
  }
//}}}

//{{{
uint32_t cs43l22_SetVolume (uint16_t DeviceAddr, uint8_t Volume) {

  uint32_t counter = 0;
  uint8_t convertedvol = VOLUME_CONVERT (Volume);

  if(convertedvol > 0xE6) {
    // Set the Master volume
    counter += CODEC_IO_Write (DeviceAddr, CS43L22_REG_MASTER_A_VOL, convertedvol - 0xE7);
    counter += CODEC_IO_Write (DeviceAddr, CS43L22_REG_MASTER_B_VOL, convertedvol - 0xE7);
    }
  else {
    // Set the Master volume
    counter += CODEC_IO_Write (DeviceAddr, CS43L22_REG_MASTER_A_VOL, convertedvol + 0x19);
    counter += CODEC_IO_Write (DeviceAddr, CS43L22_REG_MASTER_B_VOL, convertedvol + 0x19);
    }

  return counter;
  }
//}}}
uint32_t cs43l22_SetFrequency (uint16_t DeviceAddr, uint32_t AudioFreq) { return 0; }
//{{{
uint32_t cs43l22_SetMute (uint16_t DeviceAddr, uint32_t Cmd) {

  uint32_t counter = 0;

  // Set the Mute mode
  if (Cmd == AUDIO_MUTE_ON) {
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_POWER_CTL2, 0xFF);
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_HEADPHONE_A_VOL, 0x01);
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_HEADPHONE_B_VOL, 0x01);
    }
  else {
    // AUDIO_MUTE_OFF Disable the Mute
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_HEADPHONE_A_VOL, 0x00);
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_HEADPHONE_B_VOL, 0x00);
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_POWER_CTL2, OutputDev);
    }

  return counter;
  }
//}}}
//{{{
uint32_t cs43l22_SetOutputMode (uint16_t DeviceAddr, uint8_t Output) {

  uint32_t counter = 0;
  switch (Output) {
    case OUTPUT_DEVICE_SPEAKER:
      counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_POWER_CTL2, 0xFA); /* SPK always ON & HP always OFF */
      OutputDev = 0xFA;
      break;

    case OUTPUT_DEVICE_HEADPHONE:
      counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_POWER_CTL2, 0xAF); /* SPK always OFF & HP always ON */
      OutputDev = 0xAF;
      break;

    case OUTPUT_DEVICE_BOTH:
      counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_POWER_CTL2, 0xAA); /* SPK always ON & HP always ON */
      OutputDev = 0xAA;
      break;

    case OUTPUT_DEVICE_AUTO:
      counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_POWER_CTL2, 0x05); /* Detect the HP or the SPK automatically */
      OutputDev = 0x05;
      break;

    default:
      counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_POWER_CTL2, 0x05); /* Detect the HP or the SPK automatically */
      OutputDev = 0x05;
      break;
    }

  return counter;
  }
//}}}

uint32_t cs43l22_Reset (uint16_t DeviceAddr) { return 0; }

AUDIO_DrvTypeDef cs43l22_drv = {
  cs43l22_Init,
  cs43l22_DeInit,
  cs43l22_ReadID,

  cs43l22_Play,
  cs43l22_Pause,
  cs43l22_Resume,
  cs43l22_Stop,

  cs43l22_SetFrequency,
  cs43l22_SetVolume,
  cs43l22_SetMute,
  cs43l22_SetOutputMode,
  cs43l22_Reset,
  };
