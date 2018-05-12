#include "cs43l22.h"

#define VOLUME_CONVERT(Volume)    (((Volume) > 100)? 255:((uint8_t)(((Volume) * 255) / 100)))
/* Uncomment this line to enable verifying data sent to codec after each write operation (for debug purpose) */
#if !defined (VERIFY_WRITTENDATA)
/* #define VERIFY_WRITTENDATA */
#endif /* VERIFY_WRITTENDATA */

static uint8_t Is_cs43l22_Stop = 1;

volatile uint8_t OutputDev = 0;
//{{{
static uint8_t CODEC_IO_Write (uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  uint32_t result = 0;

  AUDIO_IO_Write(Addr, Reg, Value);

#ifdef VERIFY_WRITTENDATA
  /* Verify that the data has been correctly written */
  result = (AUDIO_IO_Read(Addr, Reg) == Value)? 0:1;
#endif /* VERIFY_WRITTENDATA */

  return result;
}
//}}}

//{{{
uint32_t cs43l22_Init (uint16_t DeviceAddr, uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq)
{
  uint32_t counter = 0;

  /* Initialize the Control interface of the Audio Codec */
  AUDIO_IO_Init();

  /* Keep Codec powered OFF */
  counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_POWER_CTL1, 0x01);

  /*Save Output device for mute ON/OFF procedure*/
  switch (OutputDevice)
  {
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

  counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_POWER_CTL2, OutputDev);

  /* Clock configuration: Auto detection */
  counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_CLOCKING_CTL, 0x81);

  /* Set the Slave Mode and the audio Standard */
  counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_INTERFACE_CTL1, CODEC_STANDARD);

  /* Set the Master volume */
  counter += cs43l22_SetVolume(DeviceAddr, Volume);

  /* If the Speaker is enabled, set the Mono mode and volume attenuation level */
  if(OutputDevice != OUTPUT_DEVICE_HEADPHONE)
  {
    /* Set the Speaker Mono mode */
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_PLAYBACK_CTL2, 0x06);

    /* Set the Speaker attenuation level */
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_SPEAKER_A_VOL, 0x00);
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_SPEAKER_B_VOL, 0x00);
  }

  /* Additional configuration for the CODEC. These configurations are done to reduce
  the time needed for the Codec to power off. If these configurations are removed,
  then a long delay should be added between powering off the Codec and switching
  off the I2S peripheral MCLK clock (which is the operating clock for Codec).
  If this delay is not inserted, then the codec will not shut down properly and
  it results in high noise after shut down. */

  /* Disable the analog soft ramp */
  counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_ANALOG_ZC_SR_SETT, 0x00);
  /* Disable the digital soft ramp */
  counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_MISC_CTL, 0x04);
  /* Disable the limiter attack level */
  counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_LIMIT_CTL1, 0x00);
  /* Adjust Bass and Treble levels */
  counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_TONE_CTL, 0x0F);
  /* Adjust PCM volume level */
  counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_PCMA_VOL, 0x0A);
  counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_PCMB_VOL, 0x0A);

  /* Return communication control value */
  return counter;
}
//}}}
//{{{
void cs43l22_DeInit()
{
  /* Deinitialize Audio Codec interface */
  AUDIO_IO_DeInit();
}
//}}}

//{{{
uint32_t cs43l22_ReadID (uint16_t DeviceAddr)
{
  uint8_t Value;
  /* Initialize the Control interface of the Audio Codec */
  AUDIO_IO_Init();

  Value = AUDIO_IO_Read(DeviceAddr, CS43L22_CHIPID_ADDR);
  Value = (Value & CS43L22_ID_MASK);

  return((uint32_t) Value);
}
//}}}

//{{{
uint32_t cs43l22_Play (uint16_t DeviceAddr, uint16_t* pBuffer, uint16_t Size)
{
  uint32_t counter = 0;

  if(Is_cs43l22_Stop == 1)
  {
    /* Enable the digital soft ramp */
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_MISC_CTL, 0x06);

    /* Enable Output device */
    counter += cs43l22_SetMute(DeviceAddr, AUDIO_MUTE_OFF);

    /* Power on the Codec */
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_POWER_CTL1, 0x9E);
    Is_cs43l22_Stop = 0;
  }

  /* Return communication control value */
  return counter;
}
//}}}
//{{{
uint32_t cs43l22_Pause (uint16_t DeviceAddr)
{
  uint32_t counter = 0;

  /* Pause the audio file playing */
  /* Mute the output first */
  counter += cs43l22_SetMute(DeviceAddr, AUDIO_MUTE_ON);

  /* Put the Codec in Power save mode */
  counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_POWER_CTL1, 0x01);

  return counter;
}
//}}}
//{{{
uint32_t cs43l22_Resume (uint16_t DeviceAddr)
{
  uint32_t counter = 0;
  volatile uint32_t index = 0x00;
  /* Resumes the audio file playing */
  /* Unmute the output first */
  counter += cs43l22_SetMute(DeviceAddr, AUDIO_MUTE_OFF);

  for(index = 0x00; index < 0xFF; index++);

  counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_POWER_CTL2, OutputDev);

  /* Exit the Power save mode */
  counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_POWER_CTL1, 0x9E);

  return counter;
}
//}}}
//{{{
uint32_t cs43l22_Stop (uint16_t DeviceAddr, uint32_t CodecPdwnMode)
{
  uint32_t counter = 0;

  /* Mute the output first */
  counter += cs43l22_SetMute(DeviceAddr, AUDIO_MUTE_ON);

  /* Disable the digital soft ramp */
  counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_MISC_CTL, 0x04);

  /* Power down the DAC and the speaker (PMDAC and PMSPK bits)*/
  counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_POWER_CTL1, 0x9F);

  Is_cs43l22_Stop = 1;
  return counter;
}
//}}}

//{{{
uint32_t cs43l22_SetVolume (uint16_t DeviceAddr, uint8_t Volume)
{
  uint32_t counter = 0;
  uint8_t convertedvol = VOLUME_CONVERT(Volume);

  if(convertedvol > 0xE6)
  {
    /* Set the Master volume */
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_MASTER_A_VOL, convertedvol - 0xE7);
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_MASTER_B_VOL, convertedvol - 0xE7);
  }
  else
  {
    /* Set the Master volume */
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_MASTER_A_VOL, convertedvol + 0x19);
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_MASTER_B_VOL, convertedvol + 0x19);
  }

  return counter;
}
//}}}
//{{{
uint32_t cs43l22_SetFrequency (uint16_t DeviceAddr, uint32_t AudioFreq)
{
  return 0;
}
//}}}
//{{{
uint32_t cs43l22_SetMute (uint16_t DeviceAddr, uint32_t Cmd)
{
  uint32_t counter = 0;

  /* Set the Mute mode */
  if(Cmd == AUDIO_MUTE_ON)
  {
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_POWER_CTL2, 0xFF);
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_HEADPHONE_A_VOL, 0x01);
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_HEADPHONE_B_VOL, 0x01);
  }
  else /* AUDIO_MUTE_OFF Disable the Mute */
  {
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_HEADPHONE_A_VOL, 0x00);
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_HEADPHONE_B_VOL, 0x00);
    counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_POWER_CTL2, OutputDev);
  }
  return counter;
}
//}}}
//{{{
uint32_t cs43l22_SetOutputMode (uint16_t DeviceAddr, uint8_t Output)
{
  uint32_t counter = 0;

  switch (Output)
  {
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

//{{{
uint32_t cs43l22_Reset (uint16_t DeviceAddr)
{
  return 0;
}
//}}}

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