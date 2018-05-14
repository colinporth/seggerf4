#pragma once
//{{{
#ifdef __cplusplus
 extern "C" {
#endif
//}}}
#include "stm32f4xx_hal.h"

// Audio status definition
#define AUDIO_OK                 0
#define AUDIO_ERROR              1
#define AUDIO_TIMEOUT            2

// Codec output DEVICE
#define OUTPUT_DEVICE_SPEAKER    1
#define OUTPUT_DEVICE_HEADPHONE  2
#define OUTPUT_DEVICE_BOTH       3
#define OUTPUT_DEVICE_AUTO       4

// Codec POWER DOWN modes
#define CODEC_PDWN_HW         1
#define CODEC_PDWN_SW         2

// PCM buffer output size
#define PCM_OUT_SIZE                    DEFAULT_AUDIO_IN_FREQ/1000
#define CHANNEL_DEMUX_MASK              0x55

// AudioFreq * DataSize (2 bytes) * NumChannels (Stereo: 2)
#define DEFAULT_AUDIO_IN_FREQ           I2S_AUDIOFREQ_16K
#define DEFAULT_AUDIO_IN_BIT_RESOLUTION 16
#define DEFAULT_AUDIO_IN_CHANNEL_NBR    1 // Mono = 1, Stereo = 2
#define DEFAULT_AUDIO_IN_VOLUME         64

// PDM buffer input size
#define INTERNAL_BUFF_SIZE              128*DEFAULT_AUDIO_IN_FREQ/16000*DEFAULT_AUDIO_IN_CHANNEL_NBR

extern uint16_t AudioInVolume;

void audioInit (uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq);
void audioPlay (uint16_t* pBuffer, uint32_t Size);
void audioChangeBuffer (uint16_t *pData, uint16_t Size);

void audioPause();
void audioResume();
void audioStop (uint32_t Option);
void audioSetVolume (uint8_t Volume);
void audioSetFrequency (uint32_t AudioFreq);
void audioSetMute (uint32_t Cmd);
void audioSetOutputMode (uint8_t Output);

void audioTransferComplete_CallBack();
void audioHalfTransfer_CallBack();
void audioError_CallBack();
void audioClockConfig (I2S_HandleTypeDef *hi2s, uint32_t AudioFreq, void *Params);
void audioMspInit (I2S_HandleTypeDef *hi2s, void *Params);
void audioMspDeInit (I2S_HandleTypeDef *hi2s, void *Params);

uint8_t BSP_AUDIO_IN_Init (uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr);
uint8_t BSP_AUDIO_IN_Record (uint16_t *pData, uint32_t Size);
uint8_t BSP_AUDIO_IN_Stop();
uint8_t BSP_AUDIO_IN_Pause();
uint8_t BSP_AUDIO_IN_Resume();
uint8_t BSP_AUDIO_IN_SetVolume (uint8_t Volume);
uint8_t BSP_AUDIO_IN_PDMToPCM (uint16_t *PDMBuf, uint16_t *PCMBuf);
void BSP_AUDIO_IN_TransferComplete_CallBack();
void BSP_AUDIO_IN_HalfTransfer_CallBack();
void BSP_AUDIO_IN_Error_Callback();
void BSP_AUDIO_IN_ClockConfig (I2S_HandleTypeDef *hi2s, uint32_t AudioFreq, void *Params);
void BSP_AUDIO_IN_MspInit (I2S_HandleTypeDef *hi2s, void *Params);
void BSP_AUDIO_IN_MspDeInit (I2S_HandleTypeDef *hi2s, void *Params);
//{{{
#ifdef __cplusplus
}
#endif
//}}}
