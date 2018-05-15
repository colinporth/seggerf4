#include "stm32f4xx_hal.h"

// Audio status definition
#define AUDIO_OK                 0
#define AUDIO_ERROR              1
#define AUDIO_TIMEOUT            2

// Codec output DEVICE
#define OUTPUT_DEVICE_SPEAKER    1
#define OUTPUT_DEVICE_HEADPHONE  2
#define OUTPUT_DEVICE_BOTH       3

// Codec POWER DOWN modes
#define CODEC_PDWN_HW         1
#define CODEC_PDWN_SW         2

// PCM buffer output size
#define PCM_OUT_SIZE                    DEFAULT_AUDIO_IN_FREQ/1000
#define CHANNEL_DEMUX_MASK              0x55

// sampleRate * DataSize (2 bytes) * NumChannels (Stereo: 2)
#define DEFAULT_AUDIO_IN_FREQ           I2S_AUDIOFREQ_16K
#define DEFAULT_AUDIO_IN_BIT_RESOLUTION 16
#define DEFAULT_AUDIO_IN_CHANNEL_NBR    1 // Mono = 1, Stereo = 2
#define DEFAULT_AUDIO_IN_VOLUME         64

// PDM buffer input size
#define INTERNAL_BUFF_SIZE              128*DEFAULT_AUDIO_IN_FREQ/16000*DEFAULT_AUDIO_IN_CHANNEL_NBR

extern uint16_t AudioInVolume;

void audioOutInit (uint16_t outputDevice, uint8_t volume, uint32_t sampleRate);
void audioPlay (uint16_t* buffer, uint32_t size);
void audioChangeBuffer (uint16_t* data, uint32_t size);
void audioPause();
void audioResume();
void audioStop (uint32_t Option);
void audioSetVolume (uint8_t Volume);
void audioSetMute (uint32_t Cmd);

void audioTransferComplete_CallBack();
void audioHalfTransfer_CallBack();
void audioError_CallBack();
void audioClockConfig (I2S_HandleTypeDef *hi2s, uint32_t sampleRate);
void audioMspInit (I2S_HandleTypeDef* hi2s);
void audioMspDeInit (I2S_HandleTypeDef* hi2s);

uint8_t audioInInit (uint32_t sampleRate, uint32_t BitRes, uint32_t ChnlNbr);
uint8_t audioInRecord (uint16_t *pData, uint32_t Size);
uint8_t audioInStop();
uint8_t audioInPause();
uint8_t audioInResume();
uint8_t audioInSetVolume (uint8_t Volume);
uint8_t audioInPDMToPCM (uint16_t *PDMBuf, uint16_t *PCMBuf);

void audioInClockConfig (I2S_HandleTypeDef *hi2s, uint32_t sampleRate);
void audioInMspInit (I2S_HandleTypeDef* hi2s);
void audioInMspDeInit (I2S_HandleTypeDef* hi2s);
void audioInTransferComplete_CallBack();
void audioInHalfTransfer_CallBack();
void audioInError_Callback();
