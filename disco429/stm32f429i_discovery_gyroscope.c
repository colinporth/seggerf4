#include "stm32f429i_discovery_gyroscope.h"
//{{{  struct GYRO_DrvTypeDef
typedef struct {
  void       (*Init)(uint16_t);
  void       (*DeInit)(void);
  uint8_t    (*ReadID)(void);
  void       (*Reset)(void);
  void       (*LowPower)(uint16_t);
  void       (*ConfigIT)(uint16_t);
  void       (*EnableIT)(uint8_t);
  void       (*DisableIT)(uint8_t);
  uint8_t    (*ITStatus)(uint16_t, uint16_t);
  void       (*ClearIT)(uint16_t, uint16_t);
  void       (*FilterConfig)(uint8_t);
  void       (*FilterCmd)(uint8_t);
  void       (*GetXYZ)(float *);
  } GYRO_DrvTypeDef;
//}}}
//{{{  struct GYRO_InitTypeDef
typedef struct {
  uint8_t Power_Mode;                         /* Power-down/Sleep/Normal Mode */
  uint8_t Output_DataRate;                    /* OUT data rate */
  uint8_t Axes_Enable;                        /* Axes enable */
  uint8_t Band_Width;                         /* Bandwidth selection */
  uint8_t BlockData_Update;                   /* Block Data Update */
  uint8_t Endianness;                         /* Endian Data selection */
  uint8_t Full_Scale;                         /* Full Scale selection */
  } GYRO_InitTypeDef;
//}}}
//{{{  struct GYRO_FilterConfigTypeDef
// GYRO High Pass Filter struct
typedef struct {
  uint8_t HighPassFilter_Mode_Selection;      /* Internal filter mode */
  uint8_t HighPassFilter_CutOff_Frequency;    /* High pass filter cut-off frequency */
  } GYRO_FilterConfigTypeDef;
//}}}

//{{{  defines
#define L3GD20_WHO_AM_I_ADDR          0x0F  /* device identification register */
#define L3GD20_CTRL_REG1_ADDR         0x20  /* Control register 1 */
#define L3GD20_CTRL_REG2_ADDR         0x21  /* Control register 2 */
#define L3GD20_CTRL_REG3_ADDR         0x22  /* Control register 3 */
#define L3GD20_CTRL_REG4_ADDR         0x23  /* Control register 4 */
#define L3GD20_CTRL_REG5_ADDR         0x24  /* Control register 5 */
#define L3GD20_REFERENCE_REG_ADDR     0x25  /* Reference register */
#define L3GD20_OUT_TEMP_ADDR          0x26  /* Out temp register */
#define L3GD20_STATUS_REG_ADDR        0x27  /* Status register */
#define L3GD20_OUT_X_L_ADDR           0x28  /* Output Register X */
#define L3GD20_OUT_X_H_ADDR           0x29  /* Output Register X */
#define L3GD20_OUT_Y_L_ADDR           0x2A  /* Output Register Y */
#define L3GD20_OUT_Y_H_ADDR           0x2B  /* Output Register Y */
#define L3GD20_OUT_Z_L_ADDR           0x2C  /* Output Register Z */
#define L3GD20_OUT_Z_H_ADDR           0x2D  /* Output Register Z */
#define L3GD20_FIFO_CTRL_REG_ADDR     0x2E  /* Fifo control Register */
#define L3GD20_FIFO_SRC_REG_ADDR      0x2F  /* Fifo src Register */

#define L3GD20_INT1_CFG_ADDR          0x30  /* Interrupt 1 configuration Register */
#define L3GD20_INT1_SRC_ADDR          0x31  /* Interrupt 1 source Register */
#define L3GD20_INT1_TSH_XH_ADDR       0x32  /* Interrupt 1 Threshold X register */
#define L3GD20_INT1_TSH_XL_ADDR       0x33  /* Interrupt 1 Threshold X register */
#define L3GD20_INT1_TSH_YH_ADDR       0x34  /* Interrupt 1 Threshold Y register */
#define L3GD20_INT1_TSH_YL_ADDR       0x35  /* Interrupt 1 Threshold Y register */
#define L3GD20_INT1_TSH_ZH_ADDR       0x36  /* Interrupt 1 Threshold Z register */
#define L3GD20_INT1_TSH_ZL_ADDR       0x37  /* Interrupt 1 Threshold Z register */
#define L3GD20_INT1_DURATION_ADDR     0x38  /* Interrupt 1 DURATION register */

#define I_AM_L3GD20                 ((uint8_t)0xD4)
#define I_AM_L3GD20_TR              ((uint8_t)0xD5)

#define L3GD20_MODE_POWERDOWN       ((uint8_t)0x00)
#define L3GD20_MODE_ACTIVE          ((uint8_t)0x08)

#define L3GD20_OUTPUT_DATARATE_1    ((uint8_t)0x00)
#define L3GD20_OUTPUT_DATARATE_2    ((uint8_t)0x40)
#define L3GD20_OUTPUT_DATARATE_3    ((uint8_t)0x80)
#define L3GD20_OUTPUT_DATARATE_4    ((uint8_t)0xC0)

#define L3GD20_X_ENABLE            ((uint8_t)0x02)
#define L3GD20_Y_ENABLE            ((uint8_t)0x01)
#define L3GD20_Z_ENABLE            ((uint8_t)0x04)
#define L3GD20_AXES_ENABLE         ((uint8_t)0x07)
#define L3GD20_AXES_DISABLE        ((uint8_t)0x00)

#define L3GD20_BANDWIDTH_1         ((uint8_t)0x00)
#define L3GD20_BANDWIDTH_2         ((uint8_t)0x10)
#define L3GD20_BANDWIDTH_3         ((uint8_t)0x20)
#define L3GD20_BANDWIDTH_4         ((uint8_t)0x30)

#define L3GD20_FULLSCALE_250       ((uint8_t)0x00)
#define L3GD20_FULLSCALE_500       ((uint8_t)0x10)
#define L3GD20_FULLSCALE_2000      ((uint8_t)0x20)
#define L3GD20_FULLSCALE_SELECTION ((uint8_t)0x30)

#define L3GD20_SENSITIVITY_250DPS  ((float)8.75f)         /*!< gyroscope sensitivity with 250 dps full scale [DPS/LSB]  */
#define L3GD20_SENSITIVITY_500DPS  ((float)17.50f)        /*!< gyroscope sensitivity with 500 dps full scale [DPS/LSB]  */
#define L3GD20_SENSITIVITY_2000DPS ((float)70.00f)        /*!< gyroscope sensitivity with 2000 dps full scale [DPS/LSB] */

#define L3GD20_BlockDataUpdate_Continous   ((uint8_t)0x00)
#define L3GD20_BlockDataUpdate_Single      ((uint8_t)0x80)

#define L3GD20_BLE_LSB                     ((uint8_t)0x00)
#define L3GD20_BLE_MSB                     ((uint8_t)0x40)

#define L3GD20_HIGHPASSFILTER_DISABLE      ((uint8_t)0x00)
#define L3GD20_HIGHPASSFILTER_ENABLE       ((uint8_t)0x10)

#define L3GD20_INT1                        ((uint8_t)0x00)
#define L3GD20_INT2                        ((uint8_t)0x01)

#define L3GD20_INT1INTERRUPT_DISABLE       ((uint8_t)0x00)
#define L3GD20_INT1INTERRUPT_ENABLE        ((uint8_t)0x80)

#define L3GD20_INT2INTERRUPT_DISABLE       ((uint8_t)0x00)
#define L3GD20_INT2INTERRUPT_ENABLE        ((uint8_t)0x08)

#define L3GD20_INT1INTERRUPT_LOW_EDGE      ((uint8_t)0x20)
#define L3GD20_INT1INTERRUPT_HIGH_EDGE     ((uint8_t)0x00)

#define L3GD20_BOOT_NORMALMODE             ((uint8_t)0x00)
#define L3GD20_BOOT_REBOOTMEMORY           ((uint8_t)0x80)

#define L3GD20_HPM_NORMAL_MODE_RES         ((uint8_t)0x00)
#define L3GD20_HPM_REF_SIGNAL              ((uint8_t)0x10)
#define L3GD20_HPM_NORMAL_MODE             ((uint8_t)0x20)
#define L3GD20_HPM_AUTORESET_INT           ((uint8_t)0x30)

#define L3GD20_HPFCF_0              0x00
#define L3GD20_HPFCF_1              0x01
#define L3GD20_HPFCF_2              0x02
#define L3GD20_HPFCF_3              0x03
#define L3GD20_HPFCF_4              0x04
#define L3GD20_HPFCF_5              0x05
#define L3GD20_HPFCF_6              0x06
#define L3GD20_HPFCF_7              0x07
#define L3GD20_HPFCF_8              0x08
#define L3GD20_HPFCF_9              0x09
//}}}
static GYRO_DrvTypeDef* GyroscopeDrv;

//{{{
void L3GD20_Init (uint16_t InitStruct) {

  uint8_t ctrl = 0x00;

  /* Configure the low level interface */
  GYRO_IO_Init();

  /* Write value to MEMS CTRL_REG1 register */
  ctrl = (uint8_t) InitStruct;
  GYRO_IO_Write (&ctrl, L3GD20_CTRL_REG1_ADDR, 1);

  /* Write value to MEMS CTRL_REG4 register */
  ctrl = (uint8_t) (InitStruct >> 8);
  GYRO_IO_Write (&ctrl, L3GD20_CTRL_REG4_ADDR, 1);
  }
//}}}
void L3GD20_DeInit() {}
//{{{
uint8_t L3GD20_ReadID() {

  uint8_t tmp;

  /* Configure the low level interface */
  GYRO_IO_Init();

  /* Read WHO I AM register */
  GYRO_IO_Read(&tmp, L3GD20_WHO_AM_I_ADDR, 1);

  /* Return the ID */
  return (uint8_t)tmp;
  }
//}}}
//{{{
void L3GD20_RebootCmd() {

  uint8_t tmpreg;

  /* Read CTRL_REG5 register */
  GYRO_IO_Read(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);

  /* Enable or Disable the reboot memory */
  tmpreg |= L3GD20_BOOT_REBOOTMEMORY;

  /* Write value to MEMS CTRL_REG5 register */
  GYRO_IO_Write(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);
  }
//}}}
//{{{
void L3GD20_LowPower (uint16_t InitStruct) {

  uint8_t ctrl = 0x00;

  /* Write value to MEMS CTRL_REG1 register */
  ctrl = (uint8_t) InitStruct;
  GYRO_IO_Write(&ctrl, L3GD20_CTRL_REG1_ADDR, 1);
  }
//}}}
//{{{
void L3GD20_INT1InterruptConfig (uint16_t Int1Config) {

  uint8_t ctrl_cfr = 0x00, ctrl3 = 0x00;

  /* Read INT1_CFG register */
  GYRO_IO_Read(&ctrl_cfr, L3GD20_INT1_CFG_ADDR, 1);

  /* Read CTRL_REG3 register */
  GYRO_IO_Read(&ctrl3, L3GD20_CTRL_REG3_ADDR, 1);

  ctrl_cfr &= 0x80;
  ctrl_cfr |= ((uint8_t) Int1Config >> 8);

  ctrl3 &= 0xDF;
  ctrl3 |= ((uint8_t) Int1Config);

  /* Write value to MEMS INT1_CFG register */
  GYRO_IO_Write(&ctrl_cfr, L3GD20_INT1_CFG_ADDR, 1);

  /* Write value to MEMS CTRL_REG3 register */
  GYRO_IO_Write(&ctrl3, L3GD20_CTRL_REG3_ADDR, 1);
  }
//}}}
//{{{
void L3GD20_EnableIT (uint8_t IntSel) {

  uint8_t tmpreg;

  /* Read CTRL_REG3 register */
  GYRO_IO_Read(&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);

  if (IntSel == L3GD20_INT1) {
    tmpreg &= 0x7F;
    tmpreg |= L3GD20_INT1INTERRUPT_ENABLE;
    }
  else if (IntSel == L3GD20_INT2) {
    tmpreg &= 0xF7;
    tmpreg |= L3GD20_INT2INTERRUPT_ENABLE;
    }

  /* Write value to MEMS CTRL_REG3 register */
  GYRO_IO_Write (&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);
  }
//}}}
//{{{
void L3GD20_DisableIT (uint8_t IntSel) {

  uint8_t tmpreg;

  /* Read CTRL_REG3 register */
  GYRO_IO_Read(&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);

  if (IntSel == L3GD20_INT1) {
    tmpreg &= 0x7F;
    tmpreg |= L3GD20_INT1INTERRUPT_DISABLE;
    }
  else if(IntSel == L3GD20_INT2) {
    tmpreg &= 0xF7;
    tmpreg |= L3GD20_INT2INTERRUPT_DISABLE;
    }

  /* Write value to MEMS CTRL_REG3 register */
  GYRO_IO_Write (&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);
  }
//}}}
//{{{
void L3GD20_FilterConfig (uint8_t FilterStruct) {

  /* Read CTRL_REG2 register */
  uint8_t tmpreg;
  GYRO_IO_Read (&tmpreg, L3GD20_CTRL_REG2_ADDR, 1);
  tmpreg &= 0xC0;

  /* Configure MEMS: mode and cutoff frequency */
  tmpreg |= FilterStruct;

  /* Write value to MEMS CTRL_REG2 register */
  GYRO_IO_Write (&tmpreg, L3GD20_CTRL_REG2_ADDR, 1);
  }
//}}}
//{{{
void L3GD20_FilterCmd (uint8_t HighPassFilterState) {

  /* Read CTRL_REG5 register */
  uint8_t tmpreg;
  GYRO_IO_Read (&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);
  tmpreg &= 0xEF;

  tmpreg |= HighPassFilterState;

  /* Write value to MEMS CTRL_REG5 register */
  GYRO_IO_Write (&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);
  }
//}}}
//{{{
uint8_t L3GD20_GetDataStatus() {

  uint8_t tmpreg;

  /* Read STATUS_REG register */
  GYRO_IO_Read (&tmpreg, L3GD20_STATUS_REG_ADDR, 1);

  return tmpreg;
  }
//}}}
//{{{
void L3GD20_ReadXYZAngRate (float* pfData) {

  int i = 0;

  uint8_t tmpreg = 0;
  GYRO_IO_Read (&tmpreg, L3GD20_CTRL_REG4_ADDR, 1);

  uint8_t tmpbuffer[6] = {0};
  GYRO_IO_Read (tmpbuffer, L3GD20_OUT_X_L_ADDR, 6);

  // check in the control register 4 the data alignment (Big Endian or Little Endian) 
  int16_t RawData[3] = {0};
  if (!(tmpreg & L3GD20_BLE_MSB))
    for (i = 0; i < 3; i++)
      RawData[i] = (int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
  else
    for (i = 0; i < 3; i++)
      RawData[i] = (int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);

  // Switch the sensitivity value set in the CRTL4
  float sensitivity = 0;
  switch (tmpreg & L3GD20_FULLSCALE_SELECTION) {
    case L3GD20_FULLSCALE_250:
      sensitivity = L3GD20_SENSITIVITY_250DPS;
      break;

    case L3GD20_FULLSCALE_500:
      sensitivity = L3GD20_SENSITIVITY_500DPS;
      break;

    case L3GD20_FULLSCALE_2000:
      sensitivity = L3GD20_SENSITIVITY_2000DPS;
      break;
    }

  /* Divide by sensitivity */
  for (i = 0; i < 3; i++)
    pfData[i] = (float)(RawData[i] * sensitivity);
  }
//}}}
//{{{
GYRO_DrvTypeDef L3gd20Drv = {
  L3GD20_Init,
  L3GD20_DeInit,
  L3GD20_ReadID,
  L3GD20_RebootCmd,
  L3GD20_LowPower,
  L3GD20_INT1InterruptConfig,
  L3GD20_EnableIT,
  L3GD20_DisableIT,
  0,
  0,
  L3GD20_FilterConfig,
  L3GD20_FilterCmd,
  L3GD20_ReadXYZAngRate
  };
//}}}

//{{{
uint8_t BSP_GYRO_Init() {

  uint8_t ret = GYRO_ERROR;
  uint16_t ctrl = 0x0000;
  GYRO_InitTypeDef L3GD20_InitStructure;
  GYRO_FilterConfigTypeDef L3GD20_FilterStructure={0,0};

  if((L3gd20Drv.ReadID() == I_AM_L3GD20) || (L3gd20Drv.ReadID() == I_AM_L3GD20_TR)) {
    /* Initialize the Gyroscope driver structure */
    GyroscopeDrv = &L3gd20Drv;

    /* MEMS configuration ----------------------------------------------------*/
    /* Fill the Gyroscope structure */
    L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
    L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
    L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
    L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
    L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
    L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
    L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_500;

    /* Configure MEMS: data rate, power mode, full scale and axes */
    ctrl = (uint16_t) (L3GD20_InitStructure.Power_Mode | L3GD20_InitStructure.Output_DataRate | \
                       L3GD20_InitStructure.Axes_Enable | L3GD20_InitStructure.Band_Width);

    ctrl |= (uint16_t) ((L3GD20_InitStructure.BlockData_Update | L3GD20_InitStructure.Endianness | \
                         L3GD20_InitStructure.Full_Scale) << 8);

    /* Configure the Gyroscope main parameters */
    GyroscopeDrv->Init (ctrl);

    L3GD20_FilterStructure.HighPassFilter_Mode_Selection = L3GD20_HPM_NORMAL_MODE_RES;
    L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;

    ctrl = (uint8_t) ((L3GD20_FilterStructure.HighPassFilter_Mode_Selection |\
                       L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency));

    /* Configure the Gyroscope main parameters */
    GyroscopeDrv->FilterConfig (ctrl) ;
    GyroscopeDrv->FilterCmd (L3GD20_HIGHPASSFILTER_ENABLE);

    ret = GYRO_OK;
    }
  else
    ret = GYRO_ERROR;

  return ret;
  }
//}}}
//{{{
uint8_t BSP_GYRO_ReadID() {

  uint8_t id = 0x00;
  if (GyroscopeDrv->ReadID != NULL)
    id = GyroscopeDrv->ReadID();
  return id;
  }
//}}}
//{{{
void BSP_GYRO_Reset() {

  if (GyroscopeDrv->Reset != NULL)
    GyroscopeDrv->Reset();
  }
//}}}
//{{{
void BSP_GYRO_ITConfig (GYRO_InterruptConfigTypeDef* pIntConfig) {

  uint16_t interruptconfig = 0x0000;

  if(GyroscopeDrv->ConfigIT != NULL) {
    /* Configure latch Interrupt request and axe interrupts */
    interruptconfig |= ((uint8_t)(pIntConfig->Latch_Request| \
                                  pIntConfig->Interrupt_Axes) << 8);

    interruptconfig |= (uint8_t)(pIntConfig->Interrupt_ActiveEdge);

    GyroscopeDrv->ConfigIT(interruptconfig);
    }
  }
//}}}
//{{{
void BSP_GYRO_EnableIT (uint8_t IntPin) {

  if(GyroscopeDrv->EnableIT != NULL)
    GyroscopeDrv->EnableIT(IntPin);
  }
//}}}
//{{{
void BSP_GYRO_DisableIT (uint8_t IntPin) {

  if(GyroscopeDrv->DisableIT != NULL)
    GyroscopeDrv->DisableIT(IntPin);
  }
//}}}
//{{{
void BSP_GYRO_GetXYZ (float* pfData) {
  if (GyroscopeDrv->GetXYZ != NULL)
    GyroscopeDrv->GetXYZ (pfData);
  }
//}}}
