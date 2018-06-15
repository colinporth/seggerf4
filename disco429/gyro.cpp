// L3GD20 gyro
#include "gyro.h"
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

// WHO_AM_I
#define I_AM_L3GD20                 ((uint8_t)0xD4)
#define I_AM_L3GD20_TR              ((uint8_t)0xD5)

// CTRL_REG1
#define L3GD20_OUTPUT_DATARATE_95Hz  ((uint8_t)0x00)
#define L3GD20_OUTPUT_DATARATE_190Hz ((uint8_t)0x40)
#define L3GD20_OUTPUT_DATARATE_380Hz ((uint8_t)0x80)
#define L3GD20_OUTPUT_DATARATE_760Hz ((uint8_t)0xC0)

#define L3GD20_BANDWIDTH_1         ((uint8_t)0x00)
#define L3GD20_BANDWIDTH_2         ((uint8_t)0x10)
#define L3GD20_BANDWIDTH_3         ((uint8_t)0x20)
#define L3GD20_BANDWIDTH_4         ((uint8_t)0x30)

#define L3GD20_MODE_POWERDOWN       ((uint8_t)0x00)
#define L3GD20_MODE_ACTIVE          ((uint8_t)0x08)

#define L3GD20_X_ENABLE            ((uint8_t)0x02)
#define L3GD20_Y_ENABLE            ((uint8_t)0x01)
#define L3GD20_Z_ENABLE            ((uint8_t)0x04)

// CTRL_REG2
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

// CTRL_REG4
#define L3GD20_BlockDataUpdate_Single  ((uint8_t)0x80)
#define L3GD20_BLE_MSB                 ((uint8_t)0x40)

#define L3GD20_FULLSCALE_250       ((uint8_t)0x00)
#define L3GD20_FULLSCALE_500       ((uint8_t)0x10)
#define L3GD20_FULLSCALE_2000      ((uint8_t)0x20)
#define L3GD20_FULLSCALE_SELECTION ((uint8_t)0x30)

// CTRL_REG5
#define L3GD20_REBOOTMEMORY      ((uint8_t)0x80)
#define L3GD20_FIFO_ENABLE       ((uint8_t)0x40)
#define L3GD20_HIGHPASSFILTER_ENABLE  ((uint8_t)0x10)

#define L3GD20_INT1                        ((uint8_t)0x00)
#define L3GD20_INT2                        ((uint8_t)0x01)
#define L3GD20_INT1INTERRUPT_DISABLE       ((uint8_t)0x00)
#define L3GD20_INT1INTERRUPT_ENABLE        ((uint8_t)0x80)
#define L3GD20_INT2INTERRUPT_DISABLE       ((uint8_t)0x00)
#define L3GD20_INT2INTERRUPT_ENABLE        ((uint8_t)0x08)
#define L3GD20_INT1INTERRUPT_LOW_EDGE      ((uint8_t)0x20)
#define L3GD20_INT1INTERRUPT_HIGH_EDGE     ((uint8_t)0x00)

#define L3GD20_HPM_NORMAL_MODE_RES         ((uint8_t)0x00)
#define L3GD20_HPM_REF_SIGNAL              ((uint8_t)0x10)
#define L3GD20_HPM_NORMAL_MODE             ((uint8_t)0x20)
#define L3GD20_HPM_AUTORESET_INT           ((uint8_t)0x30)

#define L3GD20_SENSITIVITY_250DPS  ((float)8.75f)   // gyroscope sensitivity with 250 dps full scale [DPS/LSB]
#define L3GD20_SENSITIVITY_500DPS  ((float)17.50f)  // gyroscope sensitivity with 500 dps full scale [DPS/LSB]
#define L3GD20_SENSITIVITY_2000DPS ((float)70.00f)  // gyroscope sensitivity with 2000 dps full scale [DPS/LSB]
//}}}

//{{{
uint8_t gyroInit() {

  GYRO_IO_Init();

  uint8_t value = L3GD20_REBOOTMEMORY;
  GYRO_IO_Write (&value, L3GD20_CTRL_REG5_ADDR, 1);
  HAL_Delay(250);

  uint8_t id;
  GYRO_IO_Read (&id, L3GD20_WHO_AM_I_ADDR, 1);

  // config CTRL_REG2
  value = L3GD20_HPFCF_0;
  GYRO_IO_Write (&value, L3GD20_CTRL_REG2_ADDR, 1);

  // config CTRL_REG4
  value = L3GD20_FULLSCALE_500;
  GYRO_IO_Write (&value, L3GD20_CTRL_REG4_ADDR, 1);

  // reset reference
  //value = 0;
  //GYRO_IO_Write (&value, L3GD20_REFERENCE_REG_ADDR, 1);

  // config CTRL_REG5
  value = L3GD20_FIFO_ENABLE | L3GD20_HIGHPASSFILTER_ENABLE;
  GYRO_IO_Write (&value, L3GD20_CTRL_REG5_ADDR, 1);

  // config FIFO_CTRL_REG_ADDR - STREAMING_MODE
  value = 0x60;
  GYRO_IO_Write (&value, L3GD20_FIFO_CTRL_REG_ADDR, 1);

  // config CTRL_REG1
  value = L3GD20_OUTPUT_DATARATE_760Hz |
          L3GD20_BANDWIDTH_1 |
          L3GD20_MODE_ACTIVE |
          L3GD20_X_ENABLE | L3GD20_Y_ENABLE | L3GD20_Z_ENABLE;
  GYRO_IO_Write (&value, L3GD20_CTRL_REG1_ADDR, 1);

  return id;
  }
//}}}
//{{{
void gyroReset() {

  // Read CTRL_REG5 register
  uint8_t tmpreg;
  GYRO_IO_Read (&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);
  tmpreg |= L3GD20_REBOOTMEMORY;
  GYRO_IO_Write (&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);
  }
//}}}
//{{{
uint8_t gyroGetStatus() {

  uint8_t value;
  GYRO_IO_Read (&value, L3GD20_STATUS_REG_ADDR, 1);
  return value;
  }
//}}}
//{{{
uint8_t gyroGetFifoSrc() {

  uint8_t value;
  GYRO_IO_Read (&value, L3GD20_FIFO_SRC_REG_ADDR, 1);
  return value;
  }
//}}}
//{{{
void gyroGetXYZ (int16_t* xyz) {
  GYRO_IO_Read ((uint8_t*)xyz, L3GD20_OUT_X_L_ADDR, 6);
  }
//}}}

//{{{
void gyroITConfig (GYRO_InterruptConfigTypeDef* pIntConfig) {

  // Configure latch Interrupt request and axe interrupts
  uint16_t interruptconfig = 0x0000;
  interruptconfig |= ((uint8_t)(pIntConfig->Latch_Request| pIntConfig->Interrupt_Axes) << 8);
  interruptconfig |= (uint8_t)(pIntConfig->Interrupt_ActiveEdge);

  // Read INT1_CFG register
  uint8_t ctrl_cfr = 0x00;
  GYRO_IO_Read(&ctrl_cfr, L3GD20_INT1_CFG_ADDR, 1);
  ctrl_cfr &= 0x80;
  ctrl_cfr |= ((uint8_t) interruptconfig >> 8);
  GYRO_IO_Write(&ctrl_cfr, L3GD20_INT1_CFG_ADDR, 1);

  // Read CTRL_REG3 register
  uint8_t ctrl3 = 0x00;
  GYRO_IO_Read (&ctrl3, L3GD20_CTRL_REG3_ADDR, 1);
  ctrl3 &= 0xDF;
  ctrl3 |= ((uint8_t) interruptconfig);
  GYRO_IO_Write(&ctrl3, L3GD20_CTRL_REG3_ADDR, 1);
  }
//}}}
//{{{
void gyroEnableIT (uint8_t IntPin) {

  uint8_t tmpreg;

  /* Read CTRL_REG3 register */
  GYRO_IO_Read(&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);

  if (IntPin == L3GD20_INT1) {
    tmpreg &= 0x7F;
    tmpreg |= L3GD20_INT1INTERRUPT_ENABLE;
    }
  else if (IntPin == L3GD20_INT2) {
    tmpreg &= 0xF7;
    tmpreg |= L3GD20_INT2INTERRUPT_ENABLE;
    }

  /* Write value to MEMS CTRL_REG3 register */
  GYRO_IO_Write (&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);
  }
//}}}
//{{{
void gyroDisableIT (uint8_t IntPin) {

  uint8_t tmpreg;

  /* Read CTRL_REG3 register */
  GYRO_IO_Read(&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);

  if (IntPin == L3GD20_INT1) {
    tmpreg &= 0x7F;
    tmpreg |= L3GD20_INT1INTERRUPT_DISABLE;
    }
  else if(IntPin == L3GD20_INT2) {
    tmpreg &= 0xF7;
    tmpreg |= L3GD20_INT2INTERRUPT_DISABLE;
    }

  /* Write value to MEMS CTRL_REG3 register */
  GYRO_IO_Write (&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);
  }
//}}}
