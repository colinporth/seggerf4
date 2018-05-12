#include "accelerometer.h"
#include "stm32f4_discovery.h"
#include <stdint.h>

//{{{  defines
#define LIS302DL_WHO_AM_I_ADDR             0x0F
//{{{  CTRL_REG1
/*******************************************************************************
*  CTRL_REG1 Register: Control Register 1
*  Read Write register
*  Default value: 0x07
*  7 DR: Data Rate selection.
*        0 - 100 Hz output data rate
*        1 - 400 Hz output data rate
*  6 PD: Power Down control.
*        0 - power down mode
*        1 - active mode
*  5 FS: Full Scale selection.
*        0 - Typical measurement range 2.3
*        1 - Typical measurement range 9.2
*  4:3 STP-STM Self Test Enable:
*              STP |  STM |   mode
*            ----------------------------
*               0  |  0   |   Normal mode
*               0  |  1   |   Self Test M
*               1  |  0   |   Self Test P
*  2 Zen: Z axis enable.
*         0 - Z axis disabled
*         1- Z axis enabled
*  1 Yen: Y axis enable.
*         0 - Y axis disabled
*         1- Y axis enabled
*  0 Xen: X axis enable.
*         0 - X axis disabled
*         1- X axis enabled
*******************************************************************************/
//}}}
#define LIS302DL_CTRL_REG1_ADDR            0x20
//{{{  CTRL_REG2
/* CTRL_REG2 Regsiter: Control Register 2
*  Read Write register
*  Default value: 0x00
*  7 SIM: SPI Serial Interface Mode Selection.
*         0 - 4 wire interface
*         1 - 3 wire interface
*  6 BOOT: Reboot memory content
*          0 - normal mode
*          1 - reboot memory content
*  5 Reserved
*  4 FDS: Filtered data selection.
*         0 - internal filter bypassed
*         1 - data from internal filter sent to output register
*  3 HP FF_WU2: High pass filter enabled for FreeFall/WakeUp#2.
*               0 - filter bypassed
*               1 - filter enabled
*  2 HP FF_WU1: High pass filter enabled for FreeFall/WakeUp#1.
*               0 - filter bypassed
*               1 - filter enabled
*  1:0 HP coeff2-HP coeff1 High pass filter cut-off frequency (ft) configuration.
*                 ft= ODR[hz]/6*HP coeff
*            HP coeff2 | HP coeff1 |   HP coeff
*            -------------------------------------------
*                 0     |     0     |   8
*                 0     |     1     |   16
*                 1     |     0     |   32
*                 1     |     1     |   64
*            HP coeff |  ft[hz]   |  ft[hz]   |
*                     |ODR 100Hz | ODR 400Hz  |
*            --------------------------------------------
*              00     |    2      |     8     |
*              01     |    1      |     4     |
*              10     |    0.5    |     2     |
*              11     |    0.25   |     1     |
*/
//}}}
#define LIS302DL_CTRL_REG2_ADDR            0x21
//{{{  CTRL_REG3
/*******************************************************************************
*  CTRL_REG3 Register: Interrupt Control Register
*  Read Write register
*  Default value: 0x00
*  7 IHL active: Interrupt active high/low.
*                0 - active high
*                1 - active low
*  6 PP_OD: push-pull/open-drain.
*           0 - push-pull
*           1 - open-drain
*  5:3 I2_CFG2 - I2_CFG0 Data signal on INT2 pad control bits
*  2:0 I1_CFG2 - I1_CFG0 Data signal on INT1 pad control bits
*        I1(2)_CFG2  |  I1(2)_CFG1  |  I1(2)_CFG0  | INT1(2) Pad
*        ----------------------------------------------------------
*              0     |      0       |       0      | GND
*              0     |      0       |       1      | FreeFall/WakeUp#1
*              0     |      1       |       0      | FreeFall/WakeUp#2
*              0     |      1       |       1      | FreeFall/WakeUp#1 or FreeFall/WakeUp#2
*              1     |      0       |       0      | Data ready
*              1     |      1       |       1      | Click interrupt
*******************************************************************************/
//}}}
#define LIS302DL_CTRL_REG3_ADDR            0x22
//{{{  HP_FILTER_RESET
/*******************************************************************************
*  HP_FILTER_RESET Register: Dummy register. Reading at this address zeroes
*  instantaneously the content of the internal high pass filter. If the high pass
*  filter is enabled all three axes are instantaneously set to 0g.
*  This allows to overcome the settling time of the high pass filter.
*  Read only register
*  Default value: Dummy
*******************************************************************************/
//}}}
#define LIS302DL_HP_FILTER_RESET_REG_ADDR  0x23
//{{{  STATUS_REG
/*******************************************************************************
*  STATUS_REG Register: Status Register
*  Default value: 0x00
*  7 ZYXOR: X, Y and Z axis data overrun.
*           0: no overrun has occurred
*           1: new data has overwritten the previous one before it was read
*  6 ZOR: Z axis data overrun.
*         0: no overrun has occurred
*         1: new data for Z-axis has overwritten the previous one before it was read
*  5 yOR: y axis data overrun.
*         0: no overrun has occurred
*         1: new data for y-axis has overwritten the previous one before it was read
*  4 XOR: X axis data overrun.
*         0: no overrun has occurred
*         1: new data for X-axis has overwritten the previous one before it was read
*  3 ZYXDA: X, Y and Z axis new data available
*           0: a new set of data is not yet available
*           1: a new set of data is available
*  2 ZDA: Z axis new data available.
*         0: a new set of data is not yet available
*         1: a new data for Z axis is available
*  1 YDA: Y axis new data available
*         0: a new set of data is not yet available
*         1: a new data for Y axis is available
*  0 XDA: X axis new data available
*         0: a new set of data is not yet available
*         1: a new data for X axis is available
*******************************************************************************/
//}}}
#define LIS302DL_STATUS_REG_ADDR           0x27
//{{{  OUT_X
/*******************************************************************************
*  OUT_X Register: X-axis output Data
*  Read only register
*  Default value: output
*  7:0 XD7-XD0: X-axis output Data
*******************************************************************************/
//}}}
#define LIS302DL_OUT_X_ADDR                0x29
//{{{  OUT_Y
/*******************************************************************************
*  OUT_Y Register: Y-axis output Data
*  Read only register
*  Default value: output
*  7:0 YD7-YD0: Y-axis output Data
*******************************************************************************/
//}}}
#define LIS302DL_OUT_Y_ADDR                0x2B
//{{{  OUT_Z
/*******************************************************************************
*  OUT_Z Register: Z-axis output Data
*  Read only register
*  Default value: output
*  7:0 ZD7-ZD0: Z-axis output Data
*******************************************************************************/
//}}}
#define LIS302DL_OUT_Z_ADDR                0x2D
//{{{  FF_WW_CFG_1
/*******************************************************************************
*  FF_WW_CFG_1 Register: Configuration register for Interrupt 1 source.
*  Read write register
*  Default value: 0x00
*  7 AOI: AND/OR combination of Interrupt events.
*         0: OR combination of interrupt events
*         1: AND combination of interrupt events
*  6 LIR: Latch/not latch interrupt request
*         0: interrupt request not latched
*         1: interrupt request latched
*  5 ZHIE: Enable interrupt generation on Z high event.
*          0: disable interrupt request
*          1: enable interrupt request on measured accel. value higher than preset threshold
*  4 ZLIE: Enable interrupt generation on Z low event.
*          0: disable interrupt request
*          1: enable interrupt request on measured accel. value lower than preset threshold
*  3 YHIE: Enable interrupt generation on Y high event.
*          0: disable interrupt request
*          1: enable interrupt request on measured accel. value higher than preset threshold
*  2 YLIE: Enable interrupt generation on Y low event.
*          0: disable interrupt request
*          1: enable interrupt request on measured accel. value lower than preset threshold
*  1 XHIE: Enable interrupt generation on X high event.
*          0: disable interrupt request
*          1: enable interrupt request on measured accel. value higher than preset threshold
*  0 XLIE: Enable interrupt generation on X low event.
*          0: disable interrupt request
*          1: enable interrupt request on measured accel. value lower than preset threshold
*******************************************************************************/
//}}}
#define LIS302DL_FF_WU_CFG1_REG_ADDR       0x30
//{{{  FF_WU_SRC_1
/*******************************************************************************
*  FF_WU_SRC_1 Register: Interrupt 1 source register.
*  Reading at this address clears FF_WU_SRC_1 register and the FF, WU 1 interrupt
*  and allow the refreshment of data in the FF_WU_SRC_1 register if the latched option
*  was chosen.
*  Read only register
*  Default value: 0x00
*  7 Reserved
*  6 IA: Interrupt active.
*        0: no interrupt has been generated
*        1: one or more interrupts have been generated
*  5 ZH: Z high.
*        0: no interrupt
*        1: ZH event has occurred
*  4 ZL: Z low.
*        0: no interrupt
*        1: ZL event has occurred
*  3 YH: Y high.
*        0: no interrupt
*        1: YH event has occurred
*  2 YL: Y low.
*        0: no interrupt
*        1: YL event has occurred
*  1 YH: X high.
*        0: no interrupt
*        1: XH event has occurred
*  0 YL: X low.
*        0: no interrupt
*        1: XL event has occurred
*******************************************************************************/
//}}}
#define LIS302DL_FF_WU_SRC1_REG_ADDR       0x31
//{{{  FF_WU_THS_1
/*******************************************************************************
*  FF_WU_THS_1 Register: Threshold register
*  Read Write register
*  Default value: 0x00
*  7 DCRM: Reset mode selection.
*          0 - counter resetted
*          1 - counter decremented
*  6 THS6-THS0: Free-fall/wake-up threshold value.
*******************************************************************************/
//}}}
#define LIS302DL_FF_WU_THS1_REG_ADDR       0x32
//{{{  FF_WU_DURATION_1
/*******************************************************************************
*  FF_WU_DURATION_1 Register: duration Register
*  Read Write register
*  Default value: 0x00
*  7:0 D7-D0 Duration value. (Duration steps and maximum values depend on the ODR chosen)
 ******************************************************************************/
//}}}
#define LIS302DL_FF_WU_DURATION1_REG_ADDR  0x33
//{{{  FF_WW_CFG_2
/*******************************************************************************
*  FF_WW_CFG_2 Register: Configuration register for Interrupt 2 source.
*  Read write register
*  Default value: 0x00
*  7 AOI: AND/OR combination of Interrupt events.
*         0: OR combination of interrupt events
*         1: AND combination of interrupt events
*  6 LIR: Latch/not latch interrupt request
*         0: interrupt request not latched
*         1: interrupt request latched
*  5 ZHIE: Enable interrupt generation on Z high event.
*          0: disable interrupt request
*          1: enable interrupt request on measured accel. value higher than preset threshold
*  4 ZLIE: Enable interrupt generation on Z low event.
*          0: disable interrupt request
*          1: enable interrupt request on measured accel. value lower than preset threshold
*  3 YHIE: Enable interrupt generation on Y high event.
*          0: disable interrupt request
*          1: enable interrupt request on measured accel. value higher than preset threshold
*  2 YLIE: Enable interrupt generation on Y low event.
*          0: disable interrupt request
*          1: enable interrupt request on measured accel. value lower than preset threshold
*  1 XHIE: Enable interrupt generation on X high event.
*          0: disable interrupt request
*          1: enable interrupt request on measured accel. value higher than preset threshold
*  0 XLIE: Enable interrupt generation on X low event.
*          0: disable interrupt request
*          1: enable interrupt request on measured accel. value lower than preset threshold
*******************************************************************************/
//}}}
#define LIS302DL_FF_WU_CFG2_REG_ADDR       0x34
//{{{  FF_WU_SRC_2
/*******************************************************************************
*  FF_WU_SRC_2 Register: Interrupt 2 source register.
*  Reading at this address clears FF_WU_SRC_2 register and the FF, WU 2 interrupt
*  and allow the refreshment of data in the FF_WU_SRC_2 register if the latched option
*  was chosen.
*  Read only register
*  Default value: 0x00
*  7 Reserved
*  6 IA: Interrupt active.
*        0: no interrupt has been generated
*        1: one or more interrupts have been generated
*  5 ZH: Z high.
*        0: no interrupt
*        1: ZH event has occurred
*  4 ZL: Z low.
*        0: no interrupt
*        1: ZL event has occurred
*  3 YH: Y high.
*        0: no interrupt
*        1: YH event has occurred
*  2 YL: Y low.
*        0: no interrupt
*        1: YL event has occurred
*  1 YH: X high.
*        0: no interrupt
*        1: XH event has occurred
*  0 YL: X low.
*        0: no interrupt
*        1: XL event has occurred
*******************************************************************************/
//}}}
#define LIS302DL_FF_WU_SRC2_REG_ADDR       0x35
//{{{  FF_WU_THS_2
/*******************************************************************************
*  FF_WU_THS_2 Register: Threshold register
*  Read Write register
*  Default value: 0x00
*  7 DCRM: Reset mode selection.
*          0 - counter resetted
*          1 - counter decremented
*  6 THS6-THS0: Free-fall/wake-up threshold value.
*******************************************************************************/
//}}}
#define LIS302DL_FF_WU_THS2_REG_ADDR       0x36
//{{{  FF_WU_DURATION_2
/*******************************************************************************
*  FF_WU_DURATION_2 Register: duration Register
*  Read Write register
*  Default value: 0x00
*  7:0 D7-D0 Duration value. (Duration steps and maximum values depend on the ODR chosen)
 ******************************************************************************/
//}}}
#define LIS302DL_FF_WU_DURATION2_REG_ADDR  0x37
//{{{  FF_WU_DURATION_2
/******************************************************************************
*  FF_WU_DURATION_2 Register: click Register
*  Read Write register
*  Default value: 0x00
*  7 Reserved
*  6 LIR: Latch Interrupt request.
*         0: interrupt request not latched
*         1: interrupt request latched
*  5 Double_Z: Enable interrupt generation on double click event on Z axis.
*              0: disable interrupt request
*              1: enable interrupt request
*  4 Single_Z: Enable interrupt generation on single click event on Z axis.
*              0: disable interrupt request
*              1: enable interrupt request
*  3 Double_Y: Enable interrupt generation on double click event on Y axis.
*              0: disable interrupt request
*              1: enable interrupt request
*  2 Single_Y: Enable interrupt generation on single click event on Y axis.
*              0: disable interrupt request
*              1: enable interrupt request
*  1 Double_X: Enable interrupt generation on double click event on X axis.
*              0: disable interrupt request
*              1: enable interrupt request
*  0 Single_y: Enable interrupt generation on single click event on X axis.
*              0: disable interrupt request
*              1: enable interrupt request
 ******************************************************************************/
//}}}
#define LIS302DL_CLICK_CFG_REG_ADDR        0x38
//{{{  CLICK_SRC
/******************************************************************************
*  CLICK_SRC Register: click status Register
*  Read only register
*  Default value: 0x00
*  7 Reserved
*  6 IA: Interrupt active.
*        0: no interrupt has been generated
*        1: one or more interrupts have been generated
*  5 Double_Z: Double click on Z axis event.
*        0: no interrupt
*        1: Double Z event has occurred
*  4 Single_Z: Z low.
*        0: no interrupt
*        1: Single Z event has occurred
*  3 Double_Y: Y high.
*        0: no interrupt
*        1: Double Y event has occurred
*  2 Single_Y: Y low.
*        0: no interrupt
*        1: Single Y event has occurred
*  1 Double_X: X high.
*        0: no interrupt
*        1: Double X event has occurred
*  0 Single_X: X low.
*        0: no interrupt
*        1: Single X event has occurred
*******************************************************************************/
//}}}
#define LIS302DL_CLICK_SRC_REG_ADDR        0x39
//{{{  CLICK_THSY_X
/*******************************************************************************
*  CLICK_THSY_X Register: Click threshold Y and X register
*  Read Write register
*  Default value: 0x00
*  7:4 THSy3-THSy0: Click threshold on Y axis, step 0.5g
*  3:0 THSx3-THSx0: Click threshold on X axis, step 0.5g
*******************************************************************************/
//}}}
#define LIS302DL_CLICK_THSY_X_REG_ADDR     0x3B
//{{{  CLICK_THSZ
/*******************************************************************************
*  CLICK_THSZ Register: Click threshold Z register
*  Read Write register
*  Default value: 0x00
*  7:4 Reserved
*  3:0 THSz3-THSz0: Click threshold on Z axis, step 0.5g
*******************************************************************************/
//}}}
#define LIS302DL_CLICK_THSZ_REG_ADDR       0x3C
//{{{  CLICK_TimeLimit
/*******************************************************************************
*  CLICK_TimeLimit Register: Time Limit register
*  Read Write register
*  Default value: 0x00
*  7:0 Dur7-Dur0: Time Limit value, step 0.5g
*******************************************************************************/
//}}}
#define LIS302DL_CLICK_TIMELIMIT_REG_ADDR  0x3D
//{{{  CLICK_Latency
/*******************************************************************************
*  CLICK_Latency Register: Latency register
*  Read Write register
*  Default value: 0x00
*  7:0 Lat7-Lat0: Latency value, step 1msec
*******************************************************************************/
//}}}
#define LIS302DL_CLICK_LATENCY_REG_ADDR    0x3E
//{{{  CLICK_Window
/*******************************************************************************
*  CLICK_Window Register: Window register
*  Read Write register
*  Default value: 0x00
*  7:0 Win7-Win0: Window value, step 1msec
*******************************************************************************/
//}}}
#define LIS302DL_CLICK_WINDOW_REG_ADDR     0x3F

#define I_AM_LIS302DL                      0x3B

#define LIS302DL_SENSITIVITY_2_3G          18  /* 18 mg/digit*/
#define LIS302DL_SENSITIVITY_9_2G          72  /* 72 mg/digit*/

#define LIS302DL_DATARATE_100              ((uint8_t)0x00)
#define LIS302DL_DATARATE_400              ((uint8_t)0x80)

#define LIS302DL_LOWPOWERMODE_POWERDOWN    ((uint8_t)0x00)
#define LIS302DL_LOWPOWERMODE_ACTIVE       ((uint8_t)0x40)

#define LIS302DL_FULLSCALE_2_3             ((uint8_t)0x00)
#define LIS302DL_FULLSCALE_9_2             ((uint8_t)0x20)

#define LIS302DL_SELFTEST_NORMAL           ((uint8_t)0x00)
#define LIS302DL_SELFTEST_P                ((uint8_t)0x10)
#define LIS302DL_SELFTEST_M                ((uint8_t)0x08)

#define LIS302DL_X_ENABLE                  ((uint8_t)0x01)
#define LIS302DL_Y_ENABLE                  ((uint8_t)0x02)
#define LIS302DL_Z_ENABLE                  ((uint8_t)0x04)
#define LIS302DL_XYZ_ENABLE                ((uint8_t)0x07)

#define LIS302DL_SERIALINTERFACE_4WIRE     ((uint8_t)0x00)
#define LIS302DL_SERIALINTERFACE_3WIRE     ((uint8_t)0x80)

#define LIS302DL_BOOT_NORMALMODE           ((uint8_t)0x00)
#define LIS302DL_BOOT_REBOOTMEMORY         ((uint8_t)0x40)

#define LIS302DL_FILTEREDDATASELECTION_BYPASSED        ((uint8_t)0x00)
#define LIS302DL_FILTEREDDATASELECTION_OUTPUTREGISTER  ((uint8_t)0x20)

#define LIS302DL_HIGHPASSFILTERINTERRUPT_OFF       ((uint8_t)0x00)
#define LIS302DL_HIGHPASSFILTERINTERRUPT_1         ((uint8_t)0x04)
#define LIS302DL_HIGHPASSFILTERINTERRUPT_2         ((uint8_t)0x08)
#define LIS302DL_HIGHPASSFILTERINTERRUPT_1_2       ((uint8_t)0x0C)

#define LIS302DL_HIGHPASSFILTER_LEVEL_0            ((uint8_t)0x00)
#define LIS302DL_HIGHPASSFILTER_LEVEL_1            ((uint8_t)0x01)
#define LIS302DL_HIGHPASSFILTER_LEVEL_2            ((uint8_t)0x02)
#define LIS302DL_HIGHPASSFILTER_LEVEL_3            ((uint8_t)0x03)

#define LIS302DL_INTERRUPTREQUEST_NOTLATCHED       ((uint8_t)0x00)
#define LIS302DL_INTERRUPTREQUEST_LATCHED          ((uint8_t)0x40)

#define LIS302DL_CLICKINTERRUPT_XYZ_DISABLE        ((uint8_t)0x00)
#define LIS302DL_CLICKINTERRUPT_X_ENABLE           ((uint8_t)0x01)
#define LIS302DL_CLICKINTERRUPT_Y_ENABLE           ((uint8_t)0x04)
#define LIS302DL_CLICKINTERRUPT_Z_ENABLE           ((uint8_t)0x10)
#define LIS302DL_CLICKINTERRUPT_XYZ_ENABLE         ((uint8_t)0x15)

#define LIS302DL_DOUBLECLICKINTERRUPT_XYZ_DISABLE  ((uint8_t)0x00)
#define LIS302DL_DOUBLECLICKINTERRUPT_X_ENABLE     ((uint8_t)0x02)
#define LIS302DL_DOUBLECLICKINTERRUPT_Y_ENABLE     ((uint8_t)0x08)
#define LIS302DL_DOUBLECLICKINTERRUPT_Z_ENABLE     ((uint8_t)0x20)
#define LIS302DL_DOUBLECLICKINTERRUPT_XYZ_ENABLE   ((uint8_t)0x2A)
//}}}
//{{{  LIS302DL_InitTypeDef
typedef struct {
  uint8_t Power_Mode;                         /* Power-down/Active Mode */
  uint8_t Output_DataRate;                    /* OUT data rate 100 Hz / 400 Hz */
  uint8_t Axes_Enable;                        /* Axes enable */
  uint8_t Full_Scale;                         /* Full scale */
  uint8_t Self_Test;                          /* Self test */
  } LIS302DL_InitTypeDef;
//}}}
//{{{  ACCELERO_InitTypeDef
typedef struct {
  uint8_t Power_Mode;                         /* Power-down/Normal Mode */
  uint8_t AccOutput_DataRate;                 /* OUT data rate */
  uint8_t Axes_Enable;                        /* Axes enable */
  uint8_t High_Resolution;                    /* High Resolution enabling/disabling */
  uint8_t BlockData_Update;                   /* Block Data Update */
  uint8_t Endianness;                         /* Endian Data selection */
  uint8_t AccFull_Scale;                      /* Full Scale selection */
  uint8_t Communication_Mode;
  } ACCELERO_InitTypeDef;
//}}}

//{{{  LIS302DL_FilterConfigTypeDef
/* High Pass Filter struct */
typedef struct {
  uint8_t HighPassFilter_Data_Selection;      /* Internal filter bypassed or data from internal filter send to output register*/
  uint8_t HighPassFilter_CutOff_Frequency;    /* High pass filter cut-off frequency */
  uint8_t HighPassFilter_Interrupt;           /* High pass filter enabled for Freefall/WakeUp #1 or #2 */
  } LIS302DL_FilterConfigTypeDef;
//}}}
//{{{  ACCELERO_FilterConfigTypeDef
/* ACCELERO High Pass Filter struct */
typedef struct {
  uint8_t HighPassFilter_Mode_Selection;      /* Internal filter mode */
  uint8_t HighPassFilter_CutOff_Frequency;    /* High pass filter cut-off frequency */
  uint8_t HighPassFilter_AOI1;                /* HPF_enabling/disabling for AOI function on interrupt 1 */
  uint8_t HighPassFilter_AOI2;                /* HPF_enabling/disabling for AOI function on interrupt 2 */
  uint8_t HighPassFilter_Data_Sel;
  uint8_t HighPassFilter_Stat;
  } ACCELERO_FilterConfigTypeDef;
//}}}

//{{{  LIS302DL_InterruptConfigTypeDef
/* Interrupt struct */
typedef struct {
  uint8_t Latch_Request;                      /* Latch interrupt request into CLICK_SRC register*/
  uint8_t SingleClick_Axes;                   /* Single Click Axes Interrupts */
  uint8_t DoubleClick_Axes;                   /* Double Click Axes Interrupts */
  } LIS302DL_InterruptConfigTypeDef;
//}}}

//{{{
void Init (uint16_t InitStruct) {

  uint8_t ctrl = 0x00;

  // Configure the low level interface */
  ACCELERO_IO_Init();

  ctrl = (uint8_t)InitStruct;

  // Write value to MEMS CTRL_REG1 register
  ACCELERO_IO_Write (&ctrl, LIS302DL_CTRL_REG1_ADDR, 1);
  }
//}}}
//{{{
void FilterConfig (uint8_t FilterStruct) {

  uint8_t ctrl = 0x00;

  // Read CTRL_REG2 register
  ACCELERO_IO_Read (&ctrl, LIS302DL_CTRL_REG2_ADDR, 1);

  // Clear high pass filter cut-off level, interrupt and data selection bits
  ctrl &= (uint8_t)~(LIS302DL_FILTEREDDATASELECTION_OUTPUTREGISTER |
                     LIS302DL_HIGHPASSFILTER_LEVEL_3 |
                     LIS302DL_HIGHPASSFILTERINTERRUPT_1_2);
  ctrl |= FilterStruct;

  // Write value to MEMS CTRL_REG2 register
  ACCELERO_IO_Write (&ctrl, LIS302DL_CTRL_REG2_ADDR, 1);
  }
//}}}
//{{{
void InterruptConfig (LIS302DL_InterruptConfigTypeDef *LIS302DL_IntConfigStruct) {

  // Read CLICK_CFG register */
  uint8_t ctrl = 0x00;
  ACCELERO_IO_Read(&ctrl, LIS302DL_CLICK_CFG_REG_ADDR, 1);

  // Configure latch Interrupt request, click interrupts and double click interrupts
  ctrl = (uint8_t)(LIS302DL_IntConfigStruct->Latch_Request|
                   LIS302DL_IntConfigStruct->SingleClick_Axes |
                   LIS302DL_IntConfigStruct->DoubleClick_Axes);

  // Write value to MEMS CLICK_CFG register
  ACCELERO_IO_Write(&ctrl, LIS302DL_CLICK_CFG_REG_ADDR, 1);
  }
//}}}
//{{{
void LowpowerCmd (uint8_t LowPowerMode) {

  // Read CTRL_REG1 register
  uint8_t tmpreg;
  ACCELERO_IO_Read (&tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);

  // Set new low power mode configuration
  tmpreg &= (uint8_t)~LIS302DL_LOWPOWERMODE_ACTIVE;
  tmpreg |= LowPowerMode;

  // Write value to MEMS CTRL_REG1 register
  ACCELERO_IO_Write (&tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);
  }
//}}}
//{{{
void DataRateCmd (uint8_t DataRateValue) {

  // Read CTRL_REG1 register
  uint8_t tmpreg;
  ACCELERO_IO_Read (&tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);

  // Set new Data rate configuration
  tmpreg &= (uint8_t)~LIS302DL_DATARATE_400;
  tmpreg |= DataRateValue;

  // Write value to MEMS CTRL_REG1 register
  ACCELERO_IO_Write (&tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);
  }
//}}}
//{{{
void FullScaleCmd (uint8_t FS_value) {

  // Read CTRL_REG1 register
  uint8_t tmpreg;
  ACCELERO_IO_Read (&tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);

  // Set new full scale configuration
  tmpreg &= (uint8_t)~LIS302DL_FULLSCALE_9_2;
  tmpreg |= FS_value;

  // Write value to MEMS CTRL_REG1 register
  ACCELERO_IO_Write (&tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);
  }
//}}}

//{{{
uint8_t BSP_ACCELERO_Init() {

  BSP_ACCELERO_ReadID();

  // Set configuration of LIS302DL MEMS Accelerometer
  LIS302DL_InitTypeDef lis302dl_initstruct;
  lis302dl_initstruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
  lis302dl_initstruct.Output_DataRate = LIS302DL_DATARATE_100;
  lis302dl_initstruct.Axes_Enable = LIS302DL_XYZ_ENABLE;
  lis302dl_initstruct.Full_Scale = LIS302DL_FULLSCALE_2_3;
  lis302dl_initstruct.Self_Test = LIS302DL_SELFTEST_NORMAL;
  Init (lis302dl_initstruct.Output_DataRate |
        lis302dl_initstruct.Power_Mode |
        lis302dl_initstruct.Full_Scale |
        lis302dl_initstruct.Self_Test |
        lis302dl_initstruct.Axes_Enable);

  // MEMS High Pass Filter configuration
  LIS302DL_FilterConfigTypeDef lis302dl_filter;
  lis302dl_filter.HighPassFilter_Data_Selection = LIS302DL_FILTEREDDATASELECTION_OUTPUTREGISTER;
  lis302dl_filter.HighPassFilter_CutOff_Frequency = LIS302DL_HIGHPASSFILTER_LEVEL_1;
  lis302dl_filter.HighPassFilter_Interrupt = LIS302DL_HIGHPASSFILTERINTERRUPT_1_2;
  FilterConfig (lis302dl_filter.HighPassFilter_Data_Selection |
                lis302dl_filter.HighPassFilter_CutOff_Frequency |
                lis302dl_filter.HighPassFilter_Interrupt);

  return ACCELERO_OK;
  }
//}}}
//{{{
uint8_t BSP_ACCELERO_ReadID() {

  // Configure the low level interface
  ACCELERO_IO_Init();

  // Read WHO_AM_I register
  uint8_t tmp = 0;
  ACCELERO_IO_Read (&tmp, LIS302DL_WHO_AM_I_ADDR, 1);

  // Return the ID
  return tmp;
  }
//}}}
//{{{
void BSP_ACCELERO_Reset() {

  // Read CTRL_REG2 register
  uint8_t tmpreg;
  ACCELERO_IO_Read (&tmpreg, LIS302DL_CTRL_REG2_ADDR, 1);

  // Enable or Disable the reboot memory
  tmpreg |= LIS302DL_BOOT_REBOOTMEMORY;

  // Write value to MEMS CTRL_REG2 register
  ACCELERO_IO_Write (&tmpreg, LIS302DL_CTRL_REG2_ADDR, 1);
  }
//}}}
//{{{
void BSP_ACCELERO_Click_ITConfig() {

  ACCELERO_IO_ITConfig();

  // Set configuration of Internal High Pass Filter of LIS302DL
  LIS302DL_InterruptConfigTypeDef LIS302DL_InterruptStruct;
  LIS302DL_InterruptStruct.Latch_Request = LIS302DL_INTERRUPTREQUEST_LATCHED;
  LIS302DL_InterruptStruct.SingleClick_Axes = LIS302DL_CLICKINTERRUPT_Z_ENABLE;
  LIS302DL_InterruptStruct.DoubleClick_Axes = LIS302DL_DOUBLECLICKINTERRUPT_Z_ENABLE;
  InterruptConfig (&LIS302DL_InterruptStruct);

  // Configure Interrupt control register: enable Click interrupt on INT1 and INT2 on Z axis high event
  uint8_t ctrl = 0x3F;
  ACCELERO_IO_Write (&ctrl, LIS302DL_CTRL_REG3_ADDR, 1);

  // Enable Interrupt generation on click on Z axis
  ctrl = 0x50;
  ACCELERO_IO_Write (&ctrl, LIS302DL_CLICK_CFG_REG_ADDR, 1);

  // Configure Click Threshold on X/Y axis (10 x 0.5g)
  ctrl = 0xAA;
  ACCELERO_IO_Write (&ctrl, LIS302DL_CLICK_THSY_X_REG_ADDR, 1);

  // Configure Click Threshold on Z axis (10 x 0.5g)
  ctrl = 0x0A;
  ACCELERO_IO_Write (&ctrl, LIS302DL_CLICK_THSZ_REG_ADDR, 1);

  // Enable interrupt on Y axis high event
  ctrl = 0x4C;
  ACCELERO_IO_Write (&ctrl, LIS302DL_FF_WU_CFG1_REG_ADDR, 1);

  // Configure Time Limit
  ctrl = 0x03;
  ACCELERO_IO_Write (&ctrl, LIS302DL_CLICK_TIMELIMIT_REG_ADDR, 1);

  // Configure Latency
  ctrl = 0x7F;
  ACCELERO_IO_Write (&ctrl, LIS302DL_CLICK_LATENCY_REG_ADDR, 1);

  // Configure Click Window
  ctrl = 0x7F;
  ACCELERO_IO_Write (&ctrl, LIS302DL_CLICK_WINDOW_REG_ADDR, 1);
  }
//}}}
//{{{
void BSP_ACCELERO_Click_ITClear() {

  // Read click and status registers if the available MEMS Accelerometer is LIS302DL
  uint8_t clickreg = 0;
  ACCELERO_IO_Read (&clickreg, LIS302DL_CLICK_SRC_REG_ADDR, 1);

  uint8_t buffer[6];
  ACCELERO_IO_Read (buffer, LIS302DL_STATUS_REG_ADDR, 6);
  }
//}}}
//{{{
void BSP_ACCELERO_GetXYZ (int16_t* pDataXYZ) {

  //ACCELERO_IO_Read (&crtl, LIS302DL_CTRL_REG1_ADDR, 1);

  int8_t buffer[5];
  ACCELERO_IO_Read ((uint8_t*)buffer, LIS302DL_OUT_X_ADDR, 5);
  pDataXYZ[0] = buffer[0];
  pDataXYZ[1] = buffer[2];
  pDataXYZ[2] = buffer[4];
  }
//}}}
