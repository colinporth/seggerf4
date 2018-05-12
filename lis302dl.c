#include "lis302dl.h"

//{{{
void LIS302DL_Init (uint16_t InitStruct)
{
  uint8_t ctrl = 0x00;

  /* Configure the low level interface */
  ACCELERO_IO_Init();

  ctrl = (uint8_t) InitStruct;

  /* Write value to MEMS CTRL_REG1 register */
  ACCELERO_IO_Write(&ctrl, LIS302DL_CTRL_REG1_ADDR, 1);
}
//}}}
//{{{
void LIS302DL_DeInit()
{

}
//}}}

//{{{
uint8_t LIS302DL_ReadID()
{
  uint8_t tmp = 0;

  /* Configure the low level interface */
  ACCELERO_IO_Init();

  /* Read WHO_AM_I register */
  ACCELERO_IO_Read(&tmp, LIS302DL_WHO_AM_I_ADDR, 1);

  /* Return the ID */
  return (uint16_t)tmp;
}
//}}}

//{{{
void LIS302DL_FilterConfig (uint8_t FilterStruct)
{
  uint8_t ctrl = 0x00;

  /* Read CTRL_REG2 register */
  ACCELERO_IO_Read(&ctrl, LIS302DL_CTRL_REG2_ADDR, 1);

  /* Clear high pass filter cut-off level, interrupt and data selection bits */
  ctrl &= (uint8_t)~(LIS302DL_FILTEREDDATASELECTION_OUTPUTREGISTER | \
                     LIS302DL_HIGHPASSFILTER_LEVEL_3 | \
                     LIS302DL_HIGHPASSFILTERINTERRUPT_1_2);

  ctrl |= FilterStruct;

  /* Write value to MEMS CTRL_REG2 register */
  ACCELERO_IO_Write(&ctrl, LIS302DL_CTRL_REG2_ADDR, 1);
}
//}}}

//{{{
void LIS302DL_InterruptConfig (LIS302DL_InterruptConfigTypeDef *LIS302DL_IntConfigStruct)
{
  uint8_t ctrl = 0x00;

  /* Read CLICK_CFG register */
  ACCELERO_IO_Read(&ctrl, LIS302DL_CLICK_CFG_REG_ADDR, 1);

  /* Configure latch Interrupt request, click interrupts and double click interrupts */
  ctrl = (uint8_t)(LIS302DL_IntConfigStruct->Latch_Request| \
                   LIS302DL_IntConfigStruct->SingleClick_Axes | \
                   LIS302DL_IntConfigStruct->DoubleClick_Axes);

  /* Write value to MEMS CLICK_CFG register */
  ACCELERO_IO_Write(&ctrl, LIS302DL_CLICK_CFG_REG_ADDR, 1);
}
//}}}
//{{{
void LIS302DL_Click_IntConfig()
{
  uint8_t ctrl = 0x00;
  LIS302DL_InterruptConfigTypeDef   LIS302DL_InterruptStruct;

  ACCELERO_IO_ITConfig();

  /* Set configuration of Internal High Pass Filter of LIS302DL */
  LIS302DL_InterruptStruct.Latch_Request = LIS302DL_INTERRUPTREQUEST_LATCHED;
  LIS302DL_InterruptStruct.SingleClick_Axes = LIS302DL_CLICKINTERRUPT_Z_ENABLE;
  LIS302DL_InterruptStruct.DoubleClick_Axes = LIS302DL_DOUBLECLICKINTERRUPT_Z_ENABLE;
  LIS302DL_InterruptConfig(&LIS302DL_InterruptStruct);

  /* Configure Interrupt control register: enable Click interrupt on INT1 and
  INT2 on Z axis high event */
  ctrl = 0x3F;
  ACCELERO_IO_Write(&ctrl, LIS302DL_CTRL_REG3_ADDR, 1);

  /* Enable Interrupt generation on click on Z axis */
  ctrl = 0x50;
  ACCELERO_IO_Write(&ctrl, LIS302DL_CLICK_CFG_REG_ADDR, 1);

  /* Configure Click Threshold on X/Y axis (10 x 0.5g) */
  ctrl = 0xAA;
  ACCELERO_IO_Write(&ctrl, LIS302DL_CLICK_THSY_X_REG_ADDR, 1);

  /* Configure Click Threshold on Z axis (10 x 0.5g) */
  ctrl = 0x0A;
  ACCELERO_IO_Write(&ctrl, LIS302DL_CLICK_THSZ_REG_ADDR, 1);

  /* Enable interrupt on Y axis high event */
  ctrl = 0x4C;
  ACCELERO_IO_Write(&ctrl, LIS302DL_FF_WU_CFG1_REG_ADDR, 1);

  /* Configure Time Limit */
  ctrl = 0x03;
  ACCELERO_IO_Write(&ctrl, LIS302DL_CLICK_TIMELIMIT_REG_ADDR, 1);

  /* Configure Latency */
  ctrl = 0x7F;
  ACCELERO_IO_Write(&ctrl, LIS302DL_CLICK_LATENCY_REG_ADDR, 1);

  /* Configure Click Window */
  ctrl = 0x7F;
  ACCELERO_IO_Write(&ctrl, LIS302DL_CLICK_WINDOW_REG_ADDR, 1);
}
//}}}

//{{{
void LIS302DL_Click_IntClear()
{
  uint8_t buffer[6], clickreg = 0;

  /* Read click and status registers if the available MEMS Accelerometer is LIS302DL */
  ACCELERO_IO_Read(&clickreg, LIS302DL_CLICK_SRC_REG_ADDR, 1);
  ACCELERO_IO_Read(buffer, LIS302DL_STATUS_REG_ADDR, 6);
}
//}}}

//{{{
void LIS302DL_LowpowerCmd (uint8_t LowPowerMode)
{
  uint8_t tmpreg;

  /* Read CTRL_REG1 register */
  ACCELERO_IO_Read(&tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);

  /* Set new low power mode configuration */
  tmpreg &= (uint8_t)~LIS302DL_LOWPOWERMODE_ACTIVE;
  tmpreg |= LowPowerMode;

  /* Write value to MEMS CTRL_REG1 register */
  ACCELERO_IO_Write(&tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);
}
//}}}
//{{{
void LIS302DL_DataRateCmd (uint8_t DataRateValue)
{
  uint8_t tmpreg;

  /* Read CTRL_REG1 register */
  ACCELERO_IO_Read(&tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);

  /* Set new Data rate configuration */
  tmpreg &= (uint8_t)~LIS302DL_DATARATE_400;
  tmpreg |= DataRateValue;

  /* Write value to MEMS CTRL_REG1 register */
  ACCELERO_IO_Write(&tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);
}
//}}}
//{{{
void LIS302DL_FullScaleCmd (uint8_t FS_value)
{
  uint8_t tmpreg;

  /* Read CTRL_REG1 register */
  ACCELERO_IO_Read(&tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);

  /* Set new full scale configuration */
  tmpreg &= (uint8_t)~LIS302DL_FULLSCALE_9_2;
  tmpreg |= FS_value;

  /* Write value to MEMS CTRL_REG1 register */
  ACCELERO_IO_Write(&tmpreg, LIS302DL_CTRL_REG1_ADDR, 1);
}
//}}}
//{{{
void LIS302DL_RebootCmd()
{
  uint8_t tmpreg;
  /* Read CTRL_REG2 register */
  ACCELERO_IO_Read(&tmpreg, LIS302DL_CTRL_REG2_ADDR, 1);

  /* Enable or Disable the reboot memory */
  tmpreg |= LIS302DL_BOOT_REBOOTMEMORY;

  /* Write value to MEMS CTRL_REG2 register */
  ACCELERO_IO_Write(&tmpreg, LIS302DL_CTRL_REG2_ADDR, 1);
}
//}}}

//{{{
void LIS302DL_ReadACC (int16_t *pData)
{
  int8_t buffer[6];
  int16_t pnRawData[3];
  uint8_t sensitivity = LIS302DL_SENSITIVITY_2_3G;
  uint8_t crtl, i = 0x00;

  ACCELERO_IO_Read(&crtl, LIS302DL_CTRL_REG1_ADDR, 1);
  ACCELERO_IO_Read((uint8_t*)buffer, LIS302DL_OUT_X_ADDR, 6);

  for(i=0; i<3; i++)
  {
    pnRawData[i] = buffer[2*i];
  }

  switch(crtl & LIS302DL_FULLSCALE_9_2)
  {
    /* FS bit = 0 ==> Sensitivity typical value = 18milligals/digit*/
  case LIS302DL_FULLSCALE_2_3:
    sensitivity = LIS302DL_SENSITIVITY_2_3G;
    break;

    /* FS bit = 1 ==> Sensitivity typical value = 72milligals/digit*/
  case LIS302DL_FULLSCALE_9_2:
    sensitivity = LIS302DL_SENSITIVITY_9_2G;
    break;

  default:
    break;
  }

  /* Obtain the mg value for the three axis */
  for(i=0; i<3; i++)
  {
    pData[i]=(pnRawData[i] * sensitivity);
  }
}
//}}}

ACCELERO_DrvTypeDef Lis302dlDrv = {
  LIS302DL_Init,
  LIS302DL_DeInit,
  LIS302DL_ReadID,
  LIS302DL_RebootCmd,
  LIS302DL_Click_IntConfig,
  0,
  0,
  0,
  0,
  LIS302DL_Click_IntClear,
  LIS302DL_FilterConfig,
  0,
  LIS302DL_ReadACC,
  };
