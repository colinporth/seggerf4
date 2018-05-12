#include "stm32f4_discovery_accelerometer.h"

static ACCELERO_DrvTypeDef *AcceleroDrv;

//{{{
uint8_t BSP_ACCELERO_Init()
{
  uint8_t ret = ACCELERO_ERROR;
  uint16_t ctrl = 0x0000;

  LIS302DL_InitTypeDef         lis302dl_initstruct;
  LIS302DL_FilterConfigTypeDef lis302dl_filter = {0,0,0};

  if(Lis302dlDrv.ReadID() == I_AM_LIS302DL)
  {
    /* Initialize the accelerometer driver structure */
    AcceleroDrv = &Lis302dlDrv;

    /* Set configuration of LIS302DL MEMS Accelerometer *********************/
    lis302dl_initstruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
    lis302dl_initstruct.Output_DataRate = LIS302DL_DATARATE_100;
    lis302dl_initstruct.Axes_Enable = LIS302DL_XYZ_ENABLE;
    lis302dl_initstruct.Full_Scale = LIS302DL_FULLSCALE_2_3;
    lis302dl_initstruct.Self_Test = LIS302DL_SELFTEST_NORMAL;

    /* Configure MEMS: data rate, power mode, full scale, self test and axes */
    ctrl = (uint16_t) (lis302dl_initstruct.Output_DataRate | lis302dl_initstruct.Power_Mode | \
                       lis302dl_initstruct.Full_Scale | lis302dl_initstruct.Self_Test | \
                       lis302dl_initstruct.Axes_Enable);

    /* Configure the accelerometer main parameters */
    AcceleroDrv->Init(ctrl);

    /* MEMS High Pass Filter configuration */
    lis302dl_filter.HighPassFilter_Data_Selection = LIS302DL_FILTEREDDATASELECTION_OUTPUTREGISTER;
    lis302dl_filter.HighPassFilter_CutOff_Frequency = LIS302DL_HIGHPASSFILTER_LEVEL_1;
    lis302dl_filter.HighPassFilter_Interrupt = LIS302DL_HIGHPASSFILTERINTERRUPT_1_2;

    /* Configure MEMS high pass filter cut-off level, interrupt and data selection bits */
    ctrl = (uint8_t)(lis302dl_filter.HighPassFilter_Data_Selection | \
                     lis302dl_filter.HighPassFilter_CutOff_Frequency | \
                     lis302dl_filter.HighPassFilter_Interrupt);

    /* Configure the accelerometer LPF main parameters */
    AcceleroDrv->FilterConfig(ctrl);

    ret = ACCELERO_OK;
  }
  else
  {
    ret = ACCELERO_ERROR;
  }
  return ret;
}
//}}}

//{{{
uint8_t BSP_ACCELERO_ReadID()
{
  uint8_t id = 0x00;

  if(AcceleroDrv->ReadID != NULL)
  {
    id = AcceleroDrv->ReadID();
  }
  return id;
}
//}}}
//{{{
void BSP_ACCELERO_Reset()
{
  if(AcceleroDrv->Reset != NULL)
  {
    AcceleroDrv->Reset();
  }
}
//}}}

//{{{
void BSP_ACCELERO_Click_ITConfig()
{
  if(AcceleroDrv->ConfigIT != NULL)
  {
    AcceleroDrv->ConfigIT();
  }
}
//}}}
//{{{
void BSP_ACCELERO_Click_ITClear()
{
  if(AcceleroDrv->ClearIT != NULL)
  {
    AcceleroDrv->ClearIT();
  }
}
//}}}

//{{{
void BSP_ACCELERO_GetXYZ (int16_t* pDataXYZ)
{
  int16_t SwitchXY = 0;

  if(AcceleroDrv->GetXYZ != NULL)
  {
    AcceleroDrv->GetXYZ(pDataXYZ);

    /* Switch X and Y Axes in case of LIS302DL MEMS */
    if(AcceleroDrv == &Lis302dlDrv)
    {
      SwitchXY  = pDataXYZ[0];
      pDataXYZ[0] = pDataXYZ[1];
      /* Invert Y Axis to be compliant with LIS3DSH MEMS */
      pDataXYZ[1] = -SwitchXY;
    }
  }
}
//}}}
