#include "stm32f429i_discovery.h"
//{{{  i2c defines
#define IO_I2C_ADDRESS                         0x82
#define TS_I2C_ADDRESS                         0x82

#define DISCOVERY_I2Cx                          I2C3
#define DISCOVERY_I2Cx_CLOCK_ENABLE()           __I2C3_CLK_ENABLE()
#define DISCOVERY_I2Cx_FORCE_RESET()            __I2C3_FORCE_RESET()
#define DISCOVERY_I2Cx_RELEASE_RESET()          __I2C3_RELEASE_RESET()
#define DISCOVERY_I2Cx_SDA_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOC_CLK_ENABLE()
#define DISCOVERY_I2Cx_SCL_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()
#define DISCOVERY_I2Cx_SDA_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOC_CLK_DISABLE()

/* Definition for DISCO I2Cx Pins */
#define DISCOVERY_I2Cx_SCL_PIN                  GPIO_PIN_8
#define DISCOVERY_I2Cx_SCL_GPIO_PORT            GPIOA
#define DISCOVERY_I2Cx_SCL_SDA_AF               GPIO_AF4_I2C3
#define DISCOVERY_I2Cx_SDA_PIN                  GPIO_PIN_9
#define DISCOVERY_I2Cx_SDA_GPIO_PORT            GPIOC

/* Definition for IOE I2Cx's NVIC */
#define DISCOVERY_I2Cx_EV_IRQn                  I2C3_EV_IRQn
#define DISCOVERY_I2Cx_ER_IRQn                  I2C3_ER_IRQn

#define BSP_I2C_SPEED                           100000

#define I2Cx_TIMEOUT_MAX                        0x3000 
//}}}
//{{{  spi defines
#define DISCOVERY_SPIx                    SPI5
#define DISCOVERY_SPIx_CLK_ENABLE()       __SPI5_CLK_ENABLE()
#define DISCOVERY_SPIx_GPIO_PORT          GPIOF                      /* GPIOF */
#define DISCOVERY_SPIx_AF                 GPIO_AF5_SPI5
#define DISCOVERY_SPIx_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOF_CLK_ENABLE()
#define DISCOVERY_SPIx_GPIO_CLK_DISABLE() __HAL_RCC_GPIOF_CLK_DISABLE()
#define DISCOVERY_SPIx_SCK_PIN            GPIO_PIN_7                 /* PF.07 */
#define DISCOVERY_SPIx_MISO_PIN           GPIO_PIN_8                 /* PF.08 */
#define DISCOVERY_SPIx_MOSI_PIN           GPIO_PIN_9                 /* PF.09 */

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define SPIx_TIMEOUT_MAX           ((uint32_t)0x1000)

/* Read/Write command */
#define READWRITE_CMD              ((uint8_t)0x80)

/* Multiple byte read/write command */
#define MULTIPLEBYTE_CMD           ((uint8_t)0x40)

/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                 ((uint8_t)0x00)
//}}}
//{{{  STMPE811 defines
#define STMPE811_INT_PIN                        GPIO_PIN_15
#define STMPE811_INT_GPIO_PORT                  GPIOA
#define STMPE811_INT_CLK_ENABLE()               __HAL_RCC_GPIOA_CLK_ENABLE()
#define STMPE811_INT_CLK_DISABLE()              __HAL_RCC_GPIOA_CLK_DISABLE()
#define STMPE811_INT_EXTI                       EXTI15_10_IRQn
#define STMPE811_INT_EXTIHandler                EXTI15_10_IRQHandler
//}}}
//{{{  gyro defines
/* Chip Select macro definition */
#define GYRO_CS_LOW()       HAL_GPIO_WritePin(GYRO_CS_GPIO_PORT, GYRO_CS_PIN, GPIO_PIN_RESET)
#define GYRO_CS_HIGH()      HAL_GPIO_WritePin(GYRO_CS_GPIO_PORT, GYRO_CS_PIN, GPIO_PIN_SET)

#define GYRO_CS_PIN                             GPIO_PIN_1                  /* PC.01 */
#define GYRO_CS_GPIO_PORT                       GPIOC                       /* GPIOC */
#define GYRO_CS_GPIO_CLK_ENABLE()               __GPIOC_CLK_ENABLE()
#define GYRO_CS_GPIO_CLK_DISABLE()              __GPIOC_CLK_DISABLE()

#define GYRO_INT_GPIO_CLK_ENABLE()              __GPIOA_CLK_ENABLE()
#define GYRO_INT_GPIO_CLK_DISABLE()             __GPIOA_CLK_DISABLE()
#define GYRO_INT_GPIO_PORT                      GPIOA                       /* GPIOA */
#define GYRO_INT1_PIN                           GPIO_PIN_1                  /* PA.01 */
#define GYRO_INT1_EXTI_IRQn                     EXTI1_IRQn
#define GYRO_INT2_PIN                           GPIO_PIN_2                  /* PA.02 */
#define GYRO_INT2_EXTI_IRQn                     EXTI2_IRQn
//}}}
//{{{  led defines
#define LED3_PIN                                GPIO_PIN_13
#define LED3_GPIO_PORT                          GPIOG
#define LED3_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOG_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOG_CLK_DISABLE()

#define LED4_PIN                                GPIO_PIN_14
#define LED4_GPIO_PORT                          GPIOG
#define LED4_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOG_CLK_ENABLE()
#define LED4_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOG_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)  do{if((__INDEX__) == 0) LED3_GPIO_CLK_ENABLE(); else \
                                            if((__INDEX__) == 1) LED4_GPIO_CLK_ENABLE(); \
                                            }while(0)
#define LEDx_GPIO_CLK_DISABLE(__INDEX__) do{if((__INDEX__) == 0) LED3_GPIO_CLK_DISABLE(); else \
                                            if((__INDEX__) == 1) LED4_GPIO_CLK_DISABLE(); \
                                            }while(0)
//}}}
//{{{  button defines
#define KEY_BUTTON_PIN                         GPIO_PIN_0
#define KEY_BUTTON_GPIO_PORT                   GPIOA
#define KEY_BUTTON_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define KEY_BUTTON_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()
#define KEY_BUTTON_EXTI_IRQn                   EXTI0_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)     do{if((__INDEX__) == 0) KEY_BUTTON_GPIO_CLK_ENABLE(); \
                                                 }while(0)
#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)    do{if((__INDEX__) == 0) KEY_BUTTON_GPIO_CLK_DISABLE(); \
                                                 }while(0)
//}}}

// vars
GPIO_TypeDef* GPIO_PORT[LEDn] = {LED3_GPIO_PORT, LED4_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED3_PIN, LED4_PIN};

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {KEY_BUTTON_GPIO_PORT};
const uint16_t BUTTON_PIN[BUTTONn] = {KEY_BUTTON_PIN};
const uint8_t BUTTON_IRQn[BUTTONn] = {KEY_BUTTON_EXTI_IRQn};

uint32_t I2cxTimeout = I2Cx_TIMEOUT_MAX; /*<! Value of Timeout when I2C communication fails */
uint32_t SpixTimeout = SPIx_TIMEOUT_MAX; /*<! Value of Timeout when SPI communication fails */

I2C_HandleTypeDef I2cHandle;
static SPI_HandleTypeDef SpiHandle;
static uint8_t Is_LCD_IO_Initialized = 0;

//{{{
static void I2Cx_MspInit (I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  if (hi2c->Instance == DISCOVERY_I2Cx)
  {
    /* Configure the GPIOs ---------------------------------------------------*/
    /* Enable GPIO clock */
    DISCOVERY_I2Cx_SDA_GPIO_CLK_ENABLE();
    DISCOVERY_I2Cx_SCL_GPIO_CLK_ENABLE();

    /* Configure I2C Tx as alternate function  */
    GPIO_InitStruct.Pin       = DISCOVERY_I2Cx_SCL_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = DISCOVERY_I2Cx_SCL_SDA_AF;
    HAL_GPIO_Init(DISCOVERY_I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);

    /* Configure I2C Rx as alternate function  */
    GPIO_InitStruct.Pin = DISCOVERY_I2Cx_SDA_PIN;
    HAL_GPIO_Init(DISCOVERY_I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);


    /* Configure the Discovery I2Cx peripheral -------------------------------*/
    /* Enable I2C3 clock */
    DISCOVERY_I2Cx_CLOCK_ENABLE();

    /* Force the I2C Peripheral Clock Reset */
    DISCOVERY_I2Cx_FORCE_RESET();

    /* Release the I2C Peripheral Clock Reset */
    DISCOVERY_I2Cx_RELEASE_RESET();

    /* Enable and set Discovery I2Cx Interrupt to the highest priority */
    HAL_NVIC_SetPriority(DISCOVERY_I2Cx_EV_IRQn, 0x00, 0);
    HAL_NVIC_EnableIRQ(DISCOVERY_I2Cx_EV_IRQn);

    /* Enable and set Discovery I2Cx Interrupt to the highest priority */
    HAL_NVIC_SetPriority(DISCOVERY_I2Cx_ER_IRQn, 0x00, 0);
    HAL_NVIC_EnableIRQ(DISCOVERY_I2Cx_ER_IRQn);
  }
}
//}}}
//{{{
static void I2Cx_Init()
{
  if(HAL_I2C_GetState(&I2cHandle) == HAL_I2C_STATE_RESET)
  {
    I2cHandle.Instance              = DISCOVERY_I2Cx;
    I2cHandle.Init.ClockSpeed       = BSP_I2C_SPEED;
    I2cHandle.Init.DutyCycle        = I2C_DUTYCYCLE_2;
    I2cHandle.Init.OwnAddress1      = 0;
    I2cHandle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    I2cHandle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLED;
    I2cHandle.Init.OwnAddress2      = 0;
    I2cHandle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLED;
    I2cHandle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLED;

    /* Init the I2C */
    I2Cx_MspInit(&I2cHandle);
    HAL_I2C_Init(&I2cHandle);
  }
}
//}}}
//{{{
static void I2Cx_Error()
{
  /* De-initialize the SPI communication BUS */
  HAL_I2C_DeInit(&I2cHandle);

  /* Re-Initialize the SPI communication BUS */
  I2Cx_Init();
}
//}}}
//{{{
static void I2Cx_ITConfig()
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable the GPIO EXTI Clock */
  STMPE811_INT_CLK_ENABLE();

  GPIO_InitStruct.Pin   = STMPE811_INT_PIN;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Mode  = GPIO_MODE_IT_FALLING;
  HAL_GPIO_Init(STMPE811_INT_GPIO_PORT, &GPIO_InitStruct);

  /* Enable and set GPIO EXTI Interrupt to the highest priority */
  HAL_NVIC_SetPriority((IRQn_Type)(STMPE811_INT_EXTI), 0x00, 0x00);
  HAL_NVIC_EnableIRQ((IRQn_Type)(STMPE811_INT_EXTI));
}
//}}}
//{{{
static void I2Cx_WriteData (uint8_t Addr, uint8_t Reg, uint8_t Value)
  {
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write(&I2cHandle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, I2cxTimeout);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initialize the BUS */
    I2Cx_Error();
  }
}
//}}}
//{{{
static void I2Cx_WriteBuffer (uint8_t Addr, uint8_t Reg,  uint8_t *pBuffer, uint16_t Length)
  {
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write(&I2cHandle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Length, I2cxTimeout);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initialize the BUS */
    I2Cx_Error();
  }
}
//}}}
//{{{
static uint8_t I2Cx_ReadData (uint8_t Addr, uint8_t Reg)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0;

  status = HAL_I2C_Mem_Read(&I2cHandle, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2cxTimeout);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initialize the BUS */
    I2Cx_Error();

  }
  return value;
}

//}}}
//{{{
static uint8_t I2Cx_ReadBuffer (uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read(&I2cHandle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Length, I2cxTimeout);

  /* Check the communication status */
  if(status == HAL_OK)
  {
    return 0;
  }
  else
  {
    /* Re-Initialize the BUS */
    I2Cx_Error();

    return 1;
  }
}
//}}}

//{{{
static void SPIx_MspInit (SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable SPIx clock */
  DISCOVERY_SPIx_CLK_ENABLE();

  /* Enable DISCOVERY_SPI GPIO clock */
  DISCOVERY_SPIx_GPIO_CLK_ENABLE();

  /* configure SPI SCK, MOSI and MISO */
  GPIO_InitStructure.Pin    = (DISCOVERY_SPIx_SCK_PIN | DISCOVERY_SPIx_MOSI_PIN | DISCOVERY_SPIx_MISO_PIN);
  GPIO_InitStructure.Mode   = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull   = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed  = GPIO_SPEED_MEDIUM;
  GPIO_InitStructure.Alternate = DISCOVERY_SPIx_AF;
  HAL_GPIO_Init(DISCOVERY_SPIx_GPIO_PORT, &GPIO_InitStructure);
}
//}}}
//{{{
static void SPIx_Init()
{
  if(HAL_SPI_GetState(&SpiHandle) == HAL_SPI_STATE_RESET)
  {
    /* SPI configuration -----------------------------------------------------*/
    SpiHandle.Instance = DISCOVERY_SPIx;
    /* SPI baudrate is set to 5.6 MHz (PCLK2/SPI_BaudRatePrescaler = 90/16 = 5.625 MHz)
       to verify these constraints:
       - ILI9341 LCD SPI interface max baudrate is 10MHz for write and 6.66MHz for read
       - l3gd20 SPI interface max baudrate is 10MHz for write/read
       - PCLK2 frequency is set to 90 MHz
    */
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;

    /* On STM32F429I-Discovery, LCD ID cannot be read then keep a common configuration */
    /* for LCD and GYRO (SPI_DIRECTION_2LINES) */
    /* Note: To read a register a LCD, SPI_DIRECTION_1LINE should be set */
    SpiHandle.Init.Direction      = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase       = SPI_PHASE_1EDGE;
    SpiHandle.Init.CLKPolarity    = SPI_POLARITY_LOW;
    SpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    SpiHandle.Init.CRCPolynomial  = 7;
    SpiHandle.Init.DataSize       = SPI_DATASIZE_8BIT;
    SpiHandle.Init.FirstBit       = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.NSS            = SPI_NSS_SOFT;
    SpiHandle.Init.TIMode         = SPI_TIMODE_DISABLED;
    SpiHandle.Init.Mode           = SPI_MODE_MASTER;

    SPIx_MspInit(&SpiHandle);
    HAL_SPI_Init(&SpiHandle);
  }
}
//}}}
//{{{
static void SPIx_Error()
{
  /* De-initialize the SPI communication BUS */
  HAL_SPI_DeInit(&SpiHandle);

  /* Re- Initialize the SPI communication BUS */
  SPIx_Init();
}
//}}}
//{{{
static uint32_t SPIx_Read (uint8_t ReadSize)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t readvalue;

  status = HAL_SPI_Receive(&SpiHandle, (uint8_t*) &readvalue, ReadSize, SpixTimeout);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initialize the BUS */
    SPIx_Error();
  }

  return readvalue;
}
//}}}
//{{{
static void SPIx_Write (uint16_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_Transmit(&SpiHandle, (uint8_t*) &Value, 1, SpixTimeout);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initialize the BUS */
    SPIx_Error();
  }
}
//}}}
//{{{
static uint8_t SPIx_WriteRead (uint8_t Byte)
{
  uint8_t receivedbyte = 0;

  /* Send a Byte through the SPI peripheral */
  /* Read byte from the SPI bus */
  if(HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*) &Byte, (uint8_t*) &receivedbyte, 1, SpixTimeout) != HAL_OK)
  {
    SPIx_Error();
  }

  return receivedbyte;
}
//}}}

//{{{
void IOE_Init()
{
  I2Cx_Init();
}
//}}}
//{{{
void IOE_ITConfig()
{
  I2Cx_ITConfig();
}
//}}}
//{{{
void IOE_Write (uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  I2Cx_WriteData(Addr, Reg, Value);
}
//}}}
//{{{
uint8_t IOE_Read (uint8_t Addr, uint8_t Reg)
{
  return I2Cx_ReadData(Addr, Reg);
}
//}}}
//{{{
void IOE_WriteMultiple (uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length)
{
  I2Cx_WriteBuffer(Addr, Reg, pBuffer, Length);
}
//}}}
//{{{
uint16_t IOE_ReadMultiple (uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length)
{
 return I2Cx_ReadBuffer(Addr, Reg, pBuffer, Length);
}
//}}}
//{{{
void IOE_Delay (uint32_t Delay)
{
  HAL_Delay(Delay);
}
//}}}

//{{{
void GYRO_IO_Init()
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure the Gyroscope Control pins ------------------------------------*/
  /* Enable CS GPIO clock and Configure GPIO PIN for Gyroscope Chip select */
  GYRO_CS_GPIO_CLK_ENABLE();
  GPIO_InitStructure.Pin = GYRO_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GYRO_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect: Chip Select high */
  GYRO_CS_HIGH();

  /* Enable INT1, INT2 GPIO clock and Configure GPIO PINs to detect Interrupts */
  GYRO_INT_GPIO_CLK_ENABLE();
  GPIO_InitStructure.Pin = GYRO_INT1_PIN | GYRO_INT2_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Pull= GPIO_NOPULL;
  HAL_GPIO_Init(GYRO_INT_GPIO_PORT, &GPIO_InitStructure);

  SPIx_Init();
}
//}}}
//{{{
void GYRO_IO_Write (uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
  /* Configure the MS bit:
       - When 0, the address will remain unchanged in multiple read/write commands.
       - When 1, the address will be auto incremented in multiple read/write commands.
  */
  if(NumByteToWrite > 0x01)
  {
    WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  GYRO_CS_LOW();

  /* Send the Address of the indexed register */
  SPIx_WriteRead(WriteAddr);

  /* Send the data that will be written into the device (MSB First) */
  while(NumByteToWrite >= 0x01)
  {
    SPIx_WriteRead(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }

  /* Set chip select High at the end of the transmission */
  GYRO_CS_HIGH();
}
//}}}
//{{{
void GYRO_IO_Read (uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
  if(NumByteToRead > 0x01)
  {
    ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  }
  else
  {
    ReadAddr |= (uint8_t)READWRITE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  GYRO_CS_LOW();

  /* Send the Address of the indexed register */
  SPIx_WriteRead(ReadAddr);

  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to Gyroscope (Slave device) */
    *pBuffer = SPIx_WriteRead(DUMMY_BYTE);
    NumByteToRead--;
    pBuffer++;
  }

  /* Set chip select High at the end of the transmission */
  GYRO_CS_HIGH();
}
//}}}

//{{{
void BSP_LED_Init (Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIO_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}
//}}}
//{{{
void BSP_LED_On (Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
}
//}}}
//{{{
void BSP_LED_Off (Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}
//}}}
//{{{
void BSP_LED_Toggle (Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}
//}}}

//{{{
void BSP_PB_Init (Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable the BUTTON Clock */
  BUTTONx_GPIO_CLK_ENABLE(Button);

  if (ButtonMode == BUTTON_MODE_GPIO)
  {
    /* Configure Button pin as input */
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);
  }

  if (ButtonMode == BUTTON_MODE_EXTI)
  {
    /* Configure Button pin as input with External interrupt */
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0x00);
    HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  }
}
//}}}
//{{{
uint32_t BSP_PB_GetState (Button_TypeDef Button)
{
  return HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}
//}}}
