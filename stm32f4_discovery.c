#include "stm32f4_discovery.h"

//{{{  Led defines
#define LED4_PIN                         GPIO_PIN_12
#define LED4_GPIO_PORT                   GPIOD
#define LED4_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOD_CLK_ENABLE()
#define LED4_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOD_CLK_DISABLE()

#define LED3_PIN                         GPIO_PIN_13
#define LED3_GPIO_PORT                   GPIOD
#define LED3_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOD_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOD_CLK_DISABLE()

#define LED5_PIN                         GPIO_PIN_14
#define LED5_GPIO_PORT                   GPIOD
#define LED5_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOD_CLK_ENABLE()
#define LED5_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOD_CLK_DISABLE()

#define LED6_PIN                         GPIO_PIN_15
#define LED6_GPIO_PORT                   GPIOD
#define LED6_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOD_CLK_ENABLE()
#define LED6_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOD_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__) do{if((__INDEX__) == 0) LED4_GPIO_CLK_ENABLE(); else \
                                           if((__INDEX__) == 1) LED3_GPIO_CLK_ENABLE(); else \
                                           if((__INDEX__) == 2) LED5_GPIO_CLK_ENABLE(); else \
                                           if((__INDEX__) == 3) LED6_GPIO_CLK_ENABLE(); \
                                           }while(0)

#define LEDx_GPIO_CLK_DISABLE(__INDEX__) do{if((__INDEX__) == 0) LED4_GPIO_CLK_DISABLE(); else \
                                            if((__INDEX__) == 1) LED3_GPIO_CLK_DISABLE(); else \
                                            if((__INDEX__) == 2) LED5_GPIO_CLK_DISABLE(); else \
                                            if((__INDEX__) == 3) LED6_GPIO_CLK_DISABLE(); \
                                            }while(0)
//}}}
//{{{  button defines
#define KEY_BUTTON_PIN                GPIO_PIN_0
#define KEY_BUTTON_GPIO_PORT          GPIOA
#define KEY_BUTTON_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define KEY_BUTTON_GPIO_CLK_DISABLE() __HAL_RCC_GPIOA_CLK_DISABLE()
#define KEY_BUTTON_EXTI_IRQn          EXTI0_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    do{if((__INDEX__) == 0) KEY_BUTTON_GPIO_CLK_ENABLE(); \
                                                }while(0)

#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)    do{if((__INDEX__) == 0) KEY_BUTTON_GPIO_CLK_DISABLE(); \
                                                 }while(0)
//}}}
//{{{  spi defines
#define DISCOVERY_SPIx                              SPI1
#define DISCOVERY_SPIx_CLK_ENABLE()                 __HAL_RCC_SPI1_CLK_ENABLE()
#define DISCOVERY_SPIx_GPIO_PORT                    GPIOA                      /* GPIOA */
#define DISCOVERY_SPIx_AF                           GPIO_AF5_SPI1
#define DISCOVERY_SPIx_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOA_CLK_ENABLE()
#define DISCOVERY_SPIx_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOA_CLK_DISABLE()
#define DISCOVERY_SPIx_SCK_PIN                      GPIO_PIN_5                 /* PA.05 */
#define DISCOVERY_SPIx_MISO_PIN                     GPIO_PIN_6                 /* PA.06 */
#define DISCOVERY_SPIx_MOSI_PIN                     GPIO_PIN_7                 /* PA.07 */

#define SPIx_TIMEOUT_MAX                            0x1000 /*<! The value of the maximal timeout for BUS waiting loops */

/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                        ((uint8_t)0x00)
//}}}
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

#define READWRITE_CMD                     ((uint8_t)0x80)
#define MULTIPLEBYTE_CMD                  ((uint8_t)0x40)
//}}}
//{{{  accel defines

/* Chip Select macro definition */
#define ACCELERO_CS_LOW()       HAL_GPIO_WritePin(ACCELERO_CS_GPIO_PORT, ACCELERO_CS_PIN, GPIO_PIN_RESET)
#define ACCELERO_CS_HIGH()      HAL_GPIO_WritePin(ACCELERO_CS_GPIO_PORT, ACCELERO_CS_PIN, GPIO_PIN_SET)

#define ACCELERO_CS_PIN                        GPIO_PIN_3                 /* PE.03 */
#define ACCELERO_CS_GPIO_PORT                  GPIOE                      /* GPIOE */
#define ACCELERO_CS_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOE_CLK_ENABLE()
#define ACCELERO_CS_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOE_CLK_DISABLE()
#define ACCELERO_INT_GPIO_PORT                 GPIOE                      /* GPIOE */
#define ACCELERO_INT_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOE_CLK_ENABLE()
#define ACCELERO_INT_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOE_CLK_DISABLE()
#define ACCELERO_INT1_PIN                      GPIO_PIN_0                 /* PE.00 */
#define ACCELERO_INT1_EXTI_IRQn                EXTI0_IRQn
#define ACCELERO_INT2_PIN                      GPIO_PIN_1                 /* PE.01 */
#define ACCELERO_INT2_EXTI_IRQn                EXTI1_IRQn
//}}}

GPIO_TypeDef* GPIO_PORT[LEDn] = {LED4_GPIO_PORT, LED3_GPIO_PORT, LED5_GPIO_PORT, LED6_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED4_PIN, LED3_PIN, LED5_PIN, LED6_PIN};

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {KEY_BUTTON_GPIO_PORT};
const uint16_t BUTTON_PIN[BUTTONn] = {KEY_BUTTON_PIN};
const uint8_t BUTTON_IRQn[BUTTONn] = {KEY_BUTTON_EXTI_IRQn};

uint32_t I2cxTimeout = I2Cx_TIMEOUT_MAX;
uint32_t SpixTimeout = SPIx_TIMEOUT_MAX;

static SPI_HandleTypeDef SpiHandle;
static I2C_HandleTypeDef I2cHandle;

//{{{
static void SPIx_MspInit() {

  // Enable the SPI peripheral
  DISCOVERY_SPIx_CLK_ENABLE();

  // Enable SCK, MOSI and MISO GPIO clocks
  DISCOVERY_SPIx_GPIO_CLK_ENABLE();

  // SPI SCK, MOSI, MISO pin configuration
  GPIO_InitTypeDef   GPIO_InitStructure;
  GPIO_InitStructure.Pin = (DISCOVERY_SPIx_SCK_PIN | DISCOVERY_SPIx_MISO_PIN | DISCOVERY_SPIx_MOSI_PIN);
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull  = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
  GPIO_InitStructure.Alternate = DISCOVERY_SPIx_AF;
  HAL_GPIO_Init(DISCOVERY_SPIx_GPIO_PORT, &GPIO_InitStructure);
  }
//}}}
//{{{
static void SPIx_Init() {

  if (HAL_SPI_GetState (&SpiHandle) == HAL_SPI_STATE_RESET) {
    SpiHandle.Instance = DISCOVERY_SPIx;
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    SpiHandle.Init.Direction = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
    SpiHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
    SpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    SpiHandle.Init.CRCPolynomial = 7;
    SpiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    SpiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.NSS = SPI_NSS_SOFT;
    SpiHandle.Init.TIMode = SPI_TIMODE_DISABLED;
    SpiHandle.Init.Mode = SPI_MODE_MASTER;

    SPIx_MspInit();
    HAL_SPI_Init (&SpiHandle);
    }
  }
//}}}
//{{{
static uint8_t SPIx_WriteRead (uint8_t Byte) {

  uint8_t receivedbyte = 0;

  //  Send a Byte through the SPI peripheral, read byte from the SPI bus
  if (HAL_SPI_TransmitReceive (&SpiHandle, (uint8_t*) &Byte, (uint8_t*) &receivedbyte, 1, SpixTimeout) != HAL_OK) {
    // error
    }

  return receivedbyte;
  }
//}}}

//{{{
static void I2Cx_MspInit() {

  // Enable I2C GPIO clocks
  DISCOVERY_I2Cx_SCL_SDA_GPIO_CLK_ENABLE();

  // DISCOVERY_I2Cx SCL and SDA pins configuration
  GPIO_InitTypeDef  GPIO_InitStruct;
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
  HAL_NVIC_SetPriority(DISCOVERY_I2Cx_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DISCOVERY_I2Cx_EV_IRQn);

  // Enable and set I2Cx Interrupt to the highest priority
  HAL_NVIC_SetPriority(DISCOVERY_I2Cx_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DISCOVERY_I2Cx_ER_IRQn);
  }
//}}}
//{{{
static void I2Cx_Init() {

  if (HAL_I2C_GetState(&I2cHandle) == HAL_I2C_STATE_RESET) {
    // DISCOVERY_I2Cx peripheral configuration */
    I2cHandle.Init.ClockSpeed = BSP_I2C_SPEED;
    I2cHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
    I2cHandle.Init.OwnAddress1 = 0x33;
    I2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    I2cHandle.Instance = DISCOVERY_I2Cx;

    // Init the I2C */
    I2Cx_MspInit();
    HAL_I2C_Init(&I2cHandle);
    }
  }
//}}}
//{{{
static void I2Cx_Error (uint8_t Addr) {

  // Re-Initialize the I2C communication bus
  I2Cx_Init();
  }
//}}}
//{{{
static void I2Cx_WriteData (uint8_t Addr, uint8_t Reg, uint8_t Value) {

  HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&I2cHandle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, I2cxTimeout);
  if (status != HAL_OK)
    I2Cx_Error(Addr);
  }
//}}}
//{{{
static uint8_t  I2Cx_ReadData (uint8_t Addr, uint8_t Reg) {

  uint8_t value = 0;
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&I2cHandle, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &value, 1,I2cxTimeout);
  if (status != HAL_OK)
    I2Cx_Error(Addr);

  return value;
  }
//}}}

//{{{
void BSP_LED_Init (Led_TypeDef Led) {

  // Enable the GPIO_LED Clock
  LEDx_GPIO_CLK_ENABLE (Led);

  // Configure the GPIO_LED pin
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init (GPIO_PORT[Led], &GPIO_InitStruct);

  HAL_GPIO_WritePin (GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
  }
//}}}
//{{{
void BSP_LED_On (Led_TypeDef Led) {
  HAL_GPIO_WritePin (GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
  }
//}}}
//{{{
void BSP_LED_Off (Led_TypeDef Led) {
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
  }
//}}}
//{{{
void BSP_LED_Toggle (Led_TypeDef Led) {
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
  }
//}}}

//{{{
void BSP_PB_Init (Button_TypeDef Button, ButtonMode_TypeDef Mode) {

  // Enable the BUTTON Clock
  BUTTONx_GPIO_CLK_ENABLE(Button);

  GPIO_InitTypeDef GPIO_InitStruct;
  if (Mode == BUTTON_MODE_GPIO) {
    // Configure Button pin as input */
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init (BUTTON_PORT[Button], &GPIO_InitStruct);
    }

  if (Mode == BUTTON_MODE_EXTI) {
    // Configure Button pin as input with External interrupt */
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    HAL_GPIO_Init (BUTTON_PORT[Button], &GPIO_InitStruct);

    // Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority ((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0);
    HAL_NVIC_EnableIRQ ((IRQn_Type)(BUTTON_IRQn[Button]));
  }
}
//}}}
//{{{
uint32_t BSP_PB_GetState (Button_TypeDef Button) {
  return HAL_GPIO_ReadPin (BUTTON_PORT[Button], BUTTON_PIN[Button]);
  }
//}}}

//{{{
void ACCELERO_IO_Init() {

  // Enable CS GPIO clock and configure GPIO pin for Accelerometer Chip select
  ACCELERO_CS_GPIO_CLK_ENABLE();

  // Configure GPIO PIN for LIS Chip select
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.Pin = ACCELERO_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(ACCELERO_CS_GPIO_PORT, &GPIO_InitStructure);

  // Deselect: Chip Select high
  ACCELERO_CS_HIGH();

  SPIx_Init();
  }
//}}}
//{{{
void ACCELERO_IO_ITConfig() {

  GPIO_InitTypeDef GPIO_InitStructure;

  // Enable INT2 GPIO clock and configure GPIO PINs to detect Interrupts
  ACCELERO_INT_GPIO_CLK_ENABLE();

  // Configure GPIO PINs to detect Interrupts
  GPIO_InitStructure.Pin = ACCELERO_INT2_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (ACCELERO_INT_GPIO_PORT, &GPIO_InitStructure);

  // Enable and set Accelerometer INT2 to the lowest priority
  HAL_NVIC_SetPriority ((IRQn_Type)ACCELERO_INT2_EXTI_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ ((IRQn_Type)ACCELERO_INT2_EXTI_IRQn);
  }
//}}}
//{{{
void ACCELERO_IO_Read (uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead) {

  if(NumByteToRead > 0x01)
    ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  else
    ReadAddr |= (uint8_t)READWRITE_CMD;

  // Set chip select Low at the start of the transmission
  ACCELERO_CS_LOW();

  // Send the Address of the indexed register
  SPIx_WriteRead(ReadAddr);

  // Receive the data that will be read from the device (MSB First)
  while(NumByteToRead > 0x00) {
    // Send dummy byte (0x00) to generate the SPI clock to ACCELEROMETER (Slave device)
    *pBuffer = SPIx_WriteRead(DUMMY_BYTE);
    NumByteToRead--;
    pBuffer++;
    }

  // Set chip select High at the end of the transmission
  ACCELERO_CS_HIGH();
  }
//}}}
//{{{
void ACCELERO_IO_Write (uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite) {

  // Configure the MS bit:
  //  - When 0, the address will remain unchanged in multiple read/write commands
  //  - When 1, the address will be auto incremented in multiple read/write commands
  if (NumByteToWrite > 0x01)
    WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;

  // Set chip select Low at the start of the transmission
  ACCELERO_CS_LOW();

  // Send the Address of the indexed register
  SPIx_WriteRead (WriteAddr);

  // Send the data that will be written into the device (MSB First)
  while(NumByteToWrite >= 0x01) {
    SPIx_WriteRead (*pBuffer);
    NumByteToWrite--;
    pBuffer++;
    }

  // Set chip select High at the end of the transmission
  ACCELERO_CS_HIGH ();
  }
//}}}

//{{{
void AUDIO_IO_Init() {

  // Enable Reset GPIO Clock
  AUDIO_RESET_GPIO_CLK_ENABLE();

  // Audio reset pin configuration
  GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.Pin = AUDIO_RESET_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(AUDIO_RESET_GPIO, &GPIO_InitStruct);

  I2Cx_Init();

  // Power Down the codec
  HAL_GPIO_WritePin(AUDIO_RESET_GPIO, AUDIO_RESET_PIN, GPIO_PIN_RESET);

  // Wait for a delay to insure registers erasing
  HAL_Delay(5);

  // Power on the codec HAL_GPIO_WritePin(AUDIO_RESET_GPIO, AUDIO_RESET_PIN, GPIO_PIN_SET);

  // Wait for a delay to insure registers erasing
  HAL_Delay(5);
  }
//}}}
void AUDIO_IO_DeInit() {}
//{{{
void AUDIO_IO_Write (uint8_t Addr, uint8_t Reg, uint8_t Value) {
  I2Cx_WriteData (Addr, Reg, Value);
  }
//}}}
//{{{
uint8_t AUDIO_IO_Read (uint8_t Addr, uint8_t Reg) {
  return I2Cx_ReadData (Addr, Reg);
  }
//}}}
