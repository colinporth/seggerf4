//{{{
/**
    [..]
      The SPI HAL driver can be used as follows:

      (#) Declare a SPI_HandleTypeDef handle structure, for example:
          SPI_HandleTypeDef  hspi;

      (#)Initialize the SPI low level resources by implementing the HAL_SPI_MspInit() API:
          (##) Enable the SPIx interface clock
          (##) SPI pins configuration
              (+++) Enable the clock for the SPI GPIOs
              (+++) Configure these SPI pins as alternate function push-pull
          (##) NVIC configuration if you need to use interrupt process
              (+++) Configure the SPIx interrupt priority
              (+++) Enable the NVIC SPI IRQ handle
          (##) DMA Configuration if you need to use DMA process
              (+++) Declare a DMA_HandleTypeDef handle structure for the transmit or receive stream
              (+++) Enable the DMAx clock
              (+++) Configure the DMA handle parameters
              (+++) Configure the DMA Tx or Rx stream
              (+++) Associate the initialized hdma_tx handle to the hspi DMA Tx or Rx handle
              (+++) Configure the priority and enable the NVIC for the transfer complete interrupt on the DMA Tx or Rx stream

      (#) Program the Mode, BidirectionalMode , Data size, Baudrate Prescaler, NSS
          management, Clock polarity and phase, FirstBit and CRC configuration in the hspi Init structure.

      (#) Initialize the SPI registers by calling the HAL_SPI_Init() API:
          (++) This API configures also the low level Hardware GPIO, CLOCK, CORTEX...etc)
              by calling the customized HAL_SPI_MspInit() API.
     [..]
       Circular mode restriction:
      (#) The DMA circular mode cannot be used when the SPI is configured in these modes:
          (##) Master 2Lines RxOnly
          (##) Master 1Line Rx
      (#) The CRC feature is not managed when the DMA circular mode is enabled
      (#) When the SPI DMA Pause/Stop features are used, we must use the following APIs
          the HAL_SPI_DMAPause()/ HAL_SPI_DMAStop() only under the SPI callbacks
     [..]
       Master Receive mode restriction:
      (#) In Master unidirectional receive-only mode (MSTR =1, BIDIMODE=0, RXONLY=0) or
          bidirectional receive mode (MSTR=1, BIDIMODE=1, BIDIOE=0), to ensure that the SPI
          does not initiate a new transfer the following procedure has to be respected:
          (##) HAL_SPI_DeInit()
          (##) HAL_SPI_Init()

    Using the HAL it is not possible to reach all supported SPI frequency with the differents SPI Modes,
    the following tables resume the max SPI frequency reached with data size 8bits/16bits,
    according to frequency used on APBx Peripheral Clock (fPCLK) used by the SPI instance :

    DataSize = SPI_DATASIZE_8BIT:
    +----------------------------------------------------------------------------------------------+
    |         |                | 2Lines Fullduplex   |     2Lines RxOnly    |         1Line        |
    | Process | Tranfert mode  |---------------------|----------------------|----------------------|
    |         |                |  Master  |  Slave   |  Master   |  Slave   |  Master   |  Slave   |
    |==============================================================================================|
    |    T    |     Polling    | Fpclk/2  | Fpclk/2  |    NA     |    NA    |    NA     |   NA     |
    |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
    |    /    |     Interrupt  | Fpclk/4  | Fpclk/8  |    NA     |    NA    |    NA     |   NA     |
    |    R    |----------------|----------|----------|-----------|----------|-----------|----------|
    |    X    |       DMA      | Fpclk/2  | Fpclk/2  |    NA     |    NA    |    NA     |   NA     |
    |=========|================|==========|==========|===========|==========|===========|==========|
    |         |     Polling    | Fpclk/2  | Fpclk/2  | Fpclk/64  | Fpclk/2  | Fpclk/64  | Fpclk/2  |
    |         |----------------|----------|----------|-----------|----------|-----------|----------|
    |    R    |     Interrupt  | Fpclk/8  | Fpclk/8  | Fpclk/64  | Fpclk/2  | Fpclk/64  | Fpclk/2  |
    |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
    |         |       DMA      | Fpclk/2  | Fpclk/2  | Fpclk/64  | Fpclk/2  | Fpclk/128 | Fpclk/2  |
    |=========|================|==========|==========|===========|==========|===========|==========|
    |         |     Polling    | Fpclk/2  | Fpclk/4  |     NA    |    NA    | Fpclk/2   | Fpclk/64 |
    |         |----------------|----------|----------|-----------|----------|-----------|----------|
    |    T    |     Interrupt  | Fpclk/2  | Fpclk/4  |     NA    |    NA    | Fpclk/2   | Fpclk/64 |
    |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
    |         |       DMA      | Fpclk/2  | Fpclk/2  |     NA    |    NA    | Fpclk/2   | Fpclk/128|
    +----------------------------------------------------------------------------------------------+

    DataSize = SPI_DATASIZE_16BIT:
    +----------------------------------------------------------------------------------------------+
    |         |                | 2Lines Fullduplex   |     2Lines RxOnly    |         1Line        |
    | Process | Tranfert mode  |---------------------|----------------------|----------------------|
    |         |                |  Master  |  Slave   |  Master   |  Slave   |  Master   |  Slave   |
    |==============================================================================================|
    |    T    |     Polling    | Fpclk/2  | Fpclk/2  |    NA     |    NA    |    NA     |   NA     |
    |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
    |    /    |     Interrupt  | Fpclk/4  | Fpclk/4  |    NA     |    NA    |    NA     |   NA     |
    |    R    |----------------|----------|----------|-----------|----------|-----------|----------|
    |    X    |       DMA      | Fpclk/2  | Fpclk/2  |    NA     |    NA    |    NA     |   NA     |
    |=========|================|==========|==========|===========|==========|===========|==========|
    |         |     Polling    | Fpclk/2  | Fpclk/2  | Fpclk/64  | Fpclk/2  | Fpclk/32  | Fpclk/2  |
    |         |----------------|----------|----------|-----------|----------|-----------|----------|
    |    R    |     Interrupt  | Fpclk/4  | Fpclk/4  | Fpclk/64  | Fpclk/2  | Fpclk/64  | Fpclk/2  |
    |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
    |         |       DMA      | Fpclk/2  | Fpclk/2  | Fpclk/64  | Fpclk/2  | Fpclk/128 | Fpclk/2  |
    |=========|================|==========|==========|===========|==========|===========|==========|
    |         |     Polling    | Fpclk/2  | Fpclk/2  |     NA    |    NA    | Fpclk/2   | Fpclk/32 |
    |         |----------------|----------|----------|-----------|----------|-----------|----------|
    |    T    |     Interrupt  | Fpclk/2  | Fpclk/2  |     NA    |    NA    | Fpclk/2   | Fpclk/64 |
    |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
    |         |       DMA      | Fpclk/2  | Fpclk/2  |     NA    |    NA    | Fpclk/2   | Fpclk/128|
    +----------------------------------------------------------------------------------------------+
     [..]
       (@) The max SPI frequency depend on SPI data size (8bits, 16bits),
           SPI mode(2 Lines fullduplex, 2 lines RxOnly, 1 line TX/RX) and Process mode (Polling, IT, DMA).
       (@)
            (+@) TX/RX processes are HAL_SPI_TransmitReceive(), HAL_SPI_TransmitReceive_IT() and HAL_SPI_TransmitReceive_DMA()
            (+@) RX processes are HAL_SPI_Receive(), HAL_SPI_Receive_IT() and HAL_SPI_Receive_DMA()
            (+@) TX processes are HAL_SPI_Transmit(), HAL_SPI_Transmit_IT() and HAL_SPI_Transmit_DMA()

    This subsection provides a set of functions allowing to manage the SPI
    data transfers.

    [..] The SPI supports master and slave mode :

    (#) There are two modes of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
            The HAL status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode: The communication is performed using Interrupts
            or DMA, These APIs return the HAL status.
            The end of the data processing will be indicated through the
            dedicated SPI IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.
            The HAL_SPI_TxCpltCallback(), HAL_SPI_RxCpltCallback() and HAL_SPI_TxRxCpltCallback() user callbacks
            will be executed respectively at the end of the transmit or Receive process
            The HAL_SPI_ErrorCallback()user callback will be executed when a communication error is detected

    (#) APIs provided for these 2 transfer modes (Blocking mode or Non blocking mode using either Interrupt or DMA)
        exist for 1Line (simplex) and 2Lines (full duplex) modes.
  */
//}}}
#include "stm32f4xx_hal.h"

#define SPI_DEFAULT_TIMEOUT 100U

//{{{
static HAL_StatusTypeDef SPI_WaitFlagStateUntilTimeout (SPI_HandleTypeDef *hspi, uint32_t Flag, uint32_t State, uint32_t Timeout, uint32_t Tickstart) {

  while((((hspi->Instance->SR & Flag) == (Flag)) ? SET : RESET) != State) {
    if(Timeout != HAL_MAX_DELAY) {
      if((Timeout == 0U) || ((HAL_GetTick()-Tickstart) >= Timeout)) {
        /* Disable the SPI and reset the CRC: the CRC value should be cleared
        //on both master and slave sides in order to resynchronize the master
        //and slave for their respective CRC calculation */

        /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
        __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

        if((hspi->Init.Mode == SPI_MODE_MASTER)&&((hspi->Init.Direction == SPI_DIRECTION_1LINE)||(hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY)))
          /* Disable SPI peripheral */
          __HAL_SPI_DISABLE(hspi);

        /* Reset CRC Calculation */
        if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
          SPI_RESET_CRC(hspi);

        hspi->State= HAL_SPI_STATE_READY;
        return HAL_TIMEOUT;
        }
      }
    }

  return HAL_OK;
  }
//}}}
//{{{
static HAL_StatusTypeDef SPI_CheckFlag_BSY (SPI_HandleTypeDef *hspi, uint32_t Timeout, uint32_t Tickstart) {

  // Control the BSY flag
  if (SPI_WaitFlagStateUntilTimeout (hspi, SPI_FLAG_BSY, RESET, Timeout, Tickstart) != HAL_OK) {
    SET_BIT (hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
    return HAL_TIMEOUT;
    }

  return HAL_OK;
  }
//}}}

//{{{
static void SPI_CloseRxTx_ISR (SPI_HandleTypeDef *hspi) { uint32_t tickstart = 0U;

  __IO uint32_t count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);
  /* Init tickstart for timeout managment*/
  tickstart = HAL_GetTick();

  /* Disable ERR interrupt */
  __HAL_SPI_DISABLE_IT(hspi, SPI_IT_ERR);

  /* Wait until TXE flag is set */
  do {
    if(count-- == 0U) {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
      break;
      }
    }
  while ((hspi->Instance->SR & SPI_FLAG_TXE) == RESET);

  /* Check the end of the transaction */
  if(SPI_CheckFlag_BSY(hspi, SPI_DEFAULT_TIMEOUT, tickstart)!=HAL_OK)
    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);

  /* Clear overrun flag in 2 Lines communication mode because received is not read */
  if(hspi->Init.Direction == SPI_DIRECTION_2LINES)
    __HAL_SPI_CLEAR_OVRFLAG(hspi);

  if (hspi->ErrorCode == HAL_SPI_ERROR_NONE) {
    if(hspi->State == HAL_SPI_STATE_BUSY_RX) {
      hspi->State = HAL_SPI_STATE_READY;
      HAL_SPI_RxCpltCallback(hspi);
      }
    else {
      hspi->State = HAL_SPI_STATE_READY;
      HAL_SPI_TxRxCpltCallback(hspi);
      }
    }
  else {
    hspi->State = HAL_SPI_STATE_READY;
    HAL_SPI_ErrorCallback(hspi);
    }
  }
//}}}
//{{{
static void SPI_CloseRx_ISR (SPI_HandleTypeDef *hspi) {

  /* Disable RXNE and ERR interrupt */
  __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));

  /* Check the end of the transaction */
  if((hspi->Init.Mode == SPI_MODE_MASTER)&&((hspi->Init.Direction == SPI_DIRECTION_1LINE)||(hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY)))
    __HAL_SPI_DISABLE(hspi);

  /* Clear overrun flag in 2 Lines communication mode because received is not read */
  if (hspi->Init.Direction == SPI_DIRECTION_2LINES)
    __HAL_SPI_CLEAR_OVRFLAG(hspi);
  hspi->State = HAL_SPI_STATE_READY;

  if (hspi->ErrorCode == HAL_SPI_ERROR_NONE)
    HAL_SPI_RxCpltCallback(hspi);
  else
    HAL_SPI_ErrorCallback(hspi);
 }
//}}}
//{{{
static void SPI_CloseTx_ISR (SPI_HandleTypeDef *hspi) {

  uint32_t tickstart = 0U;
  __IO uint32_t count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);

  // Init tickstart for timeout management
  tickstart = HAL_GetTick();

  // Wait until TXE flag is set
  do {
    if(count-- == 0U) {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
      break;
      }
    } while ((hspi->Instance->SR & SPI_FLAG_TXE) == RESET);

  // Disable TXE and ERR interrupt
  __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_ERR));

  // Check Busy flag
  if (SPI_CheckFlag_BSY(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);

  // Clear overrun flag in 2 Lines communication mode because received is not read
  if (hspi->Init.Direction == SPI_DIRECTION_2LINES)
    __HAL_SPI_CLEAR_OVRFLAG(hspi);

  hspi->State = HAL_SPI_STATE_READY;
  if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
    HAL_SPI_ErrorCallback(hspi);
  else
    HAL_SPI_TxCpltCallback(hspi);
  }
//}}}

//{{{
static void SPI_RxISR_8BIT (struct __SPI_HandleTypeDef *hspi) {

  *hspi->pRxBuffPtr++ = (*(__IO uint8_t *)&hspi->Instance->DR);
  hspi->RxXferCount--;

  if (hspi->RxXferCount == 0U)
    SPI_CloseRx_ISR(hspi);
  }
//}}}
//{{{
static void SPI_TxISR_8BIT (struct __SPI_HandleTypeDef *hspi) {

  *(__IO uint8_t *)&hspi->Instance->DR = (*hspi->pTxBuffPtr++);
  hspi->TxXferCount--;

  if (hspi->TxXferCount == 0U)
    SPI_CloseTx_ISR(hspi);
  }
//}}}
//{{{
static void SPI_2linesRxISR_8BIT (struct __SPI_HandleTypeDef *hspi) {

  // Receive data in 8bit mode
  *hspi->pRxBuffPtr++ = *((__IO uint8_t *)&hspi->Instance->DR);
  hspi->RxXferCount--;

  // check end of the reception
  if (hspi->RxXferCount == 0U) {
    // Disable RXNE interrupt
    __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));
    if (hspi->TxXferCount == 0U)
      SPI_CloseRxTx_ISR(hspi);
   }
  }
//}}}
//{{{
static void SPI_2linesTxISR_8BIT (struct __SPI_HandleTypeDef *hspi) {

  *(__IO uint8_t *)&hspi->Instance->DR = (*hspi->pTxBuffPtr++);
  hspi->TxXferCount--;

  // check the end of the transmission */
  if (hspi->TxXferCount == 0U) {
    // Disable TXE interrupt */
    __HAL_SPI_DISABLE_IT(hspi, SPI_IT_TXE);

    if(hspi->RxXferCount == 0U)
      SPI_CloseRxTx_ISR(hspi);
    }
  }
//}}}
//{{{
static void SPI_RxISR_16BIT (struct __SPI_HandleTypeDef *hspi) {

  *((uint16_t *)hspi->pRxBuffPtr) = hspi->Instance->DR;
  hspi->pRxBuffPtr += sizeof(uint16_t);
  hspi->RxXferCount--;

  if (hspi->RxXferCount == 0U)
    SPI_CloseRx_ISR(hspi);
  }
//}}}
//{{{
static void SPI_TxISR_16BIT (struct __SPI_HandleTypeDef *hspi) {

  // Transmit data in 16 Bit mode */
  hspi->Instance->DR = *((uint16_t *)hspi->pTxBuffPtr);
  hspi->pTxBuffPtr += sizeof(uint16_t);
  hspi->TxXferCount--;

  if (hspi->TxXferCount == 0U)
    SPI_CloseTx_ISR(hspi);
  }
//}}}
//{{{
static void SPI_2linesRxISR_16BIT (struct __SPI_HandleTypeDef *hspi) {

  // Receive data in 16 Bit mode */
  *((uint16_t*)hspi->pRxBuffPtr) = hspi->Instance->DR;
  hspi->pRxBuffPtr += sizeof(uint16_t);
  hspi->RxXferCount--;

  if(hspi->RxXferCount == 0U) {
    // Disable RXNE interrupt */
    __HAL_SPI_DISABLE_IT(hspi, SPI_IT_RXNE);

    if (hspi->TxXferCount == 0U)
      SPI_CloseRxTx_ISR(hspi);
    }
  }
//}}}
//{{{
static void SPI_2linesTxISR_16BIT (struct __SPI_HandleTypeDef *hspi) {

  // Transmit data in 16 Bit mode */
  hspi->Instance->DR = *((uint16_t *)hspi->pTxBuffPtr);
  hspi->pTxBuffPtr += sizeof(uint16_t);
  hspi->TxXferCount--;

  // Enable CRC Transmission */
  if (hspi->TxXferCount == 0U) {
    // Disable TXE interrupt */
    __HAL_SPI_DISABLE_IT(hspi, SPI_IT_TXE);

    if (hspi->RxXferCount == 0U)
      SPI_CloseRxTx_ISR(hspi);
    }
  }
//}}}
//{{{
static void SPI_AbortRx_ISR (SPI_HandleTypeDef *hspi) {

  __IO uint32_t tmpreg = 0U;
  __IO uint32_t count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);

  // Wait until TXE flag is set
  do {
    if(count-- == 0U) {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
      break;
      }
    } while((hspi->Instance->SR & SPI_FLAG_TXE) == RESET);

  // Disable SPI Peripheral
  __HAL_SPI_DISABLE(hspi);

  // Disable TXEIE, RXNEIE and ERRIE(mode fault event, overrun error, TI frame error) interrupts
  CLEAR_BIT(hspi->Instance->CR2, (SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE));

  // Flush DR Register
  tmpreg = (*(__IO uint32_t *)&hspi->Instance->DR);

  // To avoid GCC warning
  UNUSED(tmpreg);
  }
//}}}
//{{{
static void SPI_AbortTx_ISR (SPI_HandleTypeDef *hspi) {

  // Disable TXEIE, RXNEIE and ERRIE(mode fault event, overrun error, TI frame error) interrupts
  CLEAR_BIT(hspi->Instance->CR2, (SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE));

  // Disable SPI Peripheral
  __HAL_SPI_DISABLE(hspi);
  }
//}}}

//{{{
static void SPI_DMATransmitCplt (DMA_HandleTypeDef *hdma) {

  SPI_HandleTypeDef* hspi = (SPI_HandleTypeDef*)((DMA_HandleTypeDef* )hdma)->Parent;
  uint32_t tickstart = 0U;

  // Init tickstart for timeout managment*/
  tickstart = HAL_GetTick();

  // DMA Normal Mode */
  if((hdma->Instance->CR & DMA_SxCR_CIRC) == 0U) {
    // Disable Tx DMA Request */
    CLEAR_BIT (hspi->Instance->CR2, SPI_CR2_TXDMAEN);

    // Check the end of the transaction */
    if(SPI_CheckFlag_BSY(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);

    // Clear overrun flag in 2 Lines communication mode because received data is not read */
    if (hspi->Init.Direction == SPI_DIRECTION_2LINES)
      __HAL_SPI_CLEAR_OVRFLAG(hspi);

    hspi->TxXferCount = 0U;
    hspi->State = HAL_SPI_STATE_READY;

    if (hspi->ErrorCode != HAL_SPI_ERROR_NONE) {
      HAL_SPI_ErrorCallback(hspi);
      return;
      }
    }

  HAL_SPI_TxCpltCallback(hspi);
  }
//}}}
//{{{
static void SPI_DMAReceiveCplt (DMA_HandleTypeDef *hdma) {

  SPI_HandleTypeDef* hspi = ( SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  if ((hdma->Instance->CR & DMA_SxCR_CIRC) == 0U) {
    // Disable Rx/Tx DMA Request (done by default to handle the case master rx direction 2 lines) */
    CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

    // Check the end of the transaction */
    if((hspi->Init.Mode == SPI_MODE_MASTER)&&((hspi->Init.Direction == SPI_DIRECTION_1LINE)||(hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY)))
      __HAL_SPI_DISABLE(hspi);

    hspi->RxXferCount = 0U;
    hspi->State = HAL_SPI_STATE_READY;


    if(hspi->ErrorCode != HAL_SPI_ERROR_NONE) {
      HAL_SPI_ErrorCallback(hspi);
      return;
      }
    }

  HAL_SPI_RxCpltCallback(hspi);
  }
//}}}
//{{{
static void SPI_DMATransmitReceiveCplt (DMA_HandleTypeDef *hdma) {

  SPI_HandleTypeDef* hspi = ( SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  uint32_t tickstart = 0U;

  // Init tickstart for timeout management*/
  tickstart = HAL_GetTick();

  if((hdma->Instance->CR & DMA_SxCR_CIRC) == 0U) {
    // Check the end of the transaction */
    if(SPI_CheckFlag_BSY(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);

    // Disable Rx/Tx DMA Request */
    CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

    hspi->TxXferCount = 0U;
    hspi->RxXferCount = 0U;
    hspi->State = HAL_SPI_STATE_READY;

    if(hspi->ErrorCode != HAL_SPI_ERROR_NONE) {
      HAL_SPI_ErrorCallback(hspi);
      return;
      }
    }

  HAL_SPI_TxRxCpltCallback(hspi);
  }
//}}}
//{{{
static void SPI_DMAHalfTransmitCplt (DMA_HandleTypeDef *hdma) {

  SPI_HandleTypeDef* hspi = ( SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  HAL_SPI_TxHalfCpltCallback(hspi);
  }
//}}}
//{{{
static void SPI_DMAHalfReceiveCplt (DMA_HandleTypeDef *hdma) {

  SPI_HandleTypeDef* hspi = ( SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  HAL_SPI_RxHalfCpltCallback(hspi);
  }
//}}}
//{{{
static void SPI_DMAHalfTransmitReceiveCplt (DMA_HandleTypeDef *hdma) {

  SPI_HandleTypeDef* hspi = ( SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  HAL_SPI_TxRxHalfCpltCallback(hspi);
  }
//}}}
//{{{
static void SPI_DMAError (DMA_HandleTypeDef *hdma) {

  SPI_HandleTypeDef* hspi = (SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  // Stop the disable DMA transfer on SPI side */
  CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

  SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_DMA);
  hspi->State = HAL_SPI_STATE_READY;
  HAL_SPI_ErrorCallback(hspi);
  }
//}}}
//{{{
static void SPI_DMAAbortOnError (DMA_HandleTypeDef *hdma) {

  SPI_HandleTypeDef* hspi = ( SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  hspi->RxXferCount = 0U;
  hspi->TxXferCount = 0U;
  HAL_SPI_ErrorCallback(hspi);
  }
//}}}
//{{{
static void SPI_DMATxAbortCallback (DMA_HandleTypeDef *hdma) {

  __IO uint32_t count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);
  SPI_HandleTypeDef* hspi = ( SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  hspi->hdmatx->XferAbortCallback = NULL;

  /* Disable Tx DMA Request */
  CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN );

  /* Wait until TXE flag is set */
  do {
    if(count-- == 0U) {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
      break;
      }
    } while((hspi->Instance->SR & SPI_FLAG_TXE) == RESET);

  // Check if an Abort process is still ongoing */
  if (hspi->hdmarx != NULL)
    if (hspi->hdmarx->XferAbortCallback != NULL)
      return;

  // No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
  hspi->RxXferCount = 0U;
  hspi->TxXferCount = 0U;

  // Reset errorCode */
  hspi->ErrorCode = HAL_SPI_ERROR_NONE;

  // Clear the Error flags in the SR register */
  __HAL_SPI_CLEAR_FREFLAG(hspi);

  // Restore hspi->State to Ready */
  hspi->State  = HAL_SPI_STATE_READY;

  // Call user Abort complete callback */
  HAL_SPI_AbortCpltCallback(hspi);
  }
//}}}
//{{{
static void SPI_DMARxAbortCallback (DMA_HandleTypeDef *hdma) {

  SPI_HandleTypeDef* hspi = ( SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  __HAL_SPI_DISABLE(hspi);

  hspi->hdmarx->XferAbortCallback = NULL;

  // Disable Rx DMA Request */
  CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_RXDMAEN);

  // Check if an Abort process is still ongoing */
  if (hspi->hdmatx != NULL)
    if (hspi->hdmatx->XferAbortCallback != NULL)
      return;

  // No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
  hspi->RxXferCount = 0U;
  hspi->TxXferCount = 0U;

  // Reset errorCode */
  hspi->ErrorCode = HAL_SPI_ERROR_NONE;

  // Clear the Error flags in the SR register */
  __HAL_SPI_CLEAR_OVRFLAG(hspi);
  __HAL_SPI_CLEAR_FREFLAG(hspi);

  // Restore hspi->State to Ready */
  hspi->State  = HAL_SPI_STATE_READY;

  // Call user Abort complete callback */
  HAL_SPI_AbortCpltCallback(hspi);
  }
//}}}

__weak void HAL_SPI_TxCpltCallback (SPI_HandleTypeDef *hspi) { UNUSED(hspi); }
__weak void HAL_SPI_RxCpltCallback (SPI_HandleTypeDef *hspi) { UNUSED(hspi); }
__weak void HAL_SPI_TxRxCpltCallback (SPI_HandleTypeDef *hspi) { UNUSED(hspi); }
__weak void HAL_SPI_RxHalfCpltCallback (SPI_HandleTypeDef *hspi) { UNUSED(hspi); }
__weak void HAL_SPI_TxRxHalfCpltCallback (SPI_HandleTypeDef *hspi) { UNUSED(hspi); }
__weak void HAL_SPI_ErrorCallback (SPI_HandleTypeDef *hspi) { UNUSED(hspi); }
__weak void HAL_SPI_AbortCpltCallback (SPI_HandleTypeDef *hspi) { UNUSED(hspi); }

//{{{
HAL_StatusTypeDef HAL_SPI_Init (SPI_HandleTypeDef *hspi) {

  hspi->State = HAL_SPI_STATE_BUSY;
  hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

  __HAL_SPI_DISABLE (hspi);

  // Configure SPI Mode, Communication Mode, Data size, Clock polarity and phase, NSS management
  // Communication speed, First bit and CRC calculation state */
  WRITE_REG(hspi->Instance->CR1, (hspi->Init.Mode | hspi->Init.Direction | hspi->Init.DataSize |
                                  hspi->Init.CLKPolarity | hspi->Init.CLKPhase | (hspi->Init.NSS & SPI_CR1_SSM) |
                                  hspi->Init.BaudRatePrescaler | hspi->Init.FirstBit  | hspi->Init.CRCCalculation) );

  // Configure : NSS management
  WRITE_REG(hspi->Instance->CR2, (((hspi->Init.NSS >> 16U) & SPI_CR2_SSOE) | hspi->Init.TIMode));

#if defined(SPI_I2SCFGR_I2SMOD)
  /* Activate the SPI mode (Make sure that I2SMOD bit in I2SCFGR register is reset) */
  CLEAR_BIT(hspi->Instance->I2SCFGR, SPI_I2SCFGR_I2SMOD);
#endif /* USE_SPI_CRC */

  hspi->ErrorCode = HAL_SPI_ERROR_NONE;
  hspi->State     = HAL_SPI_STATE_READY;

  return HAL_OK;
  }
//}}}

HAL_SPI_StateTypeDef HAL_SPI_GetState (SPI_HandleTypeDef *hspi) { return hspi->State; }
uint32_t HAL_SPI_GetError (SPI_HandleTypeDef *hspi) { return hspi->ErrorCode; }

//{{{
HAL_StatusTypeDef HAL_SPI_Transmit (SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout) {

  uint32_t tickstart = 0U;
  HAL_StatusTypeDef errorcode = HAL_OK;

  // Set the transaction information
  hspi->State       = HAL_SPI_STATE_BUSY_TX;
  hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
  hspi->pTxBuffPtr  = (uint8_t*)pData;
  hspi->TxXferSize  = Size;
  hspi->TxXferCount = Size;

  // Init field not used in handle to zero
  hspi->pRxBuffPtr  = (uint8_t*)NULL;
  hspi->RxXferSize  = 0U;
  hspi->RxXferCount = 0U;
  hspi->TxISR       = NULL;
  hspi->RxISR       = NULL;

  // Configure communication direction : 1Line
  if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
    SPI_1LINE_TX (hspi);

  // Check if the SPI is already enabled
  if( (hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
    __HAL_SPI_ENABLE (hspi);

  tickstart = HAL_GetTick();
  if (hspi->Init.DataSize == SPI_DATASIZE_16BIT) {
    //{{{  transmit data in 16 Bit mode
    if ((hspi->Init.Mode == SPI_MODE_SLAVE) || (hspi->TxXferCount == 0x01)) {
      hspi->Instance->DR = *((uint16_t *)pData);
      pData += sizeof(uint16_t);
      hspi->TxXferCount--;
      }

    // Transmit data in 16 Bit mode */
    while (hspi->TxXferCount > 0U) {
      // Wait until TXE flag is set to send data */
      if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE)) {
        hspi->Instance->DR = *((uint16_t *)pData);
        pData += sizeof(uint16_t);
        hspi->TxXferCount--;
        }
      else {
        /* Timeout management */
        if ((Timeout == 0U) || ((Timeout != HAL_MAX_DELAY) && ((HAL_GetTick()-tickstart) >=  Timeout))) {
          errorcode = HAL_TIMEOUT;
          goto error;
          }
        }
      }
    }
    //}}}
  else {
    //{{{  Transmit data in 8 Bit mode
    if ((hspi->Init.Mode == SPI_MODE_SLAVE) || (hspi->TxXferCount == 0x01)) {
      *((__IO uint8_t*)&hspi->Instance->DR) = (*pData);
      pData += sizeof(uint8_t);
      hspi->TxXferCount--;
    }
    while (hspi->TxXferCount > 0U) {
      /* Wait until TXE flag is set to send data */
      if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE)) {
        *((__IO uint8_t*)&hspi->Instance->DR) = (*pData);
        pData += sizeof(uint8_t);
        hspi->TxXferCount--;
        }
      else {
        /* Timeout management */
        if((Timeout == 0U) || ((Timeout != HAL_MAX_DELAY) && ((HAL_GetTick()-tickstart) >=  Timeout))) {
          errorcode = HAL_TIMEOUT;
          goto error;
          }
        }
      }
    }
    //}}}

  // Wait until TXE flag
  if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_TXE, SET, Timeout, tickstart) != HAL_OK) {
    errorcode = HAL_TIMEOUT;
    goto error;
    }

  // Check Busy flag
  if (SPI_CheckFlag_BSY (hspi, Timeout, tickstart) != HAL_OK) {
    errorcode = HAL_ERROR;
    hspi->ErrorCode = HAL_SPI_ERROR_FLAG;
    goto error;
    }

  // Clear overrun flag in 2 Lines communication mode because received is not read
  if (hspi->Init.Direction == SPI_DIRECTION_2LINES)
    __HAL_SPI_CLEAR_OVRFLAG(hspi);

  if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
    errorcode = HAL_ERROR;

error:
  hspi->State = HAL_SPI_STATE_READY;
  return errorcode;
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_SPI_Receive (SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout) {

  uint32_t tickstart = 0U;
  HAL_StatusTypeDef errorcode = HAL_OK;

  if ((hspi->Init.Mode == SPI_MODE_MASTER) && (hspi->Init.Direction == SPI_DIRECTION_2LINES)) {
    hspi->State = HAL_SPI_STATE_BUSY_RX;
    // Call transmit-receive function to send Dummy data on Tx line and generate clock on CLK line */
    return HAL_SPI_TransmitReceive(hspi,pData,pData,Size,Timeout);
    }

  // Init tickstart for timeout management*/
  tickstart = HAL_GetTick();

  // Set the transaction information */
  hspi->State       = HAL_SPI_STATE_BUSY_RX;
  hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
  hspi->pRxBuffPtr  = (uint8_t *)pData;
  hspi->RxXferSize  = Size;
  hspi->RxXferCount = Size;

  // Init field not used in handle to zero */
  hspi->pTxBuffPtr  = (uint8_t *)NULL;
  hspi->TxXferSize  = 0U;
  hspi->TxXferCount = 0U;
  hspi->RxISR       = NULL;
  hspi->TxISR       = NULL;

  // Configure communication direction: 1Line */
  if(hspi->Init.Direction == SPI_DIRECTION_1LINE)
    SPI_1LINE_RX(hspi);

  // Check if the SPI is already enabled */
  if((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
    __HAL_SPI_ENABLE(hspi);

  // Receive data in 8 Bit mode */
  if (hspi->Init.DataSize == SPI_DATASIZE_8BIT) {
    // Transfer loop */
    while (hspi->RxXferCount > 0U) {
      // Check the RXNE flag */
      if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)) {
        // read the received data */
        (* (uint8_t *)pData)= *(__IO uint8_t *)&hspi->Instance->DR;
        pData += sizeof(uint8_t);
        hspi->RxXferCount--;
        }
      else {
        // Timeout management */
        if((Timeout == 0U) || ((Timeout != HAL_MAX_DELAY) && ((HAL_GetTick()-tickstart) >=  Timeout))) {
          errorcode = HAL_TIMEOUT;
          goto error;
          }
        }
      }
    }
  else {
    // Transfer loop */
    while(hspi->RxXferCount > 0U) {
      // Check the RXNE flag */
      if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)) {
        *((uint16_t*)pData) = hspi->Instance->DR;
        pData += sizeof(uint16_t);
        hspi->RxXferCount--;
        }
      else {
        // Timeout management */
        if((Timeout == 0U) || ((Timeout != HAL_MAX_DELAY) && ((HAL_GetTick()-tickstart) >=  Timeout))) {
          errorcode = HAL_TIMEOUT;
          goto error;
          }
        }
      }
    }

  // Check the end of the transaction */
  if ((hspi->Init.Mode == SPI_MODE_MASTER)&&((hspi->Init.Direction == SPI_DIRECTION_1LINE)||(hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY)))
    __HAL_SPI_DISABLE(hspi);
  if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
    errorcode = HAL_ERROR;

error :
  hspi->State = HAL_SPI_STATE_READY;
  return errorcode;
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_SPI_TransmitReceive (SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout) {

  uint32_t tmp = 0U, tmp1 = 0U;
  uint32_t tickstart = 0U;

  // Variable used to alternate Rx and Tx during transfer */
  uint32_t txallowed = 1U;
  HAL_StatusTypeDef errorcode = HAL_OK;

  // Init tickstart for timeout management*/
  tickstart = HAL_GetTick();

  tmp  = hspi->State;
  tmp1 = hspi->Init.Mode;

  if(!((tmp == HAL_SPI_STATE_READY) || \
    ((tmp1 == SPI_MODE_MASTER) && (hspi->Init.Direction == SPI_DIRECTION_2LINES) && (tmp == HAL_SPI_STATE_BUSY_RX)))) {
    errorcode = HAL_BUSY;
    goto error;
    }

  if((pTxData == NULL) || (pRxData == NULL) || (Size == 0))
  { errorcode = HAL_ERROR;
    goto error;
    }

  // Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
  if(hspi->State == HAL_SPI_STATE_READY)
    hspi->State = HAL_SPI_STATE_BUSY_TX_RX;

  /* Set the transaction information */
  hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
  hspi->pRxBuffPtr  = (uint8_t *)pRxData;
  hspi->RxXferCount = Size;
  hspi->RxXferSize  = Size;
  hspi->pTxBuffPtr  = (uint8_t *)pTxData;
  hspi->TxXferCount = Size;
  hspi->TxXferSize  = Size;

  //Init field not used in handle to zero */
  hspi->RxISR       = NULL;
  hspi->TxISR       = NULL;

  // Check if the SPI is already enabled */
  if((hspi->Instance->CR1 &SPI_CR1_SPE) != SPI_CR1_SPE)
    __HAL_SPI_ENABLE(hspi);

  // Transmit and Receive data in 16 Bit mode */
  if(hspi->Init.DataSize == SPI_DATASIZE_16BIT) {
    if((hspi->Init.Mode == SPI_MODE_SLAVE) || (hspi->TxXferCount == 0x01U)) {
      hspi->Instance->DR = *((uint16_t *)pTxData);
      pTxData += sizeof(uint16_t);
      hspi->TxXferCount--;
    }
    while ((hspi->TxXferCount > 0U) || (hspi->RxXferCount > 0U)) {
      // Check TXE flag */
      if(txallowed && (hspi->TxXferCount > 0U) && (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE))) {
        hspi->Instance->DR = *((uint16_t *)pTxData);
        pTxData += sizeof(uint16_t);
        hspi->TxXferCount--;
        // Next Data is a reception (Rx). Tx not allowed */
        txallowed = 0U;
        }

      // Check RXNE flag */
      if((hspi->RxXferCount > 0U) && (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE))) {
        *((uint16_t *)pRxData) = hspi->Instance->DR;
        pRxData += sizeof(uint16_t);
        hspi->RxXferCount--;
        // Next Data is a Transmission (Tx). Tx is allowed */
        txallowed = 1U;
        }
      if((Timeout != HAL_MAX_DELAY) && ((HAL_GetTick()-tickstart) >=  Timeout)) {
        errorcode = HAL_TIMEOUT;
        goto error;
        }
      }
    }

  // Transmit and Receive data in 8 Bit mode */
  else {
    if((hspi->Init.Mode == SPI_MODE_SLAVE) || (hspi->TxXferCount == 0x01U)) {
      *((__IO uint8_t*)&hspi->Instance->DR) = (*pTxData);
      pTxData += sizeof(uint8_t);
      hspi->TxXferCount--;
      }
    while((hspi->TxXferCount > 0U) || (hspi->RxXferCount > 0U)) {
      // check TXE flag */
      if(txallowed && (hspi->TxXferCount > 0U) && (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE))) {
        *(__IO uint8_t *)&hspi->Instance->DR = (*pTxData++);
        hspi->TxXferCount--;
        /* Next Data is a reception (Rx). Tx not allowed */
        txallowed = 0U;
        }

      // Wait until RXNE flag is reset */
      if((hspi->RxXferCount > 0U) && (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE))) {
        (*(uint8_t *)pRxData++) = hspi->Instance->DR;
        hspi->RxXferCount--;
        // Next Data is a Transmission (Tx). Tx is allowed */
        txallowed = 1U;
        }
      if((Timeout != HAL_MAX_DELAY) && ((HAL_GetTick()-tickstart) >=  Timeout)) {
        errorcode = HAL_TIMEOUT;
        goto error;
        }
      }
    }

  // Wait until TXE flag */
  if(SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_TXE, SET, Timeout, tickstart) != HAL_OK) {
    errorcode = HAL_TIMEOUT;
    goto error;
    }

  // Check Busy flag */
  if (SPI_CheckFlag_BSY(hspi, Timeout, tickstart) != HAL_OK) {
    errorcode = HAL_ERROR;
    hspi->ErrorCode = HAL_SPI_ERROR_FLAG;
    goto error;
    }

  // Clear overrun flag in 2 Lines communication mode because received is not read */
  if (hspi->Init.Direction == SPI_DIRECTION_2LINES)
    __HAL_SPI_CLEAR_OVRFLAG(hspi);

error :
  hspi->State = HAL_SPI_STATE_READY;
  return errorcode;
  }
//}}}

//{{{
/**
  * @brief  Transmit an amount of data in non-blocking mode with Interrupt.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_Transmit_IT (SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
  HAL_StatusTypeDef errorcode = HAL_OK;

  if((pData == NULL) || (Size == 0))
  {
    errorcode = HAL_ERROR;
    goto error;
  }

  if(hspi->State != HAL_SPI_STATE_READY)
  {
    errorcode = HAL_BUSY;
    goto error;
  }

  /* Set the transaction information */
  hspi->State       = HAL_SPI_STATE_BUSY_TX;
  hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
  hspi->pTxBuffPtr  = (uint8_t *)pData;
  hspi->TxXferSize  = Size;
  hspi->TxXferCount = Size;

  /* Init field not used in handle to zero */
  hspi->pRxBuffPtr  = (uint8_t *)NULL;
  hspi->RxXferSize  = 0U;
  hspi->RxXferCount = 0U;
  hspi->RxISR       = NULL;

  /* Set the function for IT treatment */
  if(hspi->Init.DataSize > SPI_DATASIZE_8BIT )
  {
    hspi->TxISR = SPI_TxISR_16BIT;
  }
  else
  {
    hspi->TxISR = SPI_TxISR_8BIT;
  }

  /* Configure communication direction : 1Line */
  if(hspi->Init.Direction == SPI_DIRECTION_1LINE)
  {
    SPI_1LINE_TX(hspi);
  }

  if (hspi->Init.Direction == SPI_DIRECTION_2LINES)
  {
    /* Enable TXE interrupt */
    __HAL_SPI_ENABLE_IT(hspi, (SPI_IT_TXE));
  }
  else
  {
    /* Enable TXE and ERR interrupt */
    __HAL_SPI_ENABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_ERR));
  }

  /* Check if the SPI is already enabled */
  if((hspi->Instance->CR1 &SPI_CR1_SPE) != SPI_CR1_SPE)
  {
    /* Enable SPI peripheral */
    __HAL_SPI_ENABLE(hspi);
  }

error :
  return errorcode;
}
//}}}
//{{{
/**
  * @brief  Receive an amount of data in non-blocking mode with Interrupt.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_Receive_IT (SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
  HAL_StatusTypeDef errorcode = HAL_OK;

  if((hspi->Init.Direction == SPI_DIRECTION_2LINES) && (hspi->Init.Mode == SPI_MODE_MASTER))
  {
     hspi->State = HAL_SPI_STATE_BUSY_RX;
     /* Call transmit-receive function to send Dummy data on Tx line and generate clock on CLK line */
     return HAL_SPI_TransmitReceive_IT(hspi, pData, pData, Size);
  }

  if(hspi->State != HAL_SPI_STATE_READY)
  {
    errorcode = HAL_BUSY;
    goto error;
  }

  if((pData == NULL) || (Size == 0))
  {
    errorcode = HAL_ERROR;
    goto error;
  }

  /* Set the transaction information */
  hspi->State       = HAL_SPI_STATE_BUSY_RX;
  hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
  hspi->pRxBuffPtr  = (uint8_t *)pData;
  hspi->RxXferSize  = Size;
  hspi->RxXferCount = Size;

  /* Init field not used in handle to zero */
  hspi->pTxBuffPtr  = (uint8_t *)NULL;
  hspi->TxXferSize  = 0U;
  hspi->TxXferCount = 0U;
  hspi->TxISR       = NULL;

  /* Set the function for IT treatment */
  if(hspi->Init.DataSize > SPI_DATASIZE_8BIT )
  {
    hspi->RxISR = SPI_RxISR_16BIT;
  }
  else
  {
    hspi->RxISR = SPI_RxISR_8BIT;
  }

  /* Configure communication direction : 1Line */
  if(hspi->Init.Direction == SPI_DIRECTION_1LINE)
  {
    SPI_1LINE_RX(hspi);
  }


  /* Enable TXE and ERR interrupt */
  __HAL_SPI_ENABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));

  /* Note : The SPI must be enabled after unlocking current process
            to avoid the risk of SPI interrupt handle execution before current
            process unlock */

  /* Check if the SPI is already enabled */
  if((hspi->Instance->CR1 &SPI_CR1_SPE) != SPI_CR1_SPE)
  {
    /* Enable SPI peripheral */
    __HAL_SPI_ENABLE(hspi);
  }

error :
  /* Process Unlocked */
  return errorcode;
}
//}}}
//{{{
/**
  * @brief  Transmit and Receive an amount of data in non-blocking mode with Interrupt.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pTxData pointer to transmission data buffer
  * @param  pRxData pointer to reception data buffer
  * @param  Size amount of data to be sent and received
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT (SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
  uint32_t tmp = 0U, tmp1 = 0U;
  HAL_StatusTypeDef errorcode = HAL_OK;

  tmp  = hspi->State;
  tmp1 = hspi->Init.Mode;

  if(!((tmp == HAL_SPI_STATE_READY) || \
    ((tmp1 == SPI_MODE_MASTER) && (hspi->Init.Direction == SPI_DIRECTION_2LINES) && (tmp == HAL_SPI_STATE_BUSY_RX))))
  {
    errorcode = HAL_BUSY;
    goto error;
  }

  if((pTxData == NULL ) || (pRxData == NULL ) || (Size == 0))
  {
    errorcode = HAL_ERROR;
    goto error;
  }

  /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
  if(hspi->State == HAL_SPI_STATE_READY)
  {
    hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
  }

  /* Set the transaction information */
  hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
  hspi->pTxBuffPtr  = (uint8_t *)pTxData;
  hspi->TxXferSize  = Size;
  hspi->TxXferCount = Size;
  hspi->pRxBuffPtr  = (uint8_t *)pRxData;
  hspi->RxXferSize  = Size;
  hspi->RxXferCount = Size;

  /* Set the function for IT treatment */
  if(hspi->Init.DataSize > SPI_DATASIZE_8BIT )
  {
    hspi->RxISR     = SPI_2linesRxISR_16BIT;
    hspi->TxISR     = SPI_2linesTxISR_16BIT;
  }
  else
  {
    hspi->RxISR     = SPI_2linesRxISR_8BIT;
    hspi->TxISR     = SPI_2linesTxISR_8BIT;
  }

  /* Enable TXE, RXNE and ERR interrupt */
  __HAL_SPI_ENABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

  /* Check if the SPI is already enabled */
  if((hspi->Instance->CR1 &SPI_CR1_SPE) != SPI_CR1_SPE)
  {
    /* Enable SPI peripheral */
    __HAL_SPI_ENABLE(hspi);
  }

error :
  /* Process Unlocked */
  return errorcode;
}
//}}}

//{{{
HAL_StatusTypeDef HAL_SPI_Transmit_DMA (SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size) {

  HAL_StatusTypeDef errorcode = HAL_OK;

  /* Set the transaction information */
  hspi->State       = HAL_SPI_STATE_BUSY_TX;
  hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
  hspi->pTxBuffPtr  = (uint8_t *)pData;
  hspi->TxXferSize  = Size;
  hspi->TxXferCount = Size;

  /* Init field not used in handle to zero */
  hspi->pRxBuffPtr  = (uint8_t *)NULL;
  hspi->TxISR       = NULL;
  hspi->RxISR       = NULL;
  hspi->RxXferSize  = 0U;
  hspi->RxXferCount = 0U;

  /* Configure communication direction : 1Line */
  if(hspi->Init.Direction == SPI_DIRECTION_1LINE)
    SPI_1LINE_TX(hspi);

  /* Set the SPI TxDMA Half transfer complete callback */
  hspi->hdmatx->XferHalfCpltCallback = SPI_DMAHalfTransmitCplt;

  /* Set the SPI TxDMA transfer complete callback */
  hspi->hdmatx->XferCpltCallback = SPI_DMATransmitCplt;

  /* Set the DMA error callback */
  hspi->hdmatx->XferErrorCallback = SPI_DMAError;

  /* Set the DMA AbortCpltCallback */
  hspi->hdmatx->XferAbortCallback = NULL;

  /* Enable the Tx DMA Stream */
  HAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)hspi->pTxBuffPtr, (uint32_t)&hspi->Instance->DR, hspi->TxXferCount);

  /* Check if the SPI is already enabled */
  if((hspi->Instance->CR1 &SPI_CR1_SPE) != SPI_CR1_SPE)
    __HAL_SPI_ENABLE(hspi);

  /* Enable the SPI Error Interrupt Bit */
  SET_BIT(hspi->Instance->CR2, SPI_CR2_ERRIE);

  /* Enable Tx DMA Request */
  SET_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN);

error :
  /* Process Unlocked */
  return errorcode;
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_SPI_Receive_DMA (SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
  HAL_StatusTypeDef errorcode = HAL_OK;

  if((hspi->Init.Direction == SPI_DIRECTION_2LINES)&&(hspi->Init.Mode == SPI_MODE_MASTER)) {
     hspi->State = HAL_SPI_STATE_BUSY_RX;
     /* Call transmit-receive function to send Dummy data on Tx line and generate clock on CLK line */
     return HAL_SPI_TransmitReceive_DMA(hspi, pData, pData, Size);
    }

  /* Set the transaction information */
  hspi->State       = HAL_SPI_STATE_BUSY_RX;
  hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
  hspi->pRxBuffPtr  = (uint8_t *)pData;
  hspi->RxXferSize  = Size;
  hspi->RxXferCount = Size;

  /*Init field not used in handle to zero */
  hspi->RxISR       = NULL;
  hspi->TxISR       = NULL;
  hspi->TxXferSize  = 0U;
  hspi->TxXferCount = 0U;

  /* Configure communication direction : 1Line */
  if(hspi->Init.Direction == SPI_DIRECTION_1LINE)
    SPI_1LINE_RX(hspi);

  /* Set the SPI RxDMA Half transfer complete callback */
  hspi->hdmarx->XferHalfCpltCallback = SPI_DMAHalfReceiveCplt;

  /* Set the SPI Rx DMA transfer complete callback */
  hspi->hdmarx->XferCpltCallback = SPI_DMAReceiveCplt;

  /* Set the DMA error callback */
  hspi->hdmarx->XferErrorCallback = SPI_DMAError;

 /* Set the DMA AbortCpltCallback */
  hspi->hdmarx->XferAbortCallback = NULL;

  /* Enable the Rx DMA Stream */
  HAL_DMA_Start_IT(hspi->hdmarx, (uint32_t)&hspi->Instance->DR, (uint32_t)hspi->pRxBuffPtr, hspi->RxXferCount);

  /* Check if the SPI is already enabled */
  if((hspi->Instance->CR1 &SPI_CR1_SPE) != SPI_CR1_SPE)
    __HAL_SPI_ENABLE(hspi);

  /* Enable the SPI Error Interrupt Bit */
  SET_BIT(hspi->Instance->CR2, SPI_CR2_ERRIE);

  /* Enable Rx DMA Request */
  SET_BIT(hspi->Instance->CR2, SPI_CR2_RXDMAEN);

error:
  /* Process Unlocked */
  return errorcode;
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA (SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size) {

  uint32_t tmp = 0U, tmp1 = 0U;
  HAL_StatusTypeDef errorcode = HAL_OK;

  tmp  = hspi->State;
  tmp1 = hspi->Init.Mode;

  /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
  if(hspi->State == HAL_SPI_STATE_READY)
    hspi->State = HAL_SPI_STATE_BUSY_TX_RX;

  /* Set the transaction information */
  hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
  hspi->pTxBuffPtr  = (uint8_t*)pTxData;
  hspi->TxXferSize  = Size;
  hspi->TxXferCount = Size;
  hspi->pRxBuffPtr  = (uint8_t*)pRxData;
  hspi->RxXferSize  = Size;
  hspi->RxXferCount = Size;

  /* Init field not used in handle to zero */
  hspi->RxISR       = NULL;
  hspi->TxISR       = NULL;

  /* Check if we are in Rx only or in Rx/Tx Mode and configure the DMA transfer complete callback */
  if(hspi->State == HAL_SPI_STATE_BUSY_RX) {
    /* Set the SPI Rx DMA Half transfer complete callback */
    hspi->hdmarx->XferHalfCpltCallback = SPI_DMAHalfReceiveCplt;
    hspi->hdmarx->XferCpltCallback     = SPI_DMAReceiveCplt;
  }
  else {
    /* Set the SPI Tx/Rx DMA Half transfer complete callback */
    hspi->hdmarx->XferHalfCpltCallback = SPI_DMAHalfTransmitReceiveCplt;
    hspi->hdmarx->XferCpltCallback     = SPI_DMATransmitReceiveCplt;
    }

  /* Set the DMA error callback */
  hspi->hdmarx->XferErrorCallback = SPI_DMAError;

  /* Set the DMA AbortCpltCallback */
  hspi->hdmarx->XferAbortCallback = NULL;

  /* Enable the Rx DMA Stream */
  HAL_DMA_Start_IT(hspi->hdmarx, (uint32_t)&hspi->Instance->DR, (uint32_t)hspi->pRxBuffPtr, hspi->RxXferCount);

  /* Enable Rx DMA Request */
  SET_BIT(hspi->Instance->CR2, SPI_CR2_RXDMAEN);

  /* Set the SPI Tx DMA transfer complete callback as NULL because the communication closing
  is performed in DMA reception complete callback  */
  hspi->hdmatx->XferHalfCpltCallback = NULL;
  hspi->hdmatx->XferCpltCallback     = NULL;
  hspi->hdmatx->XferErrorCallback    = NULL;
  hspi->hdmatx->XferAbortCallback    = NULL;

  /* Enable the Tx DMA Stream */
  HAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)hspi->pTxBuffPtr, (uint32_t)&hspi->Instance->DR, hspi->TxXferCount);

  /* Check if the SPI is already enabled */
  if((hspi->Instance->CR1 &SPI_CR1_SPE) != SPI_CR1_SPE)
    __HAL_SPI_ENABLE(hspi);
  /* Enable the SPI Error Interrupt Bit */
  SET_BIT(hspi->Instance->CR2, SPI_CR2_ERRIE);

  /* Enable Tx DMA Request */
  SET_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN);

error :
  return errorcode;
  }
//}}}

//{{{
HAL_StatusTypeDef HAL_SPI_Abort (SPI_HandleTypeDef *hspi) {

  __IO uint32_t count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);

  /* Disable TXEIE, RXNEIE and ERRIE(mode fault event, overrun error, TI frame error) interrupts */
  if(HAL_IS_BIT_SET(hspi->Instance->CR2, SPI_CR2_TXEIE))
    hspi->TxISR = SPI_AbortTx_ISR;

  if(HAL_IS_BIT_SET(hspi->Instance->CR2, SPI_CR2_RXNEIE))
    hspi->RxISR = SPI_AbortRx_ISR;

  /* Clear ERRIE interrupts in case of DMA Mode */
  CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_ERRIE);

  /* Disable the SPI DMA Tx or SPI DMA Rx request if enabled */
  if ((HAL_IS_BIT_SET(hspi->Instance->CR2, SPI_CR2_TXDMAEN)) || (HAL_IS_BIT_SET(hspi->Instance->CR2, SPI_CR2_RXDMAEN))) {
    /* Abort the SPI DMA Tx channel : use blocking DMA Abort API (no callback) */
    if(hspi->hdmatx != NULL) {
      /* Set the SPI DMA Abort callback :
      will lead to call HAL_SPI_AbortCpltCallback() at end of DMA abort procedure */
      hspi->hdmatx->XferAbortCallback = NULL;

      /* Abort DMA Tx Handle linked to SPI Peripheral */
      HAL_DMA_Abort(hspi->hdmatx);

      /* Disable Tx DMA Request */
      CLEAR_BIT(hspi->Instance->CR2, (SPI_CR2_TXDMAEN));

      /* Wait until TXE flag is set */
      do {
        if(count-- == 0U) {
          SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
          break;
          }
        } while((hspi->Instance->SR & SPI_FLAG_TXE) == RESET);
      }

    /* Abort the SPI DMA Rx channel : use blocking DMA Abort API (no callback) */
    if(hspi->hdmarx != NULL) {
      /* Set the SPI DMA Abort callback :
      will lead to call HAL_SPI_AbortCpltCallback() at end of DMA abort procedure */
      hspi->hdmarx->XferAbortCallback = NULL;

      /* Abort DMA Rx Handle linked to SPI Peripheral */
      HAL_DMA_Abort(hspi->hdmarx);

      /* Disable peripheral */
      __HAL_SPI_DISABLE(hspi);

      /* Disable Rx DMA Request */
      CLEAR_BIT(hspi->Instance->CR2, (SPI_CR2_RXDMAEN));

    }
  }
  /* Reset Tx and Rx transfer counters */
  hspi->RxXferCount = 0U;
  hspi->TxXferCount = 0U;

  /* Reset errorCode */
  hspi->ErrorCode = HAL_SPI_ERROR_NONE;

  /* Clear the Error flags in the SR register */
  __HAL_SPI_CLEAR_OVRFLAG(hspi);
  __HAL_SPI_CLEAR_FREFLAG(hspi);

  /* Restore hspi->state to ready */
  hspi->State = HAL_SPI_STATE_READY;

  return HAL_OK;
}
//}}}
//{{{
HAL_StatusTypeDef HAL_SPI_Abort_IT (SPI_HandleTypeDef *hspi) {

  uint32_t abortcplt;

  /* Change Rx and Tx Irq Handler to Disable TXEIE, RXNEIE and ERRIE interrupts */
  if(HAL_IS_BIT_SET(hspi->Instance->CR2, SPI_CR2_TXEIE))
    hspi->TxISR = SPI_AbortTx_ISR;

  if(HAL_IS_BIT_SET(hspi->Instance->CR2, SPI_CR2_RXNEIE))
    hspi->RxISR = SPI_AbortRx_ISR;

  /* Clear ERRIE interrupts in case of DMA Mode */
  CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_ERRIE);

  abortcplt = 1U;

  /* If DMA Tx and/or DMA Rx Handles are associated to SPI Handle, DMA Abort complete callbacks should be initialised
     before any call to DMA Abort functions */
  /* DMA Tx Handle is valid */
  if(hspi->hdmatx != NULL) {
    /* Set DMA Abort Complete callback if UART DMA Tx request if enabled.
       Otherwise, set it to NULL */
    if(HAL_IS_BIT_SET(hspi->Instance->CR2, SPI_CR2_TXDMAEN))
      hspi->hdmatx->XferAbortCallback = SPI_DMATxAbortCallback;
    else
      hspi->hdmatx->XferAbortCallback = NULL;
    }

  /* DMA Rx Handle is valid */
  if(hspi->hdmarx != NULL) {
    /* Set DMA Abort Complete callback if UART DMA Rx request if enabled.
       Otherwise, set it to NULL */
    if(HAL_IS_BIT_SET(hspi->Instance->CR2, SPI_CR2_RXDMAEN))
      hspi->hdmarx->XferAbortCallback = SPI_DMARxAbortCallback;
    else
      hspi->hdmarx->XferAbortCallback = NULL;
    }

  /* Disable the SPI DMA Tx or the SPI Rx request if enabled */
  if((HAL_IS_BIT_SET(hspi->Instance->CR2, SPI_CR2_TXDMAEN)) && (HAL_IS_BIT_SET(hspi->Instance->CR2, SPI_CR2_RXDMAEN))) {
    /* Abort the SPI DMA Tx channel */
    if(hspi->hdmatx != NULL) {
      /* Abort DMA Tx Handle linked to SPI Peripheral */
      if(HAL_DMA_Abort_IT(hspi->hdmatx) != HAL_OK)
        hspi->hdmatx->XferAbortCallback = NULL;
      else
        abortcplt = 0U;
      }

    /* Abort the SPI DMA Rx channel */
    if(hspi->hdmarx != NULL) {
      /* Abort DMA Rx Handle linked to SPI Peripheral */
      if(HAL_DMA_Abort_IT(hspi->hdmarx)!=  HAL_OK) {
        hspi->hdmarx->XferAbortCallback = NULL;
        abortcplt = 1U;
        }
      else
        abortcplt = 0U;
      }
    }

  /* Disable the SPI DMA Tx or the SPI Rx request if enabled */
  if (HAL_IS_BIT_SET(hspi->Instance->CR2, SPI_CR2_TXDMAEN)) {
    /* Abort the SPI DMA Tx channel */
    if(hspi->hdmatx != NULL) {
      /* Abort DMA Tx Handle linked to SPI Peripheral */
      if(HAL_DMA_Abort_IT(hspi->hdmatx) != HAL_OK)
        hspi->hdmatx->XferAbortCallback = NULL;
      else
        abortcplt = 0U;
      }
    }

  /* Disable the SPI DMA Tx or the SPI Rx request if enabled */
  if (HAL_IS_BIT_SET(hspi->Instance->CR2, SPI_CR2_RXDMAEN)) {
    /* Abort the SPI DMA Rx channel */
    if(hspi->hdmarx != NULL) {
      /* Abort DMA Rx Handle linked to SPI Peripheral */
      if(HAL_DMA_Abort_IT(hspi->hdmarx)!=  HAL_OK)
        hspi->hdmarx->XferAbortCallback = NULL;
      else
        abortcplt = 0U;
      }
    }

  if(abortcplt == 1U) {
    /* Reset Tx and Rx transfer counters */
    hspi->RxXferCount = 0U;
    hspi->TxXferCount = 0U;

    /* Reset errorCode */
    hspi->ErrorCode = HAL_SPI_ERROR_NONE;

    /* Clear the Error flags in the SR register */
    __HAL_SPI_CLEAR_OVRFLAG(hspi);
    __HAL_SPI_CLEAR_FREFLAG(hspi);

    /* Restore hspi->State to Ready */
    hspi->State = HAL_SPI_STATE_READY;

    /* As no DMA to be aborted, call directly user Abort complete callback */
    HAL_SPI_AbortCpltCallback(hspi);
    }
  return HAL_OK;
}
//}}}
//{{{
HAL_StatusTypeDef HAL_SPI_DMAPause(SPI_HandleTypeDef *hspi) {

  /* Disable the SPI DMA Tx & Rx requests */
  CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
  return HAL_OK;
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_SPI_DMAResume (SPI_HandleTypeDef *hspi) {

  /* Enable the SPI DMA Tx & Rx requests */
  SET_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
  return HAL_OK;
  }
//}}}
//{{{
HAL_StatusTypeDef HAL_SPI_DMAStop (SPI_HandleTypeDef *hspi) {

  /* Abort the SPI DMA tx Stream */
  if(hspi->hdmatx != NULL)
    HAL_DMA_Abort(hspi->hdmatx);

  /* Abort the SPI DMA rx Stream */
  if(hspi->hdmarx != NULL)
    HAL_DMA_Abort(hspi->hdmarx);

  /* Disable the SPI DMA Tx & Rx requests */
  CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
  hspi->State = HAL_SPI_STATE_READY;
  return HAL_OK;
  }
//}}}

//{{{
void HAL_SPI_IRQHandler (SPI_HandleTypeDef *hspi) {

  uint32_t itsource = hspi->Instance->CR2;
  uint32_t itflag   = hspi->Instance->SR;

  // SPI in mode Receiver
  if (((itflag & SPI_FLAG_OVR) == RESET) &&
     ((itflag & SPI_FLAG_RXNE) != RESET) && ((itsource & SPI_IT_RXNE) != RESET)) {
    hspi->RxISR(hspi);
    return;
    }

  // SPI in mode Transmitter
  if (((itflag & SPI_FLAG_TXE) != RESET) && ((itsource & SPI_IT_TXE) != RESET)) {
    hspi->TxISR(hspi);
    return;
    }

  // SPI in Error Treatment
  if(((itflag & (SPI_FLAG_MODF | SPI_FLAG_OVR | SPI_FLAG_FRE)) != RESET) && ((itsource & SPI_IT_ERR) != RESET)) {
    // SPI Overrun error interrupt occurred
    if((itflag & SPI_FLAG_OVR) != RESET) {
      if(hspi->State != HAL_SPI_STATE_BUSY_TX) {
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_OVR);
        __HAL_SPI_CLEAR_OVRFLAG(hspi);
        }
      else {
        __HAL_SPI_CLEAR_OVRFLAG(hspi);
        return;
        }
      }

    // SPI Mode Fault error interrupt occurred
    if ((itflag & SPI_FLAG_MODF) != RESET) {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_MODF);
      __HAL_SPI_CLEAR_MODFFLAG(hspi);
      }

    // SPI Frame error interrupt occurred
    if ((itflag & SPI_FLAG_FRE) != RESET) {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FRE);
      __HAL_SPI_CLEAR_FREFLAG(hspi);
      }

    if (hspi->ErrorCode != HAL_SPI_ERROR_NONE) {
      // Disable all interrupts */
      __HAL_SPI_DISABLE_IT(hspi, SPI_IT_RXNE | SPI_IT_TXE | SPI_IT_ERR);

      hspi->State = HAL_SPI_STATE_READY;
      // Disable the SPI DMA requests if enabled */
      if ((HAL_IS_BIT_SET(itsource, SPI_CR2_TXDMAEN))||(HAL_IS_BIT_SET(itsource, SPI_CR2_RXDMAEN))) {
        CLEAR_BIT(hspi->Instance->CR2, (SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN));

        /* Abort the SPI DMA Rx channel */
        if(hspi->hdmarx != NULL) {
          /* Set the SPI DMA Abort callback :
          will lead to call HAL_SPI_ErrorCallback() at end of DMA abort procedure */
          hspi->hdmarx->XferAbortCallback = SPI_DMAAbortOnError;
          HAL_DMA_Abort_IT(hspi->hdmarx);
          }
        /* Abort the SPI DMA Tx channel */
        if(hspi->hdmatx != NULL) {
          /* Set the SPI DMA Abort callback :
          will lead to call HAL_SPI_ErrorCallback() at end of DMA abort procedure */
          hspi->hdmatx->XferAbortCallback = SPI_DMAAbortOnError;
          HAL_DMA_Abort_IT(hspi->hdmatx);
          }
        }
      else
        HAL_SPI_ErrorCallback(hspi);
      }
    return;
    }
  }
//}}}
