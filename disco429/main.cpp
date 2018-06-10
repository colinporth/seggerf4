// main.cpp
//{{{  includes
#include "cLcd.h"
#include "ps2.h"

#include "stm32f429i_discovery.h"

#include "fatFs.h"
#include "diskio.h"

#include "cDecodePic.h"
//}}}
//{{{  sdram defines
#define SDRAM_BANK1_ADDR  ((uint32_t)0xC0000000)
#define SDRAM_BANK1_LEN   ((uint32_t)0x01000000)
#define SDRAM_BANK2_ADDR  ((uint32_t)0xD0000000)
#define SDRAM_BANK2_LEN   ((uint32_t)0x00800000)

#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)
//}}}
//{{{  etm defines
#define ITM_LAR_KEY   0xC5ACCE55
//{{{  ETM_Type       0xE0041000
// http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ihi0014q/Chdfiagc.html
// http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ddi0337i/CHDBGEED.html
// - LAR:       Allow write access to other ETM registers
// - CR:        Enable/disable tracing
// - TRIGGER:   Select tracing trigger event
// - SR:        Current status
// - TECR1      Select areas of code where to enable trace
// - TECR2      Select comparator for trace enable
// - TEEVR      Select event for trace enable

typedef struct {
    __IO uint32_t CR;           /* Main Control Register */
    __IO uint32_t CCR;          /* Configuration Code Register */
    __IO uint32_t TRIGGER;      /* Trigger Event */
    __IO uint32_t ASICCR;       /* ASIC Control Register */
    __IO uint32_t SR;           /* ETM Status Register */
    __IO uint32_t SCR;          /* System Configuration Register */
    __IO uint32_t TSSCR;        /* TraceEnable Start/Stop Control Register */
    __IO uint32_t TECR2;        /* TraceEnable Control 2 */
    __IO uint32_t TEEVR;        /* TraceEnable Event Register */
    __IO uint32_t TECR1;        /* TraceEnable Control 1 */
    __IO uint32_t FFRR;         /* FIFOFULL Region Register */
    __IO uint32_t FFLR;         /* FIFOFULL Level Register */
    __IO uint32_t VDEVR;        /* ViewData Event Register */
    __IO uint32_t VDCR1;        /* ViewData Control 1 */
    __IO uint32_t VDCR2;        /* ViewData Control 2 */
    __IO uint32_t VDCR3;        /* ViewData Control 3 */
    __IO uint32_t ACVR[16];     /* Address Comparator Value Registers */
    __IO uint32_t ACTR[16];     /* Address Comparator Access Type Registers */
    __IO uint32_t DCVR[16];     /* Data Comparator Value Registers */
    __IO uint32_t DCMR[16];     /* Data Comparator Mask Registers */
    __IO uint32_t CNTRLDVR[4];  /* Counter Reload Value Registers */
    __IO uint32_t CNTENR[4];    /* Counter Enable Registers */
    __IO uint32_t CNTRLDEVR[4]; /* Counter Reload Event Registers */
    __IO uint32_t CNTVR[4];     /* Counter Value Registers */
    __IO uint32_t SQabEVR[6];   /* Sequencer State Transition Event Registers */
    __IO uint32_t RESERVED0;
    __IO uint32_t SQR;          /* Current Sequencer State Register */
    __IO uint32_t EXTOUTEVR[4]; /* External Output Event Registers */
    __IO uint32_t CIDCVR[3];    /* Context ID Comparator Value Registers */
    __IO uint32_t CIDCMR;       /* Context ID Comparator Mask Register */
    __IO uint32_t IMPL[8];      /* Implementation specific registers */
    __IO uint32_t SYNCFR;       /* Synchronization Frequency Register */
    __IO uint32_t IDR;          /* ETM ID Register */
    __IO uint32_t CCER;         /* Configuration Code Extension Register */
    __IO uint32_t EXTINSELR;    /* Extended External Input Selection Register */
    __IO uint32_t TESSEICR;     /* TraceEnable Start/Stop EmbeddedICE Control Register */
    __IO uint32_t EIBCR;        /* EmbeddedICE Behavior Control Register */
    __IO uint32_t TSEVR;        /* Timestamp Event Register, ETMv3.5 */
    __IO uint32_t AUXCR;        /* Auxiliary Control Register, ETMv3.5 */
    __IO uint32_t TRACEIDR;     /* CoreSight Trace ID Register */
    __IO uint32_t RESERVED1;
    __IO uint32_t IDR2;         /* ETM ID Register 2 */
    __IO uint32_t RESERVED2[13];
    __IO uint32_t VMIDCVR;      /* VMID Comparator Value Register, ETMv3.5 */
    __IO uint32_t RESERVED3[47];
    __IO uint32_t OSLAR;        /* OS Lock Access Register */
    __IO uint32_t OSLSR;        /* OS Lock Status Register */
    __IO uint32_t OSSRR;        /* OS Save and Restore Register */
    __IO uint32_t RESERVED4;
    __IO uint32_t PDCR;         /* Power Down Control Register, ETMv3.5 */
    __IO uint32_t PDSR;         /* Device Power-Down Status Register */
    __IO uint32_t RESERVED5[762];
    __IO uint32_t ITCTRL;       /* Integration Mode Control Register */
    __IO uint32_t RESERVED6[39];
    __IO uint32_t CLAIMSET;     /* Claim Tag Set Register */
    __IO uint32_t CLAIMCLR;     /* Claim Tag Clear Register */
    __IO uint32_t RESERVED7[2];
    __IO uint32_t LAR;          /* Lock Access Register */
    __IO uint32_t LSR;          /* Lock Status Register */
    __IO uint32_t AUTHSTATUS;   /* Authentication Status Register */
    __IO uint32_t RESERVED8[3];
    __IO uint32_t DEVID;        /* CoreSight Device Configuration Register */
    __IO uint32_t DEVTYPE;      /* CoreSight Device Type Register */
    __IO uint32_t PIDR4;        /* Peripheral ID4 */
    __IO uint32_t PIDR5;        /* Peripheral ID5 */
    __IO uint32_t PIDR6;        /* Peripheral ID6 */
    __IO uint32_t PIDR7;        /* Peripheral ID7 */
    __IO uint32_t PIDR0;        /* Peripheral ID0 */
    __IO uint32_t PIDR1;        /* Peripheral ID1 */
    __IO uint32_t PIDR2;        /* Peripheral ID2 */
    __IO uint32_t PIDR3;        /* Peripheral ID3 */
    __IO uint32_t CIDR0;        /* Component ID0 */
    __IO uint32_t CIDR1;        /* Component ID1 */
    __IO uint32_t CIDR2;        /* Component ID2 */
    __IO uint32_t CIDR3;        /* Component ID3 */
  } ETM_Type;
//}}}

#define ETM_BASE 0xE0041000
#define ETM ((ETM_Type*)ETM_BASE)

//{{{  ETM bit defines
#define ETM_CR_POWERDOWN                0x00000001
#define ETM_CR_MONITORCPRT              0x00000002
#define ETM_CR_TRACE_DATA               0x00000004
#define ETM_CR_TRACE_ADDR               0x00000008
#define ETM_CR_PORTSIZE_1BIT            0x00200000
#define ETM_CR_PORTSIZE_2BIT            0x00200010
#define ETM_CR_PORTSIZE_4BIT            0x00000000
#define ETM_CR_PORTSIZE_8BIT            0x00000010
#define ETM_CR_PORTSIZE_16BIT           0x00000020
#define ETM_CR_STALL_PROCESSOR          0x00000080
#define ETM_CR_BRANCH_OUTPUT            0x00000100
#define ETM_CR_DEBUGREQ                 0x00000200
#define ETM_CR_PROGRAMMING              0x00000400
#define ETM_CR_ETMEN                    0x00000800
#define ETM_CR_CYCLETRACE               0x00001000
#define ETM_CR_CONTEXTID_8BIT           0x00004000
#define ETM_CR_CONTEXTID_16BIT          0x00008000
#define ETM_CR_CONTEXTID_32BIT          0x0000C000
#define ETM_CR_CONTEXTID_8BIT           0x00004000
#define ETM_CR_PORTMODE_ONCHIP          0x00000000
#define ETM_CR_PORTMODE_2_1             0x00010000
#define ETM_CR_PORTMODE_IMPL            0x00030000
#define ETM_CR_PORTMODE_1_1             0x00002000
#define ETM_CR_PORTMODE_1_2             0x00022000
#define ETM_CR_PORTMODE_1_3             0x00012000
#define ETM_CR_PORTMODE_1_4             0x00032000
#define ETM_CR_SUPPRESS_DATA            0x00040000
#define ETM_CR_FILTER_CPRT              0x00080000
#define ETM_CR_DATA_ONLY                0x00100000
#define ETM_CR_BLOCK_DEBUGGER           0x00400000
#define ETM_CR_BLOCK_SOFTWARE           0x00800000
#define ETM_CR_ACCESS                   0x01000000
#define ETM_CR_PROCSEL_Pos              25
#define ETM_CR_TIMESTAMP                0x10000000
#define ETM_CR_VMID                     0x40000000

#define ETM_SR_PROGSTATUS               0x00000002
#define ETM_SR_TRIGSTATUS               0x00000008

#define ETM_TECR1_EXCLUDE               0x01000000
#define ETM_TECR1_TSSEN                 0x02000000

#define ETM_FFRR_EXCLUDE                0x01000000

#define ETM_LAR_KEY                     0xC5ACCE55

#define ETM_TraceMode() ETM->CR &= ~ETM_CR_PROGRAMMING
#define ETM_SetupMode() ETM->CR |= ETM_CR_PROGRAMMING
//}}}
//}}}

cLcd* lcd = nullptr;

UART_HandleTypeDef DebugUartHandle;
//{{{
void initDebugUart() {

  __HAL_RCC_USART1_FORCE_RESET();
  __HAL_RCC_USART1_RELEASE_RESET();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  // PA9 - USART1 tx pin configuration
  GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.Pin       = GPIO_PIN_9;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  // PA10 - USART1 rx pin configuration
  //GPIO_InitStruct.Pin = GPIO_PIN_10;
  //HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  // 8 Bits, One Stop bit, Parity = None, RTS,CTS flow control
  DebugUartHandle.Instance   = USART1;
  DebugUartHandle.Init.BaudRate   = 230400;
  DebugUartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  DebugUartHandle.Init.StopBits   = UART_STOPBITS_1;
  DebugUartHandle.Init.Parity     = UART_PARITY_NONE;
  DebugUartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  DebugUartHandle.Init.Mode       = UART_MODE_TX;
  //DebugUartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init (&DebugUartHandle);
  }
//}}}

//{{{  system
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
uint32_t SystemCoreClock = 16000000;
int globalCounter = 0;

//{{{
void configureSWO() {

  DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN; // Enable IO trace pins
  if (!(DBGMCU->CR & DBGMCU_CR_TRACE_IOEN))
    return;

  // default 64k baud rate
  // SWOSpeed in Hz, note that cpuCoreFreqHz is expected to be match the CPU core clock
  //uint32_t SWOSpeed = 64000;
  uint32_t SWOSpeed = 2000000;
  uint32_t cpuCoreFreqHz = 180000000;
  uint32_t SWOPrescaler = (cpuCoreFreqHz / SWOSpeed) - 1;

  // enable trace in core debug
  CoreDebug->DEMCR = CoreDebug_DEMCR_TRCENA_Msk;

  // TPI SPPR - Selected PIN Protocol Register =  protocol for trace output 2: SWO NRZ, 1: SWO Manchester encoding
  TPI->SPPR = 0x00000002;

  // TPI Async Clock Prescaler Register =  Scale the baud rate of the asynchronous output
  TPI->ACPR = SWOPrescaler;

  // ITM Lock Access Register = ITM_LAR_KEY = C5ACCE55 enables write access to Control Register 0xE00 :: 0xFFC
  ITM->LAR = ITM_LAR_KEY;

  // ITM Trace Control Register
  ITM->TCR = ITM_TCR_TraceBusID_Msk | ITM_TCR_SWOENA_Msk | ITM_TCR_SYNCENA_Msk | ITM_TCR_ITMENA_Msk;

  // ITM Trace Privilege Register =
  ITM->TPR = ITM_TPR_PRIVMASK_Msk;

  // ITM Trace Enable Register =  Enabled tracing on stimulus ports. One bit per stimulus port
  ITM->TER = 0xFFFFFFFF;

  // DWT Control Register =
  //*((volatile unsigned*)(ITM_BASE + 0x01000)) = 0x400003FE;
  DWT->CTRL = 0x400003FE;

  // TPI Formatter and Flush Control Register =
  //*((volatile unsigned*)(ITM_BASE + 0x40304)) = 0x00000100;
  TPI->FFCR = 0x00000100;
  }
//}}}
//{{{
void configureDtrace1() {

  __HAL_RCC_GPIOE_CLK_ENABLE();
  //*((uint32_t*)(0x40023830)) |= 0x00000010;
  *((uint32_t*)(0x40021000)) = 0x00002AA0;    // GPIOE_MODER:   PE2..PE6 = Alternate function mode
  *((uint32_t*)(0x40021008)) = 0x00003FF0;    // GPIOE_OSPEEDR: PE2..PE6 = 100 MHz speed
  *((uint32_t*)(0x4002100C)) = 0x00000000;    // GPIOE_PUPDR:   PE2..PE6 = No Pull-up/Pull-down
  *((uint32_t*)(0x40021020)) = 0x00000000;    // GPIOE_AFRL:    PE2..PE6 = AF0

  //DBGMCU->CR = DBGMCU_CR_TRACE_IOEN | DBGMCU1_CR_TRACE_MODE_0 | DBGMCU1_CR_TRACE_MODE_1; // Enable IO trace pins
  //DBGMCU->CR = 0xE0;
  DBGMCU->CR = 0x60;
  if (!(DBGMCU->CR & DBGMCU_CR_TRACE_IOEN))
    return;

  // default 64k baud rate
  // SWOSpeed in Hz, note that cpuCoreFreqHz is expected to be match the CPU core clock
  //uint32_t SWOSpeed = 64000;
  uint32_t SWOSpeed = 2000000;
  uint32_t cpuCoreFreqHz = 168000000;
  uint32_t SWOPrescaler = (cpuCoreFreqHz / SWOSpeed) - 1;

  // enable trace in core debug
  CoreDebug->DEMCR = CoreDebug_DEMCR_TRCENA_Msk;

  // TPI SPPR - Selected PIN Protocol Register =  protocol for trace output 2: SWO NRZ, 1: SWO Manchester encoding
  //TPI->SPPR = 0x00000002;
  TPI->SPPR = 0x00000000;  // sync
  //TPI->CSPSR = 0x08;       // 4 pins TRACED[0..3], PE2 clk, PE3,4,5,6 data
  TPI->CSPSR = 0x01;       // 4 pins TRACED[0..3], PE2 clk, PE3,4,5,6 data

  // TPI Async Clock Prescaler Register =  Scale the baud rate of the asynchronous output
  TPI->ACPR = SWOPrescaler;

  // ITM Lock Access Register = ITM_LAR_KEY = C5ACCE55 enables write access to Control Register 0xE00 :: 0xFFC
  ITM->LAR = ITM_LAR_KEY;

  // ITM Trace Control Register
  ITM->TCR = ITM_TCR_TraceBusID_Msk | ITM_TCR_SWOENA_Msk | ITM_TCR_SYNCENA_Msk | ITM_TCR_ITMENA_Msk;

  // ITM Trace Privilege Register =
  ITM->TPR = ITM_TPR_PRIVMASK_Msk;

  // ITM Trace Enable Register =  Enabled tracing on stimulus ports. One bit per stimulus port
  ITM->TER = 0xFFFFFFFF;

  // DWT Control Register =
  //*((volatile unsigned*)(ITM_BASE + 0x01000)) = 0x400003FE;
  DWT->CTRL = 0x400003FE;

  // TPI Formatter and Flush Control Register =
  //*((volatile unsigned*)(ITM_BASE + 0x40304)) = 0x00000100;
  TPI->FFCR = 0x00000102;
}
//}}}
//{{{
void configureDtrace4() {

  __HAL_RCC_GPIOE_CLK_ENABLE();
  //*((uint32_t*)(0x40023830)) |= 0x00000010;
  *((uint32_t*)(0x40021000)) = 0x00002AA0;    // GPIOE_MODER:   PE2..PE6 = Alternate function mode
  *((uint32_t*)(0x40021008)) = 0x00003FF0;    // GPIOE_OSPEEDR: PE2..PE6 = 100 MHz speed
  *((uint32_t*)(0x4002100C)) = 0x00000000;    // GPIOE_PUPDR:   PE2..PE6 = No Pull-up/Pull-down
  *((uint32_t*)(0x40021020)) = 0x00000000;    // GPIOE_AFRL:    PE2..PE6 = AF0

  //DBGMCU->CR = DBGMCU_CR_TRACE_IOEN | DBGMCU1_CR_TRACE_MODE_0 | DBGMCU1_CR_TRACE_MODE_1; // Enable IO trace pins
  DBGMCU->CR = 0xE0;
  //DBGMCU->CR = 0x60;
  if (!(DBGMCU->CR & DBGMCU_CR_TRACE_IOEN))
    return;

  // default 64k baud rate
  // SWOSpeed in Hz, note that cpuCoreFreqHz is expected to be match the CPU core clock
  //uint32_t SWOSpeed = 64000;
  uint32_t SWOSpeed = 2000000;
  uint32_t cpuCoreFreqHz = 180000000;
  uint32_t SWOPrescaler = (cpuCoreFreqHz / SWOSpeed) - 1;

  // enable trace in core debug
  CoreDebug->DEMCR = CoreDebug_DEMCR_TRCENA_Msk;

  // TPI SPPR - Selected PIN Protocol Register =  protocol for trace output 2: SWO NRZ, 1: SWO Manchester encoding
  //TPI->SPPR = 0x00000002;
  TPI->SPPR = 0x00000000;  // sync
  TPI->CSPSR = 0x08;       // 4 pins TRACED[0..3], PE2 clk, PE3,4,5,6 data
  //TPI->CSPSR = 0x01;       // 4 pins TRACED[0..3], PE2 clk, PE3,4,5,6 data

  // TPI Async Clock Prescaler Register =  Scale the baud rate of the asynchronous output
  TPI->ACPR = SWOPrescaler;

  // ITM Lock Access Register = ITM_LAR_KEY = C5ACCE55 enables write access to Control Register 0xE00 :: 0xFFC
  ITM->LAR = ITM_LAR_KEY;

  // ITM Trace Control Register
  ITM->TCR = ITM_TCR_TraceBusID_Msk | ITM_TCR_SWOENA_Msk | ITM_TCR_SYNCENA_Msk | ITM_TCR_ITMENA_Msk;

  // ITM Trace Privilege Register =
  ITM->TPR = ITM_TPR_PRIVMASK_Msk;

  // ITM Trace Enable Register =  Enabled tracing on stimulus ports. One bit per stimulus port
  ITM->TER = 0xFFFFFFFF;

  // DWT Control Register =
  //*((volatile unsigned*)(ITM_BASE + 0x01000)) = 0x400003FE;
  DWT->CTRL = 0x400003FE;

  // TPI Formatter and Flush Control Register =
  //*((volatile unsigned*)(ITM_BASE + 0x40304)) = 0x00000100;
  TPI->FFCR = 0x00000102;
}
//}}}
//{{{
void configureTracing() {
//#define ETM_TraceMode() ETM->CR &= ~ETM_CR_PROGRAMMING
//#define ETM_SetupMode() ETM->CR |= ETM_CR_PROGRAMMING

  DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN; // Enable IO trace pins
  if (!(DBGMCU->CR & DBGMCU_CR_TRACE_IOEN))
    return;

  // default 64k baud rate
  // SWOSpeed in Hz, note that cpuCoreFreqHz is expected to be match the CPU core clock
  //uint32_t SWOSpeed = 64000;
  uint32_t SWOSpeed = 8000000;
  uint32_t cpuCoreFreqHz = 168000000;
  uint32_t SWOPrescaler = (cpuCoreFreqHz / SWOSpeed) - 1;

  // Configure Trace Port Interface Unit */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable access to registers
  TPI->ACPR = SWOPrescaler;
  TPI->SPPR = 2;     // Pin protocol = NRZ/USART
  TPI->FFCR = 0x102; // TPIU packet framing enabled when bit 2 set,  0x100 only DWT/ITM and not ETM.

  // Configure PC sampling and exception trace
  DWT->CTRL = (1 << DWT_CTRL_CYCTAP_Pos) |     // Prescaler for PC sampling, 0 = x32, 1 = x512
              (0 << DWT_CTRL_POSTPRESET_Pos) | // Postscaler for PC sampling, Divider = value + 1
              (1 << DWT_CTRL_PCSAMPLENA_Pos) | // Enable PC sampling
              (2 << DWT_CTRL_SYNCTAP_Pos) |    // Sync packet interval 0=Off, 1=2^23 cycles, 2=2^25, 3=2^27
              (1 << DWT_CTRL_EXCTRCENA_Pos) |  // Enable exception trace
              (1 << DWT_CTRL_CYCCNTENA_Pos);   // Enable cycle counter

  // Configure instrumentation trace macroblock
  ITM->LAR = 0xC5ACCE55;
  ITM->TCR = (1 << ITM_TCR_TraceBusID_Pos) | // Trace bus ID for TPIU
             (1 << ITM_TCR_DWTENA_Pos) |     // Enable events from DWT
             (1 << ITM_TCR_SYNCENA_Pos) |    // Enable sync packets
             (1 << ITM_TCR_ITMENA_Pos);      // Main enable for ITM
  ITM->TER = 0xFFFFFFFF; // Enable all stimulus ports

  // Configure embedded trace macroblock
  ETM->LAR = 0xC5ACCE55;
  ETM_SetupMode();
  ETM->CR = ETM_CR_ETMEN |           // Enable ETM output port
            ETM_CR_STALL_PROCESSOR | // Stall processor when fifo is full
            ETM_CR_BRANCH_OUTPUT;    // Report all branches
  ETM->TRACEIDR = 2;                 // Trace bus ID for TPIU
  ETM->TECR1 = ETM_TECR1_EXCLUDE;    // Trace always enabled
  ETM->FFRR = ETM_FFRR_EXCLUDE;      // Stalling always enabled
  ETM->FFLR = 24;                    // Stall <N bytes free FIFO (1..24), larger less latency, but more stalls
                                     // Note: we do not enable ETM trace yet, only for specific parts of code.
  }
//}}}
//{{{
void configureWatchpoint() {
// how to configure DWT to monitor a watchpoint, data value reported when watchpoint hit

  // Monitor all accesses to GPIOC (range length 32 bytes)
  DWT->COMP0 = (uint32_t)GPIOD;
  DWT->MASK0 = 5;
  DWT->FUNCTION0 = (2 << DWT_FUNCTION_FUNCTION_Pos) | // Report data and addr on watchpoint hit
                   (1 << DWT_FUNCTION_EMITRANGE_Pos);

  // Monitor all accesses to globalCounter (range length 4 bytes)
  DWT->COMP1 = (uint32_t)&globalCounter;
  DWT->MASK1 = 2;
  DWT->FUNCTION1 = (3 << DWT_FUNCTION_FUNCTION_Pos); // Report data and PC on watchpoint hit
  }
//}}}
//{{{
uint32_t my_ITM_SendChar (uint32_t port, uint32_t ch) {

  // if ITM enabled && ITM Port #0 enabled
  if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) && ((ITM->TER & 1UL) != 0UL)) {
    while (ITM->PORT[port].u32 == 0UL) {
      __NOP();
      }
    ITM->PORT[port].u8 = (uint8_t)ch;
    }
  return (ch);
  }
//}}}

//{{{
void SystemCoreClockUpdate() {

  uint32_t tmp = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;

  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;
  switch (tmp) {
    case 0x00:  /* HSI used as system clock source */
      SystemCoreClock = HSI_VALUE;
      break;

    case 0x04:  /* HSE used as system clock source */
      SystemCoreClock = HSE_VALUE;
      break;

    case 0x08:  /* PLL used as system clock source */
      /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N SYSCLK = PLL_VCO / PLL_P */
      pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
      pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;

      if (pllsource != 0) /* HSE used as PLL clock source */
        pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      else /* HSI used as PLL clock source */
        pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);

      pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;
      SystemCoreClock = pllvco/pllp;
      break;

    default:
      SystemCoreClock = HSI_VALUE;
      break;
    }

  /* Compute HCLK frequency --------------------------------------------------*/
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];

  /* HCLK frequency */
  SystemCoreClock >>= tmp;
  }
//}}}
//{{{
void SystemInit() {

  // FPU settings
  SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  // set CP10 and CP11 Full Access

  // Reset the RCC clock configuration to the default reset state
  // Set HSION bit
  RCC->CR |= (uint32_t)0x00000001;

  // Reset CFGR register
  RCC->CFGR = 0x00000000;

  // Reset HSEON, CSSON and PLLON bits
  RCC->CR &= (uint32_t)0xFEF6FFFF;

  // Reset PLLCFGR register
  RCC->PLLCFGR = 0x24003010;

  // Reset HSEBYP bit
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  // Disable all interrupts
  RCC->CIR = 0x00000000;

  // Configure the Vector Table location add offset address
  SCB->VTOR = FLASH_BASE; // Vector Table Relocation in Internal FLASH
  }
//}}}

//{{{
void SystemClockConfig180() {
//  System Clock source            = PLL (HSE)
//    SYSCLK(Hz)                     = 180000000
//    HCLK(Hz)                       = 180000000
//    AHB Prescaler                  = 1
//    APB1 Prescaler                 = 4
//    APB2 Prescaler                 = 2
//    HSE Frequency(Hz)              = 8000000
//    PLL_M                          = 8
//    PLL_N                          = 360
//    PLL_P                          = 2
//    PLL_Q                          = 7
//    VDD(V)                         = 3.3
//    Main regulator output voltage  = Scale1 mode
//    Flash Latency(WS)              = 5

  // Enable Power Control clock
  __HAL_RCC_PWR_CLK_ENABLE();

  // The voltage scaling allows optimizing the power consumption when the device is
  // clocked below the maximum system frequency, to update the voltage scaling value
  // regarding system frequency refer to product datasheet.
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  // Enable HSE Oscillator and activate PLL with HSE as source
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig (&RCC_OscInitStruct);

  // Activate the Over-Drive mode
  HAL_PWREx_EnableOverDrive();

  // PLLSAI_VCO Input  = HSE_VALUE / PLL_M = 1mhz
  // PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN     = 130mhz
  // PLLLCDCLK         = PLLSAI_VCO Output / PLLSAIR    = 130/2 = 65mhz
  // LTDC clock        = PLLLCDCLK / LTDC_PLLSAI_DIVR_2 = 65/2  = 32.5mhz
  RCC_PeriphCLKInitTypeDef rccPeriphClkInit;
  rccPeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  rccPeriphClkInit.PLLSAI.PLLSAIN = LTDC_CLOCK_4;  // hclk = 192mhz, 138/4 = 34.5mhz
  rccPeriphClkInit.PLLSAI.PLLSAIR = 2;
  rccPeriphClkInit.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  HAL_RCCEx_PeriphCLKConfig (&rccPeriphClkInit);

  //rccPeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDIO | RCC_PERIPHCLK_CK48;
  //rccPeriphClkInit.SdioClockSelection = RCC_SDIOCLKSOURCE_CK48;
  //rccPeriphClkInit.Clk48ClockSelection = RCC_CK48CLKSOURCE_PLLSAIP;
  //rccPeriphClkInit.PLLSAI.PLLSAIN = 384;
  //rccPeriphClkInit.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  //HAL_RCCEx_PeriphCLKConfig (&rccPeriphClkInit);

  // Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_5);
  }
//}}}
//{{{
void memoryTest() {

  uint32_t readAddress = SDRAM_BANK1_ADDR;
  uint32_t phase = 0;
  while (1) {
    uint32_t len = (readAddress == SDRAM_BANK1_ADDR) ? SDRAM_BANK1_LEN : SDRAM_BANK2_LEN;
    uint32_t reportMask = 0xFFFFF;

    //  write
    for (uint32_t i = readAddress; i < readAddress + len; i++)
      *(uint8_t*)readAddress = (i+phase) & 0xFF;

    // read
    uint8_t readErr = 0;
    for (uint32_t i = readAddress; i < readAddress + len; i++) {
      uint8_t read = *(uint8_t*)readAddress;
      if (read != ((i+phase) & 0xFF)) {
        readErr++;
        //printf ("add:%x exp:%x got:%x\n", i, (i+phase) & 0xFF, read);
        }

      if ((i & reportMask) == reportMask) {
        if (readErr)
          printf ("add:%x err:%x rate:%d\n", (unsigned int)i, (unsigned int)readErr, (int)((readErr * 100) / reportMask));
        else
          printf ("add:%lx ok\n", i);

        if (readErr) { // red
          //BSP_LED_Off (LED3);
          //BSP_LED_On (LED4);
          readErr = 0;
          }
        else { // green
          //BSP_LED_Off (LED4);
          //BSP_LED_On (LED3);
          }
        }
      }

    phase++;
    readAddress = (readAddress == SDRAM_BANK1_ADDR) ? SDRAM_BANK2_ADDR : SDRAM_BANK1_ADDR;
    }
  }
//}}}
//}}}
//{{{  sdRam
//{{{
void sdramGpioInit() {
// Timing configuration 90 MHz SD clock frequency (180MHz/2)
//     PG08 <-> FMC_SDCLK
//     PC00 <-> FMC_SDNWE
//     PC02 <-> FMC_SDNE0  BANK1 address 0xC0000000        PB06 <-> FMC_SDNE1  BANK2 address 0xD0000000
//     PB05 <-> FMC_SDCKE1                                 PC03 <-> FMC_SDCKE0
// PD14..15 <-> FMC_D00..01    PF00..05 <-> FMC_A00..05    PE00 <-> FMC_NBL0
// PD00..01 <-> FMC_D02..03    PF12..15 <-> FMC_A06..09    PE01 <-> FMC_NBL1
// PE07..15 <-> FMC_D04..12    PG00..01 <-> FMC_A10..11    PG15 <-> FMC_NCAS
// PD08..10 <-> FMC_D13..15                                PF11 <-> FMC_NRAS

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_FMC_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_Init_Structure;
  GPIO_Init_Structure.Mode = GPIO_MODE_AF_PP;
  GPIO_Init_Structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_Init_Structure.Pull = GPIO_NOPULL;
  GPIO_Init_Structure.Alternate = GPIO_AF12_FMC;

  GPIO_Init_Structure.Pin = GPIO_PIN_5 | GPIO_PIN_6;
  HAL_GPIO_Init (GPIOB, &GPIO_Init_Structure);

  GPIO_Init_Structure.Pin = GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3;
  HAL_GPIO_Init (GPIOC, &GPIO_Init_Structure);

  GPIO_Init_Structure.Pin = GPIO_PIN_0 | GPIO_PIN_1  | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                            GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init (GPIOD, &GPIO_Init_Structure);

  GPIO_Init_Structure.Pin = GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_7 | GPIO_PIN_8  | GPIO_PIN_9  |
                            GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init (GPIOE, &GPIO_Init_Structure);

  GPIO_Init_Structure.Pin = GPIO_PIN_0  | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3  | GPIO_PIN_4 | GPIO_PIN_5 |
                            GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init (GPIOF, &GPIO_Init_Structure);

  GPIO_Init_Structure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_15;
  HAL_GPIO_Init (GPIOG, &GPIO_Init_Structure);
  }
//}}}
//{{{
void sdramBank1Init() {
// Micron MT48LC8M16A2 – 2m x 16bits x 4banks
// PC2, PC3 - bank 1 - 0xC0000000 : 0xC10000000 - len 0x1000000

  SDRAM_HandleTypeDef hsdram;
  hsdram.Instance = FMC_SDRAM_DEVICE;
  hsdram.Init.SDBank             = FMC_SDRAM_BANK1;
  hsdram.Init.ColumnBitsNumber   = FMC_SDRAM_COLUMN_BITS_NUM_9; // 8,9,10,11
  hsdram.Init.RowBitsNumber      = FMC_SDRAM_ROW_BITS_NUM_12;   // 11,12,13
  hsdram.Init.MemoryDataWidth    = FMC_SDRAM_MEM_BUS_WIDTH_16;  // 8,16,32
  hsdram.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram.Init.CASLatency         = FMC_SDRAM_CAS_LATENCY_3;
  hsdram.Init.WriteProtection    = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram.Init.SDClockPeriod      = FMC_SDRAM_CLOCK_PERIOD_3;
  //hsdram.Init.ReadBurst          = FMC_SDRAM_RBURST_DISABLE;
  hsdram.Init.ReadBurst          = FMC_SDRAM_RBURST_ENABLE;
  hsdram.Init.ReadPipeDelay      = FMC_SDRAM_RPIPE_DELAY_1;

  FMC_SDRAM_TimingTypeDef SDRAM_Timing;
  SDRAM_Timing.LoadToActiveDelay    = 2; // TMRD: 2 Clock cycles
  SDRAM_Timing.ExitSelfRefreshDelay = 7; // TXSR: min = 70ns (6 x 11.90ns)
  SDRAM_Timing.SelfRefreshTime      = 4; // TRAS: min = 42ns (4 x 11.90ns) max=120k (ns)
  SDRAM_Timing.RowCycleDelay        = 7; // TRC:  min = 63 (6 x 11.90ns)
  SDRAM_Timing.WriteRecoveryTime    = 2; // TWR:  2 Clock cycles
  SDRAM_Timing.RPDelay              = 2; // TRP:  15ns => 2 x 11.90ns
  SDRAM_Timing.RCDDelay             = 2; // TRCD: 15ns => 2 x 11.90ns

  if (HAL_SDRAM_Init (&hsdram, &SDRAM_Timing) != HAL_OK) {
    printf ("HAL_SDRAM_Init error\n");
    while (1) {}
    }

  // Configure clock configuration enable command
  FMC_SDRAM_CommandTypeDef Command;
  Command.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
  Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber = 1;
  Command.ModeRegisterDefinition = 0;
  HAL_SDRAM_SendCommand (&hsdram, &Command, 0x1000);

  // Insert 100 ms delay
  HAL_Delay (100);

  // Configure PALL (precharge all) command
  Command.CommandMode = FMC_SDRAM_CMD_PALL;
  Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber = 1;
  Command.ModeRegisterDefinition = 0;
  HAL_SDRAM_SendCommand (&hsdram, &Command, 0x1000);

  // Configure Auto-Refresh command
  Command.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber = 4;
  Command.ModeRegisterDefinition = 0;
  HAL_SDRAM_SendCommand (&hsdram, &Command, 0x1000);

  Command.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
  Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber = 1;
  Command.ModeRegisterDefinition =
    SDRAM_MODEREG_BURST_LENGTH_2 | SDRAM_MODEREG_CAS_LATENCY_3 | SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;
  HAL_SDRAM_SendCommand (&hsdram, &Command, 0x1000);

  // Set refresh rate counter //* (15.62 us x Freq) - 20 - SDRAM refresh counter (90MHz SD clock)
  HAL_SDRAM_ProgramRefreshRate (&hsdram, 0x1386);
  //HAL_SDRAM_ProgramRefreshRate (&hsdram, 0x0569);
  }
//}}}
//{{{
void sdramBank2Init() {
// PB6, PB5 - Bank2 - 0xD0000000 : 0xD0800000 len 0x800000
  SDRAM_HandleTypeDef hsdram;
  hsdram.Instance = FMC_SDRAM_DEVICE;
  hsdram.Init.SDBank             = FMC_SDRAM_BANK2;
  hsdram.Init.ColumnBitsNumber   = FMC_SDRAM_COLUMN_BITS_NUM_8;  // 8,9,10,11
  hsdram.Init.RowBitsNumber      = FMC_SDRAM_ROW_BITS_NUM_12;    // 11,12,13
  hsdram.Init.MemoryDataWidth    = FMC_SDRAM_MEM_BUS_WIDTH_16;   // 8,16,32
  hsdram.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram.Init.CASLatency         = FMC_SDRAM_CAS_LATENCY_3;
  hsdram.Init.WriteProtection    = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram.Init.SDClockPeriod      = FMC_SDRAM_CLOCK_PERIOD_3;
  //hsdram.Init.ReadBurst          = FMC_SDRAM_RBURST_DISABLE;
  hsdram.Init.ReadBurst          = FMC_SDRAM_RBURST_ENABLE;
  hsdram.Init.ReadPipeDelay      = FMC_SDRAM_RPIPE_DELAY_1;

  FMC_SDRAM_TimingTypeDef SDRAM_Timing;
  SDRAM_Timing.LoadToActiveDelay    = 2; // TMRD: 2 Clock cycles
  SDRAM_Timing.ExitSelfRefreshDelay = 7; // TXSR: min = 70ns (6 x 11.90ns)
  SDRAM_Timing.SelfRefreshTime      = 4; // TRAS: min = 42ns (4 x 11.90ns) max=120k (ns)
  SDRAM_Timing.RowCycleDelay        = 7; // TRC:  min = 63 (6 x 11.90ns)
  SDRAM_Timing.WriteRecoveryTime    = 2; // TWR:  2 Clock cycles
  SDRAM_Timing.RPDelay              = 2; // TRP:  15ns => 2 x 11.90ns
  SDRAM_Timing.RCDDelay             = 2; // TRCD: 15ns => 2 x 11.90ns

  if (HAL_SDRAM_Init (&hsdram, &SDRAM_Timing) != HAL_OK) {
    printf ("HAL_SDRAM_Init error\n");
    while (1) {}
    }

  // Configure clock configuration enable command
  FMC_SDRAM_CommandTypeDef Command;
  Command.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
  Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber = 1;
  Command.ModeRegisterDefinition = 0;
  HAL_SDRAM_SendCommand (&hsdram, &Command, 0x1000);

  // Insert 100 ms delay
  HAL_Delay (100);

  // Configure PALL (precharge all) command
  Command.CommandMode = FMC_SDRAM_CMD_PALL;
  Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber = 1;
  Command.ModeRegisterDefinition = 0;
  HAL_SDRAM_SendCommand (&hsdram, &Command, 0x1000);

  // Configure Auto-Refresh command
  Command.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber = 4;
  Command.ModeRegisterDefinition = 0;
  HAL_SDRAM_SendCommand (&hsdram, &Command, 0x1000);

  // Program external memory mode register
  Command.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
  Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber = 1;
  Command.ModeRegisterDefinition = // burstSequential, operatingModeNormal
    SDRAM_MODEREG_WRITEBURST_MODE_SINGLE | SDRAM_MODEREG_CAS_LATENCY_3 | SDRAM_MODEREG_BURST_LENGTH_4;
  HAL_SDRAM_SendCommand (&hsdram, &Command, 0x1000);

  // Set refresh rate counter //* (15.62 us x Freq) - 20 - SDRAM refresh counter (90MHz SD clock)
  HAL_SDRAM_ProgramRefreshRate  (&hsdram, 0x0569);
  }
//}}}
//}}}
//{{{  sd
typedef enum { MSD_OK, MSD_ERROR, MSD_ERROR_SD_NOT_PRESENT, MSD_NO_HIGH_SPEED } MSD_RESULT;
//{{{  static vars
SD_HandleTypeDef uSdHandle;
HAL_SD_CardInfoTypedef uSdCardInfo;
DMA_HandleTypeDef dmaRxHandle;
DMA_HandleTypeDef dmaTxHandle;

const uint32_t mReadCacheSize = 0x40;
uint8_t* mReadCache = 0;
uint32_t mReadCacheBlock = 0xFFFFFFB0;
uint32_t mReads = 0;
uint32_t mReadHits = 0;
uint32_t mReadMultipleLen = 0;
uint32_t mReadBlock = 0xFFFFFFFF;

uint32_t mWrites = 0;
int32_t mWriteMultipleLen = 0;
uint32_t mWriteBlock = 0xFFFFFFFF;
//}}}

extern "C" { void SDIO_IRQHandler() { HAL_SD_IRQHandler (&uSdHandle); } }
extern "C" { void DMA2_Stream3_IRQHandler() { HAL_DMA_IRQHandler (uSdHandle.hdmarx); } }
extern "C" { void DMA2_Stream6_IRQHandler() { HAL_DMA_IRQHandler (uSdHandle.hdmatx); } }

//{{{
uint8_t SD_Init() {

  uSdHandle.Instance = SDIO;
  uSdHandle.Init.ClockEdge           = SDIO_CLOCK_EDGE_RISING;
  uSdHandle.Init.ClockBypass         = SDIO_CLOCK_BYPASS_DISABLE;  // SDIO_CLOCK_BYPASS_ENABLE;
  uSdHandle.Init.ClockPowerSave      = SDIO_CLOCK_POWER_SAVE_DISABLE;
  uSdHandle.Init.BusWide             = SDIO_BUS_WIDE_1B;
  uSdHandle.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  uSdHandle.Init.ClockDiv            = 0;

  __HAL_RCC_SDIO_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  //{{{  gpio init
  // sdPresent init - PC6
  GPIO_InitTypeDef gpio_init_structure;
  gpio_init_structure.Pin = GPIO_PIN_6;
  gpio_init_structure.Mode = GPIO_MODE_INPUT;
  gpio_init_structure.Pull = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init (GPIOC, &gpio_init_structure);

  // SDIO D0..D3 - PC8..PC11
  gpio_init_structure.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Alternate = GPIO_AF12_SDIO;
  HAL_GPIO_Init (GPIOC, &gpio_init_structure);

  // SDIO CMD - PD2
  gpio_init_structure.Pin = GPIO_PIN_2;
  HAL_GPIO_Init (GPIOD, &gpio_init_structure);

  // SDIO CLK - PC12
  gpio_init_structure.Pin = GPIO_PIN_12;
  //gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init (GPIOC, &gpio_init_structure);
  //}}}
  //{{{  DMA rx parameters
  dmaRxHandle.Instance                 = DMA2_Stream3;
  dmaRxHandle.Init.Channel             = DMA_CHANNEL_4;
  dmaRxHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  dmaRxHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  dmaRxHandle.Init.MemInc              = DMA_MINC_ENABLE;
  dmaRxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  dmaRxHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  dmaRxHandle.Init.Mode                = DMA_PFCTRL;
  dmaRxHandle.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
  dmaRxHandle.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
  dmaRxHandle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  dmaRxHandle.Init.MemBurst            = DMA_MBURST_INC4;
  dmaRxHandle.Init.PeriphBurst         = DMA_PBURST_INC4;
  __HAL_LINKDMA (&uSdHandle, hdmarx, dmaRxHandle);
  HAL_DMA_DeInit (&dmaRxHandle);
  HAL_DMA_Init (&dmaRxHandle);
  //}}}
  //{{{  DMA tx parameters
  dmaTxHandle.Instance                 = DMA2_Stream6;
  dmaTxHandle.Init.Channel             = DMA_CHANNEL_4;
  dmaTxHandle.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  dmaTxHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  dmaTxHandle.Init.MemInc              = DMA_MINC_ENABLE;
  dmaTxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  dmaTxHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  dmaTxHandle.Init.Mode                = DMA_PFCTRL;
  dmaTxHandle.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
  dmaTxHandle.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
  dmaTxHandle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  dmaTxHandle.Init.MemBurst            = DMA_MBURST_INC4;
  dmaTxHandle.Init.PeriphBurst         = DMA_PBURST_INC4;

  __HAL_LINKDMA (&uSdHandle, hdmatx, dmaTxHandle);
  HAL_DMA_DeInit (&dmaTxHandle);
  HAL_DMA_Init (&dmaTxHandle);
  //}}}

  HAL_NVIC_SetPriority (DMA2_Stream3_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ (DMA2_Stream3_IRQn);
  HAL_NVIC_SetPriority (DMA2_Stream6_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ (DMA2_Stream6_IRQn);

  HAL_NVIC_SetPriority (SDIO_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ (SDIO_IRQn);

  // HAL SD initialization
  if (HAL_SD_Init (&uSdHandle, &uSdCardInfo) != SD_OK)
    return MSD_ERROR;
  if (HAL_SD_WideBusOperation_Config (&uSdHandle, SDIO_BUS_WIDE_4B) != SD_OK)
    return MSD_ERROR;
  if (HAL_SD_HighSpeed (&uSdHandle) == SD_OK)
    lcd->info ("high speed ok");
  return MSD_OK;
  }
//}}}
//{{{
uint8_t SD_ITConfig() {

  //GPIO_InitTypeDef gpio_init_structure;
  //gpio_init_structure.Pin = SD_DETECT_PIN;
  //gpio_init_structure.Pull = GPIO_PULLUP;
  //gpio_init_structure.Speed = GPIO_SPEED_FAST;
  //gpio_init_structure.Mode = GPIO_MODE_IT_RISING_FALLING;
  //HAL_GPIO_Init (SD_DETECT_GPIO_PORT, &gpio_init_structure);

  // Enable and set SD detect EXTI Interrupt to the lowest priority */
  //HAL_NVIC_SetPriority ((IRQn_Type)(SD_DETECT_EXTI_IRQn), 0x0F, 0x00);
  //HAL_NVIC_EnableIRQ ((IRQn_Type)(SD_DETECT_EXTI_IRQn));

  return MSD_OK;
  }
//}}}

//{{{
bool SD_present() {
  return !(GPIOC->IDR & GPIO_PIN_6);
  }
//}}}
//{{{
int8_t SD_IsReady() {
  return (SD_present() && (HAL_SD_GetStatus (&uSdHandle) == SD_TRANSFER_OK)) ? 0 : -1;
  }
//}}}
//{{{
int8_t SD_GetCapacity (uint32_t* block_num, uint16_t* block_size) {

  if (SD_present()) {
    HAL_SD_CardInfoTypedef info;
    HAL_SD_Get_CardInfo (&uSdHandle, &info);
    *block_num = (info.CardCapacity) / 512 - 1;
    *block_size = 512;
    return 0;
    }

  return -1;
  }
//}}}

//{{{
HAL_SD_TransferStateTypedef SD_GetStatus() {
  return HAL_SD_GetStatus (&uSdHandle);
  }
//}}}
//{{{
void SD_GetCardInfo (HAL_SD_CardInfoTypedef* CardInfo) {
  HAL_SD_Get_CardInfo (&uSdHandle, CardInfo);
  }
//}}}
//{{{
std::string SD_info() {
  return "r:" + dec (mReadHits) + ":" + dec (mReads) + ":"  + dec (mReadBlock + mReadMultipleLen) +
         " w:" + dec (mWrites);
  }
//}}}

//{{{
uint8_t SD_Read (uint8_t* buf, uint32_t blk_addr, uint16_t blocks) {

  uint32_t tmp = (uint32_t)buf;

  auto result = HAL_SD_ReadBlocks_DMA (&uSdHandle, (uint32_t*)buf, blk_addr * 512, 512, blocks);
  if (result != SD_OK) {
    lcd->debug ("HAL_SD_ReadBlocks_DMA:" + hex (tmp) + " " + dec (blk_addr) + " num:" + dec (blocks) +  dec (result));
    return MSD_ERROR;
    }

  result = HAL_SD_CheckReadOperation (&uSdHandle, 500);
  if (result != SD_OK) {
    lcd->debug ("HAL_SD_CheckReadOperation:" + hex (tmp) + " " + dec (blk_addr) + " num:" + dec (blocks) +  dec (result));
    return MSD_ERROR;
    }
  //SCB_InvalidateDCache_by_Addr ((uint32_t*)((uint32_t)buf & 0xFFFFFFE0), (blocks * 512) + 32);

  return MSD_OK;
  }
//}}}
//{{{
uint8_t SD_Write (uint8_t* buf, uint32_t blk_addr, uint16_t blocks) {

  //if (HAL_SD_WriteBlocks (&uSdHandle, (uint32_t*)buf, blk_addr * 512, blocks) != SD_OK)
  //  return MSD_ERROR;
  //can't remove ?
  HAL_SD_CheckWriteOperation (&uSdHandle, 0xFFFFFFFF);

  return MSD_OK;
  }
//}}}
//{{{
int8_t SD_ReadCached (uint8_t* buf, uint32_t blk_addr, uint16_t blocks) {

  return SD_Read (buf, blk_addr, blocks);
  }
//}}}
//{{{
int8_t SD_WriteCached (uint8_t* buf, uint32_t blk_addr, uint16_t blocks) {

  if (SD_present()) {
    mWrites++;
    SD_Write (buf, blk_addr, blocks);

    mReadCacheBlock = 0xFFFFFFF0;
    if (blk_addr != mWriteBlock + mWriteMultipleLen) {
      if (mWriteMultipleLen) {
        // flush pending multiple
        //cLcd::debug ("wm:" + dec (mWriteBlock) + "::" + dec (mWriteMultipleLen));
        mWriteMultipleLen = 0;
        }
      mWriteBlock = blk_addr;
      }
    mWriteMultipleLen += blocks;

    return 0;
    }

  return -1;
  }
//}}}

//{{{
uint8_t SD_Erase (uint64_t StartAddr, uint64_t EndAddr) {
  if (HAL_SD_Erase (&uSdHandle, StartAddr, EndAddr) != SD_OK)
    return MSD_ERROR;
  else
    return MSD_OK;
  }
//}}}

// diskio
static volatile DSTATUS Stat = STA_NOINIT;
//{{{
DSTATUS diskInitialize() {

  return SD_GetStatus() == SD_TRANSFER_OK ? 0 : STA_NOINIT;
  }
//}}}

//{{{
DSTATUS diskStatus() {

  return SD_GetStatus() == SD_TRANSFER_OK ? 0 : STA_NOINIT;
  }
//}}}
//{{{
DRESULT diskIoctl (BYTE cmd, void* buff) {

  DRESULT res = RES_ERROR;

  switch (cmd) {
    // Make sure that no pending write process
    case CTRL_SYNC :
      res = RES_OK;
      break;

    // Get number of sectors on the disk (DWORD)
    case GET_SECTOR_COUNT : {
      HAL_SD_CardInfoTypedef CardInfo;
      SD_GetCardInfo (&CardInfo);
      *(DWORD*)buff = CardInfo.CardCapacity / SECTOR_SIZE;
      res = RES_OK;
      break;
      }

    // Get R/W sector size (WORD)
    case GET_SECTOR_SIZE :
      *(WORD*)buff = SECTOR_SIZE;
      res = RES_OK;
      break;

    // Get erase block size in unit of sector (DWORD)
    case GET_BLOCK_SIZE :
      *(DWORD*)buff = SECTOR_SIZE;
      res = RES_OK;
      break;

    default:
      res = RES_PARERR;
    }

  return res;
  }
//}}}

//{{{
DRESULT diskRead (BYTE* buffer, DWORD sector, UINT count) {

  if ((uint32_t)buffer & 0x03) {
    lcd->info ("diskRead align b:" + hex ((int)buffer) + " sec:" + dec (sector) + " num:" + dec (count));

    // not 32bit aligned, dma fails,
    auto tempBuffer = (uint8_t*)pvPortMalloc (count * SECTOR_SIZE);

    // read into 32bit aligned tempBuffer
    auto result = SD_ReadCached (tempBuffer, sector, count) == MSD_OK ? RES_OK : RES_ERROR;
    memcpy (buffer, tempBuffer, count * SECTOR_SIZE);

    vPortFree (tempBuffer);
    return result;
    }

  else {
    //lcd->info ("diskRead - sec:" + dec (sector) + " num:" + dec (count));
    return SD_ReadCached ((uint8_t*)buffer, sector, count) == MSD_OK ? RES_OK : RES_ERROR;
    }
  }
//}}}
//{{{
DRESULT diskWrite (const BYTE* buffer, DWORD sector, UINT count) {
  return SD_WriteCached ((uint8_t*)buffer, (uint64_t)(sector * SECTOR_SIZE), count) == MSD_OK ? RES_OK : RES_ERROR;
  }
//}}}
//}}}

std::vector<std::string> mFileNames;
//{{{
void listDirectory (std::string directoryName, std::string ext, std::string ext1) {

  cDirectory directory (directoryName);
  if (directory.getError()) {
    //{{{  open error
    lcd->info (COL_RED, "directory open error:"  + dec (directory.getError()));
    return;
    }
    //}}}

  cFileInfo fileInfo;
  while ((directory.find (fileInfo) == FR_OK) && !fileInfo.getEmpty()) {
    if (fileInfo.getBack()) {
      }
    else if (fileInfo.isDirectory()) {
      listDirectory (directoryName + "/" + fileInfo.getName(), ext, ext1);
      }
    else if (ext.empty() | fileInfo.matchExtension (ext.c_str()) || fileInfo.matchExtension (ext1.c_str())) {
      mFileNames.push_back (directoryName + "/" + fileInfo.getName());
      }
    }
  }
//}}}
//{{{
int main() {

  HAL_Init();
  SystemClockConfig180();
  initDebugUart();
  BSP_PB_Init (BUTTON_KEY, BUTTON_MODE_GPIO);
  //configureDtrace4();
  sdramGpioInit();
  sdramBank1Init();
  sdramBank2Init();

  HeapRegion_t xHeapRegions[] = {
    //{(uint8_t*)0x10000000, 0x10000 },
    {(uint8_t*)SDRAM_BANK2_ADDR + (LCD_WIDTH*LCD_HEIGHT*4), SDRAM_BANK2_LEN - (LCD_WIDTH*LCD_HEIGHT*4) },
    { nullptr, 0 } };
  heapInit (xHeapRegions);
  //{{{  init frameBuffer
  memset ((void*)SDRAM_BANK2_ADDR, 0, (LCD_WIDTH*LCD_HEIGHT*4));

  lcd = new cLcd (SDRAM_BANK2_ADDR, SDRAM_BANK2_ADDR + (LCD_WIDTH*LCD_HEIGHT*2));
  const std::string kHello = "built " + std::string(__TIME__) + " on " + std::string(__DATE__) +
                              " heap:" + dec (0x800000 - (LCD_WIDTH*LCD_HEIGHT*4));
  lcd->init ("stm32F429disco test - " + kHello);

  lcd->displayOn();
  lcd->render();
  //}}}

  if (BSP_PB_GetState (BUTTON_KEY) == GPIO_PIN_SET) {
    //{{{  ps2
    initPs2gpio();
    initPs2touchpad();
    }
    //}}}
  else {
    //{{{  sd
    if (SD_present()) {
      int ret = SD_Init();
      lcd->debug ("SDinit " + dec(ret));

      cFatFs* fatFs = cFatFs::create();
      if (fatFs->mount() != FR_OK)
        lcd->debug ("fatFs mount problem");
      else
        lcd->debug ("SD label:" + fatFs->getLabel() + " vsn:" + hex (fatFs->getVolumeSerialNumber()) +
                   " freeSectors:" + dec (fatFs->getFreeSectors()));
      listDirectory ("", "", "");
      }
    else
      lcd->debug ("no SD card");
    }
    //}}}

  for (auto fileStr : mFileNames) {
    auto t0 = HAL_GetTick();
    //{{{  read file and header
    cFile file (fileStr, FA_OPEN_EXISTING | FA_READ);

    auto buf = (uint8_t*)pvPortMalloc (file.getSize());
    if (!buf)
      lcd->debug ("no buf");

    auto bytesRead = 0;
    file.read (buf, file.getSize(), bytesRead);
    //}}}
    iPic* pic = new cDecodePic ();
    auto tRead = HAL_GetTick();

    //{{{  scale calc
    // calc scale
    //auto scaleShift = 0;
    //auto scale = 1;
    //while ((scaleShift < 3) &&
    //       ((width / scale > lcd->getWidth()) || (height /scale > lcd->getHeight()))) {
    //  scale *= 2;
    //  scaleShift++;
    //  }
    //width >>= scaleShift;
    //height >>= scaleShift;
    //}}}
    pic->setPic (buf, file.getSize());
    vPortFree (buf);
    auto tDecode = HAL_GetTick();
    auto width = pic->getWidth();
    auto height = pic->getHeight();
    lcd->info ("fileSize:" + dec(file.getSize()) + " " + dec(width) + ":" + dec(height) +
               " " + dec(tRead-t0) + ":" + dec(tDecode-tRead) + "ms " + fileStr);

    if (pic->getPic()) {
      cLcd::cTile picTile (pic->getPic(), pic->getComponents(), width,
                           0,0, width > lcd->getWidth() ? lcd->getWidth() : width,
                                height > lcd->getHeight() ? lcd->getHeight() : height);
      lcd->startRender();
      lcd->clear (COL_BLACK);
      lcd->copy (picTile, 0,0);
      lcd->sizeCpuBi (picTile, 0,0, lcd->getWidth(), lcd->getHeight());
      picTile.free();
      lcd->endRender (true);
      }
    else
      lcd->debug ("- no piccy");

    delete (pic);
    }

  HAL_Delay (60000);

  while (true) {
    lcd->startRender();
    lcd->clear (COL_BLACK);
    //std::string str = "x:" + dec(touchX) + " y:" + dec(touchY) + " z:" + dec(touchZ);
    //lcd->text (COL_YELLOW, getFontHeight(), str, 100, 200, lcd->getWidth(), getBoxHeight());
    //auto x = lcd->getWidth() - ((touchY-1300)/4);
    //auto y = lcd->getHeight() - ((touchX-1300)/4);
    //lcd->ellipse (COL_YELLOW, x, y, touchZ, touchZ);
    lcd->endRender (true);
    //uint16_t value = PS2get();
    //char ch = (char)value;
    //std::string charStr (1, ch);
    //std::string releaseStr = (value & 0x100) ? " release " : " ";
    //lcd->info ("key:" + charStr + releaseStr + hex (value));
    }
  }
//}}}
