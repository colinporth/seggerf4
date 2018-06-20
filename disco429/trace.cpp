// trace
//{{{  trace defines
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
