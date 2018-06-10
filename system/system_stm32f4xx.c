// system_stm32f4xx.c
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

uint32_t SystemCoreClock = 168000000;

const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
//{{{
void SystemCoreClockUpdate() {

  // Get SYSCLK source
  uint32_t tmp = RCC->CFGR & RCC_CFGR_SWS;
  switch (tmp) {
    case 0x00:  // HSI system clock source
      SystemCoreClock = HSI_VALUE;
      break;

    case 0x04:  // HSE system clock source
      SystemCoreClock = HSE_VALUE;
      break;

    case 0x08: { // PLL system clock source
      // PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
      // SYSCLK = PLL_VCO / PLL_P
      uint32_t pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
      uint32_t pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
      uint32_t pllvco = (pllsource != 0) ? (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6)
                                         : (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      uint32_t pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;
      SystemCoreClock = pllvco / pllp;
      break;
      }

    default:
      SystemCoreClock = HSI_VALUE;
      break;
    }

  // Compute HCLK frequency, Get HCLK prescaler
  SystemCoreClock >>= AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  }
//}}}

//{{{
void SystemInit() {

  // FPU - set CP10 and CP11 Full Access
  SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));

  // reset the RCC clock configuration to the default reset state, Set HSION bit
  RCC->CR |= (uint32_t)0x00000001;

  // reset CFGR register
  RCC->CFGR = 0x00000000;

  // reset HSEON, CSSON and PLLON bits
  RCC->CR &= (uint32_t)0xFEF6FFFF;

  // reset PLLCFGR register
  RCC->PLLCFGR = 0x24003010;

  // reset HSEBYP bit
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  // disable all interrupts
  RCC->CIR = 0x00000000;

  SCB->VTOR = FLASH_BASE;
  }
//}}}
