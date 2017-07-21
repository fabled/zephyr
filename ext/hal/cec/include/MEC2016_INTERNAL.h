
/****************************************************************************************************//**
 * @file     MCHP_MEC2016_internal.h
 *
 * @brief    CMSIS Cortex-M4 Peripheral Access Layer Header File for
 *           MCHP_MEC2016_internal from Microchip Technology Inc..
 *
 * @version  V1.0
 * @date     25. September 2015
 *
 * @note     Generated with SVDConv V2.87e
 *           from CMSIS SVD File 'MCHP_MEC2016_internal.svd' Version 1.0,
 *
 * @par      ARM Limited (ARM) is supplying this software for use with Cortex-M
 *           processor based microcontroller, but can be equally used for other
 *           suitable processor architectures. This file can be freely distributed.
 *           Modifications to this file shall be clearly marked.
 *
 *           THIS SOFTWARE IS PROVIDED "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *           OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *           MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *           ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *           CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *******************************************************************************************************/



/** @addtogroup Microchip Technology Inc.
  * @{
  */

/** @addtogroup MCHP_MEC2016_internal
  * @{
  */

#ifndef MCHP_MEC2016_INTERNAL_H
#define MCHP_MEC2016_INTERNAL_H

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum {
/* -------------------  Cortex-M4 Processor Exceptions Numbers  ------------------- */
  Reset_IRQn                    = -15,              /*!<   1  Reset Vector, invoked on Power up and warm reset                 */
  NonMaskableInt_IRQn           = -14,              /*!<   2  Non maskable Interrupt, cannot be stopped or preempted           */
  HardFault_IRQn                = -13,              /*!<   3  Hard Fault, all classes of Fault                                 */
  MemoryManagement_IRQn         = -12,              /*!<   4  Memory Management, MPU mismatch, including Access Violation
                                                         and No Match                                                          */
  BusFault_IRQn                 = -11,              /*!<   5  Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory
                                                         related Fault                                                         */
  UsageFault_IRQn               = -10,              /*!<   6  Usage Fault, i.e. Undef Instruction, Illegal State Transition    */
  SVCall_IRQn                   =  -5,              /*!<  11  System Service Call via SVC instruction                          */
  DebugMonitor_IRQn             =  -4,              /*!<  12  Debug Monitor                                                    */
  PendSV_IRQn                   =  -2,              /*!<  14  Pendable request for system service                              */
  SysTick_IRQn                  =  -1,              /*!<  15  System Tick Timer                                                */
/* ----------------MCHP_MEC2016_internal Specific Interrupt Numbers---------------- */
  GPIO_140_176_IRQn             =   0,              /*!<   0  GPIO[140:176], GIRQ08                                            */
  GPIO_100_137_IRQn             =   1,              /*!<   1  GPIO[100:137], GIRQ09                                            */
  GPIO_040_076_IRQn             =   2,              /*!<   2  GPIO[040:076], GIRQ10                                            */
  GPIO_000_036_IRQn             =   3,              /*!<   3  GPIO[000:036], GIRQ11                                            */
  GPIO_200_236_IRQn             =   4,              /*!<   4  GPIO[200:236], GIRQ12                                            */
  MSVW00_06_IRQn                =  15,              /*!<  15  MSVW[00:06]_SRC[0:3], GIRQ 24                                    */
  MSVW07_10_IRQn                =  16,              /*!<  16  MSVW[07:10]_SRC[0:3], GIRQ 25                                    */
  GPIO_240_257_IRQn             =  17,              /*!<  17  GPIO[240:257], GIRQ26                                            */
  SMB0_IRQn                     =  20,              /*!<  20  SMB0, GIRQ 13.0                                                  */
  SMB1_IRQn                     =  21,              /*!<  21  SMB1                                                             */
  SMB2_IRQn                     =  22,              /*!<  22  SMB2                                                             */
  SMB3_IRQn                     =  23,              /*!<  23  SMB3                                                             */
  DMA0_IRQn                     =  24,              /*!<  24  DMA0, GIRQ14.0                                                   */
  DMA1_IRQn                     =  25,              /*!<  25  DMA1                                                             */
  DMA2_IRQn                     =  26,              /*!<  26  DMA2                                                             */
  DMA3_IRQn                     =  27,              /*!<  27  DMA3                                                             */
  DMA4_IRQn                     =  28,              /*!<  28  DMA4                                                             */
  DMA5_IRQn                     =  29,              /*!<  29  DMA5                                                             */
  DMA6_IRQn                     =  30,              /*!<  30  DMA6                                                             */
  DMA7_IRQn                     =  31,              /*!<  31  DMA7                                                             */
  DMA8_IRQn                     =  32,              /*!<  32  DMA8                                                             */
  DMA9_IRQn                     =  33,              /*!<  33  DMA9                                                             */
  DMA10_IRQn                    =  34,              /*!<  34  DMA10                                                            */
  DMA11_IRQn                    =  35,              /*!<  35  DMA11                                                            */
  DMA12_IRQn                    =  36,              /*!<  36  DMA12                                                            */
  DMA13_IRQn                    =  37,              /*!<  37  DMA13                                                            */
  UART_0_IRQn                   =  40,              /*!<  40  UART 0, GIRQ 15.0                                                */
  UART_1_IRQn                   =  41,              /*!<  41  UART 1, GIRQ 15.1                                                */
  EMI_0_IRQn                    =  42,              /*!<  42  EMI_0, GIRQ 15.2                                                 */
  EMI_1_IRQn                    =  43,              /*!<  43  EMI_1, GIRQ 15.3                                                 */
  EMI_2_IRQn                    =  44,              /*!<  44  EMI_2, GIRQ 15.4                                                 */
  ACPIEC0_IBF_IRQn              =  45,              /*!<  45  ACPIEC[0] IBF, GIRQ 15.5                                         */
  ACPIEC0_OBF_IRQn              =  46,              /*!<  46  ACPIEC[0] OBF, GIRQ 15.6                                         */
  ACPIEC1_IBF_IRQn              =  47,              /*!<  47  ACPIEC[1] IBF, GIRQ 15.7                                         */
  ACPIEC1_OBF_IRQn              =  48,              /*!<  48  ACPIEC[1] OBF, GIRQ 15.8                                         */
  ACPIEC2_IBF_IRQn              =  49,              /*!<  49  ACPIEC[2] IBF, GIRQ 15.9                                         */
  ACPIEC2_OBF_IRQn              =  50,              /*!<  50  ACPIEC[2] OBF, GIRQ 15.10                                        */
  ACPIEC3_IBF_IRQn              =  51,              /*!<  51  ACPIEC[3] IBF, GIRQ 15.11                                        */
  ACPIEC3_OBF_IRQn              =  52,              /*!<  52  ACPIEC[3] OBF, GIRQ 15.12                                        */
  ACPIEC4_IBF_IRQn              =  53,              /*!<  53  ACPIEC[4] IBF, GIRQ 15.13                                        */
  ACPIEC4_OBF_IRQn              =  54,              /*!<  54  ACPIEC[4] OBF, GIRQ 15.14                                        */
  ACPIPM1_CTL_IRQn              =  55,              /*!<  55  ACPIPM1_CTL, GIRQ 15.10                                          */
  ACPIPM1_EN_IRQn               =  56,              /*!<  56  ACPIPM1_EN, GIRQ 15.11                                           */
  ACPIPM1_STS_IRQn              =  57,              /*!<  57  ACPIPM1_STS, GIRQ 15.12                                          */
  KBC8042_OBF_IRQn              =  58,              /*!<  58  8042EM OBF, GIRQ 15.18                                           */
  KBC8042_IBF_IRQn              =  59,              /*!<  59  8042EM IBF, GIRQ 15.19                                           */
  MAILBOX_IRQn                  =  60,              /*!<  60  MAILBOX, GIRQ 15.20                                              */
  MAILBOX_DATA_IRQn             =  61,              /*!<  61  MAILBOX DATA, GIRQ 15.21                                         */
  PORT80_DEBUG_0_IRQn           =  62,              /*!<  62  PORT80_DEBUG_0, GIRQ 15.22                                       */
  PORT80_DEBUG_1_IRQn           =  63,              /*!<  63  PORT80_DEBUG_1, GIRQ 15.23                                       */
  ASIF_INT_IRQn                 =  64,              /*!<  64  ASIF_INT, GIRQ 15.24                                             */
  PECIHOST_IRQn                 =  70,              /*!<  70  PECIHOST, GIRQ 17.0                                              */
  TACH_0_IRQn                   =  71,              /*!<  71  TACH_0, GIRQ 17.1                                                */
  TACH_1_IRQn                   =  72,              /*!<  72  TACH_1, GIRQ 17.2                                                */
  TACH_2_IRQn                   =  73,              /*!<  73  TACH_2, GIRQ 17.3                                                */
  RPM2PWM_0_FAIL_IRQn           =  74,              /*!<  74  RPM2PWM_0 Fail, GIRQ 17.4                                        */
  RPM2PWM_0_STALL_IRQn          =  75,              /*!<  75  RPM2PWM_0 Stall, GIRQ 17.5                                       */
  RPM2PWM_1_FAIL_IRQn           =  76,              /*!<  76  RPM2PWM_1 Fail, GIRQ 17.6                                        */
  RPM2PWM_1_STALL_IRQn          =  77,              /*!<  77  RPM2PWM_1 Stall, GIRQ 17.7                                       */
  ADC_SNGL_IRQn                 =  78,              /*!<  78  ADC_SNGL, GIRQ 17.8                                              */
  ADC_RPT_IRQn                  =  79,              /*!<  79  ADC_RPT, GIRQ 17.9                                               */
  RC_ID_0_IRQn                  =  80,              /*!<  80  RC_ID_0, GIRQ 17.10                                              */
  RC_ID_1_IRQn                  =  81,              /*!<  81  RC_ID_1, GIRQ 17.11                                              */
  RC_ID_2_IRQn                  =  82,              /*!<  82  RC_ID_2, GIRQ 17.12                                              */
  LED_0_IRQn                    =  83,              /*!<  83  Breathing LED 0, GIRQ 17.13                                      */
  LED_1_IRQn                    =  84,              /*!<  84  Breathing LED 1, GIRQ 17.14                                      */
  LED_2_IRQn                    =  85,              /*!<  85  Breathing LED 2, GIRQ 17.15                                      */
  LED_3_IRQn                    =  86,              /*!<  86  Breathing LED 3, GIRQ 17.16                                      */
  PROCHOT_MON_IRQn              =  87,              /*!<  87  PROCHOT_MON, GIRQ 17.17                                          */
  POWERGUARD_0_IRQn             =  88,              /*!<  88  POWERGUARD_0, GIRQ 17.18                                         */
  POWERGUARD_1_IRQn             =  89,              /*!<  89  POWERGUARD_1, GIRQ 17.19                                         */
  LPC_IRQn                      =  90,              /*!<  90  LPC (GIRQ 18.0)                                                  */
  QMSPI_IRQn                    =  91,              /*!<  91  QMSPI, GIRQ 18.1                                                 */
  SPI0_TX_IRQn                  =  92,              /*!<  92  SPI0 TX, GIRQ 18.2                                               */
  SPI0_RX_IRQn                  =  93,              /*!<  93  SPI0 RX, GIRQ 18.3                                               */
  SPI1_TX_IRQn                  =  94,              /*!<  94  SPI1 TX, GIRQ 18.4                                               */
  SPI1_RX_IRQn                  =  95,              /*!<  95  SPI1 RX, GIRQ 18.5                                               */
  BCM_BUSY_CLR_0_IRQn           =  96,              /*!<  96  BCM_BUSY_CLR_0, GIRQ 18.6                                        */
  BCM_ERR_0_IRQn                =  97,              /*!<  97  BCM_ERR_0, GIRQ 18.7                                             */
  BCM_BUSY_CLR_1_IRQn           =  98,              /*!<  98  BCM_BUSY_CLR_1, GIRQ 18.8                                        */
  BCM_ERR_1_IRQn                =  99,              /*!<  99  BCM_ERR_1, GIRQ 18.9                                             */
  PS2_0_ACT_IRQn                = 100,              /*!< 100  PS2 Controller 0 Activity, GIRQ 17.14                            */
  PS2_1_ACT_IRQn                = 101,              /*!< 101  PS2 Controller 1 Activity, GIRQ 17.15                            */
  PS2_2_ACT_IRQn                = 102,              /*!< 102  PS2 Controller 2 Activity, GIRQ 17.16                            */
  INTR_PC_IRQn                  = 103,              /*!< 103  PC, GIRQ 19.0                                                    */
  INTR_BM1_IRQn                 = 104,              /*!< 104  BM1, GIRQ 19.1                                                   */
  INTR_BM2_IRQn                 = 105,              /*!< 105  BM2, GIRQ 19.2                                                   */
  INTR_LTR_IRQn                 = 106,              /*!< 106  LTR, GIRQ 19.3                                                   */
  INTR_OOB_UP_IRQn              = 107,              /*!< 107  OOB_UP, GIRQ 19.4                                                */
  INTR_OOB_DOWN_IRQn            = 108,              /*!< 108  OOB_DOWN, GIRQ 19.5                                              */
  INTR_FLASH_IRQn               = 109,              /*!< 109  FLASH, GIRQ 19.6                                                 */
  ESPI_RESET_IRQn               = 110,              /*!< 110  ESPI_RESET, GIRQ 19.7                                            */
  RTOS_TIMER_IRQn               = 111,              /*!< 111  RTOS_TIMER, GIRQ 21.0                                            */
  HTIMER0_IRQn                  = 112,              /*!< 112  HTIMER0, GIRQ 21.1                                               */
  HTIMER1_IRQn                  = 113,              /*!< 113  HTIMER1, GIRQ 21.2                                               */
  WEEK_ALARM_IRQn               = 114,              /*!< 114  WEEK_ALARM_INT, GIRQ 21.3                                        */
  SUB_WEEK_ALARM_IRQn           = 115,              /*!< 115  SUB_WEEK_ALARM_INT, GIRQ 21.4                                    */
  ONE_SECOND_IRQn               = 116,              /*!< 116  ONE_SECOND, GIRQ 21.5                                            */
  SUB_SECOND_IRQn               = 117,              /*!< 117  SUB_SECOND, GIRQ 21.6                                            */
  SYSPWR_PRES_IRQn              = 118,              /*!< 118  SYSPWR_PRES, GIRQ 21.7                                           */
  RTC_IRQn                      = 119,              /*!< 119  RTC, GIRQ 21.8                                                   */
  RTC_ALARM_IRQn                = 120,              /*!< 120  RTC ALARM, GIRQ 21.9                                             */
  VCI_OVRD_IN_IRQn              = 121,              /*!< 121  VCI_OVRD_IN, GIRQ 21.10                                          */
  VCI_IN0_IRQn                  = 122,              /*!< 122  VCI_IN0, GIRQ 21.11                                              */
  VCI_IN1_IRQn                  = 123,              /*!< 123  VCI_IN1, GIRQ 21.12                                              */
  VCI_IN2_IRQn                  = 124,              /*!< 124  VCI_IN2, GIRQ 21.13                                              */
  SVCI_IN3_IRQn                 = 125,              /*!< 125  VCI_IN3, GIRQ 21.14                                              */
  VCI_IN4_IRQn                  = 126,              /*!< 126  VCI_IN4, GIRQ 21.15                                              */
  VCI_IN5_IRQn                  = 127,              /*!< 127  VCI_IN5, GIRQ 21.16                                              */
  VCI_IN6_IRQn                  = 128,              /*!< 128  VCI_IN6, GIRQ 21.17                                              */
  PS2_0A_WK_IRQn                = 129,              /*!< 129  PS2 Controller 0 Port A Wake, GIRQ 21.18                         */
  PS2_0B_WK_IRQn                = 130,              /*!< 130  PS2 Controller 0 Port B Wake, GIRQ 21.19                         */
  PS2_1A_WK_IRQn                = 131,              /*!< 131  PS2 Controller 1 Port A Wake, GIRQ 21.20                         */
  PS2_1B_WK_IRQn                = 132,              /*!< 132  PS2 Controller 1 Port B Wake, GIRQ 21.21                         */
  PS2_2_WK_IRQn                 = 133,              /*!< 133  PS2 Controller 2 Wake, GIRQ 21.22                                */
  KSC_INT_IRQn                  = 135,              /*!< 135  KSC, GIRQ 21.25                                                  */
  TIMER0_IRQn                   = 136,              /*!< 136  TIMER_16_0, GIRQ 23.0                                            */
  TIMER1_IRQn                   = 137,              /*!< 137  TIMER_16_1, GIRQ 23.1                                            */
  TIMER2_IRQn                   = 138,              /*!< 138  TIMER_16_2, GIRQ 23.2                                            */
  TIMER3_IRQn                   = 139,              /*!< 139  TIMER_16_3, GIRQ 23.3                                            */
  TIMER4_IRQn                   = 140,              /*!< 140  TIMER_32_0, GIRQ 23.4                                            */
  TIMER5_IRQn                   = 141,              /*!< 141  TIMER_32_1, GIRQ 23.5                                            */
  COUNTER_TIMER_0_IRQn          = 142,              /*!< 142  COUNTER_TIMER_0, GIRQ 23.6                                       */
  COUNTER_TIMER_1_IRQn          = 143,              /*!< 143  COUNTER_TIMER_1, GIRQ 23.7                                       */
  COUNTER_TIMER_2_IRQn          = 144,              /*!< 144  COUNTER_TIMER_2, GIRQ 23.8                                       */
  COUNTER_TIMER_3_IRQn          = 145,              /*!< 145  COUNTER_TIMER_3, GIRQ 23.9                                       */
  CAPTURE_TIMER_IRQn            = 146,              /*!< 146  CAPTURE_TIMER, GIRQ 23.10                                        */
  CAPTURE_0_IRQn                = 147,              /*!< 147  CAPTURE_0, GIRQ 23.11                                            */
  CAPTURE_1_IRQn                = 148,              /*!< 148  CAPTURE_1, GIRQ 23.12                                            */
  CAPTURE_2_IRQn                = 149,              /*!< 149  CAPTURE_2, GIRQ 23.13                                            */
  CAPTURE_3_IRQn                = 150,              /*!< 150  CAPTURE_3, GIRQ 23.14                                            */
  CAPTURE_4_IRQn                = 151,              /*!< 151  CAPTURE_4, GIRQ 23.15                                            */
  CAPTURE_5_IRQn                = 152,              /*!< 152  CAPTURE_5, GIRQ 23.16                                            */
  COMPARE_0_IRQn                = 153,              /*!< 153  COMPARE_0, GIRQ 23.17                                            */
  COMPARE_1_IRQn                = 154,              /*!< 154  COMPARE_1, GIRQ 23.18                                            */
  EEPROM_IRQn                   = 155,              /*!< 155  EEPROM, GIRQ 18.13                                               */
  VWIRE_ENABLE_IRQn             = 156               /*!< 156  VWIRE_ENABLE, GIRQ 19.8                                          */
} IRQn_Type;


/** @addtogroup Configuration_of_CMSIS
  * @{
  */


/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* ----------------Configuration of the Cortex-M4 Processor and Core Peripherals---------------- */
#define __CM4_REV                 0x0100            /*!< Cortex-M4 Core Revision                                               */
#define __MPU_PRESENT                  1            /*!< MPU present or not                                                    */
#define __NVIC_PRIO_BITS               3            /*!< Number of Bits used for Priority Levels                               */
#define __Vendor_SysTickConfig         0            /*!< Set to 1 if different SysTick Config is used                          */
#define __FPU_PRESENT                  1            /*!< FPU present or not                                                    */
/** @} */ /* End of group Configuration_of_CMSIS */

#include "core_cm4.h"                               /*!< Cortex-M4 processor and core peripherals                              */
#include "system_MEC2016.h"                         /*!< MCHP_MEC2016_internal System                                          */


/* ================================================================================ */
/* ================       Device Specific Peripheral Section       ================ */
/* ================================================================================ */


/** @addtogroup Device_Peripheral_Registers
  * @{
  */


/* -------------------  Start of section using anonymous unions  ------------------ */
#if defined(__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined(__ICCARM__)
  #pragma language=extended
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning 586
#else
  #warning Not supported compiler type
#endif



/* ================================================================================ */
/* ================                       PCR                      ================ */
/* ================================================================================ */


/**
  * @brief The Power, Clocks, and Resets (PCR) Section identifies all the power supplies,
 clock sources, and reset inputs to the chip and defines all the derived power, clock, and reset signals.  (PCR)
  */

typedef struct {                                    /*!< PCR Structure                                                         */
  __IO uint32_t  SYS_SLP_CNTRL;                     /*!< System Sleep Control                                                  */
  __IO uint32_t  PROC_CLK_CNTRL;                    /*!< Processor Clock Control Register [7:0] Processor Clock Divide
                                                         Value (PROC_DIV)
                                                          1: divide 48 MHz Ring Oscillator by 1.
                                                          2: divide 48 MHz Ring Oscillator by 2.
                                                          3: divide 48 MHz Ring Oscillator by 3.
                                                          4: divide 48 MHz Ring Oscillator by 4.
                                                          16: divide 48 MHz Ring Oscillator by 16.
                                                          48: divide 48 MHz Ring Oscillator by 48.
                                                          No other values are supported.                                       */
  __IO uint32_t  SLOW_CLK_CNTRL;                    /*!< Configures the EC_CLK clock domain                                    */
  __IO uint32_t  OSC_ID;                            /*!< Oscillator ID Register                                                */
  __IO uint32_t  PCR_PWR_RST_STS;                   /*!< PCR Power Reset Status Register                                       */
  __IO uint32_t  PWR_RST_CNTRL;                     /*!< Power Reset Control Register                                          */
  __IO uint32_t  SYS_RST;                           /*!< System Reset Register                                                 */
  __I  uint32_t  RESERVED[5];
  __IO uint32_t  SLP_EN_0;                          /*!< Sleep Enable 0 Register                                               */
  __IO uint32_t  SLP_EN_1;                          /*!< Sleep Enable 1 Register                                               */
  __IO uint32_t  SLP_EN_2;                          /*!< Sleep Enable 2 Register                                               */
  __IO uint32_t  SLP_EN_3;                          /*!< Sleep Enable 3 Register                                               */
  __IO uint32_t  SLP_EN_4;                          /*!< Sleep Enable 4 Register                                               */
  __I  uint32_t  RESERVED1[3];
  __IO uint32_t  CLK_REQ_0;                         /*!< Clock Required 0 Register                                             */
  __IO uint32_t  CLK_REQ_1;                         /*!< Clock Required 1 Register                                             */
  __IO uint32_t  CLK_REQ_2;                         /*!< Clock Required 2 Register                                             */
  __IO uint32_t  CLK_REQ_3;                         /*!< Clock Required 3 Register                                             */
  __IO uint32_t  CLK_REQ_4;                         /*!< Clock Required 4 Register                                             */
  __I  uint32_t  RESERVED2[3];
  __IO uint32_t  RST_EN_0;                          /*!< Reset Enable 0 Register                                               */
  __IO uint32_t  RST_EN_1;                          /*!< Reset Enable 1 Register                                               */
  __IO uint32_t  RST_EN_2;                          /*!< Reset Enable 2 Register                                               */
  __IO uint32_t  RST_EN_3;                          /*!< Reset Enable 3 Register                                               */
  __IO uint32_t  RST_EN_4;                          /*!< Reset Enable 4 Register                                               */
} PCR_Type;


/* ================================================================================ */
/* ================                    DMA_MAIN                    ================ */
/* ================================================================================ */


/**
  * @brief DMA Main Registers (DMA_MAIN)
  */

typedef struct {                                    /*!< DMA_MAIN Structure                                                    */
  __IO uint8_t   DMA_MAIN_CONTROL;                  /*!< Soft reset the entire module. Enable the blocks operation.            */
  __I  uint8_t   RESERVED[3];
  __I  uint32_t  DATA_PACKET;                       /*!< Debug register that has the data that is stored in the Data
                                                         Packet. This data is read data from the currently active transfer
                                                          source.                                                              */
} DMA_MAIN_Type;


/* ================================================================================ */
/* ================                   DMA_CHAN00                   ================ */
/* ================================================================================ */


/**
  * @brief DMA Channel 00 Registers (DMA_CHAN00)
  */

typedef struct {                                    /*!< DMA_CHAN00 Structure                                                  */
  __IO uint8_t   DMA_CHANNEL_ACTIVATE;              /*!< Enable this channel for operation. The DMA Main Control: Activate
                                                         must also be enabled for this channel to be operational.              */
  __I  uint8_t   RESERVED[3];
  __IO uint32_t  MEMORY_START_ADDRESS;              /*!< This is the starting address for the Memory device.                   */
  __IO uint32_t  MEMORY_END_ADDRESS;                /*!< This is the ending address for the Memory device.                     */
  __IO uint32_t  DEVICE_ADDRESS;                    /*!< This is the Master Device address.                                    */
  __IO uint32_t  CONTROL;                           /*!< DMA Channel N Control                                                 */
  __IO uint8_t   INT_STATUS;                        /*!< DMA Channel N Interrupt Status                                        */
  __I  uint8_t   RESERVED1[3];
  __IO uint8_t   INT_EN;                            /*!< DMA CHANNEL N INTERRUPT ENABLE                                        */
  __I  uint8_t   RESERVED2[7];
  __IO uint32_t  CRC_ENABLE;                        /*!< DMA CHANNEL N CRC ENABLE                                              */
  __IO uint32_t  CRC_DATA;                          /*!< DMA CHANNEL N CRC DATA                                                */
  __IO uint32_t  CRC_POST_STATUS;                   /*!< DMA CHANNEL N CRC POST STATUS                                         */
} DMA_CHAN00_Type;


/* ================================================================================ */
/* ================                   DMA_CHAN01                   ================ */
/* ================================================================================ */


/**
  * @brief DMA Channel 01 Registers (DMA_CHAN01)
  */

typedef struct {                                    /*!< DMA_CHAN01 Structure                                                  */
  __IO uint8_t   DMA_CHANNEL_ACTIVATE;              /*!< Enable this channel for operation. The DMA Main Control: Activate
                                                         must also be enabled for this channel to be operational.              */
  __I  uint8_t   RESERVED[3];
  __IO uint32_t  MEMORY_START_ADDRESS;              /*!< This is the starting address for the Memory device.                   */
  __IO uint32_t  MEMORY_END_ADDRESS;                /*!< This is the ending address for the Memory device.                     */
  __IO uint32_t  DEVICE_ADDRESS;                    /*!< This is the Master Device address.                                    */
  __IO uint32_t  CONTROL;                           /*!< DMA Channel N Control                                                 */
  __IO uint8_t   INT_STATUS;                        /*!< DMA Channel N Interrupt Status                                        */
  __I  uint8_t   RESERVED1[3];
  __IO uint8_t   INT_EN;                            /*!< DMA CHANNEL N INTERRUPT ENABLE                                        */
  __I  uint8_t   RESERVED2[7];
  __IO uint32_t  FILL_ENABLE;                       /*!< DMA CHANNEL N FILL ENABLE                                             */
  __IO uint32_t  FILL_DATA;                         /*!< DMA CHANNEL N FILL DATA                                               */
  __IO uint32_t  FILL_STATUS;                       /*!< DMA CHANNEL N FILL STATUS                                             */
} DMA_CHAN01_Type;


/* ================================================================================ */
/* ================                   DMA_CHAN02                   ================ */
/* ================================================================================ */


/**
  * @brief DMA Channel 00 Registers (DMA_CHAN02)
  */

typedef struct {                                    /*!< DMA_CHAN02 Structure                                                  */
  __IO uint8_t   DMA_CHANNEL_ACTIVATE;              /*!< Enable this channel for operation. The DMA Main Control: Activate
                                                         must also be enabled for this channel to be operational.              */
  __I  uint8_t   RESERVED[3];
  __IO uint32_t  MEMORY_START_ADDRESS;              /*!< This is the starting address for the Memory device.                   */
  __IO uint32_t  MEMORY_END_ADDRESS;                /*!< This is the ending address for the Memory device.                     */
  __IO uint32_t  DEVICE_ADDRESS;                    /*!< This is the Master Device address.                                    */
  __IO uint32_t  CONTROL;                           /*!< DMA Channel N Control                                                 */
  __IO uint8_t   INT_STATUS;                        /*!< DMA Channel N Interrupt Status                                        */
  __I  uint8_t   RESERVED1[3];
  __IO uint8_t   INT_EN;                            /*!< DMA CHANNEL N INTERRUPT ENABLE                                        */
} DMA_CHAN02_Type;


/* ================================================================================ */
/* ================                      INTS                      ================ */
/* ================================================================================ */


/**
  * @brief The interrupt generation logic is made of 16 groups of signals, each of which
 consist of a Status register, a Enable register and a Result register. The Status and Enable are
 latched registers. The Result register is a bit by bit AND function of the Source and Enable registers.
 All the bits of the Result register are OR'ed together and AND'ed with the corresponding bit in the Block
 Select register to form the interrupt signal that is routed to the ARM interrupt controller.  (INTS)
  */

typedef struct {                                    /*!< INTS Structure                                                        */
  __IO uint32_t  GIRQ08_SRC;                        /*!< Status R/W1C                                                          */
  __IO uint32_t  GIRQ08_EN_SET;                     /*!< Write to set source enables                                           */
  __I  uint32_t  GIRQ08_RESULT;                     /*!< Read-only bitwise OR of Source and Enable                             */
  __IO uint32_t  GIRQ08_EN_CLR;                     /*!< Write to clear source enables                                         */
  __I  uint32_t  RESERVED;
  __IO uint32_t  GIRQ09_SRC;                        /*!< Status R/W1C                                                          */
  __IO uint32_t  GIRQ09_EN_SET;                     /*!< Write to set source enables                                           */
  __I  uint32_t  GIRQ09_RESULT;                     /*!< Read-only bitwise OR of Source and Enable                             */
  __IO uint32_t  GIRQ09_EN_CLR;                     /*!< Write to clear source enables                                         */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  GIRQ10_SRC;                        /*!< Status R/W1C                                                          */
  __IO uint32_t  GIRQ10_EN_SET;                     /*!< Write to set source enables                                           */
  __I  uint32_t  GIRQ10_RESULT;                     /*!< Read-only bitwise OR of Source and Enable                             */
  __IO uint32_t  GIRQ10_EN_CLR;                     /*!< Write to clear source enables                                         */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  GIRQ11_SRC;                        /*!< Status R/W1C                                                          */
  __IO uint32_t  GIRQ11_EN_SET;                     /*!< Write to set source enables                                           */
  __I  uint32_t  GIRQ11_RESULT;                     /*!< Read-only bitwise OR of Source and Enable                             */
  __IO uint32_t  GIRQ11_EN_CLR;                     /*!< Write to clear source enables                                         */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  GIRQ12_SRC;                        /*!< Status R/W1C                                                          */
  __IO uint32_t  GIRQ12_EN_SET;                     /*!< Write to set source enables                                           */
  __I  uint32_t  GIRQ12_RESULT;                     /*!< Read-only bitwise OR of Source and Enable                             */
  __IO uint32_t  GIRQ12_EN_CLR;                     /*!< Write to clear source enables                                         */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  GIRQ13_SRC;                        /*!< Status R/W1C                                                          */
  __IO uint32_t  GIRQ13_EN_SET;                     /*!< Write to set source enables                                           */
  __I  uint32_t  GIRQ13_RESULT;                     /*!< Read-only bitwise OR of Source and Enable                             */
  __IO uint32_t  GIRQ13_EN_CLR;                     /*!< Write to clear source enables                                         */
  __I  uint32_t  RESERVED5;
  __IO uint32_t  GIRQ14_SRC;                        /*!< Status R/W1C                                                          */
  __IO uint32_t  GIRQ14_EN_SET;                     /*!< Write to set source enables                                           */
  __I  uint32_t  GIRQ14_RESULT;                     /*!< Read-only bitwise OR of Source and Enable                             */
  __IO uint32_t  GIRQ14_EN_CLR;                     /*!< Write to clear source enables                                         */
  __I  uint32_t  RESERVED6;
  __IO uint32_t  GIRQ15_SRC;                        /*!< Status R/W1C                                                          */
  __IO uint32_t  GIRQ15_EN_SET;                     /*!< Write to set source enables                                           */
  __I  uint32_t  GIRQ15_RESULT;                     /*!< Read-only bitwise OR of Source and Enable                             */
  __IO uint32_t  GIRQ15_EN_CLR;                     /*!< Write to clear source enables                                         */
  __I  uint32_t  RESERVED7;
  __IO uint32_t  GIRQ16_SRC;                        /*!< Status R/W1C                                                          */
  __IO uint32_t  GIRQ16_EN_SET;                     /*!< Write to set source enables                                           */
  __I  uint32_t  GIRQ16_RESULT;                     /*!< Read-only bitwise OR of Source and Enable                             */
  __IO uint32_t  GIRQ16_EN_CLR;                     /*!< Write to clear source enables                                         */
  __I  uint32_t  RESERVED8;
  __IO uint32_t  GIRQ17_SRC;                        /*!< Status R/W1C                                                          */
  __IO uint32_t  GIRQ17_EN_SET;                     /*!< Write to set source enables                                           */
  __I  uint32_t  GIRQ17_RESULT;                     /*!< Read-only bitwise OR of Source and Enable                             */
  __IO uint32_t  GIRQ17_EN_CLR;                     /*!< Write to clear source enables                                         */
  __I  uint32_t  RESERVED9;
  __IO uint32_t  GIRQ18_SRC;                        /*!< Status R/W1C                                                          */
  __IO uint32_t  GIRQ18_EN_SET;                     /*!< Write to set source enables                                           */
  __I  uint32_t  GIRQ18_RESULT;                     /*!< Read-only bitwise OR of Source and Enable                             */
  __IO uint32_t  GIRQ18_EN_CLR;                     /*!< Write to clear source enables                                         */
  __I  uint32_t  RESERVED10;
  __IO uint32_t  GIRQ19_SRC;                        /*!< Status R/W1C                                                          */
  __IO uint32_t  GIRQ19_EN_SET;                     /*!< Write to set source enables                                           */
  __I  uint32_t  GIRQ19_RESULT;                     /*!< Read-only bitwise OR of Source and Enable                             */
  __IO uint32_t  GIRQ19_EN_CLR;                     /*!< Write to clear source enables                                         */
  __I  uint32_t  RESERVED11;
  __IO uint32_t  GIRQ20_SRC;                        /*!< Status R/W1C                                                          */
  __IO uint32_t  GIRQ20_EN_SET;                     /*!< Write to set source enables                                           */
  __I  uint32_t  GIRQ20_RESULT;                     /*!< Read-only bitwise OR of Source and Enable                             */
  __IO uint32_t  GIRQ20_EN_CLR;                     /*!< Write to clear source enables                                         */
  __I  uint32_t  RESERVED12;
  __IO uint32_t  GIRQ21_SRC;                        /*!< Status R/W1C                                                          */
  __IO uint32_t  GIRQ21_EN_SET;                     /*!< Write to set source enables                                           */
  __I  uint32_t  GIRQ21_RESULT;                     /*!< Read-only bitwise OR of Source and Enable                             */
  __IO uint32_t  GIRQ21_EN_CLR;                     /*!< Write to clear source enables                                         */
  __I  uint32_t  RESERVED13;
  __IO uint32_t  GIRQ22_SRC;                        /*!< Status R/W1C                                                          */
  __IO uint32_t  GIRQ22_EN_SET;                     /*!< Write to set source enables                                           */
  __I  uint32_t  GIRQ22_RESULT;                     /*!< Read-only bitwise OR of Source and Enable                             */
  __IO uint32_t  GIRQ22_EN_CLR;                     /*!< Write to clear source enables                                         */
  __I  uint32_t  RESERVED14;
  __IO uint32_t  GIRQ23_SRC;                        /*!< Status R/W1C                                                          */
  __IO uint32_t  GIRQ23_EN_SET;                     /*!< Write to set source enables                                           */
  __I  uint32_t  GIRQ23_RESULT;                     /*!< Read-only bitwise OR of Source and Enable                             */
  __IO uint32_t  GIRQ23_EN_CLR;                     /*!< Write to clear source enables                                         */
  __I  uint32_t  RESERVED15;
  __IO uint32_t  GIRQ24_SRC;                        /*!< Status R/W1C                                                          */
  __IO uint32_t  GIRQ24_EN_SET;                     /*!< Write to set source enables                                           */
  __I  uint32_t  GIRQ24_RESULT;                     /*!< Read-only bitwise OR of Source and Enable                             */
  __IO uint32_t  GIRQ24_EN_CLR;                     /*!< Write to clear source enables                                         */
  __I  uint32_t  RESERVED16;
  __IO uint32_t  GIRQ25_SRC;                        /*!< Status R/W1C                                                          */
  __IO uint32_t  GIRQ25_EN_SET;                     /*!< Write to set source enables                                           */
  __I  uint32_t  GIRQ25_RESULT;                     /*!< Read-only bitwise OR of Source and Enable                             */
  __IO uint32_t  GIRQ25_EN_CLR;                     /*!< Write to clear source enables                                         */
  __I  uint32_t  RESERVED17;
  __IO uint32_t  GIRQ26_SRC;                        /*!< Status R/W1C                                                          */
  __IO uint32_t  GIRQ26_EN_SET;                     /*!< Write to set source enables                                           */
  __I  uint32_t  GIRQ26_RESULT;                     /*!< Read-only bitwise OR of Source and Enable                             */
  __IO uint32_t  GIRQ26_EN_CLR;                     /*!< Write to clear source enables                                         */
  __I  uint32_t  RESERVED18[34];
  __IO uint32_t  BLOCK_ENABLE_SET;                  /*!< Block Enable Set Register                                             */
  __IO uint32_t  BLOCK_ENABLE_CLEAR;                /*!< Block Enable Clear Register.                                          */
  __I  uint32_t  BLOCK_IRQ_VECTOR;                  /*!< Block IRQ Vector Register                                             */
} INTS_Type;


/* ================================================================================ */
/* ================                       LPC                      ================ */
/* ================================================================================ */


/**
  * @brief The registers defined for the LPC Interface block are accessible by the
 various hosts as indicated by "LPC Configuration Registers", "EC-Only Registers"and "Runtime Registers".  (LPC)
  */

typedef struct {                                    /*!< LPC Structure                                                         */
  __IO uint8_t   INDEX;                             /*!< The INDEX register, which is part of the Configuration Port,
                                                         is used as a pointer to a Configuration Register Address.             */
  __IO uint8_t   DATA;                              /*!< The DATA register, which is part of the Configuration Port,
                                                         is used to read or write data to the register currently being
                                                          selected by the INDEX Register.                                      */
  __I  uint16_t  RESERVED[129];
  __I  uint32_t  BUS_MONITOR;                       /*!< LPC BUS MONITOR REGISTER                                              */
  __IO uint32_t  HOST_BUS_ERROR;                    /*!< Host Bus Error Register                                               */
  __IO uint32_t  EC_SERIRQ;                         /*!< If the LPC Logical Device is selected as the source for a Serial
                                                         Interrupt Request by an Interrupt Configuration register, this
                                                          bit is used as the interrupt source.                                 */
  __IO uint32_t  CLK_CTRL;                          /*!< Controls throughput of LPC transactions.                              */
  __I  uint32_t  RESERVED1[3];
  __IO uint32_t  BAR_INHIBIT;                       /*!< When bit Di of BAR_Inhibit is 1, the BAR for Logical Device
                                                         i is disabled and its addresses will not be claimed on the LPC
                                                          bus,
                                                          independent of the value of the Valid bit in the BAR.                */
  __I  uint32_t  RESERVED2[3];
  __IO uint16_t  BAR_INIT;                          /*!< This field is loaded into the LPC BAR at offset 60h on RESET_HOST.
                                                                                                                               */
  __I  uint16_t  RESERVED3[7];
  __IO uint32_t  SRAM_EC_BAR_0;                     /*!< SRAM EC BAR 0                                                         */
  __IO uint32_t  SRAM_EC_BAR_1;                     /*!< SRAM EC BAR 1                                                         */
  __I  uint32_t  RESERVED4[122];
  __IO uint8_t   ACTIVATE;                          /*!< The LPC Logical Device is powered and functional.                     */
  __I  uint8_t   RESERVED5[15];
  __IO uint8_t   SIRQ[16];                          /*!< The LPC Controller implements 16 IRQ channels that may be configured
                                                         to be asserted by any logical device.                                 */
  __I  uint32_t  RESERVED6[4];
  __IO uint32_t  LPC_BAR;                           /*!< LPC Interface BAR Register                                            */
  __IO uint32_t  MBX_BAR;                           /*!< Mailbox Registers Interface BAR                                       */
  __IO uint32_t  KBC_BAR;                           /*!< Keyboard Controller (8042) BAR                                        */
  __IO uint32_t  EC0_BAR;                           /*!< ACPI EC Interface 0 BAR                                               */
  __IO uint32_t  EC1_BAR;                           /*!< ACPI EC Interface 1 BAR                                               */
  __IO uint32_t  EC2_BAR;                           /*!< ACPI EC Interface 2 BAR                                               */
  __IO uint32_t  EC3_BAR;                           /*!< ACPI EC Interface 3 BAR                                               */
  __IO uint32_t  EC4_BAR;                           /*!< ACPI EC Interface 4 BAR                                               */
  __IO uint32_t  PM1_BAR;                           /*!< ACPI PM1 Interface BAR                                                */
  __IO uint32_t  LFK_BAR;                           /*!< Legacy (Fast Keyboard) Interface BAR                                  */
  __IO uint32_t  UART0_BAR;                         /*!< UART 0 BAR Register                                                   */
  __IO uint32_t  UART1_BAR;                         /*!< UART 1 BAR Register                                                   */
  __IO uint32_t  EMI0_BAR;                          /*!< EM Interface 0 BAR                                                    */
  __IO uint32_t  EMI1_BAR;                          /*!< EM Interface 1 BAR                                                    */
  __IO uint32_t  EMI2_BAR;                          /*!< EM Interface 2 BAR                                                    */
  __IO uint32_t  PORT80_0_BAR;                      /*!< BIOS Debug (Port 80) 0 BAR                                            */
  __IO uint32_t  PORT80_1_BAR;                      /*!< BIOS Debug (Port 80) 1 BAR                                            */
  __IO uint32_t  RTC_BAR;                           /*!< RTC Registers Interface BAR                                           */
  __I  uint32_t  RESERVED7;
  __IO uint32_t  LASIC_BAR;                         /*!< LASIC Registers Interface BAR                                         */
  __IO uint32_t  SRAM_0_BAR_LPC_CONFIG_DW0;         /*!< SRAM 0 BAR, LPC Configuration Register Format (Dword 0)               */
  __IO uint32_t  SRAM_0_BAR_LPC_CONFIG_DW1;         /*!< SRAM 0 BAR, LPC Configuration Register Format (Dword 1)               */
  __IO uint32_t  SRAM_1_BAR_LPC_CONFIG_DW0;         /*!< SRAM 1 BAR, LPC Configuration Register Format (DWord 0)               */
  __IO uint32_t  SRAM_1_BAR_LPC_CONFIG_DW1;         /*!< SRAM 1 BAR, LPC Configuration Register Format (DWord 1)               */
  __IO uint16_t  MBX_MEM_BAR_W0;                    /*!< Mailbox Registers I/F Memory BAR (Word 0)                             */
  __IO uint16_t  MBX_MEM_BAR_W1;                    /*!< Mailbox Registers I/F Memory BAR (Word 1)                             */
  __IO uint16_t  MBX_MEM_BAR_W2;                    /*!< Mailbox Registers I/F Memory BAR (Word 2)                             */
  __IO uint16_t  EC0_MEM_BAR_W0;                    /*!< ACPI EC Interface 0 Memory BAR (WORD 0)                               */
  __IO uint16_t  EC0_MEM_BAR_W1;                    /*!< ACPI EC Interface 0 Memory BAR (WORD 1)                               */
  __IO uint16_t  EC0_MEM_BAR_W2;                    /*!< ACPI EC Interface 0 Memory BAR (WORD 2)                               */
  __IO uint16_t  EC1_MEM_BAR_W0;                    /*!< ACPI EC Interface 1 Memory BAR (WORD 0)                               */
  __IO uint16_t  EC1_MEM_BAR_W1;                    /*!< ACPI EC Interface 1 Memory BAR (WORD 1)                               */
  __IO uint16_t  EC1_MEM_BAR_W2;                    /*!< ACPI EC Interface 1 Memory BAR (WORD 2)                               */
  __IO uint16_t  EC2_MEM_BAR_W0;                    /*!< ACPI EC Interface 2 Memory BAR (WORD 0)                               */
  __IO uint16_t  EC2_MEM_BAR_W1;                    /*!< ACPI EC Interface 2 Memory BAR (WORD 1)                               */
  __IO uint16_t  EC2_MEM_BAR_W2;                    /*!< ACPI EC Interface 2 Memory BAR (WORD 2)                               */
  __IO uint16_t  EC3_MEM_BAR_W0;                    /*!< ACPI EC Interface 3 Memory BAR (WORD 0)                               */
  __IO uint16_t  EC3_MEM_BAR_W1;                    /*!< ACPI EC Interface 3 Memory BAR (WORD 1)                               */
  __IO uint16_t  EC3_MEM_BAR_W2;                    /*!< ACPI EC Interface 3 Memory BAR (WORD 2)                               */
  __IO uint16_t  EC4_MEM_BAR_W0;                    /*!< ACPI EC Interface 4 Memory BAR (WORD 0)                               */
  __IO uint16_t  EC4_MEM_BAR_W1;                    /*!< ACPI EC Interface 4 Memory BAR (WORD 1)                               */
  __IO uint16_t  EC4_MEM_BAR_W2;                    /*!< ACPI EC Interface 4 Memory BAR (WORD 2)                               */
  __IO uint16_t  EMI0_MEM_BAR_W0;                   /*!< EM Interface 0 Memory BAR (WORD 0)                                    */
  __IO uint16_t  EMI0_MEM_BAR_W1;                   /*!< EM Interface 0 Memory BAR (WORD 1)                                    */
  __IO uint16_t  EMI0_MEM_BAR_W2;                   /*!< EM Interface 0 Memory BAR (WORD 2)                                    */
  __IO uint16_t  EMI1_MEM_BAR_W0;                   /*!< EM Interface 1 Memory BAR (WORD 0)                                    */
  __IO uint16_t  EMI1_MEM_BAR_W1;                   /*!< EM Interface 1 Memory BAR (WORD 1)                                    */
  __IO uint16_t  EMI1_MEM_BAR_W2;                   /*!< EM Interface 1 Memory BAR (WORD 2)                                    */
  __IO uint16_t  EMI2_MEM_BAR_W0;                   /*!< EM Interface 2 Memory BAR (DWORD 0)                                   */
  __IO uint16_t  EMI2_MEM_BAR_W1;                   /*!< EM Interface 2 Memory BAR (WORD 1)                                    */
  __IO uint16_t  EMI2_MEM_BAR_W2;                   /*!< EM Interface 2 Memory BAR (WORD 2)                                    */
} LPC_Type;


/* ================================================================================ */
/* ================                     ESPI_IO                    ================ */
/* ================================================================================ */


/**
  * @brief The Enhanced Serial Peripheral Interface (eSPI) is used by the system host to configure the chip and communicate
 with the logical devices implemented in the design through a series of read/write registers. It is Intel's successor to the
 Low Pin Count (LPC) bus, used in previous devices to provide System Host access to devices internal to the Embedded Controller.
 The I/O Component is one of two Logical Devices (along with the Memory Component) that provide access to all the registers in the device.  (ESPI_IO)
  */

typedef struct {                                    /*!< ESPI_IO Structure                                                     */
  __IO uint8_t   INDEX;                             /*!< The INDEX register, which is part of the Configuration Port,
                                                         is used as a pointer to a Configuration Register Address.             */
  __IO uint8_t   DATA;                              /*!< The DATA register, which is part of the Configuration Port,
                                                         is used to read or write data to the register currently being
                                                          selected by the INDEX Register.                                      */
  __I  uint16_t  RESERVED[127];
  __IO uint32_t  PC_LAST_CYCLE_DW0;                 /*!< Peripheral Channel Last Cycle Register (DWord 0)                      */
  __IO uint32_t  PC_LAST_CYCLE_DW1;                 /*!< Peripheral Channel Last Cycle Register (DWord 1)                      */
  __IO uint32_t  PC_LAST_CYCLE_DW2;                 /*!< Peripheral Channel Last Cycle Register (DWord 2)                      */
  __IO uint32_t  PC_ERROR_ADDRESS_DW0;              /*!< Peripheral Channel Error Address Register (DWord 0)                   */
  __IO uint32_t  PC_ERROR_ADDRESS_DW1;              /*!< Peripheral Channel Error Address Register (DWord 1)                   */
  __IO uint32_t  PC_STATUS;                         /*!< Peripheral Channel Status Register                                    */
  __IO uint32_t  PC_INT_ENABLE;                     /*!< Peripheral Channel Interrupt Enable Register                          */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  BAR_INHIBIT_DW0;                   /*!< BAR Inhibit Register (DWord 0)                                        */
  __IO uint32_t  BAR_INHIBIT_DW1;                   /*!< BAR Inhibit Register (DWord 1)                                        */
  __IO uint32_t  ESPI_BAR_INIT;                     /*!< eSPI BAR Init Register                                                */
  __IO uint32_t  EC_IRQ;                            /*!< EC IRQ Register                                                       */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  ESPI_IO_BASE_ADDRESS;              /*!< eSPI I/O Base Address Register                                        */
  __IO uint32_t  ESPI_MEM_BASE_ADDRESS;             /*!< eSPI Memory Base Address Register                                     */
  __IO uint32_t  MBX_BASE_ADDRESS;                  /*!< Mailbox BAR Register                                                  */
  __IO uint32_t  EM8042_BASE_ADDRESS;               /*!< 8042 Emulated Keyboard Controller BAR Register                        */
  __IO uint32_t  ACPI_EC_0_BASE_ADDRESS;            /*!< ACPI EC Channel 0 Register                                            */
  __IO uint32_t  ACPI_EC_1_BASE_ADDRESS;            /*!< ACPI EC Channel 1 BAR Register                                        */
  __IO uint32_t  ACPI_EC_2_BASE_ADDRESS;            /*!< I/O Base Address Register                                             */
  __IO uint32_t  ACPI_EC_3_BASE_ADDRESS;            /*!< I/O Base Address Register                                             */
  __IO uint32_t  ACPI_EC_4_BASE_ADDRESS;            /*!< I/O Base Address Register                                             */
  __IO uint32_t  ACPI_PM1_BASE_ADDRESS;             /*!< I/O Base Address Register                                             */
  __IO uint32_t  FAST_KDB_BASE_ADDRESS;             /*!< I/O Base Address Register                                             */
  __IO uint32_t  UART_0_BASE_ADDRESS;               /*!< I/O Base Address Register                                             */
  __IO uint32_t  UART_1_BASE_ADDRESS;               /*!< I/O Base Address Register                                             */
  __IO uint32_t  EMI_0_BASE_ADDRESS;                /*!< I/O Base Address Register                                             */
  __IO uint32_t  EMI_1_BASE_ADDRESS;                /*!< I/O Base Address Register                                             */
  __IO uint32_t  EMI_2_BASE_ADDRESS;                /*!< I/O Base Address Register                                             */
  __IO uint32_t  PORT80_0_BASE_ADDRESS;             /*!< I/O Base Address Register                                             */
  __IO uint32_t  PORT80_1_BASE_ADDRESS;             /*!< I/O Base Address Register                                             */
  __IO uint32_t  RTC_BASE_ADDRESS;                  /*!< I/O Base Address Register                                             */
  __I  uint32_t  RESERVED3[40];
  __IO uint32_t  LTR_PERIPHERAL_STATUS;             /*!< LTR Peripheral Status Register                                        */
  __IO uint32_t  LTR_PERIPHERAL_ENABLE;             /*!< LTR Peripheral Enable Register                                        */
  __IO uint32_t  LTR_PERIPHERAL_CONTROL;            /*!< LTR Peripheral Control Register                                       */
  __IO uint32_t  LTR_PERIPHERAL_MESSAGE;            /*!< LTR Peripheral Message Register                                       */
  __I  uint32_t  RESERVED4[4];
  __IO uint32_t  OOB_RECEIVE_ADDRESS;               /*!< OOB Channel Receive Address Register                                  */
  __I  uint32_t  RESERVED5;
  __IO uint32_t  OOB_TRANSMIT_ADDRESS;              /*!< OOB Channel Transmit Address Register                                 */
  __I  uint32_t  RESERVED6;
  __IO uint32_t  OOB_RECEIVE_LENGTH;                /*!< OOB Channel Receive Length Register                                   */
  __I  uint32_t  RESERVED7;
  __IO uint32_t  OOB_RECEIVE_CONTROL;               /*!< OOB Channel Receive Control Register                                  */
  __IO uint32_t  OOB_RECEIVE_INT_ENABLE;            /*!< OOB Channel Receive Interrupt Enable Register                         */
  __IO uint32_t  OOB_RECEIVE_STATUS;                /*!< OOB Channel Receive Status Register                                   */
  __IO uint32_t  OOB_TRANSMIT_CONTROL;              /*!< OOB Channel Transmit Control Register                                 */
  __IO uint32_t  OOB_TRANSMIT_INT_ENABLE;           /*!< OOB Channel Transmit Interrupt Enable Register                        */
  __IO uint32_t  OOB_TRANSMIT_STATUS;               /*!< OOB Channel Transmit Status Register                                  */
  __I  uint32_t  RESERVED8[4];
  __IO uint32_t  FLASH_CH_FLASH_ADDRESS;            /*!< Flash Access Channel Flash Address Register                           */
  __I  uint32_t  RESERVED9;
  __IO uint32_t  FLASH_CH_BUFFER_ADDRESS;           /*!< Flash Access Channel Buffer Address Register                          */
  __I  uint32_t  RESERVED10;
  __IO uint32_t  FLASH_CH_TRANSFER_LENGTH;          /*!< Flash Access Channel Transfer Length Register                         */
  __IO uint32_t  FLASH_CH_CONTROL;                  /*!< Flash Access Channel Control Register                                 */
  __IO uint32_t  FLASH_CH_INT_ENABLE;               /*!< Flash Access Channel Interrupt Enable Register                        */
  __IO uint32_t  FLASH_CH_CONFIG;                   /*!< Flash Access Channel Configuration Register                           */
  __IO uint32_t  FLASH_CH_STATUS;                   /*!< Flash Access Channel Status Register                                  */
  __I  uint32_t  RESERVED11[3];
  __IO uint8_t   VWIRE_STATUS;                      /*!< Virtual Wire Status Register                                          */
  __I  uint8_t   RESERVED12[47];
  __IO uint8_t   ESPI_CAPABILITIES_ID;              /*!< eSPI Capabilities ID Register                                         */
  __IO uint8_t   ESPI_GLOBAL_CAPABILITIES_0;        /*!< eSPI Capabilities Global Capabilities 0 Register                      */
  __IO uint8_t   ESPI_GLOBAL_CAPABILITIES_1;        /*!< eSPI Capabilities Global Capabilities 1 Register                      */
  __IO uint8_t   ESPI_PC_CAPABILITIES;              /*!< eSPI Peripheral Channel Capabilities Register                         */
  __IO uint8_t   ESPI_VWIRE_CAPABILITIES;           /*!< eSPI Virtual Wire Channel Capabilities Register                       */
  __IO uint8_t   ESPI_OOB_CAPABILITIES;             /*!< eSPI OOB Channel Capabilities Register                                */
  __IO uint8_t   ESPI_FLASH_CAPABILITIES;           /*!< eSPI Flash Channel Capabilities Register                              */
  __IO uint8_t   ESPI_PERIPHERAL_READY;             /*!< eSPI Peripheral Channel Ready Register                                */
  __IO uint8_t   ESPI_OOB_READY;                    /*!< eSPI OOB Channel Ready Register                                       */
  __IO uint8_t   ESPI_FLASH_READY;                  /*!< eSPI Flash Channel Ready Register                                     */
  __IO uint8_t   ESPI_RESET_INT_STATUS;             /*!< eSPI Reset Interrupt Status Register                                  */
  __IO uint8_t   ESPI_RESET_INT_ENABLE;             /*!< eSPI Reset Interrupt Enable Register                                  */
  __IO uint8_t   PLTRST_SOURCE;                     /*!< PLTRST Source Register                                                */
  __IO uint8_t   ESPI_VWIRE_READY;                  /*!< eSPI Virtual Wire Channel Ready Register                              */
  __I  uint16_t  RESERVED13[33];
  __IO uint8_t   ESPI_ACTIVATE;                     /*!< eSPI Activate Register                                                */
  __I  uint8_t   RESERVED14[3];
  __IO uint32_t  ESPI_IO_BAR_CONFIG_ADDRESS;        /*!< eSPI I/O Base Address Configuration Register                          */
  __IO uint32_t  ESPI_MEM_BAR_CONFIG_ADDRESS;       /*!< eSPI Memory Base Address Configuration Register                       */
  __IO uint32_t  MBX_BAR_CONFIG_ADDRESS;            /*!< Mailbox Base Address Configuration Register                           */
  __IO uint32_t  EM8042_BAR_CONFIG_ADDRESS;         /*!< 8042 Emulated Keyboard Controller Base Address Configuration
                                                         Register                                                              */
  __IO uint32_t  ACPI_EC_0_BAR_CONFIG_ADDRESS;      /*!< ACPI EC 0 Base Address Configuration Register                         */
  __IO uint32_t  ACPI_EC_1_BAR_CONFIG_ADDRESS;      /*!< ACPI EC 1 Base Address Configuration Register                         */
  __IO uint32_t  ACPI_EC_2_BAR_CONFIG_ADDRESS;      /*!< ACPI EC 2 Base Address Configuration Register                         */
  __IO uint32_t  ACPI_EC_3_BAR_CONFIG_ADDRESS;      /*!< ACPI EC 3 Base Address Configuration Register                         */
  __IO uint32_t  ACPI_EC_4_BAR_CONFIG_ADDRESS;      /*!< ACPI EC 4 Base Address Configuration Register                         */
  __IO uint32_t  ACPI_PM1_BAR_CONFIG_ADDRESS;       /*!< ACPI PM1 Base Address Configuration Register                          */
  __IO uint32_t  FAST_KBD_BAR_CONFIG_ADDRESS;       /*!< I/O Base Address Configuration Register                               */
  __IO uint32_t  UART_0_BAR_CONFIG_ADDRESS;         /*!< UART 0 Base Address Configuration Register                            */
  __IO uint32_t  UART_1_BAR_CONFIG_ADDRESS;         /*!< UART 1 Base Address Configuration Register                            */
  __IO uint32_t  EMI_0_BAR_CONFIG_ADDRESS;          /*!< Embedded Memory Interface (EMI) 2 BAR Config Register                 */
  __IO uint32_t  EMI_1_BAR_CONFIG_ADDRESS;          /*!< Embedded Memory Interface (EMI) 1 BAR Config Register                 */
  __IO uint32_t  EMI_2_BAR_CONFIG_ADDRESS;          /*!< Embedded Memory Interface (EMI) 2 BAR Config Register                 */
  __IO uint32_t  PORT80_0_BAR_CONFIG_ADDRESS;       /*!< BIOS Debug Port (Port 80) 0 BAR Config Register                       */
  __IO uint32_t  PORT80_1_BAR_CONFIG_ADDRESS;       /*!< BIOS Debug Port (Port 80) 1 BAR Config Register                       */
  __IO uint32_t  RTC_BAR_CONFIG_ADDRESS;            /*!< RTC BAR Config Register                                               */
  __I  uint32_t  RESERVED15[11];
  __IO uint8_t   MBX_HOST_SIRQ_IRQ__SELECT;         /*!< Mailbox (MBX_Host_SIRQ Interrupt) Selection Register                  */
  __IO uint8_t   MBX_HOST_SMI_IRQ_SELECT;           /*!< Mailbox (MBX_Host_SMI Interrupt) Selection Register                   */
  __IO uint8_t   KIRQ_8042_IRQ_SELECT;              /*!< 8042 (KIRQ Interrupt) Selection Register                              */
  __IO uint8_t   MIRQ_8042_IRQ_SELECT;              /*!< 8042 (MIRQ Interrupt) Selection Register                              */
  __IO uint8_t   ACPI_EC_0_OBF_IRQ_SELECT;          /*!< ACPI EC 0 (EC_OBF Interrupt) Selection Register                       */
  __IO uint8_t   ACPI_EC_1_OBF_IRQ_SELECT;          /*!< ACPI EC 1 (EC_OBF Interrupt) Selection Register                       */
  __IO uint8_t   ACPI_EC_2_OBF_IRQ_SELECT;          /*!< ACPI EC 2 (EC_OBF Interrupt) Selection Register                       */
  __IO uint8_t   ACPI_EC_3_OBF_IRQ_SELECT;          /*!< ACPI EC 3 (EC_OBF Interrupt) Selection Register                       */
  __IO uint8_t   ACPI_EC_4_OBF_IRQ_SELECT;          /*!< ACPI EC 4 (EC_OBF Interrupt) Selection Register                       */
  __IO uint8_t   UART_0_IRQ_SELECT;                 /*!< UART 0 (UART Interrupt) Selection Register                            */
  __IO uint8_t   UART_1_IRQ_SELECT;                 /*!< UART 1 (UART Interrupt) Selection Register                            */
  __IO uint8_t   EMI_0_HOST_IRQ_SELECT;             /*!< EMI 0 (Host Event Interrupt) Selection Register                       */
  __IO uint8_t   EMI_0_EC_HOST_IRQ_SELECT;          /*!< EMI 0 (EC-to-Host Interrupt) Selection Register                       */
  __IO uint8_t   EMI_1_HOST_IRQ_SELECT;             /*!< EMI 1 (Host Event Interrupt) Selection Register                       */
  __IO uint8_t   EMI_1_EC_HOST_IRQ_SELECT;          /*!< EMI 1 (EC-to-Host Interrupt) Selection Register                       */
  __IO uint8_t   EMI_2_HOST_IRQ_SELECT;             /*!< EMI 2 (Host Event Interrupt) Selection Register                       */
  __IO uint8_t   EMI_2_EC_HOST_IRQ_SELECT;          /*!< EMI 2 (EC-to-Host Interrupt) Selection Register                       */
  __IO uint8_t   RTC_IRQ_SELECT;                    /*!< RTC (RTC Interrupt) Selection Register                                */
  __IO uint8_t   EC_IRQ_SELECT;                     /*!< EC (EC_IRQ Interrupt) Selection Register                              */
  __I  uint8_t   RESERVED16[49];
  __IO uint8_t   ESPI_VWIRE_ERRORS;                 /*!< eSPI Virtual Wire Errors Register                                     */
} ESPI_IO_Type;


/* ================================================================================ */
/* ================                   ESPI_MEMORY                  ================ */
/* ================================================================================ */


/**
  * @brief The eSPI Memory Component is one of two Logical Devices (along with the I/O Component) that provide access to all the
 registers in the device.  (ESPI_MEMORY)
  */

typedef struct {                                    /*!< ESPI_MEMORY Structure                                                 */
  __I  uint32_t  RESERVED[76];
  __IO uint32_t  MBX_MEM_BASE_ADDRESS;              /*!< Mailbox Memory Base Address                                           */
  __I  uint16_t  RESERVED1[3];
  __IO uint16_t  ACPI_EC_0_MEM_BASE_ADDRESS_LSB;    /*!< ACPI EC Channel 0 Memory BAR (LSB)                                    */
  __IO uint16_t  ACPI_EC_0_MEM_BASE_ADDRESS_MSB;    /*!< ACPI EC Channel 0 Memory BAR (MSB)                                    */
  __I  uint16_t  RESERVED2[3];
  __IO uint32_t  ACPI_EC_1_MEM_BASE_ADDRESS;        /*!< ACPI EC Channel 1 Memory BAR                                          */
  __I  uint16_t  RESERVED3[3];
  __IO uint16_t  ACPI_EC_2_MEM_BASE_ADDRESS_LSB;    /*!< ACPI EC Channel 2 Memory BAR (LSB)                                    */
  __IO uint16_t  ACPI_EC_2_MEM_BASE_ADDRESS_MSB;    /*!< ACPI EC Channel 2 Memory BAR (MSB)                                    */
  __I  uint16_t  RESERVED4[3];
  __IO uint32_t  ACPI_EC_3_MEM_BASE_ADDRESS;        /*!< ACPI EC Channel 3 Memory BAR                                          */
  __I  uint16_t  RESERVED5[3];
  __IO uint16_t  ACPI_EC_4_MEM_BASE_ADDRESS_LSB;    /*!< ACPI EC Channel 4 Memory BAR (LSB)                                    */
  __IO uint16_t  ACPI_EC_4_MEM_BASE_ADDRESS_MSB;    /*!< ACPI EC Channel 4 Memory BAR (MSB)                                    */
  __I  uint16_t  RESERVED6[3];
  __IO uint32_t  EMI_0_MEM_BASE_ADDRESS;            /*!< Embedded Memory Interface (EMI) 0 Memory Base Address                 */
  __I  uint16_t  RESERVED7[3];
  __IO uint16_t  EMI_1_MEM_BASE_ADDRESS_LSB;        /*!< Embedded Memory Interface (EMI) 1 Memory Base Address (LSB)
                                                                                                                               */
  __IO uint16_t  EMI_1_MEM_BASE_ADDRESS_MSB;        /*!< Embedded Memory Interface (EMI) 1 Memory Base Address (MSB)
                                                                                                                               */
  __I  uint16_t  RESERVED8[3];
  __IO uint32_t  EMI_2_MEM_BASE_ADDRESS;            /*!< Embedded Memory Interface (EMI) 2 Memory Base Address                 */
  __I  uint32_t  RESERVED9[10];
  __IO uint16_t  SRAM_0_MEM_BASE_ADDRESS_CONF;      /*!< SRAM 0 Memory Base Address Config                                     */
  __IO uint16_t  SRAM_0_MEM_BASE_ADDRESS_LSB;       /*!< SRAM 0 Memory Base Address LSB                                        */
  __IO uint32_t  SRAM_0_MEM_BASE_ADDRESS_MSB;       /*!< SRAM 0 Memory Base Address MSB                                        */
  __I  uint16_t  RESERVED10;
  __IO uint16_t  SRAM_1_MEM_BASE_ADDRESS_CONF;      /*!< SRAM 1 Memory Base Address Config                                     */
  __IO uint16_t  SRAM_1_MEM_BASE_ADDRESS_LSB;       /*!< SRAM 1 Memory Base Address LSB                                        */
  __IO uint16_t  SRAM_1_MEM_BASE_ADDRESS_MSB;       /*!< SRAM 1 Memory Base Address MSB                                        */
  __I  uint32_t  RESERVED11[17];
  __IO uint32_t  BUS_MASTER_STATUS;                 /*!< Bus Master Status Register                                            */
  __IO uint32_t  BUS_MASTER_INT_EN;                 /*!< Bus Master Interrupt Enable Register                                  */
  __IO uint32_t  BUS_MASTER_CONFIG;                 /*!< Bus Master Configuration Register                                     */
  __I  uint32_t  RESERVED12;
  __IO uint32_t  BUS_MASTER_1_CONTROL;              /*!< Bus Master 1 Control Register                                         */
  __IO uint32_t  BUS_MASTER_1_HOST_ADDR_DW0;        /*!< Bus Master 1 Host Address Register (DWord 0)                          */
  __IO uint32_t  BUS_MASTER_1_HOST_ADDR_DW1;        /*!< Bus Master 1 Host Address Register (DWord 1)                          */
  __IO uint32_t  BUS_MASTER_1_INTERNAL_ADDR;        /*!< Bus Master 1 Internal Address Register                                */
  __I  uint32_t  RESERVED13;
  __IO uint32_t  BUS_MASTER_2_CONTROL;              /*!< Bus Master 2 Control Register                                         */
  __IO uint32_t  BUS_MASTER_2_HOST_ADDR_DW0;        /*!< Bus Master 2 Host Address Register (DWord 0)                          */
  __IO uint32_t  BUS_MASTER_2_HOST_ADDR_DW1;        /*!< Bus Master 2 Host Address Register (DWord 1)                          */
  __IO uint32_t  BUS_MASTER_2_INTERNAL_ADDR;        /*!< Bus Master 2 Internal Address Register                                */
  __I  uint32_t  RESERVED14[63];
  __IO uint16_t  MBX_MEM_BAR_CFG_W0;                /*!< Mailbox Memory BAR Configuration Register (Word 0)                    */
  __IO uint16_t  MBX_MEM_BAR_CFG_W1;                /*!< Mailbox Memory BAR Configuration Register (Word 1)                    */
  __IO uint16_t  MBX_MEM_BAR_CFG_W2;                /*!< Mailbox Memory BAR Configuration Register (Word 2)                    */
  __IO uint16_t  MBX_MEM_BAR_CFG_W3;                /*!< Mailbox Memory BAR Configuration Register (Word 3)                    */
  __IO uint16_t  MBX_MEM_BAR_CFG_W4;                /*!< Mailbox Memory BAR Configuration Register (Word 4)                    */
  __IO uint16_t  ACPI_EC_0_MEM_BAR_CFG_W0;          /*!< ACPI EC Channel 0 Memory BAR Configuration Register (Word 0)
                                                                                                                               */
  __IO uint16_t  ACPI_EC_0_MEM_BAR_CFG_W1;          /*!< ACPI EC Channel 0 Memory BAR Configuration Register (Word 1)
                                                                                                                               */
  __IO uint16_t  ACPI_EC_0_MEM_BAR_CFG_W2;          /*!< ACPI EC Channel 0 Memory BAR Configuration Register (Word 2)
                                                                                                                               */
  __IO uint16_t  ACPI_EC_0_MEM_BAR_CFG_W3;          /*!< ACPI EC Channel 0 Memory BAR Configuration Register (Word 3)
                                                                                                                               */
  __IO uint16_t  ACPI_EC_0_MEM_BAR_CFG_W4;          /*!< ACPI EC Channel 0 Memory BAR Configuration Register (Word 4)
                                                                                                                               */
  __IO uint16_t  ACPI_EC_1_MEM_BAR_CFG_W0;          /*!< ACPI EC Channel 1 Memory BAR Configuration Register (Word 0)
                                                                                                                               */
  __IO uint16_t  ACPI_EC_1_MEM_BAR_CFG_W1;          /*!< ACPI EC Channel 1 Memory BAR Configuration Register (Word 1)
                                                                                                                               */
  __IO uint16_t  ACPI_EC_1_MEM_BAR_CFG_W2;          /*!< ACPI EC Channel 1 Memory BAR Configuration Register (Word 2)
                                                                                                                               */
  __IO uint16_t  ACPI_EC_1_MEM_BAR_CFG_W3;          /*!< ACPI EC Channel 1 Memory BAR Configuration Register (Word 3)
                                                                                                                               */
  __IO uint16_t  ACPI_EC_1_MEM_BAR_CFG_W4;          /*!< ACPI EC Channel 1 Memory BAR Configuration Register (Word 4)
                                                                                                                               */
  __IO uint16_t  ACPI_EC_2_MEM_BAR_CFG_W0;          /*!< ACPI EC Channel 2 Memory BAR Configuration Register (Word 0)          */
  __IO uint16_t  ACPI_EC_2_MEM_BAR_CFG_W1;          /*!< ACPI EC Channel 2 Memory BAR Configuration Register (Word 1)          */
  __IO uint16_t  ACPI_EC_2_MEM_BAR_CFG_W2;          /*!< ACPI EC Channel 2 Memory BAR Configuration Register (Word 2)          */
  __IO uint16_t  ACPI_EC_2_MEM_BAR_CFG_W3;          /*!< ACPI EC Channel 2 Memory BAR Configuration Register (Word 3)          */
  __IO uint16_t  ACPI_EC_2_MEM_BAR_CFG_W4;          /*!< ACPI EC Channel 2 Memory BAR Configuration Register (Word 4)          */
  __IO uint16_t  ACPI_EC_3_MEM_BAR_CFG_W0;          /*!< ACPI EC Channel 3 Memory BAR Configuration Register (Word 0)          */
  __IO uint16_t  ACPI_EC_3_MEM_BAR_CFG_W1;          /*!< ACPI EC Channel 3 Memory BAR Configuration Register (Word 1)          */
  __IO uint16_t  ACPI_EC_3_MEM_BAR_CFG_W2;          /*!< ACPI EC Channel 3 Memory BAR Configuration Register (Word 2)          */
  __IO uint16_t  ACPI_EC_3_MEM_BAR_CFG_W3;          /*!< ACPI EC Channel 3 Memory BAR Configuration Register (Word 3)          */
  __IO uint16_t  ACPI_EC_3_MEM_BAR_CFG_W4;          /*!< ACPI EC Channel 3 Memory BAR Configuration Register (Word 4)          */
  __I  uint16_t  RESERVED15;
  __IO uint16_t  ACPI_EC_4_MEM_BAR_CFG_W1;          /*!< ACPI EC Channel 4 Memory BAR Configuration Register (Word 1)          */
  __IO uint16_t  ACPI_EC_4_MEM_BAR_CFG_W2;          /*!< ACPI EC Channel 4 Memory BAR Configuration Register (Word 2)          */
  __IO uint16_t  ACPI_EC_4_MEM_BAR_CFG_W3;          /*!< ACPI EC Channel 4 Memory BAR Configuration Register (Word 3)          */
  __IO uint16_t  ACPI_EC_4_MEM_BAR_CFG_W4;          /*!< ACPI EC Channel 4 Memory BAR Configuration Register (Word 4)          */
  __IO uint16_t  EMI_0_MEM_BAR_CFG_W0;              /*!< EMI 0 Memory BAR Configuration Register (Word 0)                      */
  __IO uint16_t  EMI_0_MEM_BAR_CFG_W1;              /*!< EMI 0 Memory BAR Configuration Address Register (Word 1)              */
  __IO uint16_t  EMI_0_MEM_BAR_CFG_W2;              /*!< EMI 0 Memory BAR Configuration Address Register (Word 2)              */
  __IO uint16_t  EMI_0_MEM_BAR_CFG_W3;              /*!< EMI 0 Memory BAR Configuration Address Register (Word 3)              */
  __IO uint16_t  EMI_0_MEM_BAR_CFG_W4;              /*!< EMI 0 Memory BAR Configuration Address Register (Word 4)              */
  __IO uint16_t  EMI_1_MEM_BAR_CFG_W0;              /*!< EMI 1 Memory BAR Configuration Register (Word 0) )                    */
  __IO uint16_t  EMI_1_MEM_BAR_CFG_W1;              /*!< EMI 1 Memory BAR Configuration Register (Word 1) )                    */
  __IO uint16_t  EMI_1_MEM_BAR_CFG_W2;              /*!< EMI 1 Memory BAR Configuration Register (Word 2) )                    */
  __IO uint16_t  EMI_1_MEM_BAR_CFG_W3;              /*!< EMI 1 Memory BAR Configuration Register (Word 3) )                    */
  __IO uint16_t  EMI_1_MEM_BAR_CFG_W4;              /*!< EMI 1 Memory BAR Configuration Register (Word 4) )                    */
  __IO uint16_t  EMI_2_MEM_BAR_CFG_W0;              /*!< EMI 2 Memory BAR Configuration Register (Word 0) )                    */
  __IO uint16_t  EMI_2_MEM_BAR_CFG_W1;              /*!< EMI 2 Memory BAR Configuration Register (Word 1) )                    */
  __IO uint16_t  EMI_2_MEM_BAR_CFG_W2;              /*!< EMI 2 Memory BAR Configuration Register (Word 2) )                    */
  __IO uint16_t  EMI_2_MEM_BAR_CFG_W3;              /*!< EMI 2 Memory BAR Configuration Register (Word 3) )                    */
  __IO uint16_t  EMI_2_MEM_BAR_CFG_W4;              /*!< EMI 2 Memory BAR Configuration Register (Word 4) )                    */
  __I  uint16_t  RESERVED16[17];
  __IO uint16_t  SRAM_0_MEM_BAR_CFG_W0;             /*!< SRAM BAR 0 Configuration Register (Word 0) )                          */
  __IO uint16_t  SRAM_0_MEM_BAR_CFG_W1;             /*!< SRAM BAR 0 Configuration Register (Word 1) )                          */
  __IO uint16_t  SRAM_0_MEM_BAR_CFG_W2;             /*!< SRAM BAR 0 Configuration Register (Word 2) )                          */
  __IO uint16_t  SRAM_0_MEM_BAR_CFG_W3;             /*!< SRAM BAR 0 Configuration Register (Word 3) )                          */
  __IO uint16_t  SRAM_0_MEM_BAR_CFG_W4;             /*!< SRAM BAR 0 Configuration Register (Word 4) )                          */
  __IO uint16_t  SRAM_1_MEM_BAR_CFG_W0;             /*!< SRAM BAR 1 Configuration Register (Word 0) )                          */
  __IO uint16_t  SRAM_1_MEM_BAR_CFG_W1;             /*!< SRAM BAR 1 Configuration Register (Word 1) )                          */
  __IO uint16_t  SRAM_1_MEM_BAR_CFG_W2;             /*!< SRAM BAR 1 Configuration Register (Word 2) )                          */
  __IO uint16_t  SRAM_1_MEM_BAR_CFG_W3;             /*!< SRAM BAR 1 Configuration Register (Word 3) )                          */
  __IO uint16_t  SRAM_1_MEM_BAR_CFG_W4;             /*!< SRAM BAR 1 Configuration Register (Word 4) )                          */
} ESPI_MEMORY_Type;


/* ================================================================================ */
/* ================                 ESPI_MSVW00_06                 ================ */
/* ================================================================================ */


/**
  * @brief The Virtual Wire Channel permits the System to emulate a set of wires that interconnect the system Core Logic with the EC.  (ESPI_MSVW00_06)
  */

typedef struct {                                    /*!< ESPI_MSVW00_06 Structure                                              */
  __IO uint32_t  MSVW00_DW0;                        /*!< Master-to-Slave Virtual Wire 0 Register (DW 0)                        */
  __IO uint32_t  MSVW00_DW1;                        /*!< Master-to-Slave Virtual Wire 0 Register (DW 1)                        */
  __IO uint32_t  MSVW00_DW2;                        /*!< Master-to-Slave Virtual Wire 0 Register (DW 2)                        */
  __IO uint32_t  MSVW01_DW0;                        /*!< Master-to-Slave Virtual Wire 1 Register (DW 0)                        */
  __IO uint32_t  MSVW01_DW1;                        /*!< Master-to-Slave Virtual Wire 1 Register (DW 1)                        */
  __IO uint32_t  MSVW01_DW2;                        /*!< Master-to-Slave Virtual Wire 1 Register (DW 2)                        */
  __IO uint32_t  MSVW02_DW0;                        /*!< Master-to-Slave Virtual Wire 2 Register (DW 0)                        */
  __IO uint32_t  MSVW02_DW1;                        /*!< Master-to-Slave Virtual Wire 2 Register (DW 1)                        */
  __IO uint32_t  MSVW02_DW2;                        /*!< Master-to-Slave Virtual Wire 2 Register (DW 2)                        */
  __IO uint32_t  MSVW03_DW0;                        /*!< Master-to-Slave Virtual Wire 3 Register (DW 0)                        */
  __IO uint32_t  MSVW03_DW1;                        /*!< Master-to-Slave Virtual Wire 3 Register (DW 1)                        */
  __IO uint32_t  MSVW03_DW2;                        /*!< Master-to-Slave Virtual Wire 3 Register (DW 2)                        */
  __IO uint32_t  MSVW04_DW0;                        /*!< Master-to-Slave Virtual Wire 4 Register (DW 0)                        */
  __IO uint32_t  MSVW04_DW1;                        /*!< Master-to-Slave Virtual Wire 4 Register (DW 1)                        */
  __IO uint32_t  MSVW04_DW2;                        /*!< Master-to-Slave Virtual Wire 4 Register (DW 2)                        */
  __IO uint32_t  MSVW05_DW0;                        /*!< Master-to-Slave Virtual Wire 5 Register (DW 0)                        */
  __IO uint32_t  MSVW05_DW1;                        /*!< Master-to-Slave Virtual Wire 5 Register (DW 1)                        */
  __IO uint32_t  MSVW05_DW2;                        /*!< Master-to-Slave Virtual Wire 5 Register (DW 2)                        */
  __IO uint32_t  MSVW06_DW0;                        /*!< Master-to-Slave Virtual Wire 6 Register (DW 0)                        */
  __IO uint32_t  MSVW06_DW1;                        /*!< Master-to-Slave Virtual Wire 6 Register (DW 1)                        */
  __IO uint32_t  MSVW06_DW2;                        /*!< Master-to-Slave Virtual Wire 6 Register (DW 2)                        */
} ESPI_MSVW00_06_Type;


/* ================================================================================ */
/* ================                 ESPI_MSVW07_10                 ================ */
/* ================================================================================ */


/**
  * @brief The Virtual Wire Channel permits the System to emulate a set of wires that interconnect the system Core Logic with the EC.  (ESPI_MSVW07_10)
  */

typedef struct {                                    /*!< ESPI_MSVW07_10 Structure                                              */
  __IO uint32_t  MSVW07_DW0;                        /*!< Master-to-Slave Virtual Wire 7 Register (DW 0)                        */
  __IO uint32_t  MSVW07_DW1;                        /*!< Master-to-Slave Virtual Wire 7 Register (DW 1)                        */
  __IO uint32_t  MSVW07_DW2;                        /*!< Master-to-Slave Virtual Wire 7 Register (DW 2)                        */
  __IO uint32_t  MSVW08_DW0;                        /*!< Master-to-Slave Virtual Wire 8 Register (DW 0)                        */
  __IO uint32_t  MSVW08_DW1;                        /*!< Master-to-Slave Virtual Wire 8 Register (DW 1)                        */
  __IO uint32_t  MSVW08_DW2;                        /*!< Master-to-Slave Virtual Wire 8 Register (DW 2)                        */
  __IO uint32_t  MSVW09_DW0;                        /*!< Master-to-Slave Virtual Wire 9 Register (DW 0)                        */
  __IO uint32_t  MSVW09_DW1;                        /*!< Master-to-Slave Virtual Wire 9 Register (DW 1)                        */
  __IO uint32_t  MSVW09_DW2;                        /*!< Master-to-Slave Virtual Wire 9 Register (DW 2)                        */
  __IO uint32_t  MSVW10_DW0;                        /*!< Master-to-Slave Virtual Wire 10 Register (DW 0)                       */
  __IO uint32_t  MSVW10_DW1;                        /*!< Master-to-Slave Virtual Wire 10 Register (DW 1)                       */
  __IO uint32_t  MSVW10_DW2;                        /*!< Master-to-Slave Virtual Wire 10 Register (DW 2)                       */
} ESPI_MSVW07_10_Type;


/* ================================================================================ */
/* ================                 ESPI_SMVW00_07                 ================ */
/* ================================================================================ */


/**
  * @brief The Virtual Wire Channel permits the System to emulate a set of wires that interconnect the system Core Logic with the EC.  (ESPI_SMVW00_07)
  */

typedef struct {                                    /*!< ESPI_SMVW00_07 Structure                                              */
  __IO uint32_t  SMVW00_DW0;                        /*!< Slave-to-Master Virtual Wire 0 Register (DWord 0)                     */
  __IO uint32_t  SMVW00_DW1;                        /*!< Slave-to-Master Virtual Wire 0 Register (DWord 1)                     */
  __IO uint32_t  SMVW01_DW0;                        /*!< Slave-to-Master Virtual Wire 1 Register (DWord 0)                     */
  __IO uint32_t  SMVW01_DW1;                        /*!< Slave-to-Master Virtual Wire 1 Register (DWord 1)                     */
  __IO uint32_t  SMVW02_DW0;                        /*!< Slave-to-Master Virtual Wire 2 Register (DWord 0)                     */
  __IO uint32_t  SMVW02_DW1;                        /*!< Slave-to-Master Virtual Wire 2 Register (DWord 1)                     */
  __IO uint32_t  SMVW03_DW0;                        /*!< Slave-to-Master Virtual Wire 3 Register (DWord 0)                     */
  __IO uint32_t  SMVW03_DW1;                        /*!< Slave-to-Master Virtual Wire 3 Register (DWord 1)                     */
  __IO uint32_t  SMVW04_DW0;                        /*!< Slave-to-Master Virtual Wire 4 Register (DWord 0)                     */
  __IO uint32_t  SMVW04_DW1;                        /*!< Slave-to-Master Virtual Wire 4 Register (DWord 1)                     */
  __IO uint32_t  SMVW05_DW0;                        /*!< Slave-to-Master Virtual Wire 5 Register (DWord 0)                     */
  __IO uint32_t  SMVW05_DW1;                        /*!< Slave-to-Master Virtual Wire 5 Register (DWord 1)                     */
  __IO uint32_t  SMVW06_DW0;                        /*!< Slave-to-Master Virtual Wire 6 Register (DWord 0)                     */
  __IO uint32_t  SMVW06_DW1;                        /*!< Slave-to-Master Virtual Wire 6 Register (DWord 1)                     */
  __IO uint32_t  SMVW07_DW0;                        /*!< Slave-to-Master Virtual Wire 7 Register (DWord 0)                     */
  __IO uint32_t  SMVW07_DW1;                        /*!< Slave-to-Master Virtual Wire 7 Register (DWord 1)                     */
} ESPI_SMVW00_07_Type;


/* ================================================================================ */
/* ================                       GCR                      ================ */
/* ================================================================================ */


/**
  * @brief The Logical Device Configuration registers support motherboard designs in
 which the resources required by their components are known and assigned by the BIOS
 at POST.  (GCR)
  */

typedef struct {                                    /*!< GCR Structure                                                         */
  __I  uint8_t   RESERVED[7];
  __IO uint8_t   LOGICAL_DEVICE_NUMBER;             /*!< A write to this register selects the current logical device.
                                                         This allows access to the control and configuration
                                                          registers for each logical device. Note: The Activate command
                                                          operates only on the selected logical device.                        */
  __I  uint32_t  RESERVED1[6];
  __I  uint8_t   DEVICE_ID;                         /*!< A read-only register which provides device identification.            */
  __I  uint8_t   DEVICE_REVISION;                   /*!< A read-only register which provides device revision information.
                                                                                                                               */
} GCR_Type;


/* ================================================================================ */
/* ================                       KBC                      ================ */
/* ================================================================================ */


/**
  * @brief The keyboard controller uses the EC to produce a superset of the features provided by the
 industry-standard 8042 keyboard controller. The 8042 Emulated Keyboard Controller is a Host/EC
 Message Interface with hardware assists to emulate 8042 behavior and provide Legacy GATEA20 support.  (KBC)
  */

typedef struct {                                    /*!< KBC Structure                                                         */

  union {
    __O  uint8_t   HOST_EC_DATA;                    /*!< WRITE_DATA. This 8-bit register is write-only. When written,
                                                         the C/D bit in the Keyboard Status Read Register is cleared
                                                          to '0', signifying data, and the IBF in the same register is
                                                          set to '1'. When the Runtime Register at offset 0h is read by
                                                          the Host,
                                                          it functions as the EC_HOST Data / AUX Data Register.                */
    __I  uint8_t   EC_HOST_DATA_AUX_DATA;           /*!< READ_DATA. This 8-bit register is read-only. When read by the
                                                         Host, the PCOBF and/or AUXOBF interrupts are cleared and the
                                                          OBF flag in the status register is cleared.                          */
  };
  __I  uint8_t   RESERVED[3];

  union {
    __IO uint8_t   KBD_STATUS;                      /*!< Keyboard Status Read Register. This register is a read-only
                                                         alias of the EC Keyboard Status Register.                             */
    __O  uint8_t   HOST_EC_CMD;                     /*!< WRITE_CMD. This 8-bit register is write-only and is an alias
                                                         of the register at offset 0h. When written, the C/D bit in the
                                                          Keyboard Status Read Register is set to '1', signifying a command,
                                                          and the IBF in the same register is set to '1'. When the Runtime
                                                          Register at offset 4h is read by the Host, it functions as
                                                          the Keyboard Status Read Register.                                   */
  };
  __I  uint8_t   RESERVED1[251];

  union {
    __O  uint8_t   EC_DATA;                         /*!< EC_Host Data Register                                                 */
    __I  uint8_t   HOST2EC_DATA;                    /*!< Host_EC Data/Cmd Register This register is an alias of the HOST_EC
                                                         Data / CMD Register. When read at the EC-Only offset
                                                          of 0h, it returns the data written by the Host to either Runtime
                                                          Register offset 0h or Runtime Register offset 04h.                   */
  };
  __I  uint8_t   RESERVED2[3];
  __IO uint8_t   STATUS;                            /*!< Keyboard Status Read Register                                         */
  __I  uint8_t   RESERVED3[3];
  __IO uint8_t   CONTROL;                           /*!< Keyboard Control Register                                             */
  __I  uint8_t   RESERVED4[3];
  __O  uint8_t   EC_AUX_DATA;                       /*!< EC_Host Aux Register. This 8-bit register is write-only. When
                                                         written, the C/D in the Keyboard Status Read Register is cleared
                                                          to '0', signifying data, and the IBF in the same register is
                                                          set to '1'. When the Runtime Register at offset 0h is read by
                                                          the Host, it
                                                          functions as the EC_HOST Data / AUX Data Register.                   */
  __I  uint8_t   RESERVED5[7];
  __IO uint8_t   PCOBF;                             /*!< 8042 Emulated Keyboard Controller PCOBF Register                      */
  __I  uint8_t   RESERVED6[539];
  __IO uint8_t   ACTIVATE;                          /*!< Activate Register                                                     */
} KBC_Type;


/* ================================================================================ */
/* ================                     PORT92                     ================ */
/* ================================================================================ */


/**
  * @brief The registers listed in the Configuration Register Summary table are for a single instance of the Legacy Port92/GATEA20 logic.  (PORT92)
  */

typedef struct {                                    /*!< PORT92 Structure                                                      */
  __IO uint8_t   PORT92;                            /*!< PORT92 Register: The registers listed in the Runtime Register
                                                         Summary table are for a single instance of the Legacy Port92/GATEA20
                                                          logic.                                                               */
  __I  uint8_t   RESERVED[255];
  __IO uint8_t   GATEA20;                           /*!< GATEA20 Control Register                                              */
  __I  uint8_t   RESERVED1[7];
  __O  uint8_t   SETGA20L;                          /*!< SETGA20L Register. A write to this register sets GATEA20 in
                                                         the GATEA20 Control Register.                                         */
  __I  uint8_t   RESERVED2[3];
  __IO uint8_t   RSTGA20L;                          /*!< RSTGA20L Register. A write to this register sets GATEA20 in
                                                         the GATEA20 Control Register.                                         */
  __I  uint8_t   RESERVED3[547];
  __IO uint8_t   PORT92_ENABLE;                     /*!< PORT92 Enable Register                                                */
} PORT92_Type;


/* ================================================================================ */
/* ================                    ACPI_EC0                    ================ */
/* ================================================================================ */


/**
  * @brief The ACPI Embedded Controller Interface (ACPI-ECI) provides a four byte full
 duplex data interface which is a superset of the standard ACPI Embedded Controller Interface
 (ACPI-ECI) one byte data interface. The ACPI Embedded Controller Interface (ACPI-ECI) defaults
 to the standard one byte interface.  (ACPI_EC0)
  */

typedef struct {                                    /*!< ACPI_EC0 Structure                                                    */
  __IO uint8_t   ACPI_OS_DATA_BYTE_[4];             /*!< This is byte n of the 32-bit ACPI-OS DATA BYTES[3:0]. Writes
                                                         by the ACPI_OS to the ACPI-OS DATA BYTES[n] are aliased
                                                         to the OS2EC DATA BYTES[n]. Reads by the ACPI_OS from the ACPI-OS
                                                          DATA BYTES[n] are aliased to the EC2OS DATA BYTES[n].                */

  union {
    __I  uint8_t   OS_STATUS;                       /*!< OS STATUS                                                             */
    __O  uint8_t   ACPI_OS_COMMAND;                 /*!< Writes to the this register are aliased in the OS2EC Data EC
                                                         Byte 0 Register.
                                                          Writes to this register also set the CMD and IBF bits in the
                                                          OS STATUS OS Register                                                */
  };
  __I  uint8_t   OS_BYTE_CONTROL;                   /*!< OS Byte Control Register                                              */
  __I  uint16_t  RESERVED[125];
  __IO uint8_t   EC2OS_DATA_EC_BYTE_[4];            /*!< This is byte n of the 32-bit EC2OS DATA BYTES[3:0]. Writes by
                                                         the ACPI_EC to the EC2OS DATA BYTES[3:0] are aliased to the
                                                         ACPI-OS DATA BYTES[3:0].                                              */
  __IO uint8_t   EC_STATUS;                         /*!< EC STATUS                                                             */
  __IO uint8_t   EC_BYTE_CONTROL;                   /*!< Byte Control EC-Register                                              */
  __I  uint16_t  RESERVED1;
  __IO uint8_t   OS2EC_DATA_EC_BYTE_[4];            /*!< OS_TO_EC_DATA_BYTE_n. This is byte n of the 32-bit OS2EC DATA
                                                         BYTES[3:0]. When the CMD bit in the OS STATUS OS Register
                                                         is cleared to '0', reads by the ACPI_EC from the OS2EC DATA
                                                          BYTES[3:0] are aliased to the ACPI-OS DATA BYTES[3:0].               */
} ACPI_EC0_Type;


/* ================================================================================ */
/* ================                       PM1                      ================ */
/* ================================================================================ */


/**
  * @brief The device implements the ACPI fixed registers but includes only those bits
 that apply to the power button
 sleep button and RTC alarm events. The ACPI WAK_STS, SLP_TYP and SLP_EN bits are also supported.  (PM1)
  */

typedef struct {                                    /*!< PM1 Structure                                                         */
  __I  uint8_t   RESERVED;
  __IO uint8_t   PM1_STS2;                          /*!< PM1 Status 2                                                          */
  __I  uint8_t   RESERVED1;
  __IO uint8_t   PM1_EN2;                           /*!< PM1 Enable 2                                                          */
  __I  uint8_t   RESERVED2;
  __IO uint8_t   PM1_CTRL2;                         /*!< PM1 Control 2                                                         */
  __I  uint8_t   RESERVED3[251];
  __IO uint8_t   PM1_STS_2;                         /*!< PM1 Status 2                                                          */
  __I  uint8_t   RESERVED4;
  __IO uint8_t   PM1_EN_2;                          /*!< PM1 Enable 2                                                          */
  __I  uint8_t   RESERVED5;
  __IO uint8_t   PM1_CTRL_2;                        /*!< PM1 Control 2                                                         */
  __I  uint16_t  RESERVED6[5];
  __IO uint8_t   PM_STS;                            /*!< PM1 EC PM Status                                                      */
} PM1_Type;


/* ================================================================================ */
/* ================                      EMI0                      ================ */
/* ================================================================================ */


/**
  * @brief The Embedded Memory Interface (EMI) provides a standard run-time mechanism
 for the system host to communicate with the Embedded Controller (EC) and other logical components.  (EMI0)
  */

typedef struct {                                    /*!< EMI0 Structure                                                        */
  __IO uint8_t   HOST_EC_MBX;                       /*!< Host-to-EC Mailbox Register                                           */
  __IO uint8_t   EC_HOST_MBX;                       /*!< EC-to-Host Mailbox Register                                           */
  __IO uint8_t   EC_ADDRESS_LSB;                    /*!< EC Address Access Control Register                                    */
  __IO uint8_t   EC_ADDRESS_MSB;                    /*!< EC Address Access Control Register                                    */
  __IO uint8_t   EC_DATA_BYTE[4];                   /*!< EC Data Byte Register                                                 */
  __IO uint8_t   EC_INT_SOURCE_LSB;                 /*!< Interrupt Source LSB Register                                         */
  __IO uint8_t   EC_INT_SOURCE_MSB;                 /*!< Interrupt Source MSB Register                                         */
  __IO uint8_t   EC_INT_MASK_LSB;                   /*!< Interrupt Mask LSB Register                                           */
  __IO uint8_t   EC_INT_MASK_MSB;                   /*!< Interrupt Mask MSB Register                                           */
  __IO uint8_t   APPLICATION_ID;                    /*!< Application ID Register, APPLICATION_ID When this field is 00h
                                                         it can be written with any value. When set to a non-zero value,
                                                          writing that value will clear this register to 00h. When set
                                                          to a non-zero value, writing any value other than the current
                                                          contents will have no effect.                                        */
  __I  uint8_t   RESERVED[243];
  __IO uint8_t   HOST2EC_MBX;                       /*!< Host-to-EC Mailbox Register, 8-bit mailbox used communicate
                                                         information from the system host to the embedded controller.
                                                          Writing this register generates an event to notify the embedded
                                                          controller. (R/WC)                                                   */
  __IO uint8_t   EC2HOST_MBX;                       /*!< EC-to-Host Mailbox Register, 8-bit mailbox used communicate
                                                         information from the embedded controller to the system host.
                                                          Writing this register generates an event to notify the system
                                                          host.                                                                */
  __I  uint16_t  RESERVED1;
  __IO uint32_t  MEMORY_BASE_ADDRESS_0;             /*!< Memory Base Address 0 Register [31:2] This memory base address
                                                         defines the beginning of region 0 in the Embedded Controller's
                                                          32-bit internal address space. Memory allocated to region 0
                                                          is intended to be shared between the Host and the EC. The region
                                                          defined by this base register is used when bit 15 of the EC
                                                          Address Register is 0. The access will be to a memory location
                                                          at an offset defined by the EC_Address relative to the beginning
                                                          of the region defined by this register. Therefore, a read or
                                                          write to th                                                          */
  __IO uint16_t  MEMORY_READ_LIMIT_0;               /*!< Memory Read Limit 0 Register [14:2] Whenever a read of any byte
                                                         in the EC Data Register is attempted, and bit 15 of EC_Address
                                                          is 0, the field EC_Address[14:2] in the EC_Address_Register
                                                          is compared to this field. As long as EC_Address[14:2] is less
                                                          than this field the EC_Data_Register will be loaded from the
                                                          24-bit internal address space.                                       */
  __IO uint16_t  MEMORY_WRITE_LIMIT_0;              /*!< Memory Write Limit 0 Register [14:2] Whenever a write of any
                                                         byte in EC DATA Register is attempted and bit 15 of EC_Address
                                                          is 0, the field EC_ADDRESS_MSB in the EC_Address Register is
                                                          compared to this field. As long as EC_Address[14:2] is less
                                                          than Memory_Write_Limit_0[14:2] the addressed bytes in the EC
                                                          DATA Register will be written into the internal 24-bit address
                                                          space. If EC_Address[14:2] is greater than or equal to the Memory_Write_Li
                                                         mit_0[14:2] no writes will take place.                                */
  __IO uint32_t  MEMORY_BASE_ADDRESS_1;             /*!< Memory Base Address 1 Register. [31:2] This memory base address
                                                         defines the beginning of region 1 in the Embedded Controller's
                                                          32-bit internal address space. Memory allocated to region 1
                                                          is intended to be shared between the Host and the EC. The region
                                                          defined by this base register is used when bit 15 of the EC
                                                          Address Register is 1. The access will be to a memory location
                                                          at an offset defined by the EC_Address relative to the beginning
                                                          of the region defined by this register. Therefore, a read or
                                                          write to t                                                           */
  __IO uint16_t  MEMORY_READ_LIMIT_1;               /*!< Memory Read Limit 1 Register, [14:2]: Whenever a read of any
                                                         byte in the EC Data Register is attempted, and bit 15 of EC_ADDRESS
                                                          is 1, the field EC_ADDRESS in the EC_Address_Register is compared
                                                          to this field. As long as EC_ADDRESS is less than this value,
                                                          the EC_Data_Register will be loaded from the 24-bit internal
                                                          address space.                                                       */
  __IO uint16_t  MEMORY_WRITE_LIMIT_1;              /*!< Memory Write Limit 1 Register, [14:2]: Whenever a write of any
                                                         byte in EC DATA Register is attempted and bit 15 of EC_Address
                                                          is 1, the field EC_Address[14:2] in the EC_Address Register
                                                          is compared to this field. As long as EC_Address[14:2] is less
                                                          than Memory_Write_Limit_1[14:2] the addressed bytes in the EC
                                                          DATA Register will be written into the internal 24-bit address
                                                          space. If EC_Address[14:2] is greater than or equal to the Memory_Write_Li
                                                         mit_1[14:2] no writes will take place.                                */
  __IO uint16_t  EC_SWI_SET;                        /*!< [15:1] Interrupt Set Register, Writing a bit in this field with
                                                         a '1b' sets the corresponding bit in the Interrupt Source Register
                                                          to '1b'. Writing a bit in this field with a '0b' has no effect.
                                                          Reading this field returns the current contents of the Interrupt
                                                          Source Register.                                                     */
  __IO uint16_t  CLEAR_ENABLE;                      /*!< [15:1] Host Clear Enable Register, When a bit in this field
                                                         is '0b', the corresponding bit in the Interrupt Source Register
                                                          cannot be cleared by writes to the Interrupt Source Register.
                                                          When a bit in this field is '1b', the corresponding bit in the
                                                          Interrupt Source Register can be cleared when that register
                                                          bit is written with a '1b'.                                          */
} EMI0_Type;


/* ================================================================================ */
/* ================                       MBX                      ================ */
/* ================================================================================ */


/**
  * @brief The Mailbox provides a standard run-time mechanism for the host to communicate with the Embedded Controller (EC).  (MBX)
  */

typedef struct {                                    /*!< MBX Structure                                                         */
  __IO uint8_t   INDEX;                             /*!< MBX_Index Register                                                    */
  __I  uint8_t   RESERVED[3];
  __IO uint8_t   DATA;                              /*!< MBX_Data_Register                                                     */
  __I  uint8_t   RESERVED1[251];
  __IO uint32_t  HOST_TO_EC;                        /*!< If enabled, an interrupt to the EC marked by the MBX_DATA bit
                                                         in the Interrupt Aggregator will be generated whenever the Host
                                                          writes this register. This register is cleared when written
                                                          with FFh.                                                            */
  __IO uint8_t   EC_TO_HOST;                        /*!< An EC write to this register will set bit EC_WR in the SMI Interrupt
                                                         Source Register to '1b'. If enabled, this will generate a Host
                                                          SMI. This register is cleared when written with FFh.                 */
  __I  uint8_t   RESERVED2[3];
  __IO uint32_t  SMI_SOURCE;                        /*!< SMI Interrupt Source Register                                         */
  __IO uint32_t  SMI_MASK;                          /*!< SMI Interrupt Mask Register                                           */
  __IO uint32_t  MBX_REG[8];                        /*!< Mailbox Register                                                      */
} MBX_Type;


/* ================================================================================ */
/* ================                      UART0                     ================ */
/* ================================================================================ */


/**
  * @brief The 16550 UART (Universal Asynchronous Receiver/Transmitter) is a
 full-function Two Pin Serial Port that supports the standard RS-232 Interface.  (UART0)
  */

typedef struct {                                    /*!< UART0 Structure                                                       */

  union {
    __IO uint8_t   BAUDRATE_LSB;                    /*!< UART Programmable BAUD Rate Generator (LSB) Register (DLAB=1)
                                                                                                                               */
    __IO uint8_t   RX_DATA_TX_DATA;                 /*!< UART Receive (Read) / Transmit (Write) Buffer Register (DLAB=0)
                                                                                                                               */
  };

  union {
    __IO uint8_t   INT_EN;                          /*!< UART Interrupt Enable Register (DLAB=0)                               */
    __IO uint8_t   BAUDRATE_MSB;                    /*!< UART Programmable BAUD Rate Generator (MSB) Register (DLAB=1).
                                                         [6:0] BAUD_RATE_DIVISOR_MSB, [7:7] BAUD_CLK_SEL
                                                          1=If CLK_SRC is '0', the baud clock is derived from the 1.8432MHz_Clk.
                                                          If CLK_SRC is '1', this bit has no effect
                                                          0=If CLK_SRC is '0', the baud clock is derived from the 24MHz_Clk.
                                                          If CLK_SRC is '1', this bit has no effect                            */
  };

  union {
    __IO uint8_t   INT_ID;                          /*!< UART Interrupt Identification Register                                */
    __IO uint8_t   FIFO_CR;                         /*!< UART FIFO Control Register                                            */
  };
  __IO uint8_t   LINE_CR;                           /*!< UART Line Control Register                                            */
  __IO uint8_t   MODEM_CR;                          /*!< UART Modem Control Register                                           */
  __I  uint8_t   LINE_STS;                          /*!< UART Line Status Register                                             */
  __I  uint8_t   MODEM_STS;                         /*!< UART Modem Status Register                                            */
  __IO uint8_t   SCRATCHPAD;                        /*!< UART Scratchpad Register This 8 bit read/write register has
                                                         no effect on the operation of the Serial Port. It is intended
                                                          as a scratchpad register to be used by the programmer to hold
                                                          data temporarily.                                                    */
  __I  uint32_t  RESERVED[202];
  __IO uint8_t   ACTIVATE;                          /*!< UART Activate Register. [0:0] ACTIVATE When this bit is 1, the
                                                         UART logical device is powered and functional. When this bit
                                                          is 0, the UART logical device is powered down and inactive.
                                                                                                                               */
  __I  uint8_t   RESERVED1[191];
  __IO uint8_t   CONFIG;                            /*!< UART Config Select Register                                           */
} UART0_Type;


/* ================================================================================ */
/* ================                  GPIO_000_036                  ================ */
/* ================================================================================ */


/**
  * @brief GPIO 000:036 Pin Control Registers (GPIO_000_036)
  */

typedef struct {                                    /*!< GPIO_000_036 Structure                                                */
  __IO uint32_t  GPIO_000_PIN_CONTROL;              /*!< GPIO000 Pin Control                                                   */
  __IO uint32_t  GPIO_001_PIN_CONTROL;              /*!< GPIO 001 Pin Control                                                  */
  __IO uint32_t  GPIO_002_PIN_CONTROL;              /*!< GPIO 002 Pin Control                                                  */
  __IO uint32_t  GPIO_003_PIN_CONTROL;              /*!< GPIO 003 Pin Control                                                  */
  __IO uint32_t  GPIO_004_PIN_CONTROL;              /*!< GPIO 004 Pin Control                                                  */
  __IO uint32_t  GPIO_005_PIN_CONTROL;              /*!< GPIO 005 Pin Control                                                  */
  __IO uint32_t  GPIO_006_PIN_CONTROL;              /*!< GPIO 006 Pin Control                                                  */
  __IO uint32_t  GPIO_007_PIN_CONTROL;              /*!< GPIO 007 Pin Control                                                  */
  __IO uint32_t  GPIO_010_PIN_CONTROL;              /*!< GPIO 010 Pin Control                                                  */
  __IO uint32_t  GPIO_011_PIN_CONTROL;              /*!< GPIO 011 Pin Control                                                  */
  __IO uint32_t  GPIO_012_PIN_CONTROL;              /*!< GPIO 012 Pin Control                                                  */
  __IO uint32_t  GPIO_013_PIN_CONTROL;              /*!< GPIO 013 Pin Control                                                  */
  __IO uint32_t  GPIO_014_PIN_CONTROL;              /*!< GPIO 014 Pin Control                                                  */
  __IO uint32_t  GPIO_015_PIN_CONTROL;              /*!< GPIO 015 Pin Control                                                  */
  __IO uint32_t  GPIO_016_PIN_CONTROL;              /*!< GPIO 016 Pin Control                                                  */
  __IO uint32_t  GPIO_017_PIN_CONTROL;              /*!< GPIO 017 Pin Control                                                  */
  __IO uint32_t  GPIO_020_PIN_CONTROL;              /*!< GPIO 020 Pin Control                                                  */
  __IO uint32_t  GPIO_021_PIN_CONTROL;              /*!< GPIO 021 Pin Control                                                  */
  __IO uint32_t  GPIO_022_PIN_CONTROL;              /*!< GPIO 022 Pin Control                                                  */
  __IO uint32_t  GPIO_023_PIN_CONTROL;              /*!< GPIO 023 Pin Control                                                  */
  __IO uint32_t  GPIO_024_PIN_CONTROL;              /*!< GPIO 024 Pin Control                                                  */
  __IO uint32_t  GPIO_025_PIN_CONTROL;              /*!< GPIO 025 Pin Control                                                  */
  __IO uint32_t  GPIO_026_PIN_CONTROL;              /*!< GPIO 026 Pin Control                                                  */
  __IO uint32_t  GPIO_027_PIN_CONTROL;              /*!< GPIO 027 Pin Control                                                  */
  __IO uint32_t  GPIO_030_PIN_CONTROL;              /*!< GPIO 030 Pin Control                                                  */
  __IO uint32_t  GPIO_031_PIN_CONTROL;              /*!< GPIO 031 Pin Control                                                  */
  __IO uint32_t  GPIO_032_PIN_CONTROL;              /*!< GPIO 032 Pin Control                                                  */
  __IO uint32_t  GPIO_033_PIN_CONTROL;              /*!< GPIO 033 Pin Control                                                  */
  __IO uint32_t  GPIO_034_PIN_CONTROL;              /*!< GPIO 034 Pin Control                                                  */
  __IO uint32_t  GPIO_035_PIN_CONTROL;              /*!< GPIO 035 Pin Control                                                  */
  __IO uint32_t  GPIO_036_PIN_CONTROL;              /*!< GPIO 036 Pin Control                                                  */
} GPIO_000_036_Type;


/* ================================================================================ */
/* ================                  GPIO_040_076                  ================ */
/* ================================================================================ */


/**
  * @brief GPIO 040:076 Pin Control Registers (GPIO_040_076)
  */

typedef struct {                                    /*!< GPIO_040_076 Structure                                                */
  __IO uint32_t  GPIO_040_PIN_CONTROL;              /*!< GPIO040 Pin Control                                                   */
  __IO uint32_t  GPIO_041_PIN_CONTROL;              /*!< GPIO 041 Pin Control                                                  */
  __IO uint32_t  GPIO_042_PIN_CONTROL;              /*!< GPIO 042 Pin Control                                                  */
  __IO uint32_t  GPIO_043_PIN_CONTROL;              /*!< GPIO 043 Pin Control                                                  */
  __IO uint32_t  GPIO_044_PIN_CONTROL;              /*!< GPIO 044 Pin Control                                                  */
  __IO uint32_t  GPIO_045_PIN_CONTROL;              /*!< GPIO 045 Pin Control                                                  */
  __IO uint32_t  GPIO_046_PIN_CONTROL;              /*!< GPIO 046 Pin Control                                                  */
  __IO uint32_t  GPIO_047_PIN_CONTROL;              /*!< GPIO 047 Pin Control                                                  */
  __IO uint32_t  GPIO_050_PIN_CONTROL;              /*!< GPIO 050 Pin Control                                                  */
  __IO uint32_t  GPIO_051_PIN_CONTROL;              /*!< GPIO 051 Pin Control                                                  */
  __IO uint32_t  GPIO_052_PIN_CONTROL;              /*!< GPIO 052 Pin Control                                                  */
  __IO uint32_t  GPIO_053_PIN_CONTROL;              /*!< GPIO 053 Pin Control                                                  */
  __IO uint32_t  GPIO_054_PIN_CONTROL;              /*!< GPIO 054 Pin Control                                                  */
  __IO uint32_t  GPIO_055_PIN_CONTROL;              /*!< GPIO 055 Pin Control                                                  */
  __IO uint32_t  GPIO_056_PIN_CONTROL;              /*!< GPIO 056 Pin Control                                                  */
  __IO uint32_t  GPIO_057_PIN_CONTROL;              /*!< GPIO 057 Pin Control                                                  */
  __IO uint32_t  GPIO_060_PIN_CONTROL;              /*!< GPIO 060 Pin Control                                                  */
  __IO uint32_t  GPIO_061_PIN_CONTROL;              /*!< GPIO 061 Pin Control                                                  */
  __IO uint32_t  GPIO_062_PIN_CONTROL;              /*!< GPIO 062 Pin Control                                                  */
  __IO uint32_t  GPIO_063_PIN_CONTROL;              /*!< GPIO 063 Pin Control                                                  */
  __IO uint32_t  GPIO_064_PIN_CONTROL;              /*!< GPIO 064 Pin Control                                                  */
  __IO uint32_t  GPIO_065_PIN_CONTROL;              /*!< GPIO 065 Pin Control                                                  */
  __IO uint32_t  GPIO_066_PIN_CONTROL;              /*!< GPIO 066 Pin Control                                                  */
  __IO uint32_t  GPIO_067_PIN_CONTROL;              /*!< GPIO 067 Pin Control                                                  */
  __IO uint32_t  GPIO_070_PIN_CONTROL;              /*!< GPIO 070 Pin Control                                                  */
  __IO uint32_t  GPIO_071_PIN_CONTROL;              /*!< GPIO 071 Pin Control                                                  */
  __IO uint32_t  GPIO_072_PIN_CONTROL;              /*!< GPIO 072 Pin Control                                                  */
  __IO uint32_t  GPIO_073_PIN_CONTROL;              /*!< GPIO 073 Pin Control                                                  */
  __IO uint32_t  GPIO_074_PIN_CONTROL;              /*!< GPIO 074 Pin Control                                                  */
  __IO uint32_t  GPIO_075_PIN_CONTROL;              /*!< GPIO 075 Pin Control                                                  */
  __IO uint32_t  GPIO_076_PIN_CONTROL;              /*!< GPIO 076 Pin Control                                                  */
} GPIO_040_076_Type;


/* ================================================================================ */
/* ================                  GPIO_100_137                  ================ */
/* ================================================================================ */


/**
  * @brief GPIO 100:137 Pin Control Registers (GPIO_100_137)
  */

typedef struct {                                    /*!< GPIO_100_137 Structure                                                */
  __IO uint32_t  GPIO_100_PIN_CONTROL;              /*!< GPIO100 Pin Control                                                   */
  __IO uint32_t  GPIO_101_PIN_CONTROL;              /*!< GPIO 101 Pin Control                                                  */
  __IO uint32_t  GPIO_102_PIN_CONTROL;              /*!< GPIO 102 Pin Control                                                  */
  __IO uint32_t  GPIO_103_PIN_CONTROL;              /*!< GPIO 103 Pin Control                                                  */
  __IO uint32_t  GPIO_104_PIN_CONTROL;              /*!< GPIO 104 Pin Control                                                  */
  __IO uint32_t  GPIO_105_PIN_CONTROL;              /*!< GPIO 105 Pin Control                                                  */
  __IO uint32_t  GPIO_106_PIN_CONTROL;              /*!< GPIO 106 Pin Control                                                  */
  __IO uint32_t  GPIO_107_PIN_CONTROL;              /*!< GPIO 107 Pin Control                                                  */
  __IO uint32_t  GPIO_110_PIN_CONTROL;              /*!< GPIO 110 Pin Control                                                  */
  __IO uint32_t  GPIO_111_PIN_CONTROL;              /*!< GPIO 111 Pin Control                                                  */
  __IO uint32_t  GPIO_112_PIN_CONTROL;              /*!< GPIO 112 Pin Control                                                  */
  __IO uint32_t  GPIO_113_PIN_CONTROL;              /*!< GPIO 113 Pin Control                                                  */
  __IO uint32_t  GPIO_114_PIN_CONTROL;              /*!< GPIO 114 Pin Control                                                  */
  __IO uint32_t  GPIO_115_PIN_CONTROL;              /*!< GPIO 115 Pin Control                                                  */
  __IO uint32_t  GPIO_116_PIN_CONTROL;              /*!< GPIO 116 Pin Control                                                  */
  __IO uint32_t  GPIO_117_PIN_CONTROL;              /*!< GPIO 117 Pin Control                                                  */
  __IO uint32_t  GPIO_120_PIN_CONTROL;              /*!< GPIO 120 Pin Control                                                  */
  __IO uint32_t  GPIO_121_PIN_CONTROL;              /*!< GPIO 121 Pin Control                                                  */
  __IO uint32_t  GPIO_122_PIN_CONTROL;              /*!< GPIO 122 Pin Control                                                  */
  __IO uint32_t  GPIO_123_PIN_CONTROL;              /*!< GPIO 123 Pin Control                                                  */
  __IO uint32_t  GPIO_124_PIN_CONTROL;              /*!< GPIO 124 Pin Control                                                  */
  __IO uint32_t  GPIO_125_PIN_CONTROL;              /*!< GPIO 125 Pin Control                                                  */
  __IO uint32_t  GPIO_126_PIN_CONTROL;              /*!< GPIO 126 Pin Control                                                  */
  __IO uint32_t  GPIO_127_PIN_CONTROL;              /*!< GPIO 127 Pin Control                                                  */
  __IO uint32_t  GPIO_130_PIN_CONTROL;              /*!< GPIO 130 Pin Control                                                  */
  __IO uint32_t  GPIO_131_PIN_CONTROL;              /*!< GPIO 131 Pin Control                                                  */
  __IO uint32_t  GPIO_132_PIN_CONTROL;              /*!< GPIO 132 Pin Control                                                  */
  __IO uint32_t  GPIO_133_PIN_CONTROL;              /*!< GPIO 133 Pin Control                                                  */
  __IO uint32_t  GPIO_134_PIN_CONTROL;              /*!< GPIO 134 Pin Control                                                  */
  __IO uint32_t  GPIO_135_PIN_CONTROL;              /*!< GPIO 135 Pin Control                                                  */
  __IO uint32_t  GPIO_136_PIN_CONTROL;              /*!< GPIO 136 Pin Control                                                  */
} GPIO_100_137_Type;


/* ================================================================================ */
/* ================                  GPIO_140_176                  ================ */
/* ================================================================================ */


/**
  * @brief GPIO 140:176 Pin Control Registers (GPIO_140_176)
  */

typedef struct {                                    /*!< GPIO_140_176 Structure                                                */
  __IO uint32_t  GPIO_140_PIN_CONTROL;              /*!< GPIO140 Pin Control                                                   */
  __IO uint32_t  GPIO_141_PIN_CONTROL;              /*!< GPIO 141 Pin Control                                                  */
  __IO uint32_t  GPIO_142_PIN_CONTROL;              /*!< GPIO 142 Pin Control                                                  */
  __IO uint32_t  GPIO_143_PIN_CONTROL;              /*!< GPIO 143 Pin Control                                                  */
  __IO uint32_t  GPIO_144_PIN_CONTROL;              /*!< GPIO 144 Pin Control                                                  */
  __IO uint32_t  GPIO_145_PIN_CONTROL;              /*!< GPIO 145 Pin Control                                                  */
  __IO uint32_t  GPIO_146_PIN_CONTROL;              /*!< GPIO 146 Pin Control                                                  */
  __IO uint32_t  GPIO_147_PIN_CONTROL;              /*!< GPIO 147 Pin Control                                                  */
  __IO uint32_t  GPIO_150_PIN_CONTROL;              /*!< GPIO 150 Pin Control                                                  */
  __IO uint32_t  GPIO_151_PIN_CONTROL;              /*!< GPIO 151 Pin Control                                                  */
  __IO uint32_t  GPIO_152_PIN_CONTROL;              /*!< GPIO 152 Pin Control                                                  */
  __IO uint32_t  GPIO_153_PIN_CONTROL;              /*!< GPIO 153 Pin Control                                                  */
  __IO uint32_t  GPIO_154_PIN_CONTROL;              /*!< GPIO 154 Pin Control                                                  */
  __IO uint32_t  GPIO_155_PIN_CONTROL;              /*!< GPIO 155 Pin Control                                                  */
  __IO uint32_t  GPIO_156_PIN_CONTROL;              /*!< GPIO 156 Pin Control                                                  */
  __IO uint32_t  GPIO_157_PIN_CONTROL;              /*!< GPIO 157 Pin Control                                                  */
  __IO uint32_t  GPIO_160_PIN_CONTROL;              /*!< GPIO 160 Pin Control                                                  */
  __IO uint32_t  GPIO_161_PIN_CONTROL;              /*!< GPIO 161 Pin Control                                                  */
  __IO uint32_t  GPIO_162_PIN_CONTROL;              /*!< GPIO 162 Pin Control                                                  */
  __IO uint32_t  GPIO_163_PIN_CONTROL;              /*!< GPIO 163 Pin Control                                                  */
  __IO uint32_t  GPIO_164_PIN_CONTROL;              /*!< GPIO 164 Pin Control                                                  */
  __IO uint32_t  GPIO_165_PIN_CONTROL;              /*!< GPIO 165 Pin Control                                                  */
  __IO uint32_t  GPIO_166_PIN_CONTROL;              /*!< GPIO 166 Pin Control                                                  */
  __IO uint32_t  GPIO_167_PIN_CONTROL;              /*!< GPIO 167 Pin Control                                                  */
  __IO uint32_t  GPIO_170_PIN_CONTROL;              /*!< GPIO 170 Pin Control                                                  */
  __IO uint32_t  GPIO_171_PIN_CONTROL;              /*!< GPIO 171 Pin Control                                                  */
  __IO uint32_t  GPIO_172_PIN_CONTROL;              /*!< GPIO 172 Pin Control                                                  */
  __IO uint32_t  GPIO_173_PIN_CONTROL;              /*!< GPIO 173 Pin Control                                                  */
  __IO uint32_t  GPIO_174_PIN_CONTROL;              /*!< GPIO 174 Pin Control                                                  */
  __IO uint32_t  GPIO_175_PIN_CONTROL;              /*!< GPIO 175 Pin Control                                                  */
  __IO uint32_t  GPIO_176_PIN_CONTROL;              /*!< GPIO 176 Pin Control                                                  */
} GPIO_140_176_Type;


/* ================================================================================ */
/* ================                  GPIO_200_236                  ================ */
/* ================================================================================ */


/**
  * @brief GPIO 200:236 Pin Control Registers (GPIO_200_236)
  */

typedef struct {                                    /*!< GPIO_200_236 Structure                                                */
  __IO uint32_t  GPIO_200_PIN_CONTROL;              /*!< GPIO200 Pin Control                                                   */
  __IO uint32_t  GPIO_201_PIN_CONTROL;              /*!< GPIO 201 Pin Control                                                  */
  __IO uint32_t  GPIO_202_PIN_CONTROL;              /*!< GPIO 202 Pin Control                                                  */
  __IO uint32_t  GPIO_203_PIN_CONTROL;              /*!< GPIO 203 Pin Control                                                  */
  __IO uint32_t  GPIO_204_PIN_CONTROL;              /*!< GPIO 204 Pin Control                                                  */
  __IO uint32_t  GPIO_205_PIN_CONTROL;              /*!< GPIO 205 Pin Control                                                  */
  __IO uint32_t  GPIO_206_PIN_CONTROL;              /*!< GPIO 206 Pin Control                                                  */
  __IO uint32_t  GPIO_207_PIN_CONTROL;              /*!< GPIO 207 Pin Control                                                  */
  __IO uint32_t  GPIO_210_PIN_CONTROL;              /*!< GPIO 210 Pin Control                                                  */
  __IO uint32_t  GPIO_211_PIN_CONTROL;              /*!< GPIO 211 Pin Control                                                  */
  __IO uint32_t  GPIO_212_PIN_CONTROL;              /*!< GPIO 212 Pin Control                                                  */
  __IO uint32_t  GPIO_213_PIN_CONTROL;              /*!< GPIO 213 Pin Control                                                  */
  __IO uint32_t  GPIO_214_PIN_CONTROL;              /*!< GPIO 214 Pin Control                                                  */
  __IO uint32_t  GPIO_215_PIN_CONTROL;              /*!< GPIO 215 Pin Control                                                  */
  __IO uint32_t  GPIO_216_PIN_CONTROL;              /*!< GPIO 216 Pin Control                                                  */
  __IO uint32_t  GPIO_217_PIN_CONTROL;              /*!< GPIO 217 Pin Control                                                  */
  __IO uint32_t  GPIO_220_PIN_CONTROL;              /*!< GPIO 220 Pin Control                                                  */
  __IO uint32_t  GPIO_221_PIN_CONTROL;              /*!< GPIO 221 Pin Control                                                  */
  __IO uint32_t  GPIO_222_PIN_CONTROL;              /*!< GPIO 222 Pin Control                                                  */
  __IO uint32_t  GPIO_223_PIN_CONTROL;              /*!< GPIO 223 Pin Control                                                  */
  __IO uint32_t  GPIO_224_PIN_CONTROL;              /*!< GPIO 224 Pin Control                                                  */
  __IO uint32_t  GPIO_225_PIN_CONTROL;              /*!< GPIO 225 Pin Control                                                  */
  __IO uint32_t  GPIO_226_PIN_CONTROL;              /*!< GPIO 226 Pin Control                                                  */
  __IO uint32_t  GPIO_227_PIN_CONTROL;              /*!< GPIO 227 Pin Control                                                  */
  __IO uint32_t  GPIO_230_PIN_CONTROL;              /*!< GPIO 230 Pin Control                                                  */
  __IO uint32_t  GPIO_231_PIN_CONTROL;              /*!< GPIO 231 Pin Control                                                  */
  __IO uint32_t  GPIO_232_PIN_CONTROL;              /*!< GPIO 232 Pin Control                                                  */
  __IO uint32_t  GPIO_233_PIN_CONTROL;              /*!< GPIO 233 Pin Control                                                  */
  __IO uint32_t  GPIO_234_PIN_CONTROL;              /*!< GPIO 234 Pin Control                                                  */
  __IO uint32_t  GPIO_235_PIN_CONTROL;              /*!< GPIO 235 Pin Control                                                  */
  __IO uint32_t  GPIO_236_PIN_CONTROL;              /*!< GPIO 236 Pin Control                                                  */
} GPIO_200_236_Type;


/* ================================================================================ */
/* ================                  GPIO_240_257                  ================ */
/* ================================================================================ */


/**
  * @brief GPIO 240:257 Pin Control Registers (GPIO_240_257)
  */

typedef struct {                                    /*!< GPIO_240_257 Structure                                                */
  __IO uint32_t  GPIO_240_PIN_CONTROL;              /*!< GPIO240 Pin Control                                                   */
  __IO uint32_t  GPIO_241_PIN_CONTROL;              /*!< GPIO 241 Pin Control                                                  */
  __IO uint32_t  GPIO_242_PIN_CONTROL;              /*!< GPIO 242 Pin Control                                                  */
  __IO uint32_t  GPIO_243_PIN_CONTROL;              /*!< GPIO 243 Pin Control                                                  */
  __IO uint32_t  GPIO_244_PIN_CONTROL;              /*!< GPIO 244 Pin Control                                                  */
  __IO uint32_t  GPIO_245_PIN_CONTROL;              /*!< GPIO 245 Pin Control                                                  */
  __IO uint32_t  GPIO_246_PIN_CONTROL;              /*!< GPIO 246 Pin Control                                                  */
  __IO uint32_t  GPIO_247_PIN_CONTROL;              /*!< GPIO 247 Pin Control                                                  */
  __IO uint32_t  GPIO_250_PIN_CONTROL;              /*!< GPIO 250 Pin Control                                                  */
  __IO uint32_t  GPIO_251_PIN_CONTROL;              /*!< GPIO 251 Pin Control                                                  */
  __IO uint32_t  GPIO_252_PIN_CONTROL;              /*!< GPIO 252 Pin Control                                                  */
  __IO uint32_t  GPIO_253_PIN_CONTROL;              /*!< GPIO 253 Pin Control                                                  */
  __IO uint32_t  GPIO_254_PIN_CONTROL;              /*!< GPIO 254 Pin Control                                                  */
  __IO uint32_t  GPIO_255_PIN_CONTROL;              /*!< GPIO 255 Pin Control                                                  */
  __IO uint32_t  GPIO_256_PIN_CONTROL;              /*!< GPIO 256 Pin Control                                                  */
  __IO uint32_t  GPIO_257_PIN_CONTROL;              /*!< GPIO 257 Pin Control                                                  */
} GPIO_240_257_Type;


/* ================================================================================ */
/* ================                INPUT_OUTPUT_GPIO               ================ */
/* ================================================================================ */


/**
  * @brief GPIO Input/Output Registers (INPUT_OUTPUT_GPIO)
  */

typedef struct {                                    /*!< INPUT_OUTPUT_GPIO Structure                                           */
  __IO uint32_t  INPUT_GPIO_000_036;                /*!< The GPIO Input Registers can always be used to read the state
                                                         of a pin, even when the pin is in an output mode and/or when
                                                          a
                                                          signal function other than the GPIO signal function is selected.
                                                                                                                               */
  __IO uint32_t  INPUT_GPIO_040_076;                /*!< Input GPIO[040:076]                                                   */
  __IO uint32_t  INPUT_GPIO_100_136;                /*!< Input GPIO[100:136]                                                   */
  __IO uint32_t  INPUT_GPIO_140_176;                /*!< Input GPIO[140:176]                                                   */
  __IO uint32_t  INPUT_GPIO_200_236;                /*!< Input GPIO[200:236]                                                   */
  __IO uint32_t  INPUT_GPIO_240_276;                /*!< Input GPIO[240:276]                                                   */
  __I  uint32_t  RESERVED[26];
  __IO uint32_t  OUTPUT_GPIO_000_036;               /*!< If enabled by the Output GPIO Write Enable bit, the GPIO Output
                                                         bits determine the level on the GPIO pin when the pin is
                                                          configured for the GPIO output function. On writes: If enabled
                                                          via the Output GPIO Write Enable 0: GPIO[x] out = '0', 1: GPIO[x]
                                                          out = '1'.                                                           */
  __IO uint32_t  OUPUT_GPIO_040_076;                /*!< Output GPIO[040:076]                                                  */
  __IO uint32_t  OUTPUT_GPIO_100_136;               /*!< Output GPIO[100:136]                                                  */
  __IO uint32_t  OUTPUT_GPIO_140_176;               /*!< Output GPIO[140:176]                                                  */
  __IO uint32_t  OUTPUT_GPIO_200_236;               /*!< Output GPIO[200:236]                                                  */
  __IO uint32_t  OUTPUT_GPIO_240_276;               /*!< Output GPIO[240:276]                                                  */
} INPUT_OUTPUT_GPIO_Type;


/* ================================================================================ */
/* ================               GPIO_PIN_CONTROL_2               ================ */
/* ================================================================================ */


/**
  * @brief GPIO Pin Control 2 Registers (GPIO_PIN_CONTROL_2)
  */

typedef struct {                                    /*!< GPIO_PIN_CONTROL_2 Structure                                          */
  __IO uint32_t  GPIO_000_PIN_CONTROL_2;            /*!< GPIO 000 PIN CONTROL REGISTER 2                                       */
  __IO uint32_t  GPIO_001_PIN_CONTROL_2;            /*!< GPIO 001 Pin Control 2                                                */
  __IO uint32_t  GPIO_002_PIN_CONTROL_2;            /*!< GPIO 002 Pin Control 2                                                */
  __IO uint32_t  GPIO_003_PIN_CONTROL_2;            /*!< GPIO 003 Pin Control 2                                                */
  __IO uint32_t  GPIO_004_PIN_CONTROL_2;            /*!< GPIO 004 Pin Control 2                                                */
  __IO uint32_t  GPIO_005_PIN_CONTROL_2;            /*!< GPIO 005 Pin Control 2                                                */
  __IO uint32_t  GPIO_006_PIN_CONTROL_2;            /*!< GPIO 006 Pin Control 2                                                */
  __IO uint32_t  GPIO_007_PIN_CONTROL_2;            /*!< GPIO 007 Pin Control 2                                                */
  __IO uint32_t  GPIO_010_PIN_CONTROL_2;            /*!< GPIO 010 Pin Control 2                                                */
  __IO uint32_t  GPIO_011_PIN_CONTROL_2;            /*!< GPIO 011 Pin Control 2                                                */
  __IO uint32_t  GPIO_012_PIN_CONTROL_2;            /*!< GPIO 012 Pin Control 2                                                */
  __IO uint32_t  GPIO_013_PIN_CONTROL_2;            /*!< GPIO 013 Pin Control 2                                                */
  __IO uint32_t  GPIO_014_PIN_CONTROL_2;            /*!< GPIO 014 Pin Control 2                                                */
  __IO uint32_t  GPIO_015_PIN_CONTROL_2;            /*!< GPIO 015 Pin Control 2                                                */
  __IO uint32_t  GPIO_016_PIN_CONTROL_2;            /*!< GPIO 016 Pin Control 2                                                */
  __IO uint32_t  GPIO_017_PIN_CONTROL_2;            /*!< GPIO 017 Pin Control 2                                                */
  __IO uint32_t  GPIO_020_PIN_CONTROL_2;            /*!< GPIO 020 Pin Control 2                                                */
  __IO uint32_t  GPIO_021_PIN_CONTROL_2;            /*!< GPIO 021 Pin Control 2                                                */
  __IO uint32_t  GPIO_022_PIN_CONTROL_2;            /*!< GPIO 022 Pin Control 2                                                */
  __IO uint32_t  GPIO_023_PIN_CONTROL_2;            /*!< GPIO 023 Pin Control 2                                                */
  __IO uint32_t  GPIO_024_PIN_CONTROL_2;            /*!< GPIO 024 Pin Control 2                                                */
  __IO uint32_t  GPIO_025_PIN_CONTROL_2;            /*!< GPIO 025 Pin Control 2                                                */
  __IO uint32_t  GPIO_026_PIN_CONTROL_2;            /*!< GPIO 026 Pin Control 2                                                */
  __IO uint32_t  GPIO_027_PIN_CONTROL_2;            /*!< GPIO 027 Pin Control 2                                                */
  __IO uint32_t  GPIO_030_PIN_CONTROL_2;            /*!< GPIO 030 Pin Control 2                                                */
  __IO uint32_t  GPIO_031_PIN_CONTROL_2;            /*!< GPIO 031 Pin Control 2                                                */
  __IO uint32_t  GPIO_032_PIN_CONTROL_2;            /*!< GPIO 032 Pin Control 2                                                */
  __IO uint32_t  GPIO_033_PIN_CONTROL_2;            /*!< GPIO 033 Pin Control 2                                                */
  __IO uint32_t  GPIO_034_PIN_CONTROL_2;            /*!< GPIO 034 Pin Control 2                                                */
  __IO uint32_t  GPIO_035_PIN_CONTROL_2;            /*!< GPIO 035 Pin Control 2                                                */
  __IO uint32_t  GPIO_036_PIN_CONTROL_2;            /*!< GPIO 036 Pin Control 2                                                */
  __I  uint32_t  RESERVED;
  __IO uint32_t  GPIO_040_PIN_CONTROL_2;            /*!< GPIO 040 Pin Control 2                                                */
  __IO uint32_t  GPIO_041_PIN_CONTROL_2;            /*!< GPIO 041 Pin Control 2                                                */
  __IO uint32_t  GPIO_042_PIN_CONTROL_2;            /*!< GPIO 042 Pin Control 2                                                */
  __IO uint32_t  GPIO_043_PIN_CONTROL_2;            /*!< GPIO 043 Pin Control 2                                                */
  __IO uint32_t  GPIO_044_PIN_CONTROL_2;            /*!< GPIO 044 Pin Control 2                                                */
  __IO uint32_t  GPIO_045_PIN_CONTROL_2;            /*!< GPIO 045 Pin Control 2                                                */
  __IO uint32_t  GPIO_046_PIN_CONTROL_2;            /*!< GPIO 046 Pin Control 2                                                */
  __IO uint32_t  GPIO_047_PIN_CONTROL_2;            /*!< GPIO 047 Pin Control 2                                                */
  __IO uint32_t  GPIO_050_PIN_CONTROL_2;            /*!< GPIO 050 Pin Control 2                                                */
  __IO uint32_t  GPIO_051_PIN_CONTROL_2;            /*!< GPIO 051 Pin Control 2                                                */
  __IO uint32_t  GPIO_052_PIN_CONTROL_2;            /*!< GPIO 052 Pin Control 2                                                */
  __IO uint32_t  GPIO_053_PIN_CONTROL_2;            /*!< GPIO 053 Pin Control 2                                                */
  __IO uint32_t  GPIO_054_PIN_CONTROL_2;            /*!< GPIO 054 Pin Control 2                                                */
  __IO uint32_t  GPIO_055_PIN_CONTROL_2;            /*!< GPIO 055 Pin Control 2                                                */
  __IO uint32_t  GPIO_056_PIN_CONTROL_2;            /*!< GPIO 056 Pin Control 2                                                */
  __IO uint32_t  GPIO_057_PIN_CONTROL_2;            /*!< GPIO 057 Pin Control 2                                                */
  __IO uint32_t  GPIO_060_PIN_CONTROL_2;            /*!< GPIO 060 Pin Control 2                                                */
  __IO uint32_t  GPIO_061_PIN_CONTROL_2;            /*!< GPIO 061 Pin Control 2                                                */
  __IO uint32_t  GPIO_062_PIN_CONTROL_2;            /*!< GPIO 062 Pin Control 2                                                */
  __IO uint32_t  GPIO_063_PIN_CONTROL_2;            /*!< GPIO 063 Pin Control 2                                                */
  __IO uint32_t  GPIO_064_PIN_CONTROL_2;            /*!< GPIO 064 Pin Control 2                                                */
  __IO uint32_t  GPIO_065_PIN_CONTROL_2;            /*!< GPIO 065 Pin Control 2                                                */
  __IO uint32_t  GPIO_066_PIN_CONTROL_2;            /*!< GPIO 066 Pin Control 2                                                */
  __IO uint32_t  GPIO_067_PIN_CONTROL_2;            /*!< GPIO 067 Pin Control 2                                                */
  __IO uint32_t  GPIO_070_PIN_CONTROL_2;            /*!< GPIO 070 Pin Control 2                                                */
  __IO uint32_t  GPIO_071_PIN_CONTROL_2;            /*!< GPIO 071 Pin Control 2                                                */
  __IO uint32_t  GPIO_072_PIN_CONTROL_2;            /*!< GPIO 072 Pin Control 2                                                */
  __IO uint32_t  GPIO_073_PIN_CONTROL_2;            /*!< GPIO 073 Pin Control 2                                                */
  __IO uint32_t  GPIO_074_PIN_CONTROL_2;            /*!< GPIO 074 Pin Control 2                                                */
  __IO uint32_t  GPIO_075_PIN_CONTROL_2;            /*!< GPIO 075 Pin Control 2                                                */
  __IO uint32_t  GPIO_076_PIN_CONTROL_2;            /*!< GPIO 076 Pin Control 2                                                */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  GPIO_100_PIN_CONTROL_2;            /*!< GPIO 100 Pin Control 2                                                */
  __IO uint32_t  GPIO_101_PIN_CONTROL_2;            /*!< GPIO 101 Pin Control 2                                                */
  __IO uint32_t  GPIO_102_PIN_CONTROL_2;            /*!< GPIO 102 Pin Control 2                                                */
  __IO uint32_t  GPIO_103_PIN_CONTROL_2;            /*!< GPIO 103 Pin Control 2                                                */
  __IO uint32_t  GPIO_104_PIN_CONTROL_2;            /*!< GPIO 104 Pin Control 2                                                */
  __IO uint32_t  GPIO_105_PIN_CONTROL_2;            /*!< GPIO 105 Pin Control 2                                                */
  __IO uint32_t  GPIO_106_PIN_CONTROL_2;            /*!< GPIO 106 Pin Control 2                                                */
  __IO uint32_t  GPIO_107_PIN_CONTROL_2;            /*!< GPIO 107 Pin Control 2                                                */
  __IO uint32_t  GPIO_110_PIN_CONTROL_2;            /*!< GPIO 110 Pin Control 2                                                */
  __IO uint32_t  GPIO_111_PIN_CONTROL_2;            /*!< GPIO 111 Pin Control 2                                                */
  __IO uint32_t  GPIO_112_PIN_CONTROL_2;            /*!< GPIO 112 Pin Control 2                                                */
  __IO uint32_t  GPIO_113_PIN_CONTROL_2;            /*!< GPIO 113 Pin Control 2                                                */
  __IO uint32_t  GPIO_114_PIN_CONTROL_2;            /*!< GPIO 114 Pin Control 2                                                */
  __IO uint32_t  GPIO_115_PIN_CONTROL_2;            /*!< GPIO 115 Pin Control 2                                                */
  __IO uint32_t  GPIO_116_PIN_CONTROL_2;            /*!< GPIO 116 Pin Control 2                                                */
  __IO uint32_t  GPIO_117_PIN_CONTROL_2;            /*!< GPIO 117 Pin Control 2                                                */
  __IO uint32_t  GPIO_120_PIN_CONTROL_2;            /*!< GPIO 120 Pin Control 2                                                */
  __IO uint32_t  GPIO_121_PIN_CONTROL_2;            /*!< GPIO 121 Pin Control 2                                                */
  __IO uint32_t  GPIO_122_PIN_CONTROL_2;            /*!< GPIO 122 Pin Control 2                                                */
  __IO uint32_t  GPIO_123_PIN_CONTROL_2;            /*!< GPIO 123 Pin Control 2                                                */
  __IO uint32_t  GPIO_124_PIN_CONTROL_2;            /*!< GPIO 124 Pin Control 2                                                */
  __IO uint32_t  GPIO_125_PIN_CONTROL_2;            /*!< GPIO 125 Pin Control 2                                                */
  __IO uint32_t  GPIO_126_PIN_CONTROL_2;            /*!< GPIO 126 Pin Control 2                                                */
  __IO uint32_t  GPIO_127_PIN_CONTROL_2;            /*!< GPIO 127 Pin Control 2                                                */
  __IO uint32_t  GPIO_130_PIN_CONTROL_2;            /*!< GPIO 130 Pin Control 2                                                */
  __IO uint32_t  GPIO_131_PIN_CONTROL_2;            /*!< GPIO 131 Pin Control 2                                                */
  __IO uint32_t  GPIO_132_PIN_CONTROL_2;            /*!< GPIO 132 Pin Control 2                                                */
  __IO uint32_t  GPIO_133_PIN_CONTROL_2;            /*!< GPIO 133 Pin Control 2                                                */
  __IO uint32_t  GPIO_134_PIN_CONTROL_2;            /*!< GPIO 134 Pin Control 2                                                */
  __IO uint32_t  GPIO_135_PIN_CONTROL_2;            /*!< GPIO 135 Pin Control 2                                                */
  __IO uint32_t  GPIO_136_PIN_CONTROL_2;            /*!< GPIO 136 Pin Control 2                                                */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  GPIO_140_PIN_CONTROL_2;            /*!< GPIO 140 Pin Control 2                                                */
  __IO uint32_t  GPIO_141_PIN_CONTROL_2;            /*!< GPIO 141 Pin Control 2                                                */
  __IO uint32_t  GPIO_142_PIN_CONTROL_2;            /*!< GPIO 142 Pin Control 2                                                */
  __IO uint32_t  GPIO_143_PIN_CONTROL_2;            /*!< GPIO 143 Pin Control 2                                                */
  __IO uint32_t  GPIO_144_PIN_CONTROL_2;            /*!< GPIO 144 Pin Control 2                                                */
  __IO uint32_t  GPIO_145_PIN_CONTROL_2;            /*!< GPIO 145 Pin Control 2                                                */
  __IO uint32_t  GPIO_146_PIN_CONTROL_2;            /*!< GPIO 146 Pin Control 2                                                */
  __IO uint32_t  GPIO_147_PIN_CONTROL_2;            /*!< GPIO 147 Pin Control 2                                                */
  __IO uint32_t  GPIO_150_PIN_CONTROL_2;            /*!< GPIO 150 Pin Control 2                                                */
  __IO uint32_t  GPIO_151_PIN_CONTROL_2;            /*!< GPIO 151 Pin Control 2                                                */
  __IO uint32_t  GPIO_152_PIN_CONTROL_2;            /*!< GPIO 152 Pin Control 2                                                */
  __IO uint32_t  GPIO_153_PIN_CONTROL_2;            /*!< GPIO 153 Pin Control 2                                                */
  __IO uint32_t  GPIO_154_PIN_CONTROL_2;            /*!< GPIO 154 Pin Control 2                                                */
  __IO uint32_t  GPIO_155_PIN_CONTROL_2;            /*!< GPIO 155 Pin Control 2                                                */
  __IO uint32_t  GPIO_156_PIN_CONTROL_2;            /*!< GPIO 156 Pin Control 2                                                */
  __IO uint32_t  GPIO_157_PIN_CONTROL_2;            /*!< GPIO 157 Pin Control 2                                                */
  __IO uint32_t  GPIO_160_PIN_CONTROL_2;            /*!< GPIO 160 Pin Control 2                                                */
  __IO uint32_t  GPIO_161_PIN_CONTROL_2;            /*!< GPIO 161 Pin Control 2                                                */
  __IO uint32_t  GPIO_162_PIN_CONTROL_2;            /*!< GPIO 162 Pin Control 2                                                */
  __IO uint32_t  GPIO_163_PIN_CONTROL_2;            /*!< GPIO 163 Pin Control 2                                                */
  __IO uint32_t  GPIO_164_PIN_CONTROL_2;            /*!< GPIO 164 Pin Control 2                                                */
  __IO uint32_t  GPIO_165_PIN_CONTROL_2;            /*!< GPIO 165 Pin Control 2                                                */
  __IO uint32_t  GPIO_166_PIN_CONTROL_2;            /*!< GPIO 166 Pin Control 2                                                */
  __IO uint32_t  GPIO_167_PIN_CONTROL_2;            /*!< GPIO 167 Pin Control 2                                                */
  __IO uint32_t  GPIO_170_PIN_CONTROL_2;            /*!< GPIO 170 Pin Control 2                                                */
  __IO uint32_t  GPIO_171_PIN_CONTROL_2;            /*!< GPIO 171 Pin Control 2                                                */
  __IO uint32_t  GPIO_172_PIN_CONTROL_2;            /*!< GPIO 172 Pin Control 2                                                */
  __IO uint32_t  GPIO_173_PIN_CONTROL_2;            /*!< GPIO 173 Pin Control 2                                                */
  __IO uint32_t  GPIO_174_PIN_CONTROL_2;            /*!< GPIO 174 Pin Control 2                                                */
  __IO uint32_t  GPIO_175_PIN_CONTROL_2;            /*!< GPIO 175 Pin Control 2                                                */
  __IO uint32_t  GPIO_176_PIN_CONTROL_2;            /*!< GPIO 176 Pin Control 2                                                */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  GPIO_200_PIN_CONTROL_2;            /*!< GPIO 200 Pin Control 2                                                */
  __IO uint32_t  GPIO_201_PIN_CONTROL_2;            /*!< GPIO 201 Pin Control 2                                                */
  __IO uint32_t  GPIO_202_PIN_CONTROL_2;            /*!< GPIO 202 Pin Control 2                                                */
  __IO uint32_t  GPIO_203_PIN_CONTROL_2;            /*!< GPIO 203 Pin Control 2                                                */
  __IO uint32_t  GPIO_204_PIN_CONTROL_2;            /*!< GPIO 204 Pin Control 2                                                */
  __IO uint32_t  GPIO_205_PIN_CONTROL_2;            /*!< GPIO 205 Pin Control 2                                                */
  __IO uint32_t  GPIO_206_PIN_CONTROL_2;            /*!< GPIO 206 Pin Control 2                                                */
  __IO uint32_t  GPIO_207_PIN_CONTROL_2;            /*!< GPIO 207 Pin Control 2                                                */
  __IO uint32_t  GPIO_210_PIN_CONTROL_2;            /*!< GPIO 210 Pin Control 2                                                */
  __IO uint32_t  GPIO_211_PIN_CONTROL_2;            /*!< GPIO 211 Pin Control 2                                                */
  __IO uint32_t  GPIO_212_PIN_CONTROL_2;            /*!< GPIO 212 Pin Control 2                                                */
  __IO uint32_t  GPIO_213_PIN_CONTROL_2;            /*!< GPIO 213 Pin Control 2                                                */
  __IO uint32_t  GPIO_214_PIN_CONTROL_2;            /*!< GPIO 214 Pin Control 2                                                */
  __IO uint32_t  GPIO_215_PIN_CONTROL_2;            /*!< GPIO 215 Pin Control 2                                                */
  __IO uint32_t  GPIO_216_PIN_CONTROL_2;            /*!< GPIO 216 Pin Control 2                                                */
  __IO uint32_t  GPIO_217_PIN_CONTROL_2;            /*!< GPIO 217 Pin Control 2                                                */
  __IO uint32_t  GPIO_220_PIN_CONTROL_2;            /*!< GPIO 220 Pin Control 2                                                */
  __IO uint32_t  GPIO_221_PIN_CONTROL_2;            /*!< GPIO 221 Pin Control 2                                                */
  __IO uint32_t  GPIO_222_PIN_CONTROL_2;            /*!< GPIO 222 Pin Control 2                                                */
  __IO uint32_t  GPIO_223_PIN_CONTROL_2;            /*!< GPIO 223 Pin Control 2                                                */
  __IO uint32_t  GPIO_224_PIN_CONTROL_2;            /*!< GPIO 224 Pin Control 2                                                */
  __IO uint32_t  GPIO_225_PIN_CONTROL_2;            /*!< GPIO 225 Pin Control 2                                                */
  __IO uint32_t  GPIO_226_PIN_CONTROL_2;            /*!< GPIO 226 Pin Control 2                                                */
  __IO uint32_t  GPIO_227_PIN_CONTROL_2;            /*!< GPIO 227 Pin Control 2                                                */
  __IO uint32_t  GPIO_230_PIN_CONTROL_2;            /*!< GPIO 230 Pin Control 2                                                */
  __IO uint32_t  GPIO_231_PIN_CONTROL_2;            /*!< GPIO 231 Pin Control 2                                                */
  __IO uint32_t  GPIO_232_PIN_CONTROL_2;            /*!< GPIO 232 Pin Control 2                                                */
  __IO uint32_t  GPIO_233_PIN_CONTROL_2;            /*!< GPIO 233 Pin Control 2                                                */
  __IO uint32_t  GPIO_234_PIN_CONTROL_2;            /*!< GPIO 234 Pin Control 2                                                */
  __IO uint32_t  GPIO_235_PIN_CONTROL_2;            /*!< GPIO 235 Pin Control 2                                                */
  __IO uint32_t  GPIO_236_PIN_CONTROL_2;            /*!< GPIO 236 Pin Control 2                                                */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  GPIO_240_PIN_CONTROL_2;            /*!< GPIO 240 Pin Control 2                                                */
  __IO uint32_t  GPIO_241_PIN_CONTROL_2;            /*!< GPIO 241 Pin Control 2                                                */
  __IO uint32_t  GPIO_242_PIN_CONTROL_2;            /*!< GPIO 242 Pin Control 2                                                */
  __IO uint32_t  GPIO_243_PIN_CONTROL_2;            /*!< GPIO 243 Pin Control 2                                                */
  __IO uint32_t  GPIO_244_PIN_CONTROL_2;            /*!< GPIO 244 Pin Control 2                                                */
  __IO uint32_t  GPIO_245_PIN_CONTROL_2;            /*!< GPIO 245 Pin Control 2                                                */
  __IO uint32_t  GPIO_246_PIN_CONTROL_2;            /*!< GPIO 246 Pin Control 2                                                */
  __IO uint32_t  GPIO_247_PIN_CONTROL_2;            /*!< GPIO 247 Pin Control 2                                                */
  __IO uint32_t  GPIO_250_PIN_CONTROL_2;            /*!< GPIO 250 Pin Control 2                                                */
  __IO uint32_t  GPIO_251_PIN_CONTROL_2;            /*!< GPIO 251 Pin Control 2                                                */
  __IO uint32_t  GPIO_252_PIN_CONTROL_2;            /*!< GPIO 252 Pin Control 2                                                */
  __IO uint32_t  GPIO_253_PIN_CONTROL_2;            /*!< GPIO 253 Pin Control 2                                                */
  __IO uint32_t  GPIO_254_PIN_CONTROL_2;            /*!< GPIO 254 Pin Control 2                                                */
  __IO uint32_t  GPIO_255_PIN_CONTROL_2;            /*!< GPIO 255 Pin Control 2                                                */
  __IO uint32_t  GPIO_256_PIN_CONTROL_2;            /*!< GPIO 256 Pin Control 2                                                */
  __IO uint32_t  GPIO_257_PIN_CONTROL_2;            /*!< GPIO 257 Pin Control 2                                                */
} GPIO_PIN_CONTROL_2_Type;


/* ================================================================================ */
/* ================                       WDT                      ================ */
/* ================================================================================ */


/**
  * @brief The function of the Watchdog Timer is to provide a mechanism to detect
 if the internal embedded controller has failed. When enabled, the Watchdog Timer (WDT) circuit
 will generate a WDT Event if the user program fails to reload the WDT within a specified length
 of time known as the WDT Interval.  (WDT)
  */

typedef struct {                                    /*!< WDT Structure                                                         */
  __IO uint16_t  WDT_LOAD;                          /*!< Writing this field reloads the Watch Dog Timer counter.               */
  __I  uint16_t  RESERVED;
  __IO uint16_t  WDT_CONTROL;                       /*!< WDT Control Register                                                  */
  __I  uint16_t  RESERVED1;
  __O  uint8_t   KICK;                              /*!< The WDT Kick Register is a strobe. Reads of this register return
                                                         0. Writes to this register cause the WDT to reload
                                                          the WDT Load Register value and start decrementing when the
                                                          WDT_ENABLE bit in the WDT Control Register is set to '1'. When
                                                          the WDT_ENABLE
                                                          bit in the WDT Control Register is cleared to '0', writes to
                                                          the WDT Kick Register have no effect.                                */
  __I  uint8_t   RESERVED2[3];
  __I  uint16_t  WDT_COUNT;                         /*!< This read-only register provides the current WDT count.               */
} WDT_Type;


/* ================================================================================ */
/* ================                     TIMER0                     ================ */
/* ================================================================================ */


/**
  * @brief This timer block offers a simple mechanism for firmware to maintain a time base. This timer may be instantiated as 16 bits or
 32 bits. The name of the timer instance indicates the size of the timer.  (TIMER0)
  */

typedef struct {                                    /*!< TIMER0 Structure                                                      */
  __IO uint32_t  COUNT;                             /*!< This is the value of the Timer counter. This is updated by Hardware
                                                         but may be set by Firmware.                                           */
  __IO uint32_t  PRE_LOAD;                          /*!< This is the value of the Timer pre-load for the counter. This
                                                         is used by H/W when the counter is to be restarted
                                                         automatically; this will become the new value of the counter
                                                          upon restart.                                                        */
  __IO uint32_t  STATUS;                            /*!< This is the interrupt status that fires when the timer reaches
                                                         its limit                                                             */
  __IO uint32_t  INT_EN;                            /*!< This is the interrupt enable for the status EVENT_INTERRUPT
                                                         bit in the Timer Status Register                                      */
  __IO uint32_t  CONTROL;                           /*!< Timer Control Register                                                */
} TIMER0_Type;


/* ================================================================================ */
/* ================                 COUNTER_TIMER0                 ================ */
/* ================================================================================ */


/**
  * @brief This interface is a 16-bit auto-reloading timer/counter.  (COUNTER_TIMER0)
  */

typedef struct {                                    /*!< COUNTER_TIMER0 Structure                                              */
  __IO uint32_t  TIMERX_CONTROL;                    /*!< This bit reflects the current state of the timer's Clock_Required
                                                         output signal.                                                        */
  __IO uint32_t  PRELOAD;                           /*!< This is the value of the Timer pre-load for the counter. This
                                                         is used by H/W when the counter is to be restarted automatically;
                                                          this will become the new value of the counter upon restart.
                                                                                                                               */
  __IO uint32_t  TIMERX_RELOAD;                     /*!< This register is used in Timer and One-Shot modes to set the
                                                         lower limit of the timer.                                             */
  __IO uint32_t  TIMERX_COUNT;                      /*!< This register returns the current value of the timer in all
                                                         modes.                                                                */
} COUNTER_TIMER0_Type;


/* ================================================================================ */
/* ================              CAPTURE_COMPARE_TIMER             ================ */
/* ================================================================================ */


/**
  * @brief This is a 16-bit auto-reloading timer/counter.  (CAPTURE_COMPARE_TIMER)
  */

typedef struct {                                    /*!< CAPTURE_COMPARE_TIMER Structure                                       */
  __IO uint32_t  CAPTURE_COMPARE_TIMER_CONTROL;     /*!< This register controls the capture and compare timer.                 */
  __IO uint32_t  CAPTURE_CONTROL_0;                 /*!< This register is used to configure capture and compare timers
                                                         0-3.                                                                  */
  __IO uint32_t  CAPTURE_CONTROL_1;                 /*!< This register is used to configure capture and compare timers
                                                         4-5.                                                                  */
  __IO uint32_t  FREE_RUNNING_TIMER;                /*!< This register contains the current value of the Free Running
                                                         Timer.                                                                */
  __IO uint32_t  CAPTURE_0;                         /*!< This register saves the value copied from the Free Running timer
                                                         on a programmed edge of ICT0.                                         */
  __IO uint32_t  CAPTURE_1;                         /*!< This register saves the value copied from the Free Running timer
                                                         on a programmed edge of ICT1.                                         */
  __IO uint32_t  CAPTURE_2;                         /*!< This register saves the value copied from the Free Running timer
                                                         on a programmed edge of ICT0.                                         */
  __IO uint32_t  CAPTURE_3;                         /*!< This register saves the value copied from the Free Running timer
                                                         on a programmed edge of ICT0.                                         */
  __IO uint32_t  CAPTURE_4;                         /*!< This register saves the value copied from the Free Running timer
                                                         on a programmed edge of ICT4.                                         */
  __IO uint32_t  CAPTURE_5;                         /*!< This register saves the value copied from the Free Running timer
                                                         on a programmed edge of ICT5.                                         */
  __IO uint32_t  COMPARE_0;                         /*!< A COMPARE 0 interrupt is generated when this register matches
                                                         the value in the Free Running Timer.                                  */
  __IO uint32_t  COMPARE_1;                         /*!< A COMPARE 1 interrupt is generated when this register matches
                                                         the value in the Free Running Timer.                                  */
} CAPTURE_COMPARE_TIMER_Type;


/* ================================================================================ */
/* ================                      HTM0                      ================ */
/* ================================================================================ */


/**
  * @brief The Hibernation Timer can generate a wake event to the Embedded Controller (EC) when it is in a hibernation mode.
 This block supports wake events up to 2 hours in duration. The timer is a 16-bit binary count-down timer that can be programmed
 in 30.5us and 0.125 second increments for period ranges of 30.5us to 2s or 0.125s to 136.5 minutes, respectively.  (HTM0)
  */

typedef struct {                                    /*!< HTM0 Structure                                                        */
  __IO uint16_t  HT_PRELOAD;                        /*!< [15:0] This register is used to set the Hibernation Timer Preload
                                                         value.                                                                */
  __I  uint16_t  RESERVED;
  __IO uint16_t  HT_CONTROL;                        /*!< HTimer Control Register                                               */
  __I  uint16_t  RESERVED1;
  __I  uint16_t  COUNT;                             /*!< The current state of the Hibernation Timer.                           */
} HTM0_Type;


/* ================================================================================ */
/* ================                      RTOS                      ================ */
/* ================================================================================ */


/**
  * @brief The RTOS Timer is a low-power, 32-bit timer designed to operate on the 32kHz oscillator which is available during all
 chip sleep states. This allows firmware the option to sleep the processor, enter heavy or deep chip sleep states, and
 wake after a programmed amount of time. The timer may be used as a one-shot timer or a continuous timer. When the
 timer transitions to 0 it is capable of generating a wake-capable interrupt to the embedded controller. This timer may be
 halted during debug by hardware or via a software control bit.  (RTOS)
  */

typedef struct {                                    /*!< RTOS Structure                                                        */
  __IO uint32_t  RTOS_TIMER_COUNT;                  /*!< RTOS Timer Count Register.                                            */
  __IO uint32_t  RTOS_TIMER_PRELOAD;                /*!< RTOS Timer Preload Register                                           */
  __IO uint32_t  RTOS_TIMER_CONTROL;                /*!< RTOS Timer Control Register                                           */
  __IO uint32_t  SOFT_INTERRUPT;                    /*!< Soft Interrupt Register                                               */
} RTOS_Type;


/* ================================================================================ */
/* ================                       RTC                      ================ */
/* ================================================================================ */


/**
  * @brief This is the set of registers that are automatically counted by hardware every 1 second while the block is enabled
 to run and to update. These registers are: Seconds, Minutes, Hours, Day of Week, Day of Month, Month, and Year.  (RTC)
  */

typedef struct {                                    /*!< RTC Structure                                                         */
  __IO uint8_t   SEC;                               /*!< Seconds Register                                                      */
  __IO uint8_t   SEC_ALARM;                         /*!< Seconds Alarm Register                                                */
  __IO uint8_t   MIN;                               /*!< Minutes Register                                                      */
  __IO uint8_t   MIN_ALARM;                         /*!< Minutes Alarm Register                                                */
  __IO uint8_t   HR;                                /*!< Hours Register                                                        */
  __IO uint8_t   HR_ALARM;                          /*!< Hours Alarm Register                                                  */
  __IO uint8_t   DAY_WEEK;                          /*!< Day of Week Register                                                  */
  __IO uint8_t   DAY_MONTH;                         /*!< Day of Month Register                                                 */
  __IO uint8_t   MONTH;                             /*!< Month Register                                                        */
  __IO uint8_t   YEAR;                              /*!< Year Register                                                         */
  __IO uint8_t   REG_A;                             /*!< Register A                                                            */
  __IO uint8_t   REG_B;                             /*!< Register B                                                            */
  __IO uint8_t   REG_C;                             /*!< Register C                                                            */
  __IO uint8_t   REG_D;                             /*!< Register D                                                            */
  __I  uint16_t  RESERVED;
  __IO uint32_t  CONTROL;                           /*!< RTC Control Register                                                  */
  __IO uint32_t  WEEK_ALARM;                        /*!< Week Alarm Register[7:0] - ALARM_DAY_OF_WEEK This register,
                                                         if written to a value in the range 1- -7, will inhibit the Alarm
                                                          interrupt unless this field matches the contents of the Day
                                                          of Week Register also.                                               */
  __IO uint32_t  DAYLIGHT_SAVINGS_FORWARD;          /*!< Daylight Savings Forward Register                                     */
  __IO uint32_t  DAYLIGHT_SAVINGS_BACKWARD;         /*!< Daylight Savings Backward Register                                    */
} RTC_Type;


/* ================================================================================ */
/* ================                      WEEK                      ================ */
/* ================================================================================ */


/**
  * @brief The Week Alarm Interface provides two timekeeping functions: a Week Timer and a Sub-Week Timer. Both the Week Timer
 and the Sub-Week Timer assert the Power-Up Event Output which automatically powers-up the system from the G3 state.  (WEEK)
  */

typedef struct {                                    /*!< WEEK Structure                                                        */
  __IO uint32_t  CONTROL_REGISTER;                  /*!< Control Register                                                      */
  __IO uint32_t  WEEK_ALARM_COUNTER;                /*!< Week Alarm Counter Register                                           */
  __IO uint32_t  WEEK_TIMER_COMPARE;                /*!< Week Timer Compare Register                                           */
  __IO uint32_t  CLOCK_DIVIDER;                     /*!< Clock Divider Register                                                */
  __IO uint32_t  SUB_SECOND_INT_SELECT;             /*!< Sub-Second Programmable Interrupt Select Register                     */
  __I  uint32_t  SUB_WEEK_CONTROL;                  /*!< Sub-Week Control Register                                             */
  __I  uint32_t  SUB_WEEK_ALARM_COUNTER;            /*!< Sub-Week Alarm Counter Register                                       */
  __IO uint32_t  BGPO_DATA;                         /*!< BGPO Data Register                                                    */
  __IO uint32_t  BGPO_POWER;                        /*!< BGPO Power Register                                                   */
  __IO uint32_t  BGPO_RESET;                        /*!< BGPO Reset Register                                                   */
} WEEK_Type;


/* ================================================================================ */
/* ================                      TACH0                     ================ */
/* ================================================================================ */


/**
  * @brief This block monitors TACH output signals (or locked rotor signals) from
 various types of fans, and determines their speed.  (TACH0)
  */

typedef struct {                                    /*!< TACH0 Structure                                                       */
  __IO uint32_t  TACH_CONTROL;                      /*!< TACHx Control Register                                                */
  __IO uint32_t  TACHX_STATUS;                      /*!< TACHx Status Register                                                 */
  __IO uint32_t  TACHX_HIGH_LIMIT;                  /*!< TACH HIGH LIMIT Register                                              */
  __IO uint32_t  TACHX_LOW_LIMIT;                   /*!< TACHx Low Limit Register                                              */
} TACH0_Type;


/* ================================================================================ */
/* ================                      PWM0                      ================ */
/* ================================================================================ */


/**
  * @brief This block generates a PWM output that can be used to control 4-wire fans, blinking LEDs, and other
 similar devices. Each PWM can generate an arbitrary duty cycle output at frequencies from less than 0.1 Hz to 24 MHz.
 The PWM controller can also used to generate the PROCHOT output and Speaker output.  (PWM0)
  */

typedef struct {                                    /*!< PWM0 Structure                                                        */
  __IO uint32_t  COUNTER_ON_TIME;                   /*!< This field determines both the frequency and duty cycle of the
                                                         PWM signal. Setting this field to a value of n will
                                                         cause the On time of the PWM to be n+1 cycles of the PWM Clock
                                                          Source.
                                                         When this field is set to zero and the PWMX_COUNTER_OFF_TIME
                                                          is not set to zero, the PWM_OUTPUT is held low (Full Off).           */
  __IO uint32_t  COUNTER_OFF_TIME;                  /*!< This field determine both the frequency and duty cycle of the
                                                         PWM signal. Setting this field to a value of n will
                                                         cause the Off time of the PWM to be n+1 cycles of the PWM Clock
                                                          Source.
                                                         When this field is set to zero, the PWM_OUTPUT is held high
                                                          (Full On).                                                           */
  __IO uint32_t  CONFIG;                            /*!< PWMx CONFIGURATION REGISTER                                           */
} PWM0_Type;


/* ================================================================================ */
/* ================                      PECI                      ================ */
/* ================================================================================ */


/**
  * @brief The PECI Interface allows the EC to retrieve temperature readings from PECI-compliant devices.  (PECI)
  */

typedef struct {                                    /*!< PECI Structure                                                        */
  __IO uint8_t   WRITE_DATA;                        /*!< The Write Data Register provides access to a 32-byte Transmit
                                                         FIFO.                                                                 */
  __I  uint8_t   RESERVED[3];
  __IO uint8_t   READ_DATA;                         /*!< The Read Data Register provides access to a 32-byte Receive
                                                         FIFO.                                                                 */
  __I  uint8_t   RESERVED1[3];
  __IO uint8_t   CONTROL;                           /*!< Control Register                                                      */
  __I  uint8_t   RESERVED2[3];
  __IO uint8_t   STATUS1;                           /*!< Status Register 1                                                     */
  __I  uint8_t   RESERVED3[3];
  __I  uint8_t   STATUS2;                           /*!< Status Register 2                                                     */
  __I  uint8_t   RESERVED4[3];
  __IO uint8_t   ERROR;                             /*!< Error Register                                                        */
  __I  uint8_t   RESERVED5[3];
  __IO uint8_t   INT_EN1;                           /*!< Interrupt Enable 1 Register                                           */
  __I  uint8_t   RESERVED6[3];
  __IO uint8_t   INT_EN2;                           /*!< Interrupt Enable 2 Register                                           */
  __I  uint8_t   RESERVED7[3];
  __IO uint8_t   OBT1;                              /*!< Optimal Bit Time Register (Low Byte)                                  */
  __I  uint8_t   RESERVED8[3];
  __IO uint8_t   OBT2;                              /*!< Optimal Bit Time Register (High Byte)                                 */
  __I  uint8_t   RESERVED9[27];
  __IO uint32_t  ID;                                /*!< Block ID Register                                                     */
  __IO uint32_t  REV;                               /*!< Revision Register                                                     */
} PECI_Type;


/* ================================================================================ */
/* ================                       ADC                      ================ */
/* ================================================================================ */


/**
  * @brief This block is designed to convert external analog voltage readings into digital values.  (ADC)
  */

typedef struct {                                    /*!< ADC Structure                                                         */
  __IO uint32_t  CONTROL;                           /*!< The ADC Control Register is used to control the behavior of
                                                         the Analog to Digital Converter.                                      */
  __IO uint32_t  DELAY;                             /*!< The ADC Delay register determines the delay from setting Start_Repeat
                                                         in the ADC Control Register and the start of a conversion cycle.
                                                          This register also controls the interval between conversion
                                                          cycles in repeat mode.                                               */
  __IO uint32_t  STATUS;                            /*!< The ADC Status Register indicates whether the ADC has completed
                                                         a conversion cycle. All bits are cleared by being written with
                                                          a 1.
                                                          0: conversion of the corresponding ADC channel is not complete
                                                          1: conversion of the corresponding ADC channel is complete
                                                                                                                               */
  __IO uint32_t  SINGLE_EN;                         /*!< The ADC Single Register is used to control which ADC channel
                                                         is captured during a Single-Sample conversion cycle initiated
                                                          by the Start_Single bit in the ADC Control Register.
                                                          APPLICATION NOTE: Do not change the bits in this register in
                                                          the middle of a conversion cycle to insure proper operation.
                                                          0: single cycle conversions for this channel are disabled
                                                          1: single cycle conversions for this channel are enabled             */
  __IO uint32_t  REPEAT;                            /*!< The ADC Repeat Register is used to control which ADC channels
                                                         are captured during a repeat conversion cycle initiated by the
                                                          Start_Repeat bit in the ADC Control Register.                        */
  __IO uint32_t  ADC_CHANNEL_READING[16];           /*!< All 16 ADC channels return their results into a 32-bit reading
                                                         register. In each case the low 10 bits of the reading register
                                                          return the result of the Analog to Digital conversion and the
                                                          upper 22 bits return 0.                                              */
} ADC_Type;


/* ================================================================================ */
/* ================                      FAN0                      ================ */
/* ================================================================================ */


/**
  * @brief The RPM-PWM Interface is an RPM based Fan Control Algorithm that monitors the fan's speed and automatically adjusts
 the drive to maintain the desired fan speed. This RPM based Fan Control Algorithm controls a PWM output based on a tachometer input.  (FAN0)
  */

typedef struct {                                    /*!< FAN0 Structure                                                        */
  __IO uint16_t  FAN_SETTING;                       /*!< The Fan Driver Setting used to control the output of the Fan
                                                         Driver.                                                               */
  __IO uint16_t  CONFIGURATION;                     /*!< The Fan Configuration Register controls the general operation
                                                         of the RPM based Fan Control Algorithm used by the fan driver.
                                                                                                                               */
  __IO uint8_t   PWM_DIVIDE;                        /*!< PWM Divide                                                            */
  __IO uint8_t   GAIN;                              /*!< Gain Register stores the gain terms used by the proportional
                                                         and integral portions of the RPM based Fan Control Algorithm.
                                                                                                                               */
  __IO uint8_t   SPIN_UP_CONFIGURATION;             /*!< The Fan Spin Up Configuration Register controls the settings
                                                         of Spin Up Routine.                                                   */
  __IO uint8_t   FAN_STEP;                          /*!< FAN_STEP The Fan Step value represents the maximum step size
                                                         the fan driver will take between update times                         */
  __IO uint8_t   MINIMUM_DRIVE;                     /*!< the minimum drive setting for the RPM based Fan Control Algorithm.
                                                                                                                               */
  __IO uint8_t   VALID_TACH_COUNT;                  /*!< The maximum TACH Reading Register value to indicate that the
                                                         fan is spinning properly.                                             */
  __IO uint16_t  FAN_DRIVE_FAIL_BAND;               /*!< The number of Tach counts used by the Fan Drive Fail detection
                                                         circuitry                                                             */
  __IO uint16_t  TACH_TARGET;                       /*!< The target tachometer value.                                          */
  __IO uint16_t  TACH_READING;                      /*!< [15:3] The current tachometer reading value.                          */
  __IO uint8_t   DRIVER_BASE_FREQUENCY;             /*!< [1:0] Determines the frequency range of the PWM fan driver            */
  __IO uint8_t   STATUS;                            /*!< The bits in this register are routed to interrupts.                   */
} FAN0_Type;


/* ================================================================================ */
/* ================                   PROCHOT_MON                  ================ */
/* ================================================================================ */


/**
  * @brief This block monitors the PROCHOT# signal generated by the host processor. It is designed to detect single assertions and monitor cumulative PROCHOT active time.  (PROCHOT_MON)
  */

typedef struct {                                    /*!< PROCHOT_MON Structure                                                 */
  __IO uint32_t  PHOT_CUMULATVE_COUNT;              /*!< PROCHOT Cumulative Count Register                                     */
  __IO uint32_t  PROCHOT_DC_COUNT;                  /*!< PROCHOT Duty Cycle Count Register                                     */
  __IO uint32_t  PROCHOT_DC_PERIOD;                 /*!< PROCHOT Duty Cycle Period Register                                    */
  __IO uint32_t  PROCHOT_STATUS_CONTROL;            /*!< PROCHOT Status/Control Register                                       */
  __IO uint32_t  PROCHOT_ASSERTION_COUNT;           /*!< PROCHOT Assertion Counter Register                                    */
  __IO uint32_t  PROCHOT_ASSERTION_COUNT_LIMIT;     /*!< PROCHOT Assertion Counter Register                                    */
} PROCHOT_MON_Type;


/* ================================================================================ */
/* ================                   POWERGUARD0                  ================ */
/* ================================================================================ */


/**
  * @brief This block monitors PowerGuard output signals (or locked rotor signals) from
 various types of fans, and determines their speed.  (POWERGUARD0)
  */

typedef struct {                                    /*!< POWERGUARD0 Structure                                                 */
  __IO uint32_t  LPF1_FREQ_CUTOFF_RATE;             /*!< LPF1 Frequency Cut-off Rate Register                                  */
  __IO uint32_t  LPF2_FREQ_CUTOFF_RATE;             /*!< LPF2 Frequency Cut-off Rate Register                                  */
  __IO uint32_t  DATA;                              /*!< Data Register                                                         */
  __IO uint32_t  THRESHOLD_LIMIT;                   /*!< Threshold Limit Register.                                             */
  __IO uint32_t  LOW_TIMER;                         /*!< Low Timer Register                                                    */
  __IO uint32_t  HIGH_TIMER;                        /*!< High Timer Register                                                   */
  __IO uint32_t  CONTROL_STATUS;                    /*!< Control and Status Register.                                          */
  __IO uint32_t  POWERGUARD_INT_STATUS;             /*!< PowerGuard Interrupt Status Register                                  */
  __IO uint32_t  POWERGUARD_INT_ENABLE;             /*!< PowerGuard Interrupt Enable Register                                  */
} POWERGUARD0_Type;


/* ================================================================================ */
/* ================                     EEPROM                     ================ */
/* ================================================================================ */


/**
  * @brief This block is the 2K x 8bit EEPROM.  (EEPROM)
  */

typedef struct {                                    /*!< EEPROM Structure                                                      */
  __IO uint32_t  EEPROM_MODE;                       /*!< EEPROM Mode Register                                                  */
  __IO uint32_t  EEPROM_EXECUTE;                    /*!< EEPROM Execute Register                                               */
  __IO uint32_t  EEPROM_STATUS;                     /*!< EEPROM Status Register                                                */
  __IO uint32_t  EEPROM_INTERRUPT_ENABLE;           /*!< EEPROM Interrupt Enable Register                                      */
  __IO uint32_t  EEPROM_PASSWORD;                   /*!< EEPROM Password Register                                              */
  __IO uint32_t  EEPROM_UNLOCK;                     /*!< EEPROM Unlock Register                                                */
  __I  uint32_t  RESERVED;
  __IO uint32_t  EEPROM_LOCK;                       /*!< EEPROM Lock Register                                                  */
  __IO uint8_t   EEPROM_BUFFER_BYTE_[32];           /*!< One byte of EEPROM Buffer Register. The Data buffer of 32 bytes
                                                         is used to transfer data to and from the EEPROM fabric.
                                                         For WRITES, it must be written before the WRITE command is started.
                                                          For READ STATUS and WRITE STATUS commands, only the
                                                         first byte (offset 0 in this register) is used.                       */
} EEPROM_Type;


/* ================================================================================ */
/* ================                      LED0                      ================ */
/* ================================================================================ */


/**
  * @brief The blinking/breathing hardware is implemented using a PWM. The PWM can be driven either by the 48 MHz
 clock or by a 32.768 KHz clock input. When driven by the 48 MHz clock, the PWM can be used as a standard 8-bit
 PWM in order to control a fan. When used to drive blinking or breathing LEDs, the 32.768 KHz clock source is used.  (LED0)
  */

typedef struct {                                    /*!< LED0 Structure                                                        */
  __IO uint32_t  CONFIG;                            /*!< LED Configuration                                                     */
  __IO uint32_t  LIMITS;                            /*!< LED Limits This register may be written at any time. Values
                                                         written into the register are held in an holding register, which
                                                          is transferred into the actual register at the end of a PWM
                                                          period. The two byte fields may be written independently. Reads
                                                          of this register return the current contents and not the value
                                                          of the holding register.                                             */
  __IO uint32_t  DELAY;                             /*!< LED Delay                                                             */
  __IO uint32_t  UPDATE_STEPSIZE;                   /*!< This register has eight segment fields which provide the amount
                                                         the current duty cycle is adjusted at the end of every PWM period.
                                                          Segment field selection is decoded based on the segment index.
                                                          The segment index equation utilized depends on the SYMMETRY
                                                          bit in the LED Configuration Register Register) . In Symmetric
                                                          Mode the Segment_Index[2:0] = Duty Cycle Bits[7:5]
                                                          . In Asymmetric Mode the Segment_Index[2:0] is the bit concatenation
                                                          of following: Segment_Index[2] = (FALLING RAMP TIME in Figure
                                                          30-3, "Clip                                                          */
  __IO uint32_t  UPDATE_INTERVAL;                   /*!< LED Update Interval                                                   */
  __IO uint32_t  LED_OUTPUT_DELAY;                  /*!< LED Output Delay                                                      */
} LED0_Type;


/* ================================================================================ */
/* ================                     RC_ID0                     ================ */
/* ================================================================================ */


/**
  * @brief This interface provides a single pin interface which can discriminate a number of quantized RC constants.  (RC_ID0)
  */

typedef struct {                                    /*!< RC_ID0 Structure                                                      */
  __IO uint32_t  RC_ID_CONTROL;                     /*!< RC_ID Control Register                                                */
  __IO uint32_t  RC_ID_DATA;                        /*!< Reads of this register provide the result of an RC_ID measurement.
                                                                                                                               */
} RC_ID0_Type;


/* ================================================================================ */
/* ================                       KMS                      ================ */
/* ================================================================================ */


/**
  * @brief The Keyboard Scan Interface block provides a register interface to the EC
 to directly scan an external keyboard matrix of size up to 18x8.  (KMS)
  */

typedef struct {                                    /*!< KMS Structure                                                         */
  __I  uint32_t  RESERVED;
  __IO uint32_t  KSO_CONTROL;                       /*!< KSO Select and control                                                */
  __I  uint32_t  KSI;                               /*!< [7:0] This field returns the current state of the KSI pins.
                                                                                                                               */
  __IO uint32_t  KSI_STATUS;                        /*!< [7:0] Each bit in this field is set on the falling edge of the
                                                         corresponding KSI input pin.
                                                          A KSI interrupt is generated when its corresponding status
                                                          bit and interrupt enable bit are both set. KSI interrupts are
                                                          logically ORed together to produce KSC_INT and KSC_INT_WAKE.
                                                          Writing a '1' to a bit will clear it. Writing a '0' to a bit
                                                          has no effect.                                                       */
  __IO uint32_t  KSI_INT_EN;                        /*!< [7:0] Each bit in KSI_INT_EN enables interrupt generation due
                                                         to highto-low transition on a KSI input. An interrupt is generated
                                                          when the corresponding bits in KSI_STATUS and KSI_INT_EN are
                                                          both set.                                                            */
  __IO uint32_t  EXTENDED_CONTROL;                  /*!< [0:0] PREDRIVE_ENABLE enables the PREDRIVE mode to actively
                                                         drive the KSO pins high for approximately 100ns before switching
                                                          to open-drain operation.
                                                          0=Disable predrive on KSO pins
                                                          1=Enable predrive on KSO pins.                                       */
} KMS_Type;


/* ================================================================================ */
/* ================                      SMB0                      ================ */
/* ================================================================================ */


/**
  * @brief The SMBus interface can handle standard SMBus 2.0 protocols as well as I2C interface.  (SMB0)
  */

typedef struct {                                    /*!< SMB0 Structure                                                        */

  union {
    __I  uint32_t  STATUS;                          /*!< Status Register                                                       */
    __O  uint32_t  CONTROL;                         /*!< Control Register                                                      */
  };
  __IO uint32_t  OWN;                               /*!< Own Address Register
                                                          Note that the Data Register and Own Address fields are offset
                                                         by one bit, so that programming Own Address 1 with a value of
                                                          55h will result in the value AAh being recognized as the SMB
                                                          Controller Core slave address.                                       */
  __IO uint8_t   DATA;                              /*!< This register holds the data that are either shifted out to
                                                         or shifted in from the I2C port.                                      */
  __I  uint8_t   RESERVED[3];
  __IO uint32_t  MASTER_COMMAND;                    /*!< SMBus Master Command Register                                         */
  __IO uint32_t  SLAVE_COMMAND;                     /*!< SMBus Slave Command Register                                          */
  __IO uint32_t  PEC;                               /*!< Packet Error Check (PEC) Register                                     */
  __IO uint32_t  REPEATED_START_HOLD_TIME;          /*!< Repeated Start Hold Time Register                                     */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  COMPLETION;                        /*!< Completion Register                                                   */
  __IO uint32_t  IDLE_SCALING;                      /*!< Idle Scaling Register                                                 */
  __IO uint32_t  CONFIGURATION;                     /*!< Configuration Register                                                */
  __IO uint32_t  BUS_CLOCK;                         /*!< Bus Clock Register                                                    */
  __I  uint8_t   BLOCK_ID;                          /*!< Block ID Register                                                     */
  __I  uint8_t   RESERVED2[3];
  __I  uint8_t   REVISION;                          /*!< Revision Register                                                     */
  __I  uint8_t   RESERVED3[3];
  __IO uint32_t  BIT_BANG_CONTROL;                  /*!< Bit-Bang Control Register                                             */
  __I  uint8_t   TEST;                              /*!< Test                                                                  */
  __I  uint8_t   RESERVED4[3];
  __IO uint32_t  DATA_TIMING;                       /*!< Data Timing Register                                                  */
  __IO uint32_t  TIME_OUT_SCALING;                  /*!< Time-Out Scaling Register                                             */
  __IO uint32_t  SLAVE_TRANSMIT_BUFFER;             /*!< SMBus Slave Transmit Buffer Register                                  */
  __IO uint32_t  SLAVE_RECEIVE_BUFFER;              /*!< SMBus Slave Receive Buffer Register                                   */
  __IO uint32_t  MASTER_TRANSMIT_BUFER;             /*!< SMBus Master Transmit Buffer Register                                 */
  __IO uint32_t  MASTER_RECEIVE_BUFFER;             /*!< SMBus Master Receive Buffer Register                                  */
  __I  uint32_t  RESERVED5[2];
  __IO uint32_t  WAKE_STATUS;                       /*!< WAKE STATUS Register                                                  */
  __IO uint32_t  WAKE_ENABLE;                       /*!< WAKE ENABLE Register                                                  */
} SMB0_Type;


/* ================================================================================ */
/* ================                     GP_SPI0                    ================ */
/* ================================================================================ */


/**
  * @brief The General Purpose Serial Peripheral Interface (GP-SPI) may be used
 to communicate with various peripheral devices, e.g., EEPROMS, DACs, ADCs, that use a
 standard Serial Peripheral Interface.  (GP_SPI0)
  */

typedef struct {                                    /*!< GP_SPI0 Structure                                                     */
  __IO uint32_t  ENABLE;                            /*!< [0:0] 1=Enabled. The device is fully operational
                                                          0=Disabled. Clocks are gated to conserve power and the SPDOUT
                                                         and SPI_CLK signals are set to their inactive state                   */
  __IO uint32_t  CONTROL;                           /*!< SPI Control                                                           */
  __I  uint32_t  STATUS;                            /*!< SPI Status                                                            */
  __IO uint32_t  TX_DATA;                           /*!< [7:0] A write to this register when the Tx_Data buffer is empty
                                                         (TXBE in the SPI Status Register is '1') initiates a SPI transaction.
                                                                                                                               */
  __IO uint32_t  RX_DATA;                           /*!< [7:0] This register is used to read the value returned by the
                                                         external SPI device.                                                  */
  __IO uint32_t  CLOCK_Control;                     /*!< SPI Clock Control. This register should not be changed during
                                                         an active SPI transaction.                                            */
  __IO uint32_t  CLOCK_GENERATOR;                   /*!< [5:0] PRELOAD SPI Clock Generator Preload value.                      */
} GP_SPI0_Type;


/* ================================================================================ */
/* ================                      QMSPI                     ================ */
/* ================================================================================ */


/**
  * @brief The Quad SPI Master Controller may be used to communicate with various
 peripheral devices that use a Serial Peripheral Interface, such as EEPROMS, DACs and ADCs.
 The controller can be configured to support advanced SPI Flash devices with multi-phase access protocols.  (QMSPI)
  */

typedef struct {                                    /*!< QMSPI Structure                                                       */
  __IO uint32_t  QMSPI_MODE;                        /*!< QMSPI Mode Register                                                   */
  __IO uint32_t  QMSPI_CONTROL;                     /*!< QMSPI SPI Control                                                     */
  __I  uint32_t  QMSPI_EXECUTE;                     /*!< QMSPI Execute Register                                                */
  __IO uint32_t  QMSPI_INTERFACE_CONTROL;           /*!< QMSPI Interface Control Register                                      */
  __IO uint32_t  QMSPI_STATUS;                      /*!< QMSPI Status Register                                                 */
  __IO uint32_t  QMSPI_BUFFER_COUNT_STATUS;         /*!< QMSPI Buffer Count Status Register                                    */
  __IO uint32_t  QMSPI_INTERRUPT_ENABLE;            /*!< QMSPI Interrupt Enable Register                                       */
  __IO uint32_t  QMSPI_BUFFER_COUNT_TRIGGER;        /*!< QMSPI Buffer Count Trigger Register                                   */
  __IO uint32_t  QMSPI_TRAMSMIT_BUFFER;             /*!< QMSPI Transmit Buffer Register                                        */
  __IO uint32_t  QMSPI_RECEIVE_BUFFER;              /*!< QMSPI Receive Buffer Register                                         */
  __I  uint32_t  RESERVED[2];
  __IO uint32_t  QMSPI_DESCRIPTION_BUFFER_0;        /*!< QMSPI Description Buffer 0 Register                                   */
  __IO uint32_t  QMSPI_DESCRIPTION_BUFFER_1;        /*!< QMSPI Description Buffer 1 Register                                   */
  __IO uint32_t  QMSPI_DESCRIPTION_BUFFER_2;        /*!< QMSPI Description Buffer 2 Register                                   */
  __IO uint32_t  QMSPI_DESCRIPTION_BUFFER_3;        /*!< QMSPI Description Buffer 3 Register                                   */
  __IO uint32_t  QMSPI_DESCRIPTION_BUFFER_4;        /*!< QMSPI Description Buffer 4 Register                                   */
} QMSPI_Type;


/* ================================================================================ */
/* ================                      PS2_0                     ================ */
/* ================================================================================ */


/**
  * @brief There are four PS/2 Ports which are directly controlled by the EC. The hardware implementation eliminates
 the need to bit bang I/O ports to generate PS/2 traffic, however bit banging is available via the associated GPIO pins.  (PS2_0)
  */

typedef struct {                                    /*!< PS2_0 Structure                                                       */

  union {
    __I  uint32_t  RX_DATA;                         /*!< Data received from a peripheral are recorded in this register
                                                         in bits 7:0.                                                          */
    __O  uint32_t  TX_DATA;                         /*!< Writes to bits 7:0 of this register start a transmission of
                                                         the data in this register to the peripheral                           */
  };
  __IO uint32_t  CONTROL;                           /*!< PS2 Control Register                                                  */
  __IO uint32_t  STATUS;                            /*!< PS2 Status Register                                                   */
} PS2_0_Type;


/* ================================================================================ */
/* ================                    BC_LINK0                    ================ */
/* ================================================================================ */


/**
  * @brief This block provides BC-Link connectivity to a slave device. The BC-Link
 protocol includes a start bit to signal the beginning of a message and a turnaround (TAR)
 period for bus transfer between the Master and Companion devices.  (BC_LINK0)
  */

typedef struct {                                    /*!< BC_LINK0 Structure                                                    */
  __IO uint32_t  STATUS;                            /*!< BC-Link Status                                                        */
  __IO uint32_t  ADDRESS;                           /*!< BC-Link Address Register [7:0] Address in the Companion for
                                                         the BC-Link transaction.                                              */
  __IO uint32_t  DATA;                              /*!< BC-Link Data Register [7:0] this register hold data used in
                                                         a BC-Link transaction.                                                */
  __IO uint32_t  CLOCK_SELECT;                      /*!< BC-Link Clock Select Register [7:0] DIVIDER The BC Clock is
                                                         set to the Master Clock divided by this field, or 48MHz/ (Divider
                                                          +1). The clock divider bits can only can be changed when the
                                                          BC Bus is in soft RESET (when either the Reset bit is set by
                                                          software or when the BUSY bit is set by the interface).              */
} BC_LINK0_Type;


/* ================================================================================ */
/* ================                      TFDP                      ================ */
/* ================================================================================ */


/**
  * @brief The TFDP serially transmits Embedded Controller (EC)-originated
 diagnostic vectors to an external debug trace system.  (TFDP)
  */

typedef struct {                                    /*!< TFDP Structure                                                        */
  __IO uint8_t   DEBUG_DATA;                        /*!< Debug data to be shifted out on the TFDP Debug port. While data
                                                         is being shifted out, the Host Interface will 'hold-off' additional
                                                          writes to the data register until the transfer is complete.
                                                                                                                               */
  __I  uint8_t   RESERVED[3];
  __IO uint8_t   DEBUG_CONTROL;                     /*!< Debug Control Register                                                */
} TFDP_Type;


/* ================================================================================ */
/* ================                 PORT_80_DEBUG0                 ================ */
/* ================================================================================ */


/**
  * @brief The Port 80 BIOS Debug Port emulates the functionality of a "Port 80" ISA plug-in card. In addition, a timestamp for the debug data can be
 optionally added. Diagnostic data is written by the Host Interface to the Port 80 BIOS Debug Port, which is located in the Host I/O address space.
 The Port 80 BIOS Debug Port generates an interrupt to the EC when host data is available. The EC reads this data along with the timestamp, if enabled.  (PORT_80_DEBUG0)
  */

typedef struct {                                    /*!< PORT_80_DEBUG0 Structure                                              */
  __IO uint32_t  HOST_DATA;                         /*!< Host Data Register                                                    */
  __I  uint32_t  RESERVED[63];
  __IO uint32_t  ADDRESS;                           /*!< EC Data Register.                                                     */
  __IO uint32_t  CONFIGURATION;                     /*!< Configuration Register.                                               */
  __IO uint32_t  STATUS;                            /*!< Status Register                                                       */
  __IO uint32_t  COUNT;                             /*!< Count Register                                                        */
  __I  uint32_t  RESERVED1[136];
  __IO uint32_t  ACTIVATE;                          /*!< Activate Register                                                     */
} PORT_80_DEBUG0_Type;


/* ================================================================================ */
/* ================                       VCI                      ================ */
/* ================================================================================ */


/**
  * @brief The VBAT-Powered Control Interface has VBAT-powered combinational logic and input and output signal pins.
 The block interfaces with the RTC With Date and DST Adjustment as well as the Week Alarm.  (VCI)
  */

typedef struct {                                    /*!< VCI Structure                                                         */
  __IO uint32_t  VCI_REG;                           /*!< VCI Register                                                          */
  __IO uint32_t  LATCH_ENABLE;                      /*!< Latch Enable Register                                                 */
  __IO uint32_t  LATCH_RESETS;                      /*!< Latch Resets Register                                                 */
  __IO uint32_t  VCI_INPUT_ENABLE;                  /*!< VCI Input Enable Register                                             */
  __IO uint32_t  HOLDOFF_COUNT;                     /*!< Holdoff Count Register                                                */
  __IO uint32_t  VCI_POLARITY;                      /*!< VCI Polarity Register                                                 */
  __IO uint32_t  VCI_POSEDGE_DETECT;                /*!< VCI Posedge Detect Register                                           */
  __IO uint32_t  VCI_NEGEDGE_DETECT;                /*!< VCI Negedge Detect Register                                           */
  __IO uint32_t  VCI_BUFFER_ENABLE;                 /*!< VCI Buffer Enable Register                                            */
} VCI_Type;


/* ================================================================================ */
/* ================                    VBAT_RAM                    ================ */
/* ================================================================================ */


/**
  * @brief The VBAT Powered RAM provides a 128 Byte Random Accessed Memory that is operational while the main
 power rail is operational, and will retain its values powered by battery power while the main rail is unpowered.  (VBAT_RAM)
  */

typedef struct {                                    /*!< VBAT_RAM Structure                                                    */
  __IO uint32_t  VBAT_RAM_DW_[32];                  /*!< 32-bits of VBAT powered RAM.                                          */
} VBAT_RAM_Type;


/* ================================================================================ */
/* ================                      VBAT                      ================ */
/* ================================================================================ */


/**
  * @brief The VBAT Register Bank block is a block implemented for aggregating miscellaneous
 battery-backed registers required the host and by the Embedded Controller (EC) Subsystem that are
 not unique to a block implemented in the EC subsystem.  (VBAT)
  */

typedef struct {                                    /*!< VBAT Structure                                                        */
  __IO uint8_t   PFR_STS;                           /*!< The Power-Fail and Reset Status Register collects and retains
                                                         the VBAT RST and WDT event status when VCC1 is unpowered.             */
  __I  uint8_t   RESERVED[7];
  __IO uint32_t  CLOCK_EN;                          /*!< CLOCK ENABLE                                                          */
  __I  uint32_t  RESERVED1[5];
  __IO uint32_t  MONOTONIC_COUNTER;                 /*!< MONOTONIC COUNTER                                                     */
  __IO uint32_t  COUNTER_HIWORD;                    /*!< COUNTER HIWORD                                                        */
  __IO uint32_t  VWIRE_BACKUP;                      /*!< VWIRE_BACKUP                                                          */
} VBAT_Type;


/* ================================================================================ */
/* ================                   EC_REG_BANK                  ================ */
/* ================================================================================ */


/**
  * @brief This block is designed to be accessed internally by the EC via the register interface.  (EC_REG_BANK)
  */

typedef struct {                                    /*!< EC_REG_BANK Structure                                                 */
  __I  uint32_t  RESERVED;
  __IO uint32_t  AHB_ERROR_ADDRESS;                 /*!< AHB Error Address [0:0] AHB_ERR_ADDR, In priority order:
                                                          1. AHB address is registered when an AHB error occurs on the
                                                         processor's AHB master port and the register value was
                                                          already 0. This way only the first address to generate an exception
                                                          is captured.
                                                          2. The processor can clear this register by writing any 32-bit
                                                          value to this register.                                              */
  __I  uint32_t  RESERVED1[3];
  __IO uint8_t   AHB_ERROR_CONTROL;                 /*!< AHB Error Control [0:0] AHB_ERROR_DISABLE, 0: EC memory exceptions
                                                         are enabled. 1: EC memory exceptions are disabled.                    */
  __I  uint8_t   RESERVED2[3];
  __IO uint32_t  INTERRUPT_CONTROL;                 /*!< Interrupt Control [0:0] NVIC_EN (NVIC_EN) This bit enables Alternate
                                                         NVIC IRQ's Vectors. The Alternate NVIC Vectors provides each
                                                          interrupt event with a dedicated (direct) NVIC vector.
                                                          0 = Alternate NVIC vectors disabled, 1= Alternate NVIC vectors
                                                          enabled                                                              */
  __IO uint32_t  ETM_TRACE_ENABLE;                  /*!< ETM TRACE Enable [0:0] TRACE_EN (TRACE_EN) This bit enables
                                                         the ARM TRACE debug port (ETM/ITM). The Trace Debug Interface
                                                          pins are forced to the TRACE functions. 0 = ARM TRACE port disabled,
                                                          1= ARM TRACE port enabled                                            */
  __IO uint32_t  DEBUG_Enable;                      /*!< Debug Enable Register                                                 */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  WDT_EVENT_COUNT;                   /*!< WDT Event Count [3:0] WDT_COUNT (WDT_COUNT) These EC R/W bits
                                                         are cleared to 0 on VCC1 POR, but not on a WDT.
                                                          Note: This field is written by Boot ROM firmware to indicate
                                                          the number of times a WDT fired before loading a good EC code
                                                          image.                                                               */
  __IO uint32_t  AES_HASH_BYTE_SWAP_CONTROL;        /*!< AES HASH Byte Swap Control Register.                                  */
  __I  uint32_t  RESERVED4[2];
  __IO uint32_t  SYSTEM_SHUTDOWN_RESET;             /*!< System Shutdown Reset                                                 */
  __I  uint32_t  RESERVED5;
  __IO uint32_t  MISC_TRIM;                         /*!< Misc Trim                                                             */
  __I  uint32_t  RESERVED6[6];
  __IO uint32_t  CRYPTO_SOFT_RESET;                 /*!< System Shutdown Reset                                                 */
  __I  uint32_t  RESERVED7;
  __IO uint32_t  GPIO_BANK_POWER;                   /*!< GPIO Bank Power Register                                              */
  __I  uint32_t  RESERVED8[2];
  __IO uint32_t  JTAG_MASTER_CFG;                   /*!< JTAG Master Configuration Register                                    */
  __I  uint32_t  JTAG_MASTER_STS;                   /*!< JTAG Master Status Register                                           */
  __IO uint32_t  JTAG_MASTER_TDO;                   /*!< JTAG Master TDO Register                                              */
  __IO uint32_t  JTAG_MASTER_TDI;                   /*!< JTAG Master TDI Register                                              */
  __IO uint32_t  JTAG_MASTER_TMS;                   /*!< JTAG Master TMS Register                                              */
  __IO uint32_t  JTAG_MASTER_CMD;                   /*!< JTAG Master Command Register                                          */
} EC_REG_BANK_Type;


/* ================================================================================ */
/* ================                      EFUSE                     ================ */
/* ================================================================================ */


/**
  * @brief The eFUSE block provides a means of programming and accessing the eFUSE bits.  (EFUSE)
  */

typedef struct {                                    /*!< EFUSE Structure                                                       */
  __IO uint32_t  CONTROL;                           /*!< eFUSE CONTROL Register                                                */
  __IO uint32_t  MANUAL_CONTROL;                    /*!< Manual Control Register                                               */
  __IO uint32_t  MANUAL_MODE_ADDRESS;               /*!< MANUAL MODE ADDRESS REGISTER                                          */
  __IO uint32_t  MANUAL_MODE_DATA;                  /*!< MANUAL MODE DATA REGISTER                                             */
  __IO uint32_t  EFUSE_MEMORY_DW_[128];             /*!< 512 Bytes of EFUSE Memory (IP_MEM) Represented in 128 DW chunks:
                                                         eFUSE memory read-back data. Access to this region depends on
                                                          the operating mode: NORMAL MODE: Reading any of the bytes
                                                          starting at this base will automatically start the controller
                                                          to sequence all eFUSE signals to generate read data. Wait cycles
                                                          added
                                                          to the read cycle as appropriate. MANUAL MODE: Refer to the
                                                          manual mode section for the proper procedure for accessing data
                                                          in this mode.
                                                          See REG_MAN_CTRL.MAN_EN and REG_CTRL.EXT_PRGM bits f                 */
} EFUSE_Type;


/* ================================================================================ */
/* ================                      IMSPI                     ================ */
/* ================================================================================ */


/**
  * @brief Internal Master SPI.  (IMSPI)
  */

typedef struct {                                    /*!< IMSPI Structure                                                       */
  __IO uint32_t  IMSPI_MODE;                        /*!< IMSPI Mode Register                                                   */
  __IO uint32_t  IMSPI_STATUS;                      /*!< IMSPI Status Register                                                 */
  __IO uint32_t  IMSPI_INT_ENABLE;                  /*!< IMSPI Interrupt Enable Register                                       */
  __IO uint32_t  IMSPI_TIMEOUT_CONTROL;             /*!< IMSPI Timeout Control Register                                        */
} IMSPI_Type;


/* ================================================================================ */
/* ================                      ASIF                      ================ */
/* ================================================================================ */


/**
  * @brief The auxiliary serial interface block (ASIF) allows the LPC host and EC to use index addressing to access registers residing
 in an external IC, which is connected to the EC via SPI. The block presents similar but separate logical, i.e., programming, interfaces
 to the host and EC. Control logic functions as hardware SPI driver, interpreting commands issued to these interfaces and controlling an embedded
 SPI module to effect transfers to the external IC.  (ASIF)
  */

typedef struct {                                    /*!< ASIF Structure                                                        */
  __IO uint8_t   LPC_BAL;                           /*!< LPC BAL Register                                                      */
  __IO uint8_t   LPC_BAH;                           /*!< LPC BAH Register.                                                     */
  __IO uint8_t   SCRATCH_0;                         /*!< Scratch 0 Register                                                    */
  __IO uint8_t   SCRATCH_1;                         /*!< Scratch 1 Register                                                    */
  __IO uint8_t   SCRATCH_2;                         /*!< Scratch 2 Register                                                    */
  __IO uint8_t   SCRATCH_3;                         /*!< Scratch 3 Register                                                    */
  __IO uint8_t   SCRATCH_4;                         /*!< Scratch 4 Register                                                    */
  __IO uint8_t   SCRATCH_5;                         /*!< Scratch 5 Register                                                    */
  __IO uint8_t   SCRATCH_6;                         /*!< Scratch 6 Register                                                    */
  __IO uint8_t   SCRATCH_7;                         /*!< Scratch 7 Register                                                    */
  __IO uint8_t   SCRATCH_8;                         /*!< Scratch 8 Register                                                    */
  __IO uint8_t   SCRATCH_9;                         /*!< Scratch 99 Register                                                   */
  __IO uint8_t   LPC_AIXL;                          /*!< LPC AIXL Register                                                     */
  __IO uint8_t   LPC_AIXH;                          /*!< LPC AIXH Register                                                     */

  union {
    __O  uint8_t   LPC_DATA_OUT;                    /*!< LPC Data Out Register                                                 */
    __I  uint8_t   LPC_DATA_IN;                     /*!< LPC Data In Register                                                  */
  };
  __IO uint8_t   LPC_STATUS;                        /*!< LPC Status Register                                                   */
  __I  uint32_t  RESERVED[60];
  __IO uint8_t   EC_AIXL;                           /*!< EC AIXL Register                                                      */
  __IO uint8_t   EC_AIXH;                           /*!< EC AIXH Register                                                      */

  union {
    __O  uint8_t   EC_DATA_OUT;                     /*!< EC Data Out Register                                                  */
    __I  uint8_t   EC_DATA_IN;                      /*!< EC Data In Register                                                   */
  };
  __IO uint8_t   EC_STATUS;                         /*!< EC Status Register                                                    */
  __IO uint8_t   EC_BAL;                            /*!< EC BAL Register                                                       */
  __IO uint8_t   EC_BAH;                            /*!< EC BAH Register.                                                      */
  __I  uint16_t  RESERVED1[5];
  __IO uint32_t  EC_INT_ENABLE;                     /*!< EC Interrupt Enable Register                                          */
  __IO uint32_t  EC_INT_STATUS;                     /*!< EC Interrupt Status Register                                          */
  __IO uint32_t  TIMEOUT_COUNTER;                   /*!< Timeout Counter Register.                                             */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  BLOCK_CONFIG;                      /*!< Block Configuration Register                                          */
  __I  uint32_t  RESERVED3[3];
  __IO uint32_t  SPI_CLOCK_GEN;                     /*!< SPI Clock Generator Register.                                         */
  __IO uint32_t  SPI_CONTROL;                       /*!< SPI Control Register                                                  */
  __IO uint32_t  SPI_CLOCK_CONTROL;                 /*!< SPI Clock Control Register                                            */
  __IO uint32_t  SPI_ENABLE;                        /*!< SPI Enable Register                                                   */
} ASIF_Type;


/* --------------------  End of section using anonymous unions  ------------------- */
#if defined(__CC_ARM)
  #pragma pop
#elif defined(__ICCARM__)
  /* leave anonymous unions enabled */
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning restore
#else
  #warning Not supported compiler type
#endif




/* ================================================================================ */
/* ================              Peripheral memory map             ================ */
/* ================================================================================ */

#define PCR_BASE                        0x40080100UL
#define DMA_MAIN_BASE                   0x40002400UL
#define DMA_CHAN00_BASE                 0x40002440UL
#define DMA_CHAN01_BASE                 0x40002480UL
#define DMA_CHAN02_BASE                 0x400024C0UL
#define DMA_CHAN03_BASE                 0x40002500UL
#define DMA_CHAN04_BASE                 0x40002540UL
#define DMA_CHAN05_BASE                 0x40002580UL
#define DMA_CHAN06_BASE                 0x400025C0UL
#define DMA_CHAN07_BASE                 0x40002600UL
#define DMA_CHAN08_BASE                 0x40002640UL
#define DMA_CHAN09_BASE                 0x40002680UL
#define DMA_CHAN10_BASE                 0x400026C0UL
#define DMA_CHAN11_BASE                 0x40002700UL
#define DMA_CHAN12_BASE                 0x40002740UL
#define DMA_CHAN13_BASE                 0x40002780UL
#define INTS_BASE                       0x4000E000UL
#define LPC_BASE                        0x400F3000UL
#define ESPI_IO_BASE                    0x400F3400UL
#define ESPI_MEMORY_BASE                0x400F3800UL
#define ESPI_MSVW00_06_BASE             0x400F9C00UL
#define ESPI_MSVW07_10_BASE             0x400F9C54UL
#define ESPI_SMVW00_07_BASE             0x400F9E00UL
#define GCR_BASE                        0x400FFC00UL
#define KBC_BASE                        0x400F0400UL
#define PORT92_BASE                     0x400F2000UL
#define ACPI_EC0_BASE                   0x400F0800UL
#define ACPI_EC1_BASE                   0x400F0C00UL
#define ACPI_EC2_BASE                   0x400F1000UL
#define ACPI_EC3_BASE                   0x400F1400UL
#define ACPI_EC4_BASE                   0x400F1800UL
#define PM1_BASE                        0x400F1C00UL
#define EMI0_BASE                       0x400F4000UL
#define EMI1_BASE                       0x400F4400UL
#define EMI2_BASE                       0x400F4800UL
#define MBX_BASE                        0x400F0000UL
#define UART0_BASE                      0x400F2400UL
#define UART1_BASE                      0x400F2800UL
#define GPIO_000_036_BASE               0x40081000UL
#define GPIO_040_076_BASE               0x40081080UL
#define GPIO_100_137_BASE               0x40081100UL
#define GPIO_140_176_BASE               0x40081180UL
#define GPIO_200_236_BASE               0x40081200UL
#define GPIO_240_257_BASE               0x4008127CUL
#define INPUT_OUTPUT_GPIO_BASE          0x40081300UL
#define GPIO_PIN_CONTROL_2_BASE         0x40081500UL
#define WDT_BASE                        0x40000000UL
#define TIMER0_BASE                     0x40000C00UL
#define TIMER1_BASE                     0x40000C20UL
#define TIMER2_BASE                     0x40000C40UL
#define TIMER3_BASE                     0x40000C60UL
#define TIMER4_BASE                     0x40000C80UL
#define TIMER5_BASE                     0x40000CA0UL
#define COUNTER_TIMER0_BASE             0x40000D00UL
#define COUNTER_TIMER1_BASE             0x40000D20UL
#define COUNTER_TIMER2_BASE             0x40000D40UL
#define COUNTER_TIMER3_BASE             0x40000D60UL
#define CAPTURE_COMPARE_TIMER_BASE      0x40001000UL
#define HTM0_BASE                       0x40009800UL
#define HTM1_BASE                       0x40009820UL
#define RTOS_BASE                       0x40007400UL
#define RTC_BASE                        0x400F5000UL
#define WEEK_BASE                       0x4000AC80UL
#define TACH0_BASE                      0x40006000UL
#define TACH1_BASE                      0x40006010UL
#define TACH2_BASE                      0x40006020UL
#define PWM0_BASE                       0x40005800UL
#define PWM1_BASE                       0x40005810UL
#define PWM2_BASE                       0x40005820UL
#define PWM3_BASE                       0x40005830UL
#define PWM4_BASE                       0x40005840UL
#define PWM5_BASE                       0x40005850UL
#define PWM6_BASE                       0x40005860UL
#define PWM7_BASE                       0x40005870UL
#define PWM8_BASE                       0x40005880UL
#define PWM9_BASE                       0x40005890UL
#define PWM10_BASE                      0x400058A0UL
#define PWM11_BASE                      0x400058B0UL
#define PECI_BASE                       0x40006400UL
#define ADC_BASE                        0x40007C00UL
#define FAN0_BASE                       0x4000A000UL
#define FAN1_BASE                       0x4000A080UL
#define PROCHOT_MON_BASE                0x40003400UL
#define POWERGUARD0_BASE                0x40003000UL
#define POWERGUARD1_BASE                0x40003080UL
#define EEPROM_BASE                     0x40002C00UL
#define LED0_BASE                       0x4000B800UL
#define LED1_BASE                       0x4000B900UL
#define LED2_BASE                       0x4000BA00UL
#define LED3_BASE                       0x4000BB00UL
#define RC_ID0_BASE                     0x40001400UL
#define RC_ID1_BASE                     0x40001480UL
#define RC_ID2_BASE                     0x40001500UL
#define KMS_BASE                        0x40009C00UL
#define SMB0_BASE                       0x40004000UL
#define SMB1_BASE                       0x40004400UL
#define SMB2_BASE                       0x40004800UL
#define SMB3_BASE                       0x40004C00UL
#define GP_SPI0_BASE                    0x40009400UL
#define GP_SPI1_BASE                    0x40009480UL
#define QMSPI_BASE                      0x40005400UL
#define PS2_0_BASE                      0x40009000UL
#define PS2_1_BASE                      0x40009040UL
#define PS2_2_BASE                      0x40009080UL
#define BC_LINK0_BASE                   0x4000CD00UL
#define BC_LINK1_BASE                   0x4000CD20UL
#define TFDP_BASE                       0x40008C00UL
#define PORT_80_DEBUG0_BASE             0x400F8000UL
#define PORT_80_DEBUG1_BASE             0x400F8400UL
#define VCI_BASE                        0x4000AE00UL
#define VBAT_RAM_BASE                   0x4000A800UL
#define VBAT_BASE                       0x4000A400UL
#define EC_REG_BANK_BASE                0x4000FC00UL
#define EFUSE_BASE                      0x40082000UL
#define IMSPI_BASE                      0x40220000UL
#define ASIF_BASE                       0x400FC000UL


/* ================================================================================ */
/* ================             Peripheral declaration             ================ */
/* ================================================================================ */

#define PCR                             ((PCR_Type                *) PCR_BASE)
#define DMA_MAIN                        ((DMA_MAIN_Type           *) DMA_MAIN_BASE)
#define DMA_CHAN00                      ((DMA_CHAN00_Type         *) DMA_CHAN00_BASE)
#define DMA_CHAN01                      ((DMA_CHAN01_Type         *) DMA_CHAN01_BASE)
#define DMA_CHAN02                      ((DMA_CHAN02_Type         *) DMA_CHAN02_BASE)
#define DMA_CHAN03                      ((DMA_CHAN02_Type         *) DMA_CHAN03_BASE)
#define DMA_CHAN04                      ((DMA_CHAN02_Type         *) DMA_CHAN04_BASE)
#define DMA_CHAN05                      ((DMA_CHAN02_Type         *) DMA_CHAN05_BASE)
#define DMA_CHAN06                      ((DMA_CHAN02_Type         *) DMA_CHAN06_BASE)
#define DMA_CHAN07                      ((DMA_CHAN02_Type         *) DMA_CHAN07_BASE)
#define DMA_CHAN08                      ((DMA_CHAN02_Type         *) DMA_CHAN08_BASE)
#define DMA_CHAN09                      ((DMA_CHAN02_Type         *) DMA_CHAN09_BASE)
#define DMA_CHAN10                      ((DMA_CHAN02_Type         *) DMA_CHAN10_BASE)
#define DMA_CHAN11                      ((DMA_CHAN02_Type         *) DMA_CHAN11_BASE)
#define DMA_CHAN12                      ((DMA_CHAN02_Type         *) DMA_CHAN12_BASE)
#define DMA_CHAN13                      ((DMA_CHAN02_Type         *) DMA_CHAN13_BASE)
#define INTS                            ((INTS_Type               *) INTS_BASE)
#define LPC                             ((LPC_Type                *) LPC_BASE)
#define ESPI_IO                         ((ESPI_IO_Type            *) ESPI_IO_BASE)
#define ESPI_MEMORY                     ((ESPI_MEMORY_Type        *) ESPI_MEMORY_BASE)
#define ESPI_MSVW00_06                  ((ESPI_MSVW00_06_Type     *) ESPI_MSVW00_06_BASE)
#define ESPI_MSVW07_10                  ((ESPI_MSVW07_10_Type     *) ESPI_MSVW07_10_BASE)
#define ESPI_SMVW00_07                  ((ESPI_SMVW00_07_Type     *) ESPI_SMVW00_07_BASE)
#define GCR                             ((GCR_Type                *) GCR_BASE)
#define KBC                             ((KBC_Type                *) KBC_BASE)
#define PORT92                          ((PORT92_Type             *) PORT92_BASE)
#define ACPI_EC0                        ((ACPI_EC0_Type           *) ACPI_EC0_BASE)
#define ACPI_EC1                        ((ACPI_EC0_Type           *) ACPI_EC1_BASE)
#define ACPI_EC2                        ((ACPI_EC0_Type           *) ACPI_EC2_BASE)
#define ACPI_EC3                        ((ACPI_EC0_Type           *) ACPI_EC3_BASE)
#define ACPI_EC4                        ((ACPI_EC0_Type           *) ACPI_EC4_BASE)
#define PM1                             ((PM1_Type                *) PM1_BASE)
#define EMI0                            ((EMI0_Type               *) EMI0_BASE)
#define EMI1                            ((EMI0_Type               *) EMI1_BASE)
#define EMI2                            ((EMI0_Type               *) EMI2_BASE)
#define MBX                             ((MBX_Type                *) MBX_BASE)
#define UART0                           ((UART0_Type              *) UART0_BASE)
#define UART1                           ((UART0_Type              *) UART1_BASE)
#define GPIO_000_036                    ((GPIO_000_036_Type       *) GPIO_000_036_BASE)
#define GPIO_040_076                    ((GPIO_040_076_Type       *) GPIO_040_076_BASE)
#define GPIO_100_137                    ((GPIO_100_137_Type       *) GPIO_100_137_BASE)
#define GPIO_140_176                    ((GPIO_140_176_Type       *) GPIO_140_176_BASE)
#define GPIO_200_236                    ((GPIO_200_236_Type       *) GPIO_200_236_BASE)
#define GPIO_240_257                    ((GPIO_240_257_Type       *) GPIO_240_257_BASE)
#define INPUT_OUTPUT_GPIO               ((INPUT_OUTPUT_GPIO_Type  *) INPUT_OUTPUT_GPIO_BASE)
#define GPIO_PIN_CONTROL_2              ((GPIO_PIN_CONTROL_2_Type *) GPIO_PIN_CONTROL_2_BASE)
#define WDT                             ((WDT_Type                *) WDT_BASE)
#define TIMER0                          ((TIMER0_Type             *) TIMER0_BASE)
#define TIMER1                          ((TIMER0_Type             *) TIMER1_BASE)
#define TIMER2                          ((TIMER0_Type             *) TIMER2_BASE)
#define TIMER3                          ((TIMER0_Type             *) TIMER3_BASE)
#define TIMER4                          ((TIMER0_Type             *) TIMER4_BASE)
#define TIMER5                          ((TIMER0_Type             *) TIMER5_BASE)
#define COUNTER_TIMER0                  ((COUNTER_TIMER0_Type     *) COUNTER_TIMER0_BASE)
#define COUNTER_TIMER1                  ((COUNTER_TIMER0_Type     *) COUNTER_TIMER1_BASE)
#define COUNTER_TIMER2                  ((COUNTER_TIMER0_Type     *) COUNTER_TIMER2_BASE)
#define COUNTER_TIMER3                  ((COUNTER_TIMER0_Type     *) COUNTER_TIMER3_BASE)
#define CAPTURE_COMPARE_TIMER           ((CAPTURE_COMPARE_TIMER_Type *) CAPTURE_COMPARE_TIMER_BASE)
#define HTM0                            ((HTM0_Type               *) HTM0_BASE)
#define HTM1                            ((HTM0_Type               *) HTM1_BASE)
#define RTOS                            ((RTOS_Type               *) RTOS_BASE)
#define RTC                             ((RTC_Type                *) RTC_BASE)
#define WEEK                            ((WEEK_Type               *) WEEK_BASE)
#define TACH0                           ((TACH0_Type              *) TACH0_BASE)
#define TACH1                           ((TACH0_Type              *) TACH1_BASE)
#define TACH2                           ((TACH0_Type              *) TACH2_BASE)
#define PWM0                            ((PWM0_Type               *) PWM0_BASE)
#define PWM1                            ((PWM0_Type               *) PWM1_BASE)
#define PWM2                            ((PWM0_Type               *) PWM2_BASE)
#define PWM3                            ((PWM0_Type               *) PWM3_BASE)
#define PWM4                            ((PWM0_Type               *) PWM4_BASE)
#define PWM5                            ((PWM0_Type               *) PWM5_BASE)
#define PWM6                            ((PWM0_Type               *) PWM6_BASE)
#define PWM7                            ((PWM0_Type               *) PWM7_BASE)
#define PWM8                            ((PWM0_Type               *) PWM8_BASE)
#define PWM9                            ((PWM0_Type               *) PWM9_BASE)
#define PWM10                           ((PWM0_Type               *) PWM10_BASE)
#define PWM11                           ((PWM0_Type               *) PWM11_BASE)
#define PECI                            ((PECI_Type               *) PECI_BASE)
#define ADC                             ((ADC_Type                *) ADC_BASE)
#define FAN0                            ((FAN0_Type               *) FAN0_BASE)
#define FAN1                            ((FAN0_Type               *) FAN1_BASE)
#define PROCHOT_MON                     ((PROCHOT_MON_Type        *) PROCHOT_MON_BASE)
#define POWERGUARD0                     ((POWERGUARD0_Type        *) POWERGUARD0_BASE)
#define POWERGUARD1                     ((POWERGUARD0_Type        *) POWERGUARD1_BASE)
#define EEPROM                          ((EEPROM_Type             *) EEPROM_BASE)
#define LED0                            ((LED0_Type               *) LED0_BASE)
#define LED1                            ((LED0_Type               *) LED1_BASE)
#define LED2                            ((LED0_Type               *) LED2_BASE)
#define LED3                            ((LED0_Type               *) LED3_BASE)
#define RC_ID0                          ((RC_ID0_Type             *) RC_ID0_BASE)
#define RC_ID1                          ((RC_ID0_Type             *) RC_ID1_BASE)
#define RC_ID2                          ((RC_ID0_Type             *) RC_ID2_BASE)
#define KMS                             ((KMS_Type                *) KMS_BASE)
#define SMB0                            ((SMB0_Type               *) SMB0_BASE)
#define SMB1                            ((SMB0_Type               *) SMB1_BASE)
#define SMB2                            ((SMB0_Type               *) SMB2_BASE)
#define SMB3                            ((SMB0_Type               *) SMB3_BASE)
#define GP_SPI0                         ((GP_SPI0_Type            *) GP_SPI0_BASE)
#define GP_SPI1                         ((GP_SPI0_Type            *) GP_SPI1_BASE)
#define QMSPI                           ((QMSPI_Type              *) QMSPI_BASE)
#define PS2_0                           ((PS2_0_Type              *) PS2_0_BASE)
#define PS2_1                           ((PS2_0_Type              *) PS2_1_BASE)
#define PS2_2                           ((PS2_0_Type              *) PS2_2_BASE)
#define BC_LINK0                        ((BC_LINK0_Type           *) BC_LINK0_BASE)
#define BC_LINK1                        ((BC_LINK0_Type           *) BC_LINK1_BASE)
#define TFDP                            ((TFDP_Type               *) TFDP_BASE)
#define PORT_80_DEBUG0                  ((PORT_80_DEBUG0_Type     *) PORT_80_DEBUG0_BASE)
#define PORT_80_DEBUG1                  ((PORT_80_DEBUG0_Type     *) PORT_80_DEBUG1_BASE)
#define VCI                             ((VCI_Type                *) VCI_BASE)
#define VBAT_RAM                        ((VBAT_RAM_Type           *) VBAT_RAM_BASE)
#define VBAT                            ((VBAT_Type               *) VBAT_BASE)
#define EC_REG_BANK                     ((EC_REG_BANK_Type        *) EC_REG_BANK_BASE)
#define EFUSE                           ((EFUSE_Type              *) EFUSE_BASE)
#define IMSPI                           ((IMSPI_Type              *) IMSPI_BASE)
#define ASIF                            ((ASIF_Type               *) ASIF_BASE)


/** @} */ /* End of group Device_Peripheral_Registers */
/** @} */ /* End of group MCHP_MEC2016_internal */
/** @} */ /* End of group Microchip Technology Inc. */

#ifdef __cplusplus
}
#endif


#endif  /* MCHP_MEC2016_internal_H */

