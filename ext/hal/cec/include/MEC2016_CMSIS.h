/**************************************************************************//**
 * @file     MEC2016_CMSIS.h
 * @brief    CMSIS Cortex-M# Core Peripheral Access Layer Header File for
 *           Device MEC2016
 * @version  V3.10
 * @date     23. November 2012
 *
 * @note
 *
 ******************************************************************************/
/* Copyright (c) 2012 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/


#ifndef MEC2016_H
#define MEC2016_H

#ifdef __cplusplus
 extern "C" {
#endif

/* ToDo: replace '<Device>' with your device name; add your doxyGen comment   */
/** @addtogroup <Device>_Definitions <Device> Definitions
  This file defines all structures and symbols for <Device>:
    - registers and bitfields
    - peripheral base address
    - peripheral ID
    - Peripheral definitions
  @{
*/


/******************************************************************************/
/*                Processor and Core Peripherals                              */
/******************************************************************************/
/** @addtogroup <Device>_CMSIS Device CMSIS Definitions
  Configuration of the Cortex-M# Processor and Core Peripherals
  @{
*/

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

typedef enum IRQn
{
/******  Cortex-M# Processor Exceptions Numbers ***************************************************/

  NonMaskableInt_IRQn           = -14,      /*!<  2 Non Maskable Interrupt                        */
  MemoryManagement_IRQn         = -12,      /*!<  4 Memory Management Interrupt                   */
  BusFault_IRQn                 = -11,      /*!<  5 Bus Fault Interrupt                           */
  UsageFault_IRQn               = -10,      /*!<  6 Usage Fault Interrupt                         */
  SVCall_IRQn                   = -5,       /*!< 11 SV Call Interrupt                             */
  DebugMonitor_IRQn             = -4,       /*!< 12 Debug Monitor Interrupt                       */
  PendSV_IRQn                   = -2,       /*!< 14 Pend SV Interrupt                             */
  SysTick_IRQn                  = -1,       /*!< 15 System Tick Interrupt                         */

/******  Device Specific Interrupt Numbers ********************************************************/
/* Device specific external interrupt numbers
   according the interrupt handlers defined in startup_Device.S                                   */
/* NOTE: GIRQ22 is wake only and does not generate a CPU interrupt. Therefore it is not present
 * in the interrupt vector table or the following enumerated type */
    GIRQ08_IRQn                 = 0,
    GIRQ09_IRQn                 = 1,
    GIRQ10_IRQn                 = 2,
    GIRQ11_IRQn                 = 3,
    GIRQ12_IRQn                 = 4,
    GIRQ13_IRQn                 = 5,
    GIRQ14_IRQn                 = 6,
    GIRQ15_IRQn                 = 7,
    GIRQ16_IRQn                 = 8,
    GIRQ17_IRQn                 = 9,
    GIRQ18_IRQn                 = 10,
    GIRQ19_IRQn                 = 11,
    GIRQ20_IRQn                 = 12,
    GIRQ21_IRQn                 = 13,
    GIRQ23_IRQn                 = 14,
    GIRQ24_IRQn                 = 15,
    GIRQ25_IRQn                 = 16,
    GIRQ26_IRQn                 = 17,
    /* Reserved gap, 18-19 */
    /* GIRQ's 8 - 12, 22, 24 - 26 no direct */
    /* GIRQ13 */
    SMB0_IRQn                   = 20,
    SMB1_IRQn                   = 21,
    SMB2_IRQn                   = 22,
    SMB3_IRQn                   = 23,
    DMA0_IRQn                   = 24,
    /* GIRQ14 */
    DMA1_IRQn                   = 25,
    DMA2_IRQn                   = 26,
    DMA3_IRQn                   = 27,
    DMA4_IRQn                   = 28,
    DMA5_IRQn                   = 29,
    DMA6_IRQn                   = 30,
    DMA7_IRQn                   = 31,
    DMA8_IRQn                   = 32,
    DMA9_IRQn                   = 33,
    DMA10_IRQn                  = 34,
    DMA11_IRQn                  = 35,
    DMA12_IRQn                  = 36,
    DMA13_IRQn                  = 37,
    /* Reserved gap, 38-39 */
    /* GIRQ15 */
    UART0_IRQn                  = 40,
    UART1_IRQn                  = 41,
    EMI0_IRQn                   = 42,
    EMI1_IRQn                   = 43,
    EMI2_IRQn                   = 44,
    ACPI_EC0_IBF_IRQn           = 45,
    ACPI_EC0_OBF_IRQn           = 46,
    ACPI_EC1_IBF_IRQn           = 47,
    ACPI_EC1_OBF_IRQn           = 48,
    ACPI_EC2_IBF_IRQn           = 49,
    ACPI_EC2_OBF_IRQn           = 40,
    ACPI_EC3_IBF_IRQn           = 51,
    ACPI_EC3_OBF_IRQn           = 52,
    ACPI_EC4_IBF_IRQn           = 53,
    ACPI_EC4_OBF_IRQn           = 54,
    PM1_CTL_IRQn                = 55,
    PM1_EN_IRQn                 = 56,
    PM1_STS_IRQn                = 57,
    MIF8042_OBF_IRQn            = 58,
    MIF8042_IBF_IRQn            = 59,
    MB_H2EC_IRQn                = 60,
    MB_DATA_IRQn                = 61,
    P80A_IRQn                   = 62,
    P80B_IRQn                   = 63,
    LenAsic_IRQn                = 64,
    /* GIRQ16 */
    PKE_ERR_IRQn                = 65,
    PKE_END_IRQn                = 66,
    RNG_IRQn                    = 67,
    AES_IRQn                    = 68,
    HASH_IRQn                   = 69,
    /* GIRQ17 */
    PECI_IRQn                   = 70,
    TACH0_IRQn                  = 71,
    TACH1_IRQn                  = 72,
    TACH2_IRQn                  = 73,
    R2P0_FAIL_IRQn              = 74,
    R2P0_STALL_IRQn             = 75,
    R2P1_FAIL_IRQn              = 76,
    R2P1_STALL_IRQn             = 77,
    ADC_SNGL_IRQn               = 78,
    ADC_RPT_IRQn                = 79,
    RCID0_IRQn                  = 80,
    RCID1_IRQn                  = 81,
    RCID2_IRQn                  = 82,
    LED0_IRQn                   = 83,
    LED1_IRQn                   = 84,
    LED2_IRQn                   = 85,
    LED3_IRQn                   = 86,
    PHOT_IRQn                   = 87,
    PWRGD0_IRQn                 = 88,
    PWRGD1_IRQn                 = 89,
    /* GIRQ18 */
    LPCBERR_IRQn                = 90,
    QMSPI0_IRQn                 = 91,
    GPSPI0_TX_IRQn              = 92,
    GPSPI0_RX_IRQn              = 93,
    GPSPI1_TX_IRQn              = 94,
    GPSPI1_RX_IRQn              = 95,
    BC0_BUSY_IRQn               = 96,
    BC0_ERR_IRQn                = 97,
    BC1_BUSY_IRQn               = 98,
    BC1_ERR_IRQn                = 99,
    PS2_0_IRQn                  = 100,
    PS2_1_IRQn                  = 101,
    PS2_2_IRQn                  = 102,
    /* GIRQ19 */
    ESPI_PC_IRQn                = 103,
    ESPI_BM1_IRQn               = 104,
    ESPI_BM2_IRQn               = 105,
    ESPI_LTR_IRQn               = 106,
    ESPI_OOB_UP_IRQn            = 107,
    ESPI_OOB_DN_IRQn            = 108,
    ESPI_FLASH_IRQn             = 109,
    ESPI_RESET_IRQn             = 110,
    /* GIRQ20 no direct */
    /* GIRQ21 */
    RTMR_IRQn                   = 111,
    HTMR0_IRQn                  = 112,
    HTMR1_IRQn                  = 113,
    WK_IRQn                     = 114,
    WKSUB_IRQn                  = 115,
    WKSEC_IRQn                  = 116,
    WKSUBSEC_IRQn               = 117,
    SYSPWR_IRQn                 = 118,
    RTC_IRQn                    = 119,
    RTC_ALARM_IRQn              = 120,
    VCI_OVRD_IN_IRQn            = 121,
    VCI_IN0_IRQn                = 122,
    VCI_IN1_IRQn                = 123,
    VCI_IN2_IRQn                = 124,
    VCI_IN3_IRQn                = 125,
    VCI_IN4_IRQn                = 126,
    VCI_IN5_IRQn                = 127,
    VCI_IN6_IRQn                = 128,
    PS20A_WAKE_IRQn             = 129,
    PS20B_WAKE_IRQn             = 130,
    PS21A_WAKE_IRQn             = 131,
    PS21B_WAKE_IRQn             = 132,
    PS21_WAKE_IRQn              = 133,
    ENVMON_IRQn                 = 134,
    KEYSCAN_IRQn                = 135,
    /* GIRQ22 wake only no EC connection */
    /* GIRQ23 */
    BTMR16_0_IRQn               = 136,
    BTMR16_1_IRQn               = 137,
    BTMR16_2_IRQn               = 138,
    BTMR16_3_IRQn               = 139,
    BTMR32_0_IRQn               = 140,
    BTMR32_1_IRQn               = 141,
    EVTMR0_IRQn                 = 142,
    EVTMR1_IRQn                 = 143,
    EVTMR2_IRQn                 = 144,
    EVTMR3_IRQn                 = 145,
    CAPTMR_IRQn                 = 146,
    CAP0_IRQn                   = 147,
    CAP1_IRQn                   = 148,
    CAP2_IRQn                   = 149,
    CAP3_IRQn                   = 150,
    CAP4_IRQn                   = 151,
    CAP5_IRQn                   = 152,
    CMP0_IRQn                   = 153,
    CMP1_IRQn                   = 154,
    MAX_IRQn
} IRQn_Type;

/*
 * Each external interrupt is mapped to an individual bit in enable set,
 * enable clear, pending and active registers in the NVIC. Each register
 * is 32-bits.
 * 156 external inputs requires 5 32-bit registers.
 * Each external interrupt is also mapped to a byte priority register.
 * 156 external inputs requires 156 byte registers.
 */
#define MAX_NVIC_CTRL_REGS    (5u)
#define MAX_NVIC_PRI_REGS     (156u)

/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M# Processor and Core Peripherals */
/* set the defines according your Device                                                    */
/* define the correct core revision
         __CM0_REV if your device is a CORTEX-M0 device
         __CM3_REV if your device is a CORTEX-M3 device
         __CM4_REV if your device is a CORTEX-M4 device                                           */
#define __CM4_REV                 0x0201    /*!< Core Revision r2p1                               */
#define __NVIC_PRIO_BITS          3         /*!< Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick Config is used     */
#define __MPU_PRESENT             1         /*!< MPU present or not                               */
#define __FPU_PRESENT             1        /*!< FPU present or not                                */
/*@}*/ /* end of group MEC1322_CMSIS */


/* include the correct core_cm#.h file
         core_cm0.h if your device is a CORTEX-M0 device
         core_cm3.h if your device is a CORTEX-M3 device
         core_cm4.h if your device is a CORTEX-M4 device   */
#include <core_cm4.h>
/* MEC2016 System  include file */
#include <system_MEC2016.h>


/* Register Union */
typedef union
{
    uint32_t w;
    uint16_t h[2];
    uint8_t  b[4];
} REG32_U;



/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/
/** @addtogroup MEC2016_Peripherals MEC2016 Peripherals
  MEC2016 Device Specific Peripheral registers structures
  @{
*/

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

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

/*------------- Watch Dog Timer (WDT) -----------------------------*/
/** @addtogroup MEC2016_WDT MEC2016 Watch Dog Timer (WDT)
  @{
*/
typedef struct
{
  __IO uint16_t LOAD;                       /*!< Offset: 0x0000   WDT Load Register           */
       uint16_t RSVDA;
  __IO uint8_t  CONTROL;                    /*!< Offset: 0x0004   WDT Control Register        */
       uint8_t  RSVDB[3];
  __IO uint8_t  KICK;                       /*!< Offset: 0x0008   WDT Kick Register (WO)      */
       uint8_t  RSVDC[3];
  __IO uint16_t COUNT;                      /*!< Offset: 0x000C   WDT Count Register (RO)     */
       uint16_t RSVDD;
} MEC2016_WDT_TypeDef;
/*@}*/ /* end of group MEC2016_WDT */

/*------------- 16-bit Basic Timer (BMTR16) -----------------------------*/
/** @addtogroup MEC2016_BTMR16 MEC2016 16-bit Basic Timer (BTMR16)
  @{
*/
typedef struct
{
  __IO uint16_t COUNT;                      /*!< Offset: 0x0000   BTMR16 Counter             */
       uint16_t RSVDA;
  __IO uint16_t PRELOAD;                    /*!< Offset: 0x0004   BTMR16 Preload value        */
       uint16_t RSVDB;
  __IO uint8_t  STATUS;                     /*!< Offset: 0x0008   BTMR16 Interrupt Status     */
       uint8_t  RSVDC[3];
  __IO uint8_t  IEN;                        /*!< Offset: 0x000C   BTMR16 Interrupt Enable     */
       uint8_t  RSVDD[3];
  __IO REG32_U  CONTROL;                    /*!< Offset: 0x0010   BTMR16 Control */
} MEC2016_BTMR16_TypeDef;
/*@}*/ /* end of group MEC2016_BTMR16 */

/*------------- 32-bit Basic Timer (BMTR32) -----------------------------*/
/** @addtogroup MEC2016_BTMR16 MEC2016 32-bit Basic Timer (BTMR32)
  @{
*/
typedef struct
{
  __IO uint32_t COUNT;                      /*!< Offset: 0x0000   BTMR32 Counter             */
  __IO uint32_t PRELOAD;                    /*!< Offset: 0x0004   BTMR32 Preload value        */
  __IO uint8_t  STATUS;                     /*!< Offset: 0x0008   BTMR32 Interrupt Status     */
       uint8_t  RSVDC[3];
  __IO uint8_t  IEN;                        /*!< Offset: 0x000C   BTMR32 Interrupt Enable     */
       uint8_t  RSVDD[3];
  __IO REG32_U  CONTROL;                    /*!< Offset: 0x0010   BTMR32 Control */
} MEC2016_BTMR32_TypeDef;
/*@}*/ /* end of group MEC2016_BTMR16 */

/*------------- Event Timer/Counter (EVT) -----------------------------*/
/** @addtogroup MEC2016_EVT MEC2016 Event timer/counter (EVT)
  @{
*/
typedef struct
{
  __IO uint16_t CONTROL;                    /*!< Offset: 0x0000   Event Timer/Counter Control */
       uint16_t RSVDA;
  __IO uint16_t CLK_EV_CTRL;                /*!< Offset: 0x0004   Event Timer/Counter Clock/Event Control  */
       uint16_t RSVDB;
  __IO uint16_t RELOAD;                     /*!< Offset: 0x0008   Event Timer/Counter Reload  */
       uint16_t RSVDC;
  __IO uint16_t COUNT;                      /*!< Offset: 0x000C   Event Timer/Counter Count   */
       uint16_t RSVDD;
} MEC2016_EVT_TypeDef;
/*@}*/ /* end of group MEC2016_EVT */

/*------------- Capture/Compare Timer (CCT) -----------------------------*/
/** @addtogroup MEC2016_CCT MEC2016 Capture/Compare Timer (CCT)
  @{
*/
typedef struct
{
  __IO uint16_t CONTROL;                    /*!< Offset: 0x0000   Capture/Compare Timer Control  */
       uint16_t RSVDA;
  __IO uint32_t CAP_CTRL0;                  /*!< Offset: 0x0004   Capture/Compare Timer Capture Control 0   */
  __IO uint16_t CAP_CTRL1;                  /*!< Offset: 0x0008   Capture/Compare Timer Capture Control 1   */
       uint16_t RSVDB;
  __IO uint32_t FREE_RUN;                   /*!< Offset: 0x000C   Capture/Compare Timer Free Run counter  */
  __IO uint32_t CAPTURE0;                   /*!< Offset: 0x0010   Capture/Compare Timer Capture 0 register */
  __IO uint32_t CAPTURE1;                   /*!< Offset: 0x0014   Capture/Compare Timer Capture 1 register */
  __IO uint32_t CAPTURE2;                   /*!< Offset: 0x0018   Capture/Compare Timer Capture 2 register */
  __IO uint32_t CAPTURE3;                   /*!< Offset: 0x001C   Capture/Compare Timer Capture 3 register */
  __IO uint32_t CAPTURE4;                   /*!< Offset: 0x0020   Capture/Compare Timer Capture 4 register */
  __IO uint32_t CAPTURE5;                   /*!< Offset: 0x0024   Capture/Compare Timer Capture 5 register */
  __IO uint32_t COMPARE0;                   /*!< Offset: 0x0028   Capture/Compare Timer Compare 0 register */
  __IO uint32_t COMPARE1;                   /*!< Offset: 0x002C   Capture/Compare Timer Compare 1 register */
} MEC2016_CCT_TypeDef;
/*@}*/ /* end of group MEC2016_CCT */

/*------------- RC ID (RCID) -----------------------------*/
/** @addtogroup MEC2016_RCID MEC2016 RC ID (RCID)
  @{
*/
typedef struct
{
  __IO uint16_t COUNT;                      /*!< Offset: 0x0000   RCID Control             */
       uint16_t RSVDA;
  __IO uint16_t PRELOAD;                    /*!< Offset: 0x0004   RCID Data         */
       uint16_t RSVDB;
} MEC2016_RCID_TypeDef;
/*@}*/ /* end of group MEC2016_RCID */

/*------------- DMA Main (DMAM) -----------------------------*/
/** @addtogroup MEC2016_DMAM MEC2016 DMA Main Block Enable (DMAM)
  @{
*/
typedef struct
{
  __IO uint8_t  CONTROL;                     /*!< Offset: 0x0000   DMA Main Control             */
       uint8_t  RSVDA[3];
  __IO uint32_t DATA_PKT;                   /*!< Offset: 0x0004   DMA Main Data Packet (RO)    */
  __IO uint8_t  ARB_FSM;                    /*!< Offset: 0x0008   DMA Main Arbiter FSM state (RO)  */
       uint8_t  RSVDB[3];
} MEC2016_DMAM_TypeDef;
/*@}*/ /* end of group MEC2016_DMAM */

/*------------- DMA Channel (DMACH) -----------------------------*/
/** @addtogroup MEC2016_DMACH MEC2016 DMA Channel (DMACH)
  @{
*/
#define MEC2016_MAX_DMA_CHAN    (14u)
typedef struct
{
  __IO uint8_t  ACTIVATE;                   /*!< Offset: 0x0000   DMA Channel Activate             */
       uint8_t  RSVDA[3];
  __IO uint32_t MEM_START_ADDR;             /*!< Offset: 0x0004   DMA Channel Memory Start Address */
  __IO uint32_t MEM_END_ADDR;               /*!< Offset: 0x0008   DMA Channel Memory End Address     */
  __IO uint32_t DEV_ADDR;                   /*!< Offset: 0x000C   DMA Channel Flow Control Device Address     */
  __IO uint32_t CONTROL;                    /*!< Offset: 0x0010   DMA Channel Control */
  __IO uint8_t  ISTATUS;                    /*!< Offset: 0x0014   DMA Channel Interrupt Status */
       uint8_t  RSVDB[3];
  __IO uint8_t  IEN;                        /*!< Offset: 0x0018   DMA Channel Interrupt Enable */
       uint8_t  RSVDC[3];
  __I  uint16_t FSM;                        /*!< Offset: 0x001C   DMA Channel FSM (RO)  */
       uint16_t RSVDD;
} MEC2016_DMACH_TypeDef;
/*@}*/ /* end of group MEC2016_DMACH */

/*------------- DMA Channel with ALU (DMACH_ALU) -----------------------------*/
/** @addtogroup MEC2016_DMACH_ALU MEC2016 DMA Channel with ALU (DMACH_ALU)
  @{
*/
typedef struct
{
  __IO uint8_t  ACTIVATE;                   /*!< Offset: 0x0000   DMA Channel with ALU Activate             */
       uint8_t  RSVDA[3];
  __IO uint32_t MEM_START_ADDR;             /*!< Offset: 0x0004   DMA Channel with ALU Memory Start Address */
  __IO uint32_t MEM_END_ADDR;               /*!< Offset: 0x0008   DMA Channel with ALU Memory End Address     */
  __IO uint32_t DEV_ADDR;                   /*!< Offset: 0x000C   DMA Channel with ALU Flow Control Device Address     */
  __IO uint32_t CONTROL;                    /*!< Offset: 0x0010   DMA Channel with ALU Control */
  __IO uint8_t  ISTATUS;                    /*!< Offset: 0x0014   DMA Channel with ALU Interrupt Status */
       uint8_t  RSVDB[3];
  __IO uint8_t  IEN;                        /*!< Offset: 0x0018   DMA Channel with ALU Interrupt Enable */
       uint8_t  RSVDC[3];
  __I  uint16_t FSM;                        /*!< Offset: 0x001C   DMA Channel with ALU FSM (RO)  */
       uint16_t RSVDD;
  __IO uint8_t  ALU_EN;                     /*!< Offset: 0x0020   DMA Channel with ALU Enable */
       uint8_t  RSVDE[3];
  __IO uint32_t ALU_DATA;                   /*!< Offset: 0x0024   DMA Channel with ALU Data */
  __IO uint8_t  ALU_STATUS;                 /*!< Offset: 0x0028   DMA Channel with ALU Status */
       uint8_t  RSVDF[3];
  __I  uint32_t ALU_TEST;                   /*!< Offset: 0x002C   DMA Channel with ALU Test (RO) */
} MEC2016_DMACH_ALU_TypeDef;
/*@}*/ /* end of group MEC2016_DMACH_ALU */


/*------------- EEPROM interface (EEPROM) -------------*/
/** @addtogroup MEC2016_EEPROM  MEC2016 EEPROM Interface (EEPROM)
  @{
*/

#define MEC2016_EEPROM_BUFFLEN  (32u)

typedef struct {
    __IO REG32_U  MODE;           /*!< Offset: 0x0000  EEPROM Reset/Activate */
    __IO REG32_U  EXE;            /*!< Offset: 0x0004  EEPROM Execute */
    __IO REG32_U  STATUS;         /*!< Offset: 0x0008  EEPROM Status */
    __IO REG32_U  IEN;            /*!< Offset: 0x000C  EEPROM Interrupt Enable */
    __IO REG32_U  PASSWD;         /*!< Offset: 0x0010  EEPROM Password register */
    __IO REG32_U  UNLOCK;         /*!< Offset: 0x0014  EEPROM Unlock register */
    __IO REG32_U  LOCK;           /*!< Offset: 0x0018  EEPROM Lock register */
    __IO REG32_U  TIMING;         /*!< Offset: 0x001C  EEPROM Timing register */
    __IO union {
         uint32_t w[MEC2016_EEPROM_BUFFLEN / 4];
         uint16_t h[MEC2016_EEPROM_BUFFLEN / 2];
         uint8_t  b[MEC2016_EEPROM_BUFFLEN];
    } BUFFER;                     /*!< Offset: 0x0020 - 0x3F  EEPROM Buffer registers */
} MEC2016_EEPROM_TypeDef;
/*@}*/ /* end of group MEC2016_EEPROM */


/*------------- Quad Master Serial Peripheral Interface (QMSPI) -------------*/
/** @addtogroup MEC2016_QMSPI  MEC2016 Quad Master Serial Peripheral Interface (QMSPI)
  @{
*/
#define QMSPI_MAX_DESCR (5ul)

typedef struct qmspir MEC2016_QMSPI_TypeDef;
struct qmspir {
    __IO REG32_U  MODE;           /*!< Offset: 0x0000  QMSPI Mode/ClockDiv/Rst/Activate */
    __IO REG32_U  CTRL;           /*!< Offset: 0x0004  QMSPI Control */
    __IO REG32_U  EXE;            /*!< Offset: 0x0008  QMSPI Execute */
    __IO REG32_U  IF_CTRL;        /*!< Offset: 0x000C  QMSPI Interface Control */
    __IO REG32_U  STATUS;         /*!< Offset: 0x0010  QMSPI Status */
    __I  REG32_U  BUF_CNT_STS;    /*!< Offset: 0x0014  QMSPI FIFO Buffer Count Status */
    __IO REG32_U  IEN;            /*!< Offset: 0x0018  QMSPI Interrupt Enable */
    __IO REG32_U  BUF_CNT_TRIG;   /*!< Offset: 0x001C  QMSPI FIFO Buffer Count Trigger Levels */
    __O  REG32_U  TX_FIFO;        /*!< Offset: 0x0020  QMSPI Transmit FIFO register */
    __I  REG32_U  RX_FIFO;        /*!< Offset: 0x0024  QMSPI Receive FIFO register */
         uint32_t RSVDC[2];       // +0x28 - 0x2F
    __IO REG32_U  DESCR[QMSPI_MAX_DESCR]; /*!< Offset: 0x0030 - 0x0043  QMSPI Descriptor registers */
};
/*@}*/ /* end of group MEC2016_QMSPI */

/*------------- Pulse Width Modulator (PWM) -----------------------------*/
/** @addtogroup MEC2016_RCID MEC2016 Pulse Width Modulator (PWM)
  @{
*/
typedef struct
{
  __IO uint16_t COUNT_ON;                   /*!< Offset: 0x0000   PWM On Count      */
       uint16_t RSVDA;
  __IO uint16_t COUNT_OFF;                  /*!< Offset: 0x0004   PWM Off Count     */
       uint16_t RSVDB;
  __IO uint8_t  CONFIG;                     /*!< Offset: 0x0008   PWM Configuration */
       uint8_t  RSVDC[3];
} MEC2016_PWM_TypeDef;
/*@}*/ /* end of group MEC2016_PWM */

/*------------- Tachometer (TACH) -----------------------------*/
/** @addtogroup MEC2016_TACH MEC2016 Tachometer (TACH)
  @{
*/
typedef struct
{
  __IO uint16_t CONTROL;                    /*!< Offset: 0x0000   Tach Control      */
  __IO uint16_t COUNT;                      /*!< Offset: 0x0002   Tach current counter value (RO) */
  __IO uint8_t  STATUS;                     /*!< Offset: 0x0004   Tach Status */
       uint8_t  RSVDA[3];
  __IO uint16_t limit_hi;                   /*!< Offset: 0x0008   TACH Count High Limit */
       uint16_t RSVDB;
  __IO uint16_t limit_lo;                   /*!< Offset: 0x000C   TACH Count Low Limit */
       uint16_t RSVDC;
} MEC2016_TACH_TypeDef;
/*@}*/ /* end of group MEC2016_TACH */

/*------------- Platform Environment Control Interface (PECI) -----------------------------*/
/** @addtogroup MEC2016_PECI MEC2016 Platform Environment Control Interface (PECI)
  @{
*/
typedef struct
{
  __IO uint8_t  WDATA;                    /*!< Offset: 0x0000   PECI Read data      */
       uint8_t  RSVDA[3];
  __IO uint8_t  RDATA;                    /*!< Offset: 0x0004   PECI Write data     */
       uint8_t  RSVDB[3];
  __IO uint8_t  CTRL;                     /*!< Offset: 0x0008   PECI Control */
       uint8_t  RSVDC[1];
  __IO uint16_t REQTMR;                   /*!< Offset: 0x000A   PECI Request Timer LSB */
  __IO uint8_t  STATUS1;                  /*!< Offset: 0x000C   PECI Status 1 */
       uint8_t  RSVDD[3];
  __IO uint8_t  STATUS2;                  /*!< Offset: 0x0010   PECI Status 2 */
       uint8_t  RSVDE[3];
  __IO uint8_t  ERROR;                    /*!< Offset: 0x0014   PECI Error */
       uint8_t  RSVDF[3];
  __IO uint8_t  IEN1;                     /*!< Offset: 0x0018   PECI Interrupt Enable 1 */
       uint8_t  RSVDG[3];
  __IO uint8_t  IEN2;                     /*!< Offset: 0x001C   PECI Interrupt Enable 2 */
       uint8_t  RSVDH[3];
  __IO uint8_t  OPTBTLO;                  /*!< Offset: 0x0020   PECI Optimal Bit Time LSB */
       uint8_t  RSVDI[3];
  __IO uint8_t  OPTBTHI;                  /*!< Offset: 0x0024   PECI Optimal Bit Time MSB */
       uint8_t  RSVDJ[3];
} MEC2016_PECI_TypeDef;
/*@}*/ /* end of group MEC2016_PECI */

/*------------- RTOS Timer (RTMR) -----------------------------*/
/** @addtogroup MEC2016_RTMR MEC2016 RTOS Timer (RTMR)
  @{
*/
typedef struct
{
  __IO uint32_t COUNT;                      /*!< Offset: 0x0000   RTOS Timer Count      */
  __IO uint32_t PRELOAD;                    /*!< Offset: 0x0004   RTOS Timer Pre-load  */
  __IO uint8_t  CONTROL;                    /*!< Offset: 0x0008   RTOS Timer Control */
       uint8_t  RSVDA[3];
} MEC2016_RTMR_TypeDef;
/*@}*/ /* end of group MEC2016_RTMR */

/*------------- Analog to Digital Converter (ADC) -------------*/
/** @addtogroup MEC2016_ADC MEC2016 Analog to Digital Converter (ADC)
  @{
*/
#define ADC_MAX_CHAN    (16u)

typedef struct
{
    __IO REG32_U  CONTROL;              /*!< Offset: 0x0000  ADC Control Count  */
    __IO REG32_U  DELAY;                /*!< Offset: 0x0004  ADC Delay  */
    __IO REG32_U  STATUS;               /*!< Offset: 0x0008  ADC Status */
    __IO REG32_U  SINGLE;               /*!< Offset: 0x000C  ADC Single conversion channel bit map */
    __IO REG32_U  REPEAT;               /*!< Offset: 0x0010  ADC Repeat conversion channel bit map */
    __IO REG32_U  READING[ADC_MAX_CHAN]; /*!< Offset: 0x0014 - 0x0053 ADC Channel Reading (10-bit) */
         uint32_t RSVDA[(0x78 - 0x54)/4];
    __IO uint32_t TEST1;                /*!< Offset: 0x0078 ADC TEST1 */
    __IO REG32_U  CONFIG;               /*!< Offset: 0x007C ADC Configuration */
} MEC2016_ADC_TypeDef;
/*@}*/ /* end of group MEC2016_ADC */

/*-------------  Trace FIFO Debug Port (TFDP) -----------------------------*/
/** @addtogroup MEC2016_TFDP MEC2016 Trace FIFO Debug Port (TFDP)
  @{
*/
typedef struct
{
  __IO uint8_t DATA;                        /*!< Offset: 0x0000   TFDP Data      */
       uint8_t RSVDA[3];
  __IO uint8_t CTRL;                        /*!< Offset: 0x0004   TFDP Control  */
       uint8_t RSVDB[3];
} MEC2016_TFDP_TypeDef;
/*@}*/ /* end of group MEC2016_TFDP */

/*-------------  General Purpose SPI (GPSPI) -----------------------------*/
/** @addtogroup MEC2016_GPSPI MEC2016 General Purpose SPI (GPSPI)
  @{
*/
typedef struct
{
  __IO uint8_t ENABLE;                     /*!< Offset: 0x0000   GP-SPI Enable      */
       uint8_t RSVDA[3];
  __IO uint8_t CONTROL;                    /*!< Offset: 0x0004   GP-SPI Control  */
       uint8_t RSVDB[3];
  __IO uint8_t STATUS;                     /*!< Offset: 0x0004   GP-SPI Status (Read-to-Clear) */
       uint8_t RSVDC[3];
  __O  uint8_t TX_DATA;                    /*!< Offset: 0x0004   GP-SPI TX Data (WO)  */
       uint8_t RSVDD[3];
  __I  uint8_t RX_DATA;                    /*!< Offset: 0x0004   GP-SPI RX Data (RO)  */
       uint8_t RSVDE[3];
  __IO uint8_t CLOCK_CONTROL;              /*!< Offset: 0x0004   GP-SPI Clock Control  */
       uint8_t RSVDF[3];
  __IO uint8_t CLOCK_GEN;                  /*!< Offset: 0x0004   GP-SPI Clock Generator  */
       uint8_t RSVDG[3];
} MEC2016_GPSPI_TypeDef;
/*@}*/ /* end of group MEC2016_TFDP */

/*-------------  VBAT Register Bank (VBREGS) -----------------------------*/
/** @addtogroup MEC2016_VBREGS VBAT Register Bank (VBREGS)
  @{
*/
typedef struct
{
  __IO REG32_U  PFRS;               /*!< Offset: 0x0000  VBAT Reg Bank Power Fail Reset Status   */
  __IO REG32_U  TESTA;              /*!< Offset: 0x0004  VBAT Reg Bank TESTA */
  __IO REG32_U  CLOCK_ENABLE;       /*!< Offset: 0x0008  VBAT Reg Bank Clock Enable  */
       uint32_t RSVDA;
  __IO REG32_U  TESTB;              /*!< Offset: 0x0010  VBAT Reg Bank TESTB */
  __IO REG32_U  TESTC;              /*!< Offset: 0x0014  VBAT Reg Bank TESTC */
  __IO REG32_U  TESTD;              /*!< Offset: 0x0018  VBAT Reg Bank TESTD */
  __IO REG32_U  TESTE;              /*!< Offset: 0x001C  VBAT Reg Bank TESTE */
  __IO REG32_U  MONOTONIC_COUNT;    /*!< Offset: 0x0020  VBAT Reg Bank Monotonic 32-bit Counter */
  __IO REG32_U  MONOTONIC_COUNT_HI; /*!< Offset: 0x0024  VBAT Reg Bank Monotonic Counter Hi Word */
  __IO REG32_U  VWIRE_BACKUP;       /*!< Offset: 0x0028  VBAT Reg Bank VWire Backup */
} MEC2016_VBREGS_TypeDef;
/*@}*/ /* end of group MEC2016_VBREGS */

/*------------- Breathing/Blinking LED (BBLED) -----------------------------*/
/** @addtogroup BBLED Breathing-Blinking LED (BBLED)
  @{
*/

typedef struct
{
  __IO uint32_t CONFIG;
  __IO uint32_t LIMIT;
  __IO uint32_t DELAY;
  __IO uint32_t STEP;
  __IO uint32_t INTERVAL;
} MEC2016_BBLED_TypeDef;
/*@}*/ /* end of group MEC2016_BBLED */

/*------------- Public Key Encryption Subsystem (PKE) -----------------------------*/
/** @addtogroup MEC2016_PKE Public Key Encryption (PKE)
  @{
*/
typedef struct
{
  __IO    uint32_t CONFIG;                /*!< Offset: 0x0000  Configuration */
  __IO    uint32_t COMMAND;               /*!< Offset: 0x0004  Command */
  __IO    uint32_t CONTROL;               /*!< Offset: 0x0008  Control */
  __I     uint32_t STATUS;                /*!< Offset: 0x000C  Status */
  __I     uint32_t VERSION;               /*!< Offset: 0x0010  Version */
  __IO    uint32_t LOAD_MICRO_CODE;       /*!< Offset: 0x0014  Load Micro Code */
} MEC2016_PKE_TypeDef;
/*@}*/ /* end of group MEC2016_PKE */

/*------------- Random Number Generator Subsystem (RNG) -----------------------------*/
/** @addtogroup MEC2016_NDRNG Random Number Generator (RNG)
  @{
*/
typedef struct
{
    __IO    uint32_t CONTROL;               /*!< Offset: 0x0000  Control */
    __I     uint32_t FIFO_LEVEL;            /*!< Offset: 0x0004  FIFO Level */
    __I     uint32_t VERSION;               /*!< Offset: 0x0008  Version */
} MEC2016_RNG_TypeDef;
/*@}*/ /* end of group MEC2016_RNG */

/*-------------  Hash Engine (HASH) -----------------------------*/
/** @addtogroup MEC2016_HASH Hash Accelerator (HASH)
  @{
*/
typedef struct
{
  __IO    uint32_t SHA_MODE;              /*!< Offset: 0x0000  SHA Mode */
  __IO    uint32_t NB_BLOCK;              /*!< Offset: 0x0004  NbBlock */
  __IO    uint32_t CONTROL;               /*!< Offset: 0x0008  Config */
  __I     uint32_t STATUS;                /*!< Offset: 0x000C  Status, Read to clear interrupt */
  __I     uint32_t VERSION;               /*!< Offset: 0x0010  Version */
  __I     uint32_t GENERIC_VALUE;         /*!< Offset: 0x0014  Generic Value */
  __IO    uint32_t INIT_HASH_ADDR;        /*!< Offset: 0x0018  Initial Hash value Address */
  __IO    uint32_t DATA_SOURCE_ADDR;      /*!< Offset: 0x001C  Data to hash Address */
  __IO    uint32_t HASH_RESULT_ADDR;      /*!< Offset: 0x0020  Hash result address */
} MEC2016_HASH_TypeDef;
/*@}*/ /* end of group MEC2016_HASH */

/*------------- Advanced Encryption Subsystem (AES) -----------------------------*/
/** @addtogroup MEC2016_AES Advanced Encryption Subsys (AES)
  @{
*/

#define AESHW_KEY_BITLEN    (256ul)

#define AESHW_KEY1_159_128_IDX  (0u)
#define AESHW_KEY1_191_160_IDX  (1u)
#define AESHW_KEY1_223_192_IDX  (2u)
#define AESHW_KEY1_255_224_IDX  (3u)
#define AESHW_KEY1_31_0_IDX     (4u)
#define AESHW_KEY1_63_32_IDX    (5u)
#define AESHW_KEY1_95_64_IDX    (6u)
#define AESHW_KEY1_127_96_IDX   (7u)
#define AESHW_KEY1_MAX_IDX      (8u)

#define AESHW_IV_BITLEN         (128ul)
#define AESHW_IV_31_0_IDX       (0u)
#define AESHW_IV_63_32_IDX      (1u)
#define AESHW_IV_95_64_IDX      (2u)
#define AESHW_IV_127_96_IDX     (3u)

typedef struct
{
    __IO    uint32_t CONFIG;                /*!< Offset: 0x0000  Configuration */
    __IO    uint32_t COMMAND;               /*!< Offset: 0x0004  Command */
    __IO    uint32_t CONTROL;               /*!< Offset: 0x0008  Control */
    __I     uint32_t STATUS;                /*!< Offset: 0x000C  Status */
    __I     uint32_t VERSION;               /*!< Offset: 0x0010  Version */
    __IO    uint32_t NB_HEADER;             /*!< Offset: 0x0014  Number of Headers */
    __IO    uint32_t LAST_HEADER;           /*!< Offset: 0x0018  Last Header */
    __IO    uint32_t NB_BLOCK;              /*!< Offset: 0x001C  Number of Blocks */
    __IO    uint32_t LAST_BLOCK;            /*!< Offset: 0x0020  Last Block */
    __IO    uint32_t DMA_IN;                /*!< Offset: 0x0024  DMA Input Address */
    __IO    uint32_t DMA_OUT;               /*!< Offset: 0x0028  DMA Output Address */
            uint32_t RESERVEDA[(0xFC - 0x2C)/4 + 1];
    __IO    uint32_t KEY1[(AESHW_KEY_BITLEN)>>5];/*!< Offset: 0x0100  KeyIn1[159:128]
                                              !< Offset: 0x0104  KeyIn1[191:160]
                                              !< Offset: 0x0108  KeyIn1[223:192]
                                              !< Offset: 0x010C  KeyIn1[255:224]
                                              !< Offset: 0x0110  KeyIn1[31:0]
                                              !< Offset: 0x0114  KeyIn1[63:32]
                                              !< Offset: 0x0118  KeyIn1[95:64]
                                              !< Offset: 0x011C  KeyIn1[127:96] */
    __IO    uint32_t IV[(AESHW_IV_BITLEN)>>5];   /*!< Offset: 0x0120  IV[31:0]
                                              !< Offset: 0x0124  IV[63:32]
                                              !< Offset: 0x0128  IV[95:64]
                                              !< Offset: 0x012C  IV[127:96] */
            uint32_t RESERVEDB[4];
    __IO    uint32_t KEY2[(AESHW_KEY_BITLEN)>>5];/*!< Offset: 0x0140  KeyIn1[159:128]
                                              !< Offset: 0x0144  KeyIn1[191:160]
                                              !< Offset: 0x0148  KeyIn1[223:192]
                                              !< Offset: 0x014C  KeyIn1[255:224]
                                              !< Offset: 0x0150  KeyIn1[31:0]
                                              !< Offset: 0x0154  KeyIn1[63:32]
                                              !< Offset: 0x0158  KeyIn1[95:64]
                                              !< Offset: 0x015C  KeyIn1[127:96] */
} MEC2016_AES_TypeDef;
/*@}*/ /* end of group MEC2016_AES */

/*-------------  GIRQ Aggregated Interrupt Block (GIRQ) -----------------------------*/
/** @addtogroup MEC2016_GIRQ Aggregated Interrupt (GIRQ)
  @{
*/
#define MEC2016_GIRQ_BLEN   (20ul)

typedef struct
{
  __IO uint32_t SOURCE;                   /*!< Offset: 0x0000   GIRQ Source Register (RW1C)   */
  __IO uint32_t ENABLE_SET;               /*!< Offset: 0x0004   GIRQ Enable Set Register (RW)  */
  __IO uint32_t ENABLE_CLR;               /*!< Offset: 0x0008   GIRQ Enable Clear Register (RW) */
  __IO uint32_t RESULT;                   /*!< Offset: 0x000C   GIRQ Result Register (RO) */
       uint32_t RSVDA[1];
} MEC2016_GIRQ_TypeDef;
/*@}*/ /* end of group MEC2016_GIRQ */

/*-------------  EC Interrupt Aggregator (ECIA) -----------------------------*/
/** @addtogroup MEC2016_ECIA EC Interrupt Aggregator (ECIA)
  @{
*/

#define MEC2016_GIRQ08_ID                   (0)
#define MEC2016_GIRQ09_ID                   (1)
#define MEC2016_GIRQ10_ID                   (2)
#define MEC2016_GIRQ11_ID                   (3)
#define MEC2016_GIRQ12_ID                   (4)
#define MEC2016_GIRQ13_ID                   (5)
#define MEC2016_GIRQ14_ID                   (6)
#define MEC2016_GIRQ15_ID                   (7)
#define MEC2016_GIRQ16_ID                   (8)
#define MEC2016_GIRQ17_ID                   (9)
#define MEC2016_GIRQ18_ID                   (10)
#define MEC2016_GIRQ19_ID                   (11)
#define MEC2016_GIRQ20_ID                   (12)
#define MEC2016_GIRQ21_ID                   (13)
#define MEC2016_GIRQ22_ID                   (14)
#define MEC2016_GIRQ23_ID                   (15)
#define MEC2016_GIRQ24_ID                   (16)
#define MEC2016_GIRQ25_ID                   (17)
#define MEC2016_GIRQ26_ID                   (18)
#define MEC2016_GIRQ_ID_MAX                 (19)

typedef struct
{
  MEC2016_GIRQ_TypeDef GIRQS[MEC2016_GIRQ_ID_MAX];  /*!< Offset: 0x0000 - 0x011F Array of Aggregator GIRQ blocks */
  uint32_t RSVDA[((0x200 - ((MEC2016_GIRQ_ID_MAX) * (MEC2016_GIRQ_BLEN))) / 4)]; /* offsets 0x0120 - 0x1FF */
  __IO uint32_t BLOCK_EN_SET;               /*!< Offset: 0x0200   GIRQ Source Register (RW1C)   */
  __IO uint32_t BLOCK_EN_CLR;               /*!< Offset: 0x0204   GIRQ Enable Set Register (RW)  */
  __IO uint32_t BLOCK_IRQ_VECT;             /*!< Offset: 0x0208   GIRQ Enable Clear Register (RW) */
} MEC2016_ECIA_TypeDef;
/*@}*/ /* end of group MEC2016_ECIA */

/*-------------  VBAT Powered Registers (VBR) ------------------*/
/** @addtogroup MEC2016_VBR VBAT Powered Registers (VBR)
  @{
*/
typedef struct
{
    __IO uint8_t  PWR_FAIL_RST_STS;         /*!< Offset: 0x0000  Power-Fail and Rest Status */
         uint8_t  RSVDA[3];
    __IO uint32_t TEST_ATE_REG_CTRL;        /*!< Offset: 0x0004 */
    __IO uint8_t  CLOCK_EN;                 /*!< Offset: 0x0008  32K Clock Enable */
         uint8_t  RSVDB[3];
    __IO uint32_t TESTB;                    /*!< Offset: 0x000C  */
    __IO uint32_t TESTC;                    /*!< Offset: 0x0010  */
    __IO uint32_t TESTD;                    /*!< Offset: 0x0014  */
         uint32_t RSVDC;
    __IO uint32_t TRIM_CTRL_32K;            /*!< Offset: 0x001C  */
    __IO uint32_t MONOTONIC_CNTR;           /*!< Offset: 0x0020  */
    __IO uint32_t MONOTONIC_CNTR_HIWORD;    /*!< Offset: 0x0024  */
    __IO uint32_t M2S_42H_BACKUP;           /*!< Offset: 0x0028  */
} MEC2016_VBR_TypeDef;
/*@}*/ /* end of group MEC2016_VBR */

/*-------------  EC Subystem (ECS) -----------------------------*/
/** @addtogroup MEC2016_ECS EC Subsystem (ECS)
  @{
*/
typedef struct
{
  __IO REG32_U  MSIZE;                  /*!< Offset: 0x0000   MSIZE (RO)   */
  __IO REG32_U  AHB_ERR_ADDR;           /*!< Offset: 0x0004   AHB Error Address  */
  __IO REG32_U  TESTA;                  /*!< Offset: 0x0008   */
  __IO REG32_U  TESTB;                  /*!< Offset: 0x000C   */
  __IO REG32_U  ID;                     /*!< Offset: 0x0010   */
  __IO REG32_U  AHB_ERR_CTRL;           /*!< Offset: 0x0014   AHB Error Control */
  __IO REG32_U  INTR_CTRL;              /*!< Offset: 0x0018   Interrupt Routing Control */
  __IO REG32_U  ETM_ENABLE;             /*!< Offset: 0x001C   ETM Enable */
  __IO REG32_U  JTAG_ENABLE;            /*!< Offset: 0x0020   JTAG Enable */
  __IO REG32_U  OTP_LOCK;               /*!< Offset: 0x0024   */
  __IO REG32_U  WDT_COUNT;              /*!< Offset: 0x0028   WDT Count */
  __IO REG32_U  AES_HASH_SW;            /*!< Offset: 0x002C   AES-Hash data swap control */
  __IO REG32_U  BOOTROM_SCRATCH1;       /*!< Offset: 0x0030   Boot-ROM Scratch1 */
  __IO REG32_U  VOLT_REG_TRIM;          /*!< Offset: 0x0034   Voltage Regulator Trim */
  __IO REG32_U  SYS_SHUTDOWN_RESET;     /*!< Offset: 0x0038   System Shutdown Reset */
  __IO REG32_U  ADC_BIAS_ADJ;           /*!< Offset: 0x003C   ADC Bias current adjustement */
  __IO REG32_U  BOOTROM_SCRATCH2;       /*!< Offset: 0x0040   Boot-ROM Scratch2 */
  __IO REG32_U  GPIO_PAD_TEST;          /*!< Offset: 0x0044   GPIO Pad Test */
  __IO REG32_U  BOOTROM_SCRATCH3;       /*!< Offset: 0x0048   Boot-ROM Scratch3 */
       uint32_t RSVDA[4];
  __IO REG32_U  CRYPTO_SOFT_RST;        /*!< Offset: 0x005C   Crypto Blocks Soft Reset */
  __IO REG32_U  PWR_GRD_TEST1;          /*!< Offset: 0x0060   Power Guard Test 1 */
       uint32_t RSVDB[((0x100ul - 0x60ul)>>2)-1];
  __IO REG32_U  BSCAN_ID;               /*!< Offset: 0x0100   Boundary Scan ID */
} MEC2016_ECS_TypeDef;
/*@}*/ /* end of group MEC2016_ECS */

/*-----------------------------------------------------------------------------------------------
 * EC SPB Segment 0x4008_0000 - 0x4008_XXXX
 *------------------------------------------------------------------------------------------------*/

/*-------------  Power-Clock-Reset block (PCR) -----------------------------*/
/** @addtogroup MEC2016_PCR Power-Clock-Reset block (PCR)
  @{
*/
typedef struct
{
    __IO REG32_U  SYS_SLP_CTRL;         /*!< Offset: 0x0000  System Sleep Control */
    __IO REG32_U  PROC_CLK_CTRL;        /*!< Offset: 0x0004  EC Clock Divider */
    __IO REG32_U  SLOW_CLK_CTRL;        /*!< Offset: 0x0008  Slow Clock Control */
    __IO REG32_U  OSC_ID;               /*!< Offset: 0x000C  Oscillator ID (RO) */
    __IO REG32_U  PWR_RST_STS;          /*!< Offset: 0x0010  Power Reset Status */
    __IO REG32_U  PWR_RST_CTRL;         /*!< Offset: 0x0014  Power Reset Control */
    __IO REG32_U  SYS_RESET;            /*!< Offset: 0x0018  System Reset */
    __IO REG32_U  PKE_CLK_CTRL;         /*!< Offset: 0x001C  PKE Clock control */
    __IO REG32_U  TEST_OSC_CAL;         /*!< Offset: 0x0020  TEST OSC Calibration */
         uint32_t RSVDA[3];
    __IO REG32_U  SLEEP_EN0;            /*!< Offset: 0x0030 - 0x0033 Device Sleep Enable 0  */
    __IO REG32_U  SLEEP_EN1;            /*!< Offset: 0x0034 - 0x0037 Device Sleep Enable 1  */
    __IO REG32_U  SLEEP_EN2;            /*!< Offset: 0x0038 - 0x003B Device Sleep Enable 2  */
    __IO REG32_U  SLEEP_EN3;            /*!< Offset: 0x003C - 0x003F Device Sleep Enable 3  */
    __IO REG32_U  SLEEP_EN4;            /*!< Offset: 0x0040 - 0x0043 Device Sleep Enable 4  */
         uint32_t RSVDB[3];
    __IO REG32_U  CLOCK_REQ0;           /*!< Offset: 0x0050 - 0x0053 Device Clock Required 0  */
    __IO REG32_U  CLOCK_REQ1;           /*!< Offset: 0x0054 - 0x0057 Device Clock Required 1  */
    __IO REG32_U  CLOCK_REQ2;           /*!< Offset: 0x0058 - 0x005B Device Clock Required 2  */
    __IO REG32_U  CLOCK_REQ3;           /*!< Offset: 0x005C - 0x005F Device Clock Required 3  */
    __IO REG32_U  CLOCK_REQ4;           /*!< Offset: 0x0060 - 0x0063 Device Clock Required 4  */
         uint32_t RSVDC[3];
    __IO REG32_U  RESET_EN0;            /*!< Offset: 0x0070 - 0x0073 Device Reset on Sleep Entry Enable 0 */
    __IO REG32_U  RESET_EN1;            /*!< Offset: 0x0074 - 0x0077 Device Reset on Sleep Entry Enable 1 */
    __IO REG32_U  RESET_EN2;            /*!< Offset: 0x0078 - 0x007B Device Reset on Sleep Entry Enable 2 */
    __IO REG32_U  RESET_EN3;            /*!< Offset: 0x007C - 0x007F Device Reset on Sleep Entry Enable 3 */
    __IO REG32_U  RESET_EN4;            /*!< Offset: 0x0080 - 0x0083 Device Reset on Sleep Entry Enable 4 */
} MEC2016_PCR_TypeDef;
/*@}*/ /* end of group MEC2016_PCR */

/*-------------  GPIO Pin Control (GPIO_CTRL) -----------------------------*/
/** @addtogroup MEC2016_GPIO_CTRL Pin Control (GPIO_CTRL)
  @{
*/

#define MAX_GPIO_PIN  (0xADul)
#define MAX_GPIO_BANK (6u)

#define GPIO_LOCK5_IDX  (0u)
#define GPIO_LOCK4_IDX  (1u)
#define GPIO_LOCK3_IDX  (2u)
#define GPIO_LOCK2_IDX  (3u)
#define GPIO_LOCK1_IDX  (4u)
#define GPIO_LOCK0_IDX  (5u)


typedef struct
{
  __IO REG32_U  REG[MAX_GPIO_PIN];
} MEC2016_GPIO_CTRL_TypeDef;
/*@}*/ /* end of group MEC2016_GPIO_CTRL */

/*-------------  GPIO Pin Parallel I/O (GPIO_PIO) -----------------------------*/
/** @addtogroup MEC2016_GPIO_PIO GPIO Parallel I/O (GPIO_PIO)
  @{
*/

/* TODO - Parallel GPIO register will be moving...
 * Probably offset 0x400
 */
typedef struct
{
  __IO uint32_t OUT[MAX_GPIO_BANK];  /*!< Offset: 0x280 - 0x297  Parallel output 5 banks */
       uint32_t RSVDA[2];  /* 0x298 - 0x29F */
       uint32_t RSVDB[4];  /* 0x2A0 - 0x2AF */
       uint32_t RSVDC[4];  /* 0x2B0 - 0x2BF */
       uint32_t RSVDD[4];  /* 0x2C0 - 0x2CF */
       uint32_t RSVDE[4];  /* 0x2D0 - 0x2DF */
       uint32_t RSVDF[4];  /* 0x2E0 - 0x2EF */
       uint32_t RSVDG[4];  /* 0x2F0 - 0x2FF */
  __IO uint32_t IN[MAX_GPIO_BANK];   /*!< Offset: 0x300 - 0x317  Parallel input 5 banks */
} MEC2016_GPIO_PIO_TypeDef;
/*@}*/ /* end of group MEC2016_GPIO_PIO */

/*-------------  GPIO Pin Control2 (GPIO_CTRL2) -----------------------------*/
/** @addtogroup MEC2016_GPIO_CTRL Pin Control2 (GPIO_CTRL2)
  @{
*/

typedef struct
{
  __IO REG32_U  REG[MAX_GPIO_PIN];
} MEC2016_GPIO_CTRL2_TypeDef;
/*@}*/ /* end of group MEC2016_GPIO_CTRL */


/*-------------  eFuse write-once programmable bit array (EFUSE) -----------------------------*/
/** @addtogroup MEC2016_EFUSE eFuse write-once programmable bit array (EFUSE)
  @{
*/


#define EFUSE_BITLEN        (4096ul)
#define EFUSE_BYTELEN       (4096ul >> 3)
#define EFUSE_HWORDLEN      (4096ul >> 4)

typedef struct
{
  __IO uint32_t CONTROL;                  /*!< Offset: 0x0000   Control  */
  __IO uint32_t MAN_CONTROL;              /*!< Offset: 0x0004   Manual Control  */
  __IO uint32_t MAN_MODE;                 /*!< Offset: 0x0008   Manual Mode  */
  __IO uint32_t MAN_DATA;                 /*!< Offset: 0x000C   Manual Mode Data */
  union {
    __IO uint16_t MEM16[EFUSE_HWORDLEN];  /*!< Offset: 0x0010   Memory mapped bit-array 8 or 16-bit accessible  */
    __IO uint8_t  MEM8[EFUSE_BYTELEN];
  };
} MEC2016_EFUSE_TypeDef;
/*@}*/ /* end of group MEC2016_EFUSE */


/*-----------------------------------------------------------------------------------------------
 * EC Host SPB Segment 0x400F_0000 - 0x400F_XXXX
 *------------------------------------------------------------------------------------------------*/

/*------------- UART Runtime (UARTR) -----------------------------*/
/** @addtogroup MEC2016 UART Runtime (UARTR)
  @{
*/
typedef struct
{
    __IO uint8_t RRB_WTB_BRG0;                  // +0x1C00
    __IO uint8_t IER_BRG1;                      // +0x1C01
    __IO uint8_t RIIR_WFCR;
    __IO uint8_t LCR;
    __IO uint8_t MCR;
    __IO uint8_t LSR;
    __IO uint8_t MSR;
    __IO uint8_t SCR;                           // +0x1C07
} MEC2016_UARTR_TypeDef;
/*@}*/ /* end of group MEC2016_UARTR_TypeDef */



/*------------- eSPI IO Perpheral Channel Host Access (ESPI_PCH) -------------*/
/** @addtogroup MEC2016 eSPI IO Peripheral Channel Host Access (ESPI_PCH)
  @{
*/
typedef struct
{
    __IO uint8_t  PC_INDEX;         /*!< Offset: 0x0000  Peripheral Channel Index */
    __IO uint8_t  PC_DATA;          /*!< Offset: 0x0001  Peripheral Channel Data */
} MEC2016_ESPI_PCH_TypeDef;
/*@}*/ /* end of group MEC2016_ESPI_PCH_TypeDef */

/*------------- eSPI IO Perpheral Channel EC Access (ESPI_PCEC) -------------*/
/** @addtogroup MEC2016 eSPI IO Peripheral Channel EC Access (ESPI_PCEC)
  @{
*/
typedef struct
{
    __IO REG32_U  PC_LAST_CYCLE_W0; /*!< Offset: 0x0100  PC Last Cycle Error */
    __IO REG32_U  PC_LAST_CYCLE_W1;
    __IO REG32_U  PC_LAST_CYCLE_W2;
    __IO REG32_U  PC_ERR_ADDR_LO;   /*!< Offset: 0x010C  PC Error Address */
    __IO REG32_U  PC_ERR_ADDR_HI;
    __IO REG32_U  PC_MSTR_EN_CHG;   /*!< Offset: 0x0114  PC Mastering Enable Change */
    __IO REG32_U  PC_IEN;           /*!< Offset: 0x0118  PC Interrupt Enable */
} MEC2016_ESPI_PCEC_TypeDef;
/*@}*/ /* end of group MEC2016_ESPI_PCEC_TypeDef */


/*------------- eSPI IO BAR Control (ESPI_BAR_CTRL) -------------*/
/** @addtogroup MEC2016 eSPI IO BAR Control (ESPI_BAR_CTRL)
 * EC only, contains the address mask, logical device number, & virtualization enable
  @{
*/
#define MEC2016_ESPI_IO_BAR_IDX        (0u)
#define MEC2016_ESPI_MEM_BAR_IDX       (1u)
#define MEC2016_ESPI_MBOX_BAR_IDX      (2u)
#define MEC2016_ESPI_MIF8042_BAR_IDX   (3u)
#define MEC2016_ESPI_ACPIEC0_BAR_IDX   (4u)
#define MEC2016_ESPI_ACPIEC1_BAR_IDX   (5u)
#define MEC2016_ESPI_ACPIEC2_BAR_IDX   (6u)
#define MEC2016_ESPI_ACPIEC3_BAR_IDX   (7u)
#define MEC2016_ESPI_ACPIEC4_BAR_IDX   (8u)
#define MEC2016_ESPI_ACPIPM1_BAR_IDX   (9u)
#define MEC2016_ESPI_FASTKB_BAR_IDX    (10u)
#define MEC2016_ESPI_UART0_BAR_IDX     (11u)
#define MEC2016_ESPI_UART1_BAR_IDX     (12u)
#define MEC2016_ESPI_EMI0_BAR_IDX      (13u)
#define MEC2016_ESPI_EMI1_BAR_IDX      (14u)
#define MEC2016_ESPI_EMI2_BAR_IDX      (15u)
#define MEC2016_ESPI_P80DBG0_BAR_IDX   (16u)
#define MEC2016_ESPI_P80DBG1_BAR_IDX   (17u)
#define MEC2016_ESPI_RTC_BAR_IDX       (18u)
#define MEC2016_ESPI_LASIC_BAR_IDX     (19u)
#define MEC2016_ESPI_TEST_BAR_IDX      (20u)
#define MEC2016_ESPI_MAX_BAR_IDX       (21u)

typedef struct
{
    __IO REG32_U  BAR_INHIBIT_LO;           /*!< Offset: 0x0120  BAR Inhibit */
    __IO REG32_U  BAR_INHIBIT_HI;
    __IO REG32_U  BAR_INIT;                 /*!< Offset: 0x0128  BAR Init */
    __IO REG32_U  EC_IRQ;                   /*!< Offset: 0x012C  EC Interrupt bit */
    __IO REG32_U  TESTA;                    /*!< Offset: 0x0130  TESTA */
    __IO REG32_U  IOBAR_CTRL[MEC2016_ESPI_MAX_BAR_IDX];
} MEC2016_ESPI_BAR_CTRL_TypeDef;
/*@}*/ /* end of group MEC2016_ESPI_BAR_CTRL_TypeDef */


/*------------- eSPI IO LTR (ESPI_LTR) -----------------------------*/
/** @addtogroup MEC2016 eSPI IO LTR (ESPI_LTR)
  @{
*/
typedef struct
{
    __IO REG32_U  LTR_PC_STATUS;            /*!< Offset: 0x0220  eSPI I/O LTR Peripheral Status */
    __IO REG32_U  LTR_PC_ENABLE;            /*!< Offset: 0x0224  eSPI I/O LTR Peripheral Enable */
    __IO REG32_U  LTR_PC_CONTROL;           /*!< Offset: 0x0228  eSPI I/O LTR Peripheral Control */
    __IO REG32_U  LTR_PC_MESSAGE;           /*!< Offset: 0x022C  eSPI I/O LTR Peripheral Message */
} MEC2016_ESPI_LTR_TypeDef;
/*@}*/ /* end of group MEC2016_ESPI_LTR_TypeDef */

/*------------- eSPI IO OOB Channel (ESPI_OOB) -----------------------------*/
/** @addtogroup MEC2016 eSPI IO OOB Channel (ESPI_OOB)
  @{
*/
typedef struct
{
    __IO REG32_U  RX_ADDR_LO;           /*!< Offset: 0x0240  OOB Receive Address b[31:0] */
    __IO REG32_U  RX_ADDR_HI;           /*!< Offset: 0x0244  OOB Receive Address b[63:32] */
    __IO REG32_U  TX_ADDR_LO;           /*!< Offset: 0x0248  OOB Transmit Address b[31:0] */
    __IO REG32_U  TX_ADDR_HI;           /*!< Offset: 0x024C  OOB Transmit Address b[63:32] */
    __IO REG32_U  RX_LEN;               /*!< Offset: 0x0250  OOB Receive transfer length */
    __IO REG32_U  TX_LEN;               /*!< Offset: 0x0254  OOB Transmit length */
    __IO REG32_U  RX_CTRL;              /*!< Offset: 0x0258  OOB Receive Control */
    __IO REG32_U  RX_IEN;               /*!< Offset: 0x025C  OOB Receive Interrupt Enable */
    __IO REG32_U  RX_STATUS;            /*!< Offset: 0x0260  OOB Receive Status */
    __IO REG32_U  TX_CTRL;              /*!< Offset: 0x0264  OOB Transmit Control */
    __IO REG32_U  TX_IEN;               /*!< Offset: 0x0268  OOB Transmit Interrupt Enable */
    __IO REG32_U  TX_STATUS;            /*!< Offset: 0x026C  OOB Transmit Status */
} MEC2016_ESPI_OOB_TypeDef;
/*@}*/ /* end of group MEC2016_ESPI_OOB_TypeDef */

/*------------- eSPI IO Flash Channel (ESPI_FC) -----------------------------*/
/** @addtogroup MEC2016 eSPI Flash Channel (ESPI_FC)
  @{
*/
typedef struct
{
    __IO REG32_U  FLASH_ADDR_LO;         /*!< Offset: 0x0280  FC Flash Address b[31:0] */
    __IO REG32_U  FLASH_ADDR_HI;         /*!< Offset: 0x0284  FC Flash Address b[63:32] */
    __IO REG32_U  BUF_ADDR_LO;           /*!< Offset: 0x0288  FC EC Buffer Address b[31:0] */
    __IO REG32_U  BUF_ADDR_HI;           /*!< Offset: 0x028C  FC EC Buffer Address b[63:32] */
    __IO REG32_U  XFER_LEN;              /*!< Offset: 0x0290  FC Transfer length */
    __IO REG32_U  CONTROL;               /*!< Offset: 0x0294  FC Control */
    __IO REG32_U  IEN;                   /*!< Offset: 0x0298  FC Receive Interrupt Enable */
    __IO REG32_U  CONFIG;                /*!< Offset: 0x029C  FC Configuration */
    __IO REG32_U  STATUS;                /*!< Offset: 0x02A0  FC Status */
} MEC2016_ESPI_FC_TypeDef;
/*@}*/ /* end of group MEC2016_ESPI_FC_TypeDef */

/*------------- eSPI IO Capabilities (ESPI_CAP) -----------------------------*/
/** @addtogroup MEC2016 eSPI IO Capabilities (ESPI_CAP)
  @{
*/
typedef struct
{
    __IO uint8_t  CAP_ID;               /*!< Offset: 0x02E0  Capabilities */
    __IO uint8_t  CAP0;                 /*!< Offset: 0x02E1  Global Capabilities 0 */
    __IO uint8_t  CAP1;                 /*!< Offset: 0x02E2  Global Capabilities 1 */
    __IO uint8_t  PC_CAP;               /*!< Offset: 0x02E3  Peripheral Channel Capabilities */
    __IO uint8_t  VW_CAP;               /*!< Offset: 0x02E4  Virtual Wire Channel Capabilities */
    __IO uint8_t  OOB_CAP;              /*!< Offset: 0x02E5  OOB Channel Capabilities */
    __IO uint8_t  FC_CAP;               /*!< Offset: 0x02E6  Flash Channel Capabilities */
    __IO uint8_t  PC_READY;             /*!< Offset: 0x02E7  Peripheral Channel Ready */
    __IO uint8_t  OOB_READY;            /*!< Offset: 0x02E8  OOB Channel Ready */
    __IO uint8_t  FC_READY;             /*!< Offset: 0x02E9  Flash Channel Ready */
    __IO uint8_t  ESPI_RESET_STATUS;    /*!< Offset: 0x02EA  ESPI_RESET Status */
    __IO uint8_t  ESPI_RESET_IEN;       /*!< Offset: 0x02EB  ESPI_RESET Interrupt Enable */
    __IO uint8_t  PLTRST_SRC;           /*!< Offset: 0x02EC  Platform Reset Source Select */
    __IO uint8_t  VW_READY;             /*!< Offset: 0x02ED  VWire channel Ready */
} MEC2016_ESPI_CAP_TypeDef;
/*@}*/ /* end of group MEC2016_ESPI_CAP_TypeDef */

/*------------- eSPI IO Config (ESPI_CFG) -----------------------------*/
/** @addtogroup MEC2016 eSPI IO Capabilities (ESPI_CFG)
  @{
*/
typedef struct
{
    __IO REG32_U  ACTIVATE;                 /*!< Offset: 0x0330  IO Component Activate */
    __IO REG32_U  IOBAR[MEC2016_ESPI_MAX_BAR_IDX];    /*!< Offset: 0x0334 - 0x0387 */
} MEC2016_ESPI_CFG_TypeDef;
/*@}*/ /* end of group MEC2016_ESPI_CFG_TypeDef */

/*------------- eSPI IO Serial IRQ Config (ESPI_SIRQ) -----------------------------*/
/** @addtogroup MEC2016 eSPI IO Serial IRQ Config (ESPI_SIRQ)
  @{
*/
#define MEC2016_ESPI_MBOX_SIRQ0_IDX        (0u)
#define MEC2016_ESPI_MBOX1_SIRQ1_IDX       (1u)
#define MEC2016_ESPI_MIF8042_SIRQ0_IDX     (2u)
#define MEC2016_ESPI_MIF8042_SIRQ1_IDX     (3u)
#define MEC2016_ESPI_ACPIEC0_SIRQ0_IDX     (4u)
#define MEC2016_ESPI_ACPIEC1_SIRQ0_IDX     (5u)
#define MEC2016_ESPI_ACPIEC2_SIRQ0_IDX     (6u)
#define MEC2016_ESPI_ACPIEC3_SIRQ0_IDX     (7u)
#define MEC2016_ESPI_ACPIEC4_SIRQ0_IDX     (8u)
#define MEC2016_ESPI_UART0_SIRQ0_IDX       (9u)
#define MEC2016_ESPI_UART1_SIRQ0_IDX       (10u)
#define MEC2016_ESPI_EMI0_SIRQ0_IDX        (11u)
#define MEC2016_ESPI_EMI0_SIRQ1_IDX        (12u)
#define MEC2016_ESPI_EMI1_SIRQ0_IDX        (13u)
#define MEC2016_ESPI_EMI1_SIRQ1_IDX        (14u)
#define MEC2016_ESPI_EMI2_SIRQ0_IDX        (15u)
#define MEC2016_ESPI_EMI2_SIRQ1_IDX        (16u)
#define MEC2016_ESPI_RTC_SIRQ0_IDX         (17u)
#define MEC2016_ESPI_EC_SIRQ0_IDX          (18u)
#define MEC2016_ESPI_MAX_SIRQ_IDX          (19u)

typedef struct
{
    __IO uint8_t  SERIRQ[MEC2016_ESPI_MAX_SIRQ_IDX];  /*!< Offset: 0x03AC - 0x03BF */
} MEC2016_ESPI_SIRQ_TypeDef;
/*@}*/ /* end of group MEC2016_ESPI_SIRQ_TypeDef */


/* eSPI Memory BAR's are split into two pieces.
 * EC accessible only starting at offset 0x1F0 and located on 10-byte
 * boundaries. Size is 32-bits:
 *  Contains the Mask, Logical Device Number & Virualized parameters.
 * Second pieces starting at offset 0x330 and located on 10-byte boundaries.
 * Size is 10 bytes.
 *  Contains the Valid and 64-bit Host Address fields.
 *
 */
/*------------- eSPI Memory (ESPI_MEM_HBAR) -----------------------------*/
/** @addtogroup MEC2016 eSPI Memory Component (ESPI_MEM_HBAR)
 * 10 byte registers each containing 64-bit Host Address and valid bit
  @{
*/
#define MEC2016_ESPI_MBAR_MBOX_15_0         (0u)
#define MEC2016_ESPI_MBAR_MBOX_31_16        (1u)
#define MEC2016_ESPI_MBAR_MBOX_47_32        (2u)
#define MEC2016_ESPI_MBAR_MBOX_63_48        (3u)
#define MEC2016_ESPI_MBAR_MBOX_79_64        (4u)
//
#define MEC2016_ESPI_MBAR_ACPIEC0_15_0      (5u)
#define MEC2016_ESPI_MBAR_ACPIEC0_31_16     (6u)
#define MEC2016_ESPI_MBAR_ACPIEC0_47_32     (7u)
#define MEC2016_ESPI_MBAR_ACPIEC0_63_48     (8u)
#define MEC2016_ESPI_MBAR_ACPIEC0_79_64     (9u)
//
#define MEC2016_ESPI_MBAR_ACPIEC1_15_0      (10u)
#define MEC2016_ESPI_MBAR_ACPIEC1_31_16     (11u)
#define MEC2016_ESPI_MBAR_ACPIEC1_47_32     (12u)
#define MEC2016_ESPI_MBAR_ACPIEC1_63_48     (13u)
#define MEC2016_ESPI_MBAR_ACPIEC1_79_64     (14u)
//
#define MEC2016_ESPI_MBAR_ACPIEC2_15_0      (15u)
#define MEC2016_ESPI_MBAR_ACPIEC2_31_16     (16u)
#define MEC2016_ESPI_MBAR_ACPIEC2_47_32     (17u)
#define MEC2016_ESPI_MBAR_ACPIEC2_63_48     (18u)
#define MEC2016_ESPI_MBAR_ACPIEC2_79_64     (19u)
//
#define MEC2016_ESPI_MBAR_ACPIEC3_15_0      (20u)
#define MEC2016_ESPI_MBAR_ACPIEC3_31_16     (21u)
#define MEC2016_ESPI_MBAR_ACPIEC3_47_32     (22u)
#define MEC2016_ESPI_MBAR_ACPIEC3_63_48     (23u)
#define MEC2016_ESPI_MBAR_ACPIEC3_79_64     (24u)
//
#define MEC2016_ESPI_MBAR_ACPIEC4_15_0      (25u)
#define MEC2016_ESPI_MBAR_ACPIEC4_31_16     (26u)
#define MEC2016_ESPI_MBAR_ACPIEC4_47_32     (27u)
#define MEC2016_ESPI_MBAR_ACPIEC4_63_48     (28u)
#define MEC2016_ESPI_MBAR_ACPIEC4_79_64     (29u)
//
#define MEC2016_ESPI_MBAR_EM0_15_0          (30u)
#define MEC2016_ESPI_MBAR_EM0_31_16         (31u)
#define MEC2016_ESPI_MBAR_EM0_47_32         (32u)
#define MEC2016_ESPI_MBAR_EM0_63_48         (33u)
#define MEC2016_ESPI_MBAR_EM0_79_64         (34u)
//
#define MEC2016_ESPI_MBAR_EM1_15_0          (35u)
#define MEC2016_ESPI_MBAR_EM1_31_16         (36u)
#define MEC2016_ESPI_MBAR_EM1_47_32         (37u)
#define MEC2016_ESPI_MBAR_EM1_63_48         (38u)
#define MEC2016_ESPI_MBAR_EM1_79_64         (39u)
//
#define MEC2016_ESPI_MBAR_EM2_15_0          (40u)
#define MEC2016_ESPI_MBAR_EM2_31_16         (41u)
#define MEC2016_ESPI_MBAR_EM2_47_32         (42u)
#define MEC2016_ESPI_MBAR_EM2_63_48         (43u)
#define MEC2016_ESPI_MBAR_EM2_79_64         (44u)
//
#define MEC2016_ESPI_MBAR_TEST_15_0         (45u)
#define MEC2016_ESPI_MBAR_TEST_31_16        (46u)
#define MEC2016_ESPI_MBAR_TEST_47_32        (47u)
#define MEC2016_ESPI_MBAR_TEST_63_48        (48u)
#define MEC2016_ESPI_MBAR_TEST_79_64        (49u)
//
#define MEC2016_ESPI_MBAR_MAX_IDX           (50u)


typedef struct
{
    __IO uint16_t MBAR_EC[MEC2016_ESPI_MBAR_MAX_IDX];   /*!< Offset: 0x130 - 0x193 Mem Bar EC config  */
} MEC2016_ESPI_MBAR_EC_TypeDef;
/*@}*/ /* end of group MEC2016_ESPI_MEM_HBAR_TypeDef */

typedef struct
{
    __IO uint16_t MBAR_HOST[MEC2016_ESPI_MBAR_MAX_IDX];   /*!< Offset: 0x330 - 0x393 Mem Bar Host config  */
} MEC2016_ESPI_MBAR_HOST_TypeDef;

/*@}*/ /* end of group MEC2016_ESPI_MEM_HBAR_TypeDef */


/*------------- eSPI Memory (ESPI_MEM_SRAM) -----------------------------*/
/** @addtogroup MEC2016 Virtual Wire Component (ESPI_MEM_SRAM)
 * Set SRAM BAR EC side address. (EC internal SRAM memory).
  @{
*/
typedef struct
{
    __IO REG32_U SRAM_BAR0;     /*!< Offset: 0x1F8 eSPI Memory block SRAM BAR 0 */
    __IO REG32_U SRAM_BAR1;     /*!< Offset: 0x1FC eSPI Memory block SRAM BAR 1 */
} MEC2016_ESPI_MEM_SRAM_TypeDef;
/*@}*/ /* end of group MEC2016_ESPI_MEM_SRAM_TypeDef */

/*------------- eSPI Memory (ESPI_MEM_HSRAM) -----------------------------*/
/** @addtogroup MEC2016 Virtual Wire Component (ESPI_MEM_HSRAM)
 * Set SRAM BAR Host side address.
  @{
*/
#define MEC2016_SRAM_BAR_15_0_IDX       (0u)
#define MEC2016_SRAM_BAR_31_16_IDX      (1u)
#define MEC2016_SRAM_BAR_47_32_IDX      (2u)
#define MEC2016_SRAM_BAR_63_58_IDX      (3u)
#define MEC2016_SRAM_BAR_79_64_IDX      (4u)
#define MEC2016_SRAM_BAR_IDX_MAX        (5u)

typedef struct
{

    __IO uint16_t SRAM_BAR0[MEC2016_SRAM_BAR_IDX_MAX]; /*!< Offset: 0x3AC eSPI Memory Host SRAM BAR 0 */
    __IO uint16_t SRAM_BAR1[MEC2016_SRAM_BAR_IDX_MAX]; /*!< Offset: 0x3B6 eSPI Memory Host SRAM BAR 1 */
} MEC2016_ESPI_MEM_HSRAM_TypeDef;
/*@}*/ /* end of group MEC2016_ESPI_MEM_HSRAM_TypeDef */


/*------------- eSPI Memory (ESPI_MEM_BM) -----------------------------*/
/** @addtogroup MEC2016 eSPI Memory Component Bus Master (ESPI_MEM_BM)
  @{
*/
typedef struct
{
    __IO REG32_U STATUS;            /*!< Offset: 0x0200 Bus Master Status */
    __IO REG32_U IEN;               /*!< Offset: 0x0204 Bus Master Interrupt Enable */
    __IO REG32_U CONFIG;            /*!< Offset: 0x0208 Bus Master Configuration */
        uint32_t RSVDA[1];
    __IO REG32_U CONTROL1;          /*!< Offset: 0x0210 Bus Master 1 Control */
    __IO REG32_U HOST_ADDR1_LO;     /*!< Offset: 0x0214 Bus Master 1 Host Address Lo */
    __IO REG32_U HOST_ADDR1_HI;     /*!< Offset: 0x0218 Bus Master 1 Host Address Hi */
    __IO REG32_U INTERNAL_ADDR1;    /*!< Offset: 0x021C Bus Master 1 Internal Address */
        uint32_t RSVDB[1];
    __IO REG32_U CONTROL2;          /*!< Offset: 0x0224 Bus Master 2 Control */
    __IO REG32_U HOST_ADDR2_LO;     /*!< Offset: 0x0228 Bus Master 1 Host Address Lo */
    __IO REG32_U HOST_ADDR2_HI;     /*!< Offset: 0x022C Bus Master 1 Host Address Hi */
    __IO REG32_U INTERNAL_ADDR2;    /*!< Offset: 0x0230 Bus Master 1 Internal Address */
} MEC2016_ESPI_MEM_BM_TypeDef;
/*@}*/ /* end of group MEC2016_ESPI_BM_SRAM_TypeDef */


/*------------- eSPI Master to Slave Virtual Wire (ESPI_MSVW) ----------------*/
/** @addtogroup MEC2016 Virtual Wire Component Master-to-Slave (ESPI_MSVW)
  @{
*/
#define MEC2016_ESPI_MSVW00_31_0        (0u)
#define MEC2016_ESPI_MSVW00_63_32       (1u)
#define MEC2016_ESPI_MSVW00_95_64       (2u)
//
#define MEC2016_ESPI_MSVW01_31_0        (3u)
#define MEC2016_ESPI_MSVW01_63_32       (4u)
#define MEC2016_ESPI_MSVW01_95_64       (5u)
//
#define MEC2016_ESPI_MSVW02_31_0        (6u)
#define MEC2016_ESPI_MSVW02_63_32       (7u)
#define MEC2016_ESPI_MSVW02_95_64       (8u)
//
#define MEC2016_ESPI_MSVW03_31_0        (9u)
#define MEC2016_ESPI_MSVW03_63_32       (10u)
#define MEC2016_ESPI_MSVW03_95_64       (11u)
//
#define MEC2016_ESPI_MSVW04_31_0        (12u)
#define MEC2016_ESPI_MSVW04_63_32       (13u)
#define MEC2016_ESPI_MSVW04_95_64       (14u)
//
#define MEC2016_ESPI_MSVW05_31_0        (15u)
#define MEC2016_ESPI_MSVW05_63_32       (16u)
#define MEC2016_ESPI_MSVW05_95_64       (17u)
//
#define MEC2016_ESPI_MSVW06_31_0        (18u)
#define MEC2016_ESPI_MSVW06_63_32       (19u)
#define MEC2016_ESPI_MSVW06_95_64       (20u)
//
#define MEC2016_ESPI_MSVW07_31_0        (21u)
#define MEC2016_ESPI_MSVW07_63_32       (22u)
#define MEC2016_ESPI_MSVW07_95_64       (23u)
//
#define MEC2016_ESPI_MSVW08_31_0        (24u)
#define MEC2016_ESPI_MSVW08_63_32       (25u)
#define MEC2016_ESPI_MSVW08_95_64       (26u)
//
#define MEC2016_ESPI_MSVW09_31_0        (27u)
#define MEC2016_ESPI_MSVW09_63_32       (28u)
#define MEC2016_ESPI_MSVW09_95_64       (29u)
//
#define MEC2016_ESPI_MSVW10_31_0        (30u)
#define MEC2016_ESPI_MSVW10_63_32       (31u)
#define MEC2016_ESPI_MSVW10_95_64       (32u)
//
#define MEC2016_ESPI_MSVW_MAX           (33u)

typedef struct
{
    __IO REG32_U VW[MEC2016_ESPI_MSVW_MAX];     /*<! Offset: 0x000 - 0x18B Master to Slave Virtual Wire Registers */
} MEC2016_ESPI_MSVW_TypeDef;
/*@}*/ /* end of group MEC2016_ESPI_MSVW_TypeDef */

/*------------- eSPI Master to Slave Virtual Wire (ESPI_MSVW) ----------------*/
/** @addtogroup MEC2016 Virtual Wire Component Master-to-Slave (ESPI_MSVW)
  @{
*/
#define MEC2016_ESPI_SMVW00_31_0        (0u)
#define MEC2016_ESPI_SMVW00_63_32       (1u)
#define MEC2016_ESPI_SMVW00_95_64       (2u)
//
#define MEC2016_ESPI_SMVW01_31_0        (3u)
#define MEC2016_ESPI_SMVW01_63_32       (4u)
#define MEC2016_ESPI_SMVW01_95_64       (5u)
//
#define MEC2016_ESPI_SMVW02_31_0        (6u)
#define MEC2016_ESPI_SMVW02_63_32       (7u)
#define MEC2016_ESPI_SMVW02_95_64       (8u)
//
#define MEC2016_ESPI_SMVW03_31_0        (9u)
#define MEC2016_ESPI_SMVW03_63_32       (10u)
#define MEC2016_ESPI_SMVW03_95_64       (11u)
//
#define MEC2016_ESPI_SMVW04_31_0        (12u)
#define MEC2016_ESPI_SMVW04_63_32       (13u)
#define MEC2016_ESPI_SMVW04_95_64       (14u)
//
#define MEC2016_ESPI_SMVW05_31_0        (15u)
#define MEC2016_ESPI_SMVW05_63_32       (16u)
#define MEC2016_ESPI_SMVW05_95_64       (17u)
//
#define MEC2016_ESPI_SMVW06_31_0        (18u)
#define MEC2016_ESPI_SMVW06_63_32       (19u)
#define MEC2016_ESPI_SMVW06_95_64       (20u)
//
#define MEC2016_ESPI_SMVW07_31_0        (21u)
#define MEC2016_ESPI_SMVW07_63_32       (22u)
#define MEC2016_ESPI_SMVW07_95_64       (23u)
//
#define MEC2016_ESPI_SMVW_MAX           (24u)

typedef struct
{

    __IO REG32_U VW[MEC2016_ESPI_SMVW_MAX];     /*<! Offset: 0x200 - 0x31F Slave to Master Virtual Wire Registers */
} MEC2016_ESPI_SMVW_TypeDef;

/*@}*/ /* end of group MEC2016_ESPI_SMVW_TypeDef */


/*------------- Host Global Config (HGCFG) -----------------------------*/
/** @addtogroup MEC2016 Host Global Config (HGCFG)
  @{
*/
typedef struct
{
         uint8_t  RSVDA[7];
    __IO uint8_t  LD_SEL;               /*!< Offset: 0x0007   Logical Device Number  */
         uint8_t  RSVDB[0x18];          // 0x1F - 0x08
    __IO uint8_t  DEVICE_ID;            /*!< Offset: 0x0020   Device ID  */
    __IO uint8_t  DEVICE_REV;           /*!< Offset: 0x0021   Device Revision  */
         uint8_t  RSVDC[2];             // 0x22 - 0x23
    __IO uint8_t  DEVICE_MODE;          /*!< Offset: 0x0024   Device Mode  */
         uint8_t  RSVDD[0xB];           // 0x25 - 0x2F
    __IO uint8_t  MCHP_TEST[8];         /*!< Offset: 0x0028 - 0x002F   MCHP Test  */
} MEC2016_HGCFG_TypeDef;
/*@}*/ /* end of group MEC2016_HGCFG_TypeDef */

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

/*@}*/ /* end of group <Device>_Peripherals */


/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/* ToDo: add here your device peripherals base addresses
         following is an example for timer                                    */
/** @addtogroup <Device>_MemoryMap <Device> Memory Mapping
  @{
*/

/* Peripheral and SRAM base address */
#define MEC2016_CODE_SRAM_BASE        (0x00100000UL)  /*!< (CODE SRAM     ) Base Address */
#define MEC2016_DATA_SRAM_BASE        (0x00168000UL)  /*!< (DATA SRAM     ) Base Address */
#define MEC2016_DATA_SRAM_LEN         (0x00010000UL)
#define MEC2016_DATA_SRAM_END         ((MEC2016_DATA_SRAM_BASE) + (MEC2016_DATA_SRAM_LEN))

#define MEC2016_PERIPH_BASE           (0x40000000UL)                              /*!< (Peripheral) Base Address */
#define MEC2016_PERIPH_SPB_BASE       ((MEC2016_PERIPH_BASE) + (0x00080000UL))
#define MEC2016_PERIPH_HOST_BASE      ((MEC2016_PERIPH_BASE) + (0x000F0000UL))

#define MEC2016_AHB_SEGMENT2          (0x40100000UL)

/* Peripheral memory map */
#define MEC2016_WDT0_BASE         (MEC2016_PERIPH_BASE)          /*!< (WDT0    ) Base Address */

#define MEC2016_B16TMR0_BASE      (MEC2016_PERIPH_BASE + 0x0C00UL) /*!< (16-bit Basic Timer0    ) Base Address */
#define MEC2016_B16TMR1_BASE      (MEC2016_PERIPH_BASE + 0x0C20UL) /*!< (16-bit Basic Timer1    ) Base Address */
#define MEC2016_B16TMR2_BASE      (MEC2016_PERIPH_BASE + 0x0C40UL) /*!< (16-bit Basic Timer2    ) Base Address */
#define MEC2016_B16TMR3_BASE      (MEC2016_PERIPH_BASE + 0x0C60UL) /*!< (16-bit Basic Timer3    ) Base Address */

#define MEC2016_B32TMR0_BASE      (MEC2016_PERIPH_BASE + 0x0C80UL) /*!< (32-bit Basic Timer0    ) Base Address */
#define MEC2016_B32TMR1_BASE      (MEC2016_PERIPH_BASE + 0x0CA0UL) /*!< (32-bit Basic Timer1    ) Base Address */

#define MEC2016_EVT0_BASE         (MEC2016_PERIPH_BASE + 0x0D00UL) /*!< (Capture/Compare Timer0    ) Base Address */
#define MEC2016_EVT1_BASE         (MEC2016_PERIPH_BASE + 0x0D20UL) /*!< (Capture/Compare Timer1    ) Base Address */
#define MEC2016_EVT2_BASE         (MEC2016_PERIPH_BASE + 0x0D40UL) /*!< (Capture/Compare Timer2    ) Base Address */
#define MEC2016_EVT3_BASE         (MEC2016_PERIPH_BASE + 0x0D60UL) /*!< (Capture/Compare Timer3    ) Base Address */

#define MEC2016_CCT0_BASE         (MEC2016_PERIPH_BASE + 0x1000UL) /*!< (Capture/Compare Timer0    ) Base Address */

#define MEC2016_RCID0_BASE        (MEC2016_PERIPH_BASE + 0x0140UL) /*!< (RC-ID 0    ) Base Address */
#define MEC2016_RCID1_BASE        (MEC2016_PERIPH_BASE + 0x0148UL) /*!< (RC-ID 1    ) Base Address */
#define MEC2016_RCID2_BASE        (MEC2016_PERIPH_BASE + 0x0150UL) /*!< (RC-ID 2    ) Base Address */

#define MEC2016_DMAM_BASE         (MEC2016_PERIPH_BASE + 0x2400UL) /*!< (DMA Main    ) Base Address */

#define MEC2016_DMACH0_BASE       (MEC2016_PERIPH_BASE + 0x2440UL) /*!< (DMA Channel 0    ) Base Address */
#define MEC2016_DMACH1_BASE       (MEC2016_PERIPH_BASE + 0x2480UL) /*!< (DMA Channel 1    ) Base Address */
#define MEC2016_DMACH2_BASE       (MEC2016_PERIPH_BASE + 0x24C0UL) /*!< (DMA Channel 2    ) Base Address */
#define MEC2016_DMACH3_BASE       (MEC2016_PERIPH_BASE + 0x2500UL) /*!< (DMA Channel 3    ) Base Address */
#define MEC2016_DMACH4_BASE       (MEC2016_PERIPH_BASE + 0x2540UL) /*!< (DMA Channel 4    ) Base Address */
#define MEC2016_DMACH5_BASE       (MEC2016_PERIPH_BASE + 0x2580UL) /*!< (DMA Channel 5    ) Base Address */
#define MEC2016_DMACH6_BASE       (MEC2016_PERIPH_BASE + 0x25C0UL) /*!< (DMA Channel 6    ) Base Address */
#define MEC2016_DMACH7_BASE       (MEC2016_PERIPH_BASE + 0x2600UL) /*!< (DMA Channel 7    ) Base Address */
#define MEC2016_DMACH8_BASE       (MEC2016_PERIPH_BASE + 0x2640UL) /*!< (DMA Channel 8    ) Base Address */
#define MEC2016_DMACH9_BASE       (MEC2016_PERIPH_BASE + 0x2680UL) /*!< (DMA Channel 9    ) Base Address */
#define MEC2016_DMACH10_BASE      (MEC2016_PERIPH_BASE + 0x26C0UL) /*!< (DMA Channel 10   ) Base Address */
#define MEC2016_DMACH11_BASE      (MEC2016_PERIPH_BASE + 0x2700UL) /*!< (DMA Channel 11   ) Base Address */
#define MEC2016_DMACH12_BASE      (MEC2016_PERIPH_BASE + 0x2740UL) /*!< (DMA Channel 12   ) Base Address */
#define MEC2016_DMACH13_BASE      (MEC2016_PERIPH_BASE + 0x2780UL) /*!< (DMA Channel 13   ) Base Address */

#define MEC2016_EEPROM_BASE       (MEC2016_PERIPH_BASE + 0x2C00UL) /*!< (EEPROM ) Base Address */

#define MEC2016_QMSPI0_BASE       (MEC2016_PERIPH_BASE + 0x5400UL) /*!< (QMSPI 0    ) Base Address */

#define MEC2016_PWM0_BASE         (MEC2016_PERIPH_BASE + 0x5800UL) /*!< (PWM 0  ) Base Address */
#define MEC2016_PWM1_BASE         (MEC2016_PERIPH_BASE + 0x5810UL) /*!< (PWM 1  ) Base Address */
#define MEC2016_PWM2_BASE         (MEC2016_PERIPH_BASE + 0x5820UL) /*!< (PWM 2  ) Base Address */
#define MEC2016_PWM3_BASE         (MEC2016_PERIPH_BASE + 0x5830UL) /*!< (PWM 3  ) Base Address */
#define MEC2016_PWM4_BASE         (MEC2016_PERIPH_BASE + 0x5840UL) /*!< (PWM 4  ) Base Address */
#define MEC2016_PWM5_BASE         (MEC2016_PERIPH_BASE + 0x5850UL) /*!< (PWM 5  ) Base Address */
#define MEC2016_PWM6_BASE         (MEC2016_PERIPH_BASE + 0x5860UL) /*!< (PWM 6  ) Base Address */
#define MEC2016_PWM7_BASE         (MEC2016_PERIPH_BASE + 0x5870UL) /*!< (PWM 7  ) Base Address */
#define MEC2016_PWM8_BASE         (MEC2016_PERIPH_BASE + 0x5880UL) /*!< (PWM 8  ) Base Address */
#define MEC2016_PWM9_BASE         (MEC2016_PERIPH_BASE + 0x5890UL) /*!< (PWM 9  ) Base Address */
#define MEC2016_PWM10_BASE        (MEC2016_PERIPH_BASE + 0x58A0UL) /*!< (PWM 10  ) Base Address */
#define MEC2016_PWM11_BASE        (MEC2016_PERIPH_BASE + 0x58B0UL) /*!< (PWM 11  ) Base Address */

#define MEC2016_TACH0_BASE        (MEC2016_PERIPH_BASE + 0x6000UL) /*!< (TACH 0  ) Base Address */
#define MEC2016_TACH1_BASE        (MEC2016_PERIPH_BASE + 0x6010UL) /*!< (TACH 1  ) Base Address */
#define MEC2016_TACH2_BASE        (MEC2016_PERIPH_BASE + 0x6020UL) /*!< (TACH 2  ) Base Address */

#define MEC2016_PECI_BASE         (MEC2016_PERIPH_BASE + 0x6400UL) /*!< (PECI  ) Base Address */

#define MEC2016_RTMR_BASE         (MEC2016_PERIPH_BASE + 0x7400UL) /*!< (RTOS Timer ) Base Address */

#define MEC2016_ADC_BASE          (MEC2016_PERIPH_BASE + 0x7C00UL) /*!< (ADC ) Base Address */

#define MEC2016_TFDP_BASE         (MEC2016_PERIPH_BASE + 0x8C00UL) /*!< (TFDP    ) Base Address */

#define MEC2016_GPSPI0_BASE       (MEC2016_PERIPH_BASE + 0x9400UL) /*!< (GPSPI 0    ) Base Address */
#define MEC2016_GPSPI1_BASE       (MEC2016_PERIPH_BASE + 0x9480UL) /*!< (GPSPI 1    ) Base Address */

#define MEC2016_VBREGS_BASE       (MEC2016_PERIPH_BASE + 0xA400UL) /*!< (VBAT Register Bank  ) Base Address */

#define MEC2016_LED0_BASE         (MEC2016_PERIPH_BASE + 0xB800UL) /*!< (Blinking-Breathing LED 0 ) Base Address */
#define MEC2016_LED1_BASE         (MEC2016_PERIPH_BASE + 0xB900UL) /*!< (Blinking-Breathing LED 1 ) Base Address */
#define MEC2016_LED2_BASE         (MEC2016_PERIPH_BASE + 0xBA00UL) /*!< (Blinking-Breathing LED 2 ) Base Address */
#define MEC2016_LED3_BASE         (MEC2016_PERIPH_BASE + 0xBB00UL) /*!< (Blinking-Breathing LED 3 ) Base Address */

#define MEC2016_PKE_BASE          (MEC2016_PERIPH_BASE + 0xBD00UL) /*!< (Public Key Engine ) Base Address */

#define MEC2016_RNG_BASE          (MEC2016_PERIPH_BASE + 0xBE00UL) /*!< (RNG ) Base Address */

#define MEC2016_HASH_BASE         (MEC2016_PERIPH_BASE + 0xD000UL) /*!< (Hash Engine ) Base Address */

#define MEC2016_AES_BASE          (MEC2016_PERIPH_BASE + 0xD200UL) /*!< (AES Engine ) Base Address */

#define MEC2016_ECIA_BASE         (MEC2016_PERIPH_BASE + 0xE000UL) /*!< (EC Interrupt Aggregator ) Base Address */

#define MEC2016_ECS_BASE          (MEC2016_PERIPH_BASE + 0xFC00UL) /*!< (EC Subystem ) Base Address */

/* EC SPB Segment */
#define MEC2016_PCR_BASE          (MEC2016_PERIPH_SPB_BASE + 0x0100UL) /*!< (PCR ) Base Address */

#define MEC2016_GPIO_CTRL_BASE    (MEC2016_PERIPH_SPB_BASE + 0x1000UL) /*!< (GPIO Control ) Base Address */
#define MEC2016_GPIO_LOCK_BASE    (MEC2016_PERIPH_SPB_BASE + 0x13E8UL) /*!< (GPIO Lock ) Base Address */
#define MEC2016_GPIO_PIO_BASE     (MEC2016_PERIPH_SPB_BASE + 0x1300UL) /*!< (GPIO Parallel I/O ) Base Address */
#define MEC2016_GPIO_CTRL2_BASE   (MEC2016_PERIPH_SPB_BASE + 0x1500UL) /*!< (GPIO Control2 ) Base Address */

#define MEC2016_EFUSE_BASE        (MEC2016_PERIPH_SPB_BASE + 0x2000UL) /*!< (eFUSE ) Base Address */


/* EC Host Segment */
#define MEC2016_UART0_BASE        (MEC2016_PERIPH_HOST_BASE + 0x2400UL) /*!< (UART0  ) Base Address */
#define MEC2016_UART1_BASE        (MEC2016_PERIPH_HOST_BASE + 0x2800UL) /*!< (UART1  ) Base Address */

#define MEC2016_ESPI_IO_BASE    ((MEC2016_PERIPH_HOST_BASE) + 0x3400UL) /*!< (eSPI IO ) Base Address */

#define MEC2016_ESPI_LTR_BASE   ((MEC2016_ESPI_IO_BASE) + 0x0220UL) /*!< (eSPI IO LTR ) Base Address */
#define MEC2016_ESPI_OOB_BASE   ((MEC2016_ESPI_IO_BASE) + 0x0240UL) /*!< (eSPI IO OOB Channel ) Base Address */
#define MEC2016_ESPI_FC_BASE    ((MEC2016_ESPI_IO_BASE) + 0x0280UL) /*!< (eSPI IO Flash Channel ) Base Address */
#define MEC2016_ESPI_CAP_BASE   ((MEC2016_ESPI_IO_BASE) + 0x02E0ul) /*!< (eSPI IO Capabilites ) Base Address */

#define MEC2016_ESPI_MEM_BASE   (MEC2016_PERIPH_HOST_BASE + 0x3800UL) /*!< (eSPI MEM ) Base Address */

#define MEC2016_ESPI_VW_BASE      (MEC2016_PERIPH_HOST_BASE + 0x9C00UL) /*!< (eSPI VWire ) Base Address */

#define MEC2016_GLOBAL_CFG_BASE   (MEC2016_PERIPH_HOST_BASE + 0xFF00UL) /*!< (Host Global Config  ) Base Address */

/* AHB Segment 2 */
#define MEC2016_PKE_SCMEM_BASE    ((MEC2016_AHB_SEGMENT2) + 0x0000UL)
#define MEC2016_PKE_SCMEM_END     ((MEC2016_AHB_SEGMENT2) + 0x6000UL) /* 24KB */

#define MEC2016_RNG_FIFO_BASE     ((MEC2016_AHB_SEGMENT2) + 0x8000UL)

/* Other */
#define MEC2016_DELAY_REG_BASE    (0x10000000ul)

// SMBus
#define MEC2016_SMB0_BASE                       0x40004000UL
#define MEC2016_SMB1_BASE                       0x40004400UL
#define MEC2016_SMB2_BASE                       0x40004800UL
#define MEC2016_SMB3_BASE                       0x40004C00UL

/*@}*/ /* end of group <Device>_MemoryMap */


/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/
/* ToDo: add here your device peripherals pointer definitions
         following is an example for timer                                    */

/** @addtogroup <Device>_PeripheralDecl <Device> Peripheral Declaration
  @{
*/

#define MEC2016_WDT0        ((MEC2016_WDT_TypeDef *) MEC2016_WDT0_BASE)

#define MEC2016_B16TMR0     ((MEC2016_BTMR16_TypeDef *) MEC2016_B16TMR0_BASE)
#define MEC2016_B16TMR1     ((MEC2016_BTMR16_TypeDef *) MEC2016_B16TMR1_BASE)
#define MEC2016_B16TMR2     ((MEC2016_BTMR16_TypeDef *) MEC2016_B16TMR2_BASE)
#define MEC2016_B16TMR3     ((MEC2016_BTMR16_TypeDef *) MEC2016_B16TMR3_BASE)

#define MEC2016_B32TMR0     ((MEC2016_BTMR32_TypeDef *) MEC2016_B32TMR0_BASE)
#define MEC2016_B32TMR1     ((MEC2016_BTMR32_TypeDef *) MEC2016_B32TMR1_BASE)

#define MEC2016_EVT0        ((MEC2016_EVT_TypeDef *) MEC2016_EVT0_BASE)
#define MEC2016_EVT1        ((MEC2016_EVT_TypeDef *) MEC2016_EVT1_BASE)
#define MEC2016_EVT2        ((MEC2016_EVT_TypeDef *) MEC2016_EVT2_BASE)
#define MEC2016_EVT3        ((MEC2016_EVT_TypeDef *) MEC2016_EVT3_BASE)

#define MEC2016_CCT0        ((MEC2016_CCT_TypeDef *) MEC2016_CCT0_BASE)

#define MEC2016_RCID0       ((MEC2016_RCID_TypeDef *) MEC2016_RCID0_BASE)
#define MEC2016_RCID1       ((MEC2016_RCID_TypeDef *) MEC2016_RCID1_BASE)
#define MEC2016_RCID2       ((MEC2016_RCID_TypeDef *) MEC2016_RCID2_BASE)

#define MEC2016_DMAM        ((MEC2016_DMAM_TypeDef *) MEC2016_DMAM_BASE)
/* Individual DMA Channels */
#define MEC2016_DMACH0      ((MEC2016_DMACH_ALU_TypeDef *) MEC2016_DMACH0_BASE)
#define MEC2016_DMACH1      ((MEC2016_DMACH_ALU_TypeDef *) MEC2016_DMACH1_BASE)
#define MEC2016_DMACH2      ((MEC2016_DMACH_TypeDef *) MEC2016_DMACH2_BASE)
#define MEC2016_DMACH3      ((MEC2016_DMACH_TypeDef *) MEC2016_DMACH3_BASE)
#define MEC2016_DMACH4      ((MEC2016_DMACH_TypeDef *) MEC2016_DMACH4_BASE)
#define MEC2016_DMACH5      ((MEC2016_DMACH_TypeDef *) MEC2016_DMACH5_BASE)
#define MEC2016_DMACH6      ((MEC2016_DMACH_TypeDef *) MEC2016_DMACH6_BASE)
#define MEC2016_DMACH7      ((MEC2016_DMACH_TypeDef *) MEC2016_DMACH7_BASE)
#define MEC2016_DMACH8      ((MEC2016_DMACH_TypeDef *) MEC2016_DMACH8_BASE)
#define MEC2016_DMACH9      ((MEC2016_DMACH_TypeDef *) MEC2016_DMACH9_BASE)
#define MEC2016_DMACH10     ((MEC2016_DMACH_TypeDef *) MEC2016_DMACH10_BASE)
#define MEC2016_DMACH11     ((MEC2016_DMACH_TypeDef *) MEC2016_DMACH11_BASE)
#define MEC2016_DMACH12     ((MEC2016_DMACH_TypeDef *) MEC2016_DMACH12_BASE)
#define MEC2016_DMACH13     ((MEC2016_DMACH_TypeDef *) MEC2016_DMACH13_BASE)

#define MEC2016_EEPROM      ((MEC2016_EEPROM_TypeDef*) MEC2016_EEPROM_BASE)

#define MEC2016_QMSPI0      ((MEC2016_QMSPI_TypeDef *) MEC2016_QMSPI0_BASE)

#define MEC2016_PWM0        ((MEC2016_PWM_TypeDef *) MEC2016_PWM0_BASE)
#define MEC2016_PWM1        ((MEC2016_PWM_TypeDef *) MEC2016_PWM1_BASE)
#define MEC2016_PWM2        ((MEC2016_PWM_TypeDef *) MEC2016_PWM2_BASE)
#define MEC2016_PWM3        ((MEC2016_PWM_TypeDef *) MEC2016_PWM3_BASE)
#define MEC2016_PWM4        ((MEC2016_PWM_TypeDef *) MEC2016_PWM4_BASE)
#define MEC2016_PWM5        ((MEC2016_PWM_TypeDef *) MEC2016_PWM5_BASE)
#define MEC2016_PWM6        ((MEC2016_PWM_TypeDef *) MEC2016_PWM6_BASE)
#define MEC2016_PWM7        ((MEC2016_PWM_TypeDef *) MEC2016_PWM7_BASE)
#define MEC2016_PWM8        ((MEC2016_PWM_TypeDef *) MEC2016_PWM8_BASE)
#define MEC2016_PWM9        ((MEC2016_PWM_TypeDef *) MEC2016_PWM9_BASE)
#define MEC2016_PWM10       ((MEC2016_PWM_TypeDef *) MEC2016_PWM10_BASE)
#define MEC2016_PWM11       ((MEC2016_PWM_TypeDef *) MEC2016_PWM11_BASE)

#define MEC2016_TACH0       ((MEC2016_TACH_TypeDef *) MEC2016_TACH0_BASE)
#define MEC2016_TACH1       ((MEC2016_TACH_TypeDef *) MEC2016_TACH1_BASE)
#define MEC2016_TACH2       ((MEC2016_TACH_TypeDef *) MEC2016_TACH2_BASE)

#define MEC2016_PECI        ((MEC2016_PECI_TypeDef *) MEC2016_PECI_BASE)

#define MEC2016_RTMR        ((MEC2016_RTMR_TypeDef *) MEC2016_RTMR_BASE)

#define MEC2016_ADC         ((MEC2016_ADC_TypeDef *) MEC2016_ADC_BASE)

#define MEC2016_TFDP        ((MEC2016_TFDP_TypeDef *) MEC2016_TFDP_BASE)

#define MEC2016_GPSPI0      ((MEC2016_GPSPI_TypeDef *) MEC2016_GPSPI0_BASE)
#define MEC2016_GPSPI1      ((MEC2016_GPSPI_TypeDef *) MEC2016_GPSPI1_BASE)

#define MEC2016_VBR         ((MEC2016_VBR_TypeDef*) MEC2016_VBREGS_BASE)

#define MEC2016_LED0        ((MEC2016_BBLED_TypeDef *) MEC2016_LED0_BASE)
#define MEC2016_LED1        ((MEC2016_BBLED_TypeDef *) MEC2016_LED1_BASE)
#define MEC2016_LED2        ((MEC2016_BBLED_TypeDef *) MEC2016_LED2_BASE)
#define MEC2016_LED3        ((MEC2016_BBLED_TypeDef *) MEC2016_LED3_BASE)

#define MEC2016_PKE         ((MEC2016_PKE_TypeDef *) MEC2016_PKE_BASE)

#define MEC2016_RNG         ((MEC2016_RNG_TypeDef *) MEC2016_RNG_BASE)

#define MEC2016_HASH        ((MEC2016_HASH_TypeDef *) MEC2016_HASH_BASE)

#define MEC2016_AES         ((MEC2016_AES_TypeDef *) MEC2016_AES_BASE)

#define MEC2016_ECIA        ((MEC2016_ECIA_TypeDef *) MEC2016_ECIA_BASE)

#define MEC2016_ECS         ((MEC2016_ECS_TypeDef *) MEC2016_ECS_BASE)


/*-------------- EC SPB Segment ---------------------*/
#define MEC2016_PCR         ((MEC2016_PCR_TypeDef *) MEC2016_PCR_BASE)

#define MEC2016_GPIO_CTRL   ((MEC2016_GPIO_CTRL_TypeDef *) MEC2016_GPIO_CTRL_BASE)

#define MEC2016_GPIO_LOCK   ((volatile uint32_t *) MEC2016_GPIO_LOCK_BASE)

#define MEC2016_GPIO_PIO    ((MEC2016_GPIO_PIO_TypeDef *) MEC2016_GPIO_PIO_BASE)

#define MEC2016_GPIO_PIO_IN  ((volatile uint32_t*) MEC2016_GPIO_PIO_BASE)
#define MEC2016_GPIO_PIO_OUT ((volatile uint32_t*) MEC2016_GPIO_PIO_BASE + 0x80ul)

#define MEC2016_GPIO_CTRL2  ((MEC2016_GPIO_CTRL2_TypeDef *) MEC2016_GPIO_CTRL2_BASE)

#define MEC2016_EFUSE       ((MEC2016_EFUSE_TypeDef *) MEC2016_EFUSE_BASE)


/*------------- EC Host Segment --------------------*/

#define MEC2016_UART0       ((MEC2016_UARTR_TypeDef *) MEC2016_UART0_BASE)
#define MEC2016_UART0_ACT   *((volatile uint8_t *) ((MEC2016_UART0_BASE) + 0x330ul))
#define MEC2016_UART0_CFG   *((volatile uint8_t *) ((MEC2016_UART0_BASE) + 0x3F0ul))

#define MEC2016_UART1       ((MEC2016_UARTR_TypeDef *) MEC2016_UART1_BASE)
#define MEC2016_UART1_ACT   *((volatile uint8_t *) ((MEC2016_UART1_BASE) + 0x330ul))
#define MEC2016_UART1_CFG   *((volatile uint8_t *) ((MEC2016_UART1_BASE) + 0x3F0ul))

// eSPI
#define MEC2016_ESPI_PCH    ((MEC2016_ESPI_PCH_TypeDef*) MEC2016_ESPI_IO_BASE)
#define MEC2016_ESPI_PCEC   ((MEC2016_ESPI_PCEC_TypeDef*)(MEC2016_ESPI_IO_BASE + 0x0100ul))
#define MEC2016_ESPI_BCTRL  ((MEC2016_ESPI_BAR_CTRL_TypeDef*)(MEC2016_ESPI_IO_BASE + 0x0120ul))

#define MEC2016_ESPI_LTR    ((MEC2016_ESPI_LTR_TypeDef*) MEC2016_ESPI_LTR_BASE)
#define MEC2016_ESPI_OOB    ((MEC2016_ESPI_OOB_TypeDef*) MEC2016_ESPI_OOB_BASE)
#define MEC2016_ESPI_FC     ((MEC2016_ESPI_FC_TypeDef*) MEC2016_ESPI_FC_BASE)

#define MEC2016_ESPI_VWIRE_STATUS *((volatile uint8_t*)((MEC2016_ESPI_IO_BASE) + 0x02B0ul))

#define MEC2016_ESPI_CAP    ((MEC2016_ESPI_CAP_TypeDef*) MEC2016_ESPI_CAP_BASE)

#define MEC2016_ESPI_CFG    ((MEC2016_ESPI_CFG_TypeDef*)(MEC2016_ESPI_IO_BASE + 0x0330ul))

#define MEC2016_ESPI_SIRQ   ((MEC2016_ESPI_SIRQ_TypeDef*)(MEC2016_ESPI_IO_BASE + 0x03ACul))

#define MEC2016_ESPI_VWIRE_ERROR *((volatile uint8_t*)((MEC2016_ESPI_IO_BASE) + 0x03F0ul))

// MEC2016_ESPI_MEM_BASE @ 0x400F_3800
#define MEC2016_ESPI_MBAR_EC ((MEC2016_ESPI_MBAR_EC_TypeDef*)((MEC2016_ESPI_MEM_BASE) + 0x0130ul))

#define MEC2016_ESPI_MEM_SRAM ((MEC2016_ESPI_MEM_SRAM_TypeDef*)((MEC2016_ESPI_MEM_BASE) + 0x01F8ul))

#define MEC2016_ESPI_MEM_BM ((MEC2016_ESPI_MEM_BM_TypeDef*)((MEC2016_ESPI_MEM_BASE) + 0x0200ul))

#define MEC2016_ESPI_MBAR_HOST ((MEC2016_ESPI_MBAR_HOST_TypeDef*)((MEC2016_ESPI_MEM_BASE) + 0x0330ul))

#define MEC2016_ESPI_MEM_HSRAM ((MEC2016_ESPI_MEM_HSRAM_TypeDef*)((MEC2016_ESPI_MEM_BASE) + 0x03ACul))

// MEC2016_ESPI_VW_BASE @ 0x400F_9C00

#define MEC2016_ESPI_MSVW ((MEC2016_ESPI_MSVW_TypeDef*)(MEC2016_ESPI_VW_BASE))

#define MEC2016_ESPI_SMVW ((MEC2016_ESPI_SMVW_TypeDef*)((MEC2016_ESPI_VW_BASE) + 0x0200ul))

// end eSPI

#define MEC2016_HGCFG       ((MEC2016_HGCFG_TypeDef *) MEC2016_GLOBAL_CFG_BASE)

// SMBus
#define MEC2016_SMB0        ((SMB0_Type               *) MEC2016_SMB0_BASE)
#define MEC2016_SMB1        ((SMB0_Type               *) MEC2016_SMB1_BASE)
#define MEC2016_SMB2        ((SMB0_Type               *) MEC2016_SMB2_BASE)
#define MEC2016_SMB3        ((SMB0_Type               *) MEC2016_SMB3_BASE)

/*------------- Other ---------------------------*/
#define MEC2016_DELAY_REG   *((volatile uint8_t*) MEC2016_DELAY_REG_BASE)


/*@}*/ /* end of group <Device>_PeripheralDecl */

/*@}*/ /* end of group <Device>_Definitions */

#ifdef __cplusplus
}
#endif

#endif  /* MEC2016_H */

