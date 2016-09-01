/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright (c) 2013-2015 Wind River Systems, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file SoC configuration macros for the Atmel SAM3 family processors.
 */

#ifndef _ATMEL_SAM3_SOC_H_
#define _ATMEL_SAM3_SOC_H_

/* IRQ numbers (from section 9.1, Peripheral Identifiers). */
#define IRQ_SUPC	0	/* Supply Controller                    */
#define IRQ_RSTC	1	/* Reset Controller                     */
#define IRQ_RTC		2	/* Real-time Clock                      */
#define IRQ_RTT		3	/* Real-time Timer                      */
#define IRQ_WDG		4	/* Watchdog Timer                       */
#define IRQ_PMC		5	/* Power Management Controller          */
#define IRQ_EEFC0	6	/* Enhanced Embedded Flash Controller 0 */
#define IRQ_EEFC1	7	/* Enhanced Embedded Flash Controller 1 */
#define IRQ_UART	8	/* UART                                 */
#define IRQ_PIOA	11	/* Parallel IO Controller A             */
#define IRQ_PIOB	12	/* Parallel IO Controller B             */
#define IRQ_PIOC	13	/* Parallel IO Controller C             */
#define IRQ_PIOD	14	/* Parallel IO Controller D             */
#define IRQ_PIOE	15	/* Parallel IO Controller E             */
#define IRQ_PIOF	16	/* Parallel IO Controller F             */
#define IRQ_USART0	17	/* USART #0                             */
#define IRQ_USART1	18	/* USART #1                             */
#define IRQ_USART2	19	/* USART #2                             */
#define IRQ_USART3	20	/* USART #3                             */
#define IRQ_HSMCI	21	/* High Speed Multimedia Card Interface */
#define IRQ_TWI0	22	/* Two-wire Interface #0                */
#define IRQ_TWI1	23	/* Two-wire Interface #1                */ #define IRQ_SPI0	24	/* SPI #0                               */ #define IRQ_SPI1	25	/* SPI #1                               */
#define IRQ_SSC		26	/* Synchronous Serial Controller        */
#define IRQ_TC0		27	/* Timer Counter Channel #0             */
#define IRQ_TC1		28	/* Timer Counter Channel #1             */
#define IRQ_TC2		29	/* Timer Counter Channel #2             */
#define IRQ_TC3		30	/* Timer Counter Channel #3             */
#define IRQ_TC4		31	/* Timer Counter Channel #4             */
#define IRQ_TC5		32	/* Timer Counter Channel #5             */
#define IRQ_TC6		33	/* Timer Counter Channel #6             */
#define IRQ_TC7		34	/* Timer Counter Channel #7             */
#define IRQ_TC8		35	/* Timer Counter Channel #8             */
#define IRQ_PWM		36	/* PWM Controller                       */
#define IRQ_ADC		37	/* ADC Controller                       */
#define IRQ_DACC	38	/* DAC Controller                       */
#define IRQ_DMAC	39	/* DMA Controller                       */
#define IRQ_UOTGHS	40	/* USB OTG High Speed                   */
#define IRQ_TRNG	41	/* True Random Number Generator         */
#define IRQ_EMAC	42	/* Ehternet MAC                         */
#define IRQ_CAN0	43	/* CAN Controller #0                    */
#define IRQ_CAN1	44	/* CAN Controller #1                    */


/*---------------------------------------------------------------------------*/
/** \name SYS_CTRL_CLOCK_CTRL register bit masks
 * @{
 */
#define SYS_CTRL_CLOCK_CTRL_OSC32K_CALDIS   0x02000000
#define SYS_CTRL_CLOCK_CTRL_OSC32K          0x01000000
#define SYS_CTRL_CLOCK_CTRL_AMP_DET         0x00200000
#define SYS_CTRL_CLOCK_CTRL_OSC_PD          0x00020000
#define SYS_CTRL_CLOCK_CTRL_OSC             0x00010000
#define SYS_CTRL_CLOCK_CTRL_IO_DIV          0x00000700
#define SYS_CTRL_CLOCK_CTRL_SYS_DIV         0x00000007
/** @} */
/*---------------------------------------------------------------------------*/
/** \name SYS_CTRL_CLOCK_STA register bit masks
 * @{
 */
#define SYS_CTRL_CLOCK_STA_SYNC_32K         0x04000000
#define SYS_CTRL_CLOCK_STA_OSC32K_CALDIS    0x02000000
#define SYS_CTRL_CLOCK_STA_OSC32K           0x01000000
#define SYS_CTRL_CLOCK_STA_RST              0x00C00000
#define SYS_CTRL_CLOCK_STA_RST_S            22
#define SYS_CTRL_CLOCK_STA_RST_POR          0
#define SYS_CTRL_CLOCK_STA_RST_EXT          1
#define SYS_CTRL_CLOCK_STA_RST_WDT          2
#define SYS_CTRL_CLOCK_STA_RST_CLD_SW       3
#define SYS_CTRL_CLOCK_STA_SOURCE_CHANGE    0x00100000
#define SYS_CTRL_CLOCK_STA_XOSC_STB         0x00080000
#define SYS_CTRL_CLOCK_STA_HSOSC_STB        0x00040000
#define SYS_CTRL_CLOCK_STA_OSC_PD           0x00020000
#define SYS_CTRL_CLOCK_STA_OSC              0x00010000
#define SYS_CTRL_CLOCK_STA_IO_DIV           0x00000700
#define SYS_CTRL_CLOCK_STA_RTCLK_FREQ       0x00000018
#define SYS_CTRL_CLOCK_STA_SYS_DIV          0x00000007
/** @} */
/*---------------------------------------------------------------------------*/
/** \name SYS_CTRL_RCGCGPT register bit masks
 * @{
 */
#define SYS_CTRL_RCGCGPT_GPT3   0x00000008  /**< GPT3 clock enable, CPU running */
#define SYS_CTRL_RCGCGPT_GPT2   0x00000004  /**< GPT2 clock enable, CPU running */
#define SYS_CTRL_RCGCGPT_GPT1   0x00000002  /**< GPT1 clock enable, CPU running */
#define SYS_CTRL_RCGCGPT_GPT0   0x00000001  /**< GPT0 clock enable, CPU running */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name SYS_CTRL_SCGCGPT register bit masks
 * @{
 */
#define SYS_CTRL_SCGCGPT_GPT3   0x00000008  /**< GPT3 clock enable, CPU IDLE */
#define SYS_CTRL_SCGCGPT_GPT2   0x00000004  /**< GPT2 clock enable, CPU IDLE */
#define SYS_CTRL_SCGCGPT_GPT1   0x00000002  /**< GPT1 clock enable, CPU IDLE */
#define SYS_CTRL_SCGCGPT_GPT0   0x00000001  /**< GPT0 clock enable, CPU IDLE */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name SYS_CTRL_DCGCGPT register bit masks
 * @{
 */
#define SYS_CTRL_DCGCGPT_GPT3   0x00000008  /**< GPT3 clock enable, PM0 */
#define SYS_CTRL_DCGCGPT_GPT2   0x00000004  /**< GPT2 clock enable, PM0 */
#define SYS_CTRL_DCGCGPT_GPT1   0x00000002  /**< GPT1 clock enable, PM0 */
#define SYS_CTRL_DCGCGPT_GPT0   0x00000001  /**< GPT0 clock enable, PM0 */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name SYS_CTRL_SRGPT register bits
 * @{
 */
#define SYS_CTRL_SRGPT_GPT3     0x00000008  /**< GPT3 is reset */
#define SYS_CTRL_SRGPT_GPT2     0x00000004  /**< GPT2 is reset */
#define SYS_CTRL_SRGPT_GPT1     0x00000002  /**< GPT1 is reset */
#define SYS_CTRL_SRGPT_GPT0     0x00000001  /**< GPT0 is reset */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name SYS_CTRL_RCGCSEC register bit masks
 * @{
 */
#define SYS_CTRL_RCGCSEC_AES    0x00000002  /**< AES clock enable, CPU running */
#define SYS_CTRL_RCGCSEC_PKA    0x00000001  /**< PKA clock enable, CPU running */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name SYS_CTRL_SCGCSEC register bit masks
 * @{
 */
#define SYS_CTRL_SCGCSEC_AES    0x00000002  /**< AES clock enable, CPU IDLE */
#define SYS_CTRL_SCGCSEC_PKA    0x00000001  /**< PKA clock enable, CPU IDLE */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name SYS_CTRL_DCGCSEC register bit masks
 * @{
 */
#define SYS_CTRL_DCGCSEC_AES    0x00000002  /**< AES clock enable, PM0 */
#define SYS_CTRL_DCGCSEC_PKA    0x00000001  /**< PKA clock enable, PM0 */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name SYS_CTRL_SRSEC register bits
 * @{
 */
#define SYS_CTRL_SRSEC_AES      0x00000002  /**< AES is reset */
#define SYS_CTRL_SRSEC_PKA      0x00000001  /**< PKA is reset */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name SYS_CTRL_PWRDBG register bits
 * @{
 */
#define SYS_CTRL_PWRDBG_FORCE_WARM_RESET    0x00000008
/** @} */
/*---------------------------------------------------------------------------*/
/** \name Possible values for the SYS_CTRL_CLOCK_CTRL_SYS_DIV bits
 * @{
 */
#define SYS_CTRL_CLOCK_CTRL_SYS_DIV_32MHZ   0x00000000
#define SYS_CTRL_CLOCK_CTRL_SYS_DIV_16MHZ   0x00000001
#define SYS_CTRL_CLOCK_CTRL_SYS_DIV_8MHZ    0x00000002
#define SYS_CTRL_CLOCK_CTRL_SYS_DIV_4MHZ    0x00000003
#define SYS_CTRL_CLOCK_CTRL_SYS_DIV_2MHZ    0x00000004
#define SYS_CTRL_CLOCK_CTRL_SYS_DIV_1MHZ    0x00000005
#define SYS_CTRL_CLOCK_CTRL_SYS_DIV_500KHZ  0x00000006
#define SYS_CTRL_CLOCK_CTRL_SYS_DIV_250KHZ  0x00000007
/** @} */
/*---------------------------------------------------------------------------*/
/** \name Possible values for the SYS_CTRL_CLOCK_CTRL_IO_DIV bits
 * @{
 */
#define SYS_CTRL_CLOCK_CTRL_IO_DIV_32MHZ    0x00000000
#define SYS_CTRL_CLOCK_CTRL_IO_DIV_16MHZ    0x00000100
#define SYS_CTRL_CLOCK_CTRL_IO_DIV_8MHZ     0x00000200
#define SYS_CTRL_CLOCK_CTRL_IO_DIV_4MHZ     0x00000300
#define SYS_CTRL_CLOCK_CTRL_IO_DIV_2MHZ     0x00000400
#define SYS_CTRL_CLOCK_CTRL_IO_DIV_1MHZ     0x00000500
#define SYS_CTRL_CLOCK_CTRL_IO_DIV_500KHZ   0x00000600
#define SYS_CTRL_CLOCK_CTRL_IO_DIV_250KHZ   0x00000700
/** @} */
/*---------------------------------------------------------------------------*/
/** \name SYS_CTRL_RCGCUART Register Bit-Masks
 * @{
 */
#define SYS_CTRL_RCGCUART_UART1             0x00000002  /**< UART1 Clock, CPU running */
#define SYS_CTRL_RCGCUART_UART0             0x00000001  /**< UART0 Clock, CPU running */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name SYS_CTRL_SCGCUART Register Bit-Masks
 * @{
 */
#define SYS_CTRL_SCGCUART_UART1             0x00000002  /**< UART1 Clock, CPU IDLE */
#define SYS_CTRL_SCGCUART_UART0             0x00000001  /**< UART0 Clock, CPU IDLE */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name SYS_CTRL_RCGCUART Register Bit-Masks
 * @{
 */
#define SYS_CTRL_DCGCUART_UART1             0x00000002  /**< UART1 Clock, PM0 */
#define SYS_CTRL_DCGCUART_UART0             0x00000001  /**< UART0 Clock, PM0 */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name SYS_CTRL_SRUART register bits
 * @{
 */
#define SYS_CTRL_SRUART_UART1               0x00000002  /**< UART1 module is reset */
#define SYS_CTRL_SRUART_UART0               0x00000001  /**< UART0 module is reset  */
/** @} */
/*---------------------------------------------------------------------------*/
/** \name SYS_CTRL_PMCTL register values
 * @{
 */
#define SYS_CTRL_PMCTL_PM3                  0x00000003  /**< PM3 */
#define SYS_CTRL_PMCTL_PM2                  0x00000002  /**< PM2 */
#define SYS_CTRL_PMCTL_PM1                  0x00000001  /**< PM1 */
#define SYS_CTRL_PMCTL_PM0                  0x00000000  /**< PM0 */
/** @} */



#define SYS_CTRL_ADDR 	0x400D2000

#ifndef _ASMLANGUAGE

#include <device.h>
#include <misc/util.h>
#include <drivers/rand32.h>

#include "soc_registers.h"



/* SC Register struct */
#define __SC		((volatile struct __sc *)SYS_CTRL_ADDR)

#endif /* !_ASMLANGUAGE */

#endif /* _ATMEL_SAM3_SOC_H_ */
