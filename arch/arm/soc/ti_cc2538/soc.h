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
 * @file SoC configuration macros for the TI CC2538 processors.
 */

#ifndef _TI_CC2538_SOC_H_
#define _TI_CC2538_SOC_H_

/* IRQ numbers (from table-5.2, section 5.1.2, Exception Types). 	*/
#define IRQ_GPIO_A			0	/* GPIO Port A  			*/
#define IRQ_GPIO_B			1	/* GPIO Port B  			*/
#define IRQ_GPIO_C			2	/* GPIO Port C  			*/
#define IRQ_GPIO_D			3	/* GPIO Port D  			*/
#define IRQ_UART0			5	/* Uart0   				*/
#define IRQ_UART1			6	/* Uart1 					*/
#define IRQ_SSI0			7	/* SSI0 					*/
#define IRQ_I2C				8	/* I2C   					*/
#define IRQ_ADC 			14	/* ADC  					*/
#define IRQ_WDT				18	/* Watchdog Timer   		*/
#define IRQ_GPT0A			19	/* GPTimer 0A      			*/
#define IRQ_GPT0B			20	/* GPTimer 0B      			*/
#define IRQ_GPT1A			21	/* GPTimer 1A      			*/
#define IRQ_GPT1B			22	/* GPTimer 1B      			*/
#define IRQ_GPT2A			23	/* GPTimer 2A      			*/
#define IRQ_GPT2B			24	/* GPTimer 2B      			*/
#define IRQ_ANALOG			25	/* Analog comparator  		*/
#define IRQ_RF_TXRX			26	/* RF TX/RX(Alternate)		*/
#define IRQ_RF_ERR 			27	/* RF Error(Alternate) 		*/
#define IRQ_SYSCTL			28	/* System Control 			*/
#define IRQ_FLASH			29	/* Flash memory control	*/
#define IRQ_AES_ALT			30	/* ATS(Alternate) 			*/
#define IRQ_PKA_ALT			31	/* PKA(Alterante) 			*/
#define IRQ_SM_ALT			32	/* SM Timer(Alternate)		*/
#define IRQ_MAC_ALT			33	/* MAC Timer(Alternate)	*/
#define IRQ_SSI1			34	/* SSI1  					*/
#define IRQ_GPT3A			35	/* GPTimer 3A     			*/
#define IRQ_GPT3B			36	/* GPTimer 3B      			*/
#define	IRQ_DMA_SW			46	/* DMA Software 			*/
#define IRQ_DMA_ERR			47 	/* DMA Error  				*/
#define IRQ_USB 			140	/* USB 					*/
#define IRQ_RF_CORE_RXTX	141	/* RF Core Rx/Tx 			*/
#define IRQ_RF_CORE_Err		142	/* RF Core Error 			*/
#define IRQ_AES				143	/* AES 					*/
#define IRQ_PKA 			144	/* PKA 					*/
#define IRQ_SM 				145	/* SM Timer 				*/
#define IRQ_MAC 			146	/* MAC Timer 				*/

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

#define UART0_ADDR			0x4000C000
#define UART1_ADDR			0x4000D000
#define SYS_CTRL_ADDR 		0x400D2000
#define IOC_ADDR			0x400D4000
#define GPIO_A_ADDR			0x400D9000
#define GPIO_B_ADDR			0x400DA000
#define GPIO_C_ADDR			0x400DB000
#define GPIO_D_ADDR			0x400DC000

#define IOC_PXX_SEL_UART0_TXD       0x00000000
#define IOC_PXX_SEL_UART1_RTS       0x00000001
#define IOC_PXX_SEL_UART1_TXD       0x00000002
#define IOC_PXX_SEL_SSI0_TXD        0x00000003
#define IOC_PXX_SEL_SSI0_CLKOUT     0x00000004
#define IOC_PXX_SEL_SSI0_FSSOUT     0x00000005
#define IOC_PXX_SEL_SSI0_STXSER_EN  0x00000006
#define IOC_PXX_SEL_SSI1_TXD        0x00000007
#define IOC_PXX_SEL_SSI1_CLKOUT     0x00000008
#define IOC_PXX_SEL_SSI1_FSSOUT     0x00000009
#define IOC_PXX_SEL_SSI1_STXSER_EN  0x0000000A
#define IOC_PXX_SEL_I2C_CMSSDA      0x0000000B
#define IOC_PXX_SEL_I2C_CMSSCL      0x0000000C
#define IOC_PXX_SEL_GPT0_ICP1       0x0000000D
#define IOC_PXX_SEL_GPT0_ICP2       0x0000000E
#define IOC_PXX_SEL_GPT1_ICP1       0x0000000F
#define IOC_PXX_SEL_GPT1_ICP2       0x00000010
#define IOC_PXX_SEL_GPT2_ICP1       0x00000011
#define IOC_PXX_SEL_GPT2_ICP2       0x00000012
#define IOC_PXX_SEL_GPT3_ICP1       0x00000013
#define IOC_PXX_SEL_GPT3_ICP2       0x00000014

/** \name Values for IOC_PXX_OVER
 * @{
 */
#define IOC_OVERRIDE_OE   0x00000008    /**< Output Enable */
#define IOC_OVERRIDE_PUE  0x00000004    /**< Pull Up Enable */
#define IOC_OVERRIDE_PDE  0x00000002    /**< Pull Down Enable */
#define IOC_OVERRIDE_ANA  0x00000001    /**< Analog Enable */
#define IOC_OVERRIDE_DIS  0x00000000    /**< Override Disabled */
/** @} */

#ifndef _ASMLANGUAGE

#include <device.h>
#include <misc/util.h>
#include <drivers/rand32.h>

#include "soc_registers.h"

/* SC Register struct */
#define __SC		((volatile struct __sc *)SYS_CTRL_ADDR)

/* IOC Register struct */
#define __IOC 		((volatile struct __ioc *)IOC_ADDR)

/* GPIO Register struct */
#define __GPIOA 	((volatile struct __gpio *)GPIO_A_ADDR)
#define __GPIOB		((volatile struct __gpio *)GPIO_B_ADDR)
#define __GPIOC		((volatile struct __gpio *)GPIO_C_ADDR)
#define __GPIOD		((volatile struct __gpio *)GPIO_D_ADDR)

#endif /* !_ASMLANGUAGE */

#endif /* _ATMEL_SAM3_SOC_H_ */
