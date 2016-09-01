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
 * @file
 * @brief System/hardware module for Atmel SAM3 family processor
 *
 * This module provides routines to initialize and support board-level hardware
 * for the Atmel SAM3 family processor.
 */

#include <nanokernel.h>
#include <device.h>
#include <init.h>
#include <soc.h>
#include <soc_registers.h>

#include <arch/cpu.h>

/**
 * @brief Setup various clock on SoC.
 *
 */
static ALWAYS_INLINE void clock_init(void)
{
	uint32_t val;

	/*
	* Desired Clock Ctrl configuration:
	* 32KHz source: crystal oscillator
 	* System Clock: 32 MHz
	* Power Down Unused Oscillator
 	* I/O Div: 16MHz
 	* Sys Div: 16MHz
 	* Rest: Don't care
 	*/
 	
	val = SYS_CTRL_CLOCK_CTRL_OSC_PD \
		| SYS_CTRL_CLOCK_CTRL_IO_DIV_16MHZ \
		| SYS_CTRL_CLOCK_CTRL_SYS_DIV_16MHZ;

	__SC->cc = val;

	while((__SC->cs 
		& (SYS_CTRL_CLOCK_STA_OSC32K | SYS_CTRL_CLOCK_STA_OSC)) != 0);
	
}

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */
static int ti_cc2538_init(struct device *arg)
{
	uint32_t key;

	ARG_UNUSED(arg);

	/* Note:
	 * Magic numbers below are obtained by reading the registers
	 * when the SoC was running the SAM-BA bootloader
	 * (with reserved bits set to 0).
	 */

	key = irq_lock();

	/* Setup the vector table offset register (VTOR),
	 * which is located at the beginning of flash area.
	 */
	_scs_relocate_vector_table((void *)CONFIG_FLASH_BASE_ADDRESS);


	/* Clear all faults */
	_ScbMemFaultAllFaultsReset();
	_ScbBusFaultAllFaultsReset();
	_ScbUsageFaultAllFaultsReset();

	_ScbHardFaultAllFaultsReset();

	/* Setup master clock */
	clock_init();

	/* Install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */
	NMI_INIT();

	irq_unlock(key);

	return 0;
}

SYS_INIT(ti_cc2538_init, PRIMARY, 0);
