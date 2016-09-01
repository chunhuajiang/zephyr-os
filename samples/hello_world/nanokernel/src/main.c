/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
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

#include <zephyr.h>
#define SYS_LOG_LEVEL SYS_LOG_LEVEL_INFO
#include <misc/sys_log.h>

/*
 * @file
 * @brief Hello World demo
 * Nanokernel version of hello world demo
 */

void test_with_leds(void)
{
#define HWREG(x)  				(*((volatile uint32_t *)(x)))
		
#define LED_ADDR_DATA 			0x400DB000
#define LED_ADDR_DIR			0x400DB400
#define LED_ADDR_AFSEL			0x400DB420
		
#define IOC_ADDR_PC0_OVER       0x400D40C0  
#define IOC_ADDR_PC1_OVER       0x400D40C4  
#define IOC_ADDR_PC2_OVER       0x400D40C8  
#define IOC_ADDR_PC3_OVER       0x400D40CC  
		
#define IOC_OVERRIDE_DIS  		0x00000000    
#define LED_PIN_MASK			0x0000000f
		
		
	HWREG(LED_ADDR_DIR)   = LED_PIN_MASK;
	HWREG(LED_ADDR_AFSEL) = 0;
		
	HWREG(IOC_ADDR_PC0_OVER) = IOC_OVERRIDE_DIS;
	HWREG(IOC_ADDR_PC1_OVER) = IOC_OVERRIDE_DIS;
	HWREG(IOC_ADDR_PC2_OVER) = IOC_OVERRIDE_DIS;
	HWREG(IOC_ADDR_PC3_OVER) = IOC_OVERRIDE_DIS;
	HWREG(LED_ADDR_DATA + (LED_PIN_MASK << 2)) = 4;

}

void main(void)
{
	test_with_leds();
	SYS_LOG_INF("Hello World! %s", CONFIG_ARCH);
}

