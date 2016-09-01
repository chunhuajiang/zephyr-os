/*
 * Copyright (c) 2016 http://iot-fans.xyz
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
 * @file SoC configuration macros for the TI CC2538 platform.
 *
 * Refer to the user guide for more information about these registers.
 */

#ifndef _TI_CC2538_SOC_REGS_H_
#define _TI_CC2538_SOC_REGS_H_

/* System Control */
struct __sc {

	uint32_t  cc;		 /* 0x00 Clock Ctrl    				    		 */
	uint32_t  cs;		 /* 0x04 Clock Status   						 */
	
	struct {
		uint32_t  rcgc;  /* 0x08 Clock Gating Control in Run mode            */
		uint32_t  scgc;  /* 0x0C Clock Gating Control in Sleep mode         */
		uint32_t  dcgc;  /* 0x10 Clock Gating Control in Deep sleep mode */
		uint32_t  reset; /* 0x14 Reset Ctrl                                            */
	} gpt;

	struct {
		uint32_t  rcgc;  /* 0x18 Clock Gating Control in Run mode 	     */
		uint32_t  scgc;	 /* 0x1C Clock Gating Control in Sleep mode         */
		uint32_t  dcgc;	 /* 0x20 Clock Gating Control in Deep sleep mode */
		uint32_t  reset; /* 0x24 Reset Ctrl 						    */
	} ssi;

	struct {
		uint32_t  rcgc;  /* 0x28 Clock Gating Control in Run mode 	     */
		uint32_t  scgc;	 /* 0x2C Clock Gating Control in Sleep mode 	 */
		uint32_t  dcgc;	 /* 0x30 Clock Gating Control in Deep sleep mode */
		uint32_t  reset; /* 0x34 Reset Ctrl 							 */
	} uart;

	struct {
		uint32_t  rcgc;  /* 0x38 Clock Gating Control in Run mode 	     */
		uint32_t  scgc;	 /* 0x3C Clock Gating Control in Sleep mode 	 */
		uint32_t  dcgc;	 /* 0x40 Clock Gating Control in Deep sleep mode */
		uint32_t  reset; /* 0x44 Reset Ctrl 							 */
	} i2c;

	struct {
		uint32_t  rcgc;  /* 0x48 Clock Gating Control in Run mode 		 */
		uint32_t  scgc;	 /* 0x4C Clock Gating Control in Sleep mode 	 */
		uint32_t  dcgc;	 /* 0x50 Clock Gating Control in Deep sleep mode */
		uint32_t  reset; /* 0x54 Reset Ctrl 							 */
	} sec;

	uint32_t  pm; 		/* 0x58 Power Mode control register 			 */
	uint32_t  crc;		/* 0x5C CRC on state retention 					 */

	uint32_t  rsvd__60_73[(0x74 - 0x60) / 4];
	
	uint32_t  pwrdbg;   /* 0x74 Power Debug register 					 */

	uint32_t  rsvd__78_7f[(0x80 - 0x78) / 4];

	uint32_t  cld;		/* 0x80 Clock Loss Detection				 	 */

	uint32_t  rsvd__84_93[(0x94 - 0x84) / 4];

	uint32_t  iwe; 		/* 0x94 Interrupt Wark-Up 						 */
	uint32_t  imap;		/* 0x98 Interrupt Map 							 */

	uint32_t  rsvd__9c_a7[(0xa8 - 0x9c) / 4];

	struct {
		uint32_t  rcgc; /* 0xa8 Clock Gating Control in Run mode 		 */
		uint32_t  scgc;	/* 0xac Clock Gating Control in Sleep mode 	 	 */
		uint32_t  dcgc;	/* 0xb0 Clock Gating Control in Deep sleep mode  */
	} rfc;

	uint32_t  emuovr;    /* 0xB4 Emulator override control				 */
};

#endif /* _TI_CC2538_SOC_REGS_H_ */
