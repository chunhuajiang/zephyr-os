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

struct __pads {
	uint32_t  pin0;
	uint32_t  pin1;
	uint32_t  pin2;
	uint32_t  pin3;
	uint32_t  pin4;
	uint32_t  pin5;
	uint32_t  pin6;
	uint32_t  pin7;
};

struct __sel_over {
	struct __pads pa;
	struct __pads pb;
	struct __pads pc;
	struct __pads pd;	
};

/* I/Os Control */
struct __ioc {
	struct __sel_over sel;   /* 0x000-0x07C for IOC_PXX_SEL   */
	struct __sel_over over;  /* 0x080-0x0FC for IOC_PXX_OVER */
	uint32_t  uartrxd_uart0; /* 0x100 */
	uint32_t  uartcts_uart1; /* 0x104 */
	uint32_t  uartrxd_uart1; /* 0x108 */
	uint32_t  clk_ssi_ssi0;  /* 0x10C */
	uint32_t  ssirxd_ssi0;   /* 0x110 */
	uint32_t  ssifssin_ssi0; /* 0x114 */
	uint32_t  clk_ssin_ssi0; /* 0x118 */
	uint32_t  clk_ssi_ssi1;  /* 0x11C */
	uint32_t  ssirxd_ssi1;   /* 0x120 */
	uint32_t  ssifssin_ssi1; /* 0x124 */
	uint32_t  clk_ssin_ssi1; /* 0x128 */
	uint32_t  i2cmssda;      /* 0x12C */
	uint32_t  i2cmsscl;      /* 0x130 */
	uint32_t  gpt0ocp1;      /* 0x134 */
	uint32_t  gpt0ocp2;      /* 0x138 */
	uint32_t  gpt1ocp1;      /* 0x13C */
	uint32_t  gpt1ocp2;      /* 0x140 */
	uint32_t  gpt2ocp1;      /* 0x144 */
	uint32_t  gpt2ocp2;      /* 0x148 */
	uint32_t  gpt3ocp1;      /* 0x14C */
	uint32_t  gpt3ocp2;      /* 0x150 */
};

/*  GPIO Control */
struct __gpio {
	uint32_t  data;	          /* 0x000 */
	uint32_t  rsvd_1[(0x400 - 0x004) / 4];
	uint32_t  dir;            /* 0x400  */
	uint32_t  is;             /* 0x404 */
	uint32_t  ibe;            /* 0x408 */
	uint32_t  iev;            /* 0x40C */
	uint32_t  ie;             /* 0x410 */
	uint32_t  ris;            /* 0x414 */
	uint32_t  mis;            /* 0x418 */
	uint32_t  ic;             /* 0x41C */
	uint32_t  afsel;          /* 0x420 */
	uint32_t  rsvd_2[(0x520 - 0x424) / 4];
	uint32_t  gpiolock;       /* 0x520 */
	uint32_t  gpiocr;         /* 0x524 */
	uint32_t  rsvd_3[(0x700 - 0x528) / 4];
	uint32_t  pmux;           /* 0x700 */
	uint32_t  p_edge_ctrl;    /* 0x704 */
	uint32_t  rsvd_4[(0x710 - 0x708) / 4];
	uint32_t  pi_ien;         /* 0x710 */
	uint32_t  rsvd_5[(0x718 - 0x714) / 4];
	uint32_t  irq_detect_ack; /* 0x718   */
	uint32_t  usb_irq_ack;    /* 0x71C */
	uint32_t  irq_detect_unmask; /* 0x720 */
};


#endif /* _TI_CC2538_SOC_REGS_H_ */
