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
 * @brief Driver for UART on Atmel cc2538 family processor.
 *
 * Note that there is only one UART controller on the SoC.
 * It has two wires for RX and TX, and does not have other such as
 * CTS or RTS. Also, the RX and TX are connected directly to
 * bit shifters and there is no FIFO.
 *
 * For full serial function, use the USART controller.
 *
 * (used uart_stellaris.c as template)
 */

#include <nanokernel.h>
#include <arch/cpu.h>
#include <misc/__assert.h>
#include <board.h>
#include <init.h>
#include <uart.h>
#include <sections.h>
#include <soc.h>
#include <misc/printk.h>
#include <misc/sys_log.h>


/* UART registers struct */
struct _cc2538_uart {
	/* UART registers */
	uint32_t	dr;	    /* 0x00 Data Register*/
	uint32_t	rsr_ecr;/* 0x04 Receive Status and Error Clear Register */
	uint32_t 	reserved1[(0x18 - 0x08) / 4];
	uint32_t	fr;	 	/* 0x18 Flag Register */
	uint32_t 	reserved2[(0x20 - 0x1c) / 4];
	uint32_t	ilpr;	/* 0x20 Interrupt Mask Register */
	uint32_t	ibrd;	/* 0x24 Status Register */
	uint32_t	fbrd;	/* 0x28 Receive Holding Register */
	uint32_t	lcrh;	/* 0x2C Transmit Holding Register */
	uint32_t	ctl;	/* 0x30 Baud Rate Generator Register */
	uint32_t	ifls;	/* 0x34 - 0xFF */
	uint32_t	im;		/* 0x38 Receive Pointer Reg */
	uint32_t	ris;	/* 0x3C Receive Counter Reg */
	uint32_t	mis;	/* 0x40 Transmit Pointer Reg */
	uint32_t	icr;	/* 0x44 Transmit Counter Reg */
	uint32_t	dmactl;	/* 0x48 Receive Next Pointer */
	uint32_t 	reserved3[(0x90 - 0x4c) / 4];
	uint32_t	lctl;	/* 0x90 Receive Next Counter */
	uint32_t	lss;	/* 0x94 Transmit Next Pointer */
	uint32_t	ltim;	/* 0x98 Transmit Next Counter */
	uint32_t 	reserved4[(0xa4 - 0x9c) / 4];
	uint32_t	ninebitaddr;	/* 0xa4 Transfer Control Reg */
	uint32_t	ninebitamask;	/* 0xa8 Transfer Status Reg */
	uint32_t 	reserved5[(0xfc0 - 0xac) / 4];
	uint32_t 	pp; 	/* 0xfc0 */
	uint32_t 	cc;		/* 0xfc8 */
};

/* Device data structure */
struct uart_cc2538_dev_data_t {
	uint32_t baud_rate;	/* Baud rate */

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
		uart_irq_callback_t 	cb; 	/**< Callback function pointer */
#endif

	
};

/* convenience defines */
#define DEV_CFG(dev) \
	((struct uart_device_config * const)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct uart_cc2538_dev_data_t * const)(dev)->driver_data)
#define UART_STRUCT(dev) \
	((volatile struct _cc2538_uart *)(DEV_CFG(dev))->base)

#define UART_IRQ_FLAGS 0

/* UART_FR Register Bit-Masks */
#define UART_FR_TXFE            0x00000080  /**< UART transmit FIFO empty */
#define UART_FR_RXFF            0x00000040  /**< UART receive FIFO full */
#define UART_FR_TXFF            0x00000020  /**< UART transmit FIFO full */
#define UART_FR_RXFE            0x00000010  /**< UART receive FIFO empty */
#define UART_FR_BUSY            0x00000008  /**< UART busy */
#define UART_FR_CTS             0x00000001  /**< Clear to send */

/* UART_IM Register Bit-Masks */
#define UART_IM_LME5IM          0x00008000  /**< LIN mode edge 5 intr mask */
#define UART_IM_LME1IM          0x00004000  /**< LIN mode edge 1 intr mask */
#define UART_IM_LMSBIM          0x00002000  /**< LIN mode sync break mask */
#define UART_IM_NINEBITIM       0x00001000  /**< 9-bit mode interrupt mask */
#define UART_IM_OEIM            0x00000400  /**< UART overrun error mask */
#define UART_IM_BEIM            0x00000200  /**< UART break error mask */
#define UART_IM_PEIM            0x00000100  /**< UART parity error mask */
#define UART_IM_FEIM            0x00000080  /**< UART framing error */
#define UART_IM_RTIM            0x00000040  /**< UART receive time-out mask */
#define UART_IM_TXIM            0x00000020  /**< UART transmit intr mask */
#define UART_IM_RXIM            0x00000010  /**< UART receive interrupt mask */
#define UART_IM_CTSIM           0x00000002  /**< UART CTS modem mask */

/* UART_RIS Register Bit-Masks */
#define UART_MIS_LME5MIS        0x00008000  /**< LIN mode edge 5 masked stat */
#define UART_MIS_LME1MIS        0x00004000  /**< LIN mode edge 1 masked stat */
#define UART_MIS_LMSBMIS        0x00002000  /**< LIN mode sync br masked stat */
#define UART_MIS_NINEBITMIS     0x00001000  /**< 9-bit mode masked stat */
#define UART_MIS_OEMIS          0x00000400  /**< UART overrun err masked stat */
#define UART_MIS_BEMIS          0x00000200  /**< UART break err masked stat */
#define UART_MIS_PEMIS          0x00000100  /**< UART parity err masked stat */
#define UART_MIS_FEMIS          0x00000080  /**< UART framing err masked stat */
#define UART_MIS_RTMIS          0x00000040  /**< UART RX time-out masked stat */
#define UART_MIS_TXMIS          0x00000020  /**< UART TX masked intr stat */
#define UART_MIS_RXMIS          0x00000010  /**< UART RX masked intr stat */
#define UART_MIS_CTSMIS         0x00000002  /**< UART CTS modem masked stat */


/* UART_ICR Register Bit-Masks */
#define UART_ICR_LME5IC         0x00008000  /**< LIN mode edge 5 intr clear */
#define UART_ICR_LME1IC         0x00004000  /**< LIN mode edge 1 intr clear */
#define UART_ICR_LMSBIC         0x00002000  /**< LIN mode sync br intr clear */
#define UART_ICR_NINEBITIC      0x00001000  /**< 9-bit mode intr clear */
#define UART_ICR_OEIC           0x00000400  /**< Overrun error intr clear */
#define UART_ICR_BEIC           0x00000200  /**< Break error intr clear */
#define UART_ICR_PEIC           0x00000100  /**< Parity error intr clear */
#define UART_ICR_FEIC           0x00000080  /**< Framing error intr clear */
#define UART_ICR_RTIC           0x00000040  /**< Receive time-out intr clear */
#define UART_ICR_TXIC           0x00000020  /**< Transmit intr clear */
#define UART_ICR_RXIC           0x00000010  /**< Receive intr clear */
#define UART_ICR_CTSIC          0x00000002  /**< UART CTS modem intr clear */

/*  UART_DMACTL Register Bit-Masks */
#define UART_DMACTL_DMAERR      0x00000004  /**< DMA on error */
#define UART_DMACTL_TXDMAE      0x00000002  /**< Transmit DMA enable */
#define UART_DMACTL_RXDMAE      0x00000001  /**< Receive DMA enable */

/* UART_LCRH Register Bit-Masks */
#define UART_LCRH_SPS           0x00000080  /**< UART stick parity select */
#define UART_LCRH_WLEN          0x00000060  /**< UART word length */
#define UART_LCRH_FEN           0x00000010  /**< UART enable FIFOs */
#define UART_LCRH_STP2          0x00000008  /**< UART two stop bits select */
#define UART_LCRH_EPS           0x00000004  /**< UART even parity select */
#define UART_LCRH_PEN           0x00000002  /**< UART parity enable */
#define UART_LCRH_BRK           0x00000001  /**< UART send break */


static struct uart_driver_api uart_cc2538_driver_api;

void print(char *msg, int len)
{
	int i = 0;

	for (i = 0; i < len; i++)
	{
			/* Wait for transmitter to be ready */
		while ((*((volatile int *)0x4000C018) & UART_FR_TXFF));
	
		/* send a character */
		*((volatile int *)0x4000C000) = (uint32_t)msg[i];
	}
}

/**
 * @brief Set the baud rate
 *
 * This routine set the given baud rate for the UART.
 *
 * @param dev UART device struct
 * @param baudrate Baud rate
 * @param sys_clk_freq_hz System clock frequency in Hz
 *
 * @return N/A
 */
static void baudrate_set(struct device *dev,
			 uint32_t baudrate, uint32_t sys_clk_freq_hz)
{
#if 0
	volatile struct _cc2538_uart *uart = UART_STRUCT(dev);
	struct uart_device_config * const dev_cfg = DEV_CFG(dev);
	struct uart_cc2538_dev_data_t * const dev_data = DEV_DATA(dev);
	uint32_t divisor; /* baud rate divisor */

	if ((baudrate != 0) && (dev_cfg->sys_clk_freq != 0)) {
		/* calculate baud rate divisor */
		divisor = (dev_cfg->sys_clk_freq / baudrate) >> 4;
		divisor &= 0xFFFF;

		uart->brgr = divisor;

		dev_data->baud_rate = baudrate;
	}
#endif
}

/**
 * @brief Initialize UART channel
 *
 * This routine is called to reset the chip in a quiescent state.
 * It is assumed that this function is called only once per UART.
 *
 * @param dev UART device struct
 *
 * @return 0
 */
static int uart_cc2538_init(struct device *dev)
{
	volatile struct _cc2538_uart *uart = UART_STRUCT(dev);


	/* Enable clock for the UART while Running, in Sleep and Deep Sleep */
#ifdef CONFIG_UART_CC2538_PORT_0
	if(dev->config->name == (char *)CONFIG_UART_CC2538_PORT_0_NAME) {
		__SC->uart.rcgc |= SYS_CTRL_RCGCUART_UART0;
		__SC->uart.scgc |= SYS_CTRL_SCGCUART_UART0;
		__SC->uart.dcgc |= SYS_CTRL_DCGCUART_UART0;

		/* Run on SYS_DIV */
		uart->cc = 0;


		__IOC->uartrxd_uart0 = 0;
		
		__IOC->sel.pa.pin1 = IOC_PXX_SEL_UART0_TXD;
		__IOC->over.pa.pin1 = IOC_OVERRIDE_OE;

	}
#endif
#ifdef CONFIG_UART_CC2538_PORT_1
	if (dev->config->name = (char *)CONFIG_UART_CC2538_PORT_1_NAME) {
		__SC->uart.rcgc |= SYS_CTRL_RCGCUART_UART1;
		__SC->uart.scgc |= SYS_CTRL_SCGCUART_UART1;
		__SC->uart.dcgc |= SYS_CTRL_DCGCUART_UART1;
	}
#endif


			__GPIOA->afsel |= 1 << 0;
			__GPIOA->afsel |= 1 << 1;
	
			uint32_t div;	
#define UART_CONFIG_WLEN_8                    0x00000060  // 8 bit data
#define UART_CONFIG_STOP_ONE                  0x00000000  // One stop bit
#define UART_CONFIG_PAR_NONE                  0x00000000  // No parity
			div = (((16000000 * 8) / 115200) + 1) / 2;
			uart->ibrd = div / 64;
			uart->fbrd = div % 64;
			uart->lcrh = UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE;;
	
#define UART_CTL_UARTEN                       0x00000001  // UART enable
#define UART_CTL_TXE                          0x00000100  // UART transmit enable
#define UART_CTL_RXE                          0x00000200  // UART receive enable
			uart->ctl |= UART_CTL_UARTEN;

	print("\r\nhello\r\n", 10);			
	return 0;
}

/**
 * @brief Poll the device for input.
 *
 * @param dev UART device struct
 * @param c Pointer to character
 *
 * @return 0 if a character arrived, -1 if the input buffer if empty.
 */
static int uart_cc2538_poll_in(struct device *dev, unsigned char *c)
{
	volatile struct _cc2538_uart *uart = UART_STRUCT(dev);

	if (uart->fr & UART_FR_RXFE)
		return (-1);
	
	/* got a character */
	*c = (unsigned char)uart->dr;

	return 0;
}

/**
 * @brief Output a character in polled mode.
 *
 * Checks if the transmitter is empty. If empty, a character is written to
 * the data register.
 *
 * @param dev UART device struct
 * @param c Character to send
 *
 * @return Sent character
 */
static unsigned char uart_cc2538_poll_out(struct device *dev,
					     unsigned char c)
{
	volatile struct _cc2538_uart *uart = UART_STRUCT(dev);

	/* Wait for transmitter to be ready */
	while (uart->fr & UART_FR_TXFF)
		;

	/* send a character */
	uart->dr = (uint32_t)c;
	return c;
}

#if CONFIG_UART_INTERRUPT_DRIVEN

/**
 * @brief Fill FIFO with data
 *
 * @param dev UART device struct
 * @param tx_data Data to transmit
 * @param len Number of bytes to send
 *
 * @return number of bytes sent
 */
static int uart_cc2538_fifo_fill(struct device *dev, const uint8_t *tx_data,
			      int len)
{
	volatile struct _cc2538_uart *uart = UART_STRUCT(dev);
	uint8_t num_tx = 0;
	print("uart_cc2538_fifo_fill__1\r\n", strlen("uart_cc2538_fifo_fill__1\r\n"));

	while ((len - num_tx > 0) /*&& !(uart->fr & UART_FR_TXFF)*/) {
		uart->dr = tx_data[num_tx++];
	}
	print("uart_cc2538_fifo_fill__2\r\n", strlen("uart_cc2538_fifo_fill__2\r\n"));
	//print(&(char)num_tx, 1);

	return num_tx;
}

/**
 * @brief Read data from FIFO
 *
 * @param dev UART device struct
 * @param rx_data Pointer to data container
 * @param size Container size in bytes
 *
 * @return number of bytes read
 */
static int uart_cc2538_fifo_read(struct device *dev, uint8_t *rx_data,
			      const int size)
{
	volatile struct _cc2538_uart *uart = UART_STRUCT(dev);
	uint8_t num_rx = 0;

	while ((size - num_rx > 0) && !(uart->dr & UART_FR_TXFE)) {
		rx_data[num_rx++] = uart->dr;
	}

	return num_rx;
}

/**
 * @brief Enable TX interrupt
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_cc2538_irq_tx_enable(struct device *dev)
{
	volatile struct _cc2538_uart *uart = UART_STRUCT(dev);

	uart->im |= UART_IM_TXIM;

	print("uart_cc2538_irq_tx_enable\r\n", strlen("uart_cc2538_irq_tx_enable\r\n"));
}

/**
 * @brief Disable TX interrupt in IER
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_cc2538_irq_tx_disable(struct device *dev)
{
	volatile struct _cc2538_uart *uart = UART_STRUCT(dev);

	uart->im &= ~UART_IM_TXIM;
}

/**
 * @brief Check if Tx IRQ has been raised
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is ready, 0 otherwise
 */
static int uart_cc2538_irq_tx_ready(struct device *dev)
{
	volatile struct _cc2538_uart *uart = UART_STRUCT(dev);

	if (uart->im & UART_IM_TXIM){
		return (uart->mis & UART_MIS_TXMIS) ? 1 : 0;
	} else {
		return 0;
	}
}

/**
 * @brief Enable RX interrupt in IER
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_cc2538_irq_rx_enable(struct device *dev)
{
	volatile struct _cc2538_uart *uart = UART_STRUCT(dev);

	uart->im |= UART_IM_RXIM;
}

/**
 * @brief Disable RX interrupt in IER
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_cc2538_irq_rx_disable(struct device *dev)
{
	volatile struct _cc2538_uart *uart = UART_STRUCT(dev);

	uart->im &= ~UART_IM_RXIM;
}

/**
 * @brief Check if Rx IRQ has been raised
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is ready, 0 otherwise
 */
static int uart_cc2538_irq_rx_ready(struct device *dev)
{
	volatile struct _cc2538_uart *uart = UART_STRUCT(dev);

	if (uart->im & UART_IM_RXIM){
		return (uart->mis & UART_MIS_RXMIS) ? 1 : 0;
	} else {
		return 0;
	}

}

/**
 * @brief Enable error interrupt
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_cc2538_irq_err_enable(struct device *dev)
{
#if 0
	volatile struct _cc2538_uart *uart = UART_STRUCT(dev);
	union C3 c3 = uart->c3;

	c3.field.parity_err_int_en = 1;
	c3.field.frame_err_int_en = 1;
	c3.field.noise_err_int_en = 1;
	c3.field.overrun_err_int_en = 1;
	uart->c3 = c3;
#endif
}

/**
 * @brief Disable error interrupt
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void uart_cc2538_irq_err_disable(struct device *dev)
{/*
	volatile struct _cc2538_uart *uart = UART_STRUCT(dev);
	union C3 c3 = uart->c3;

	c3.field.parity_err_int_en = 0;
	c3.field.frame_err_int_en = 0;
	c3.field.noise_err_int_en = 0;
	c3.field.overrun_err_int_en = 0;
	uart->c3 = c3;*/
}

/**
 * @brief Check if Tx or Rx IRQ is pending
 *
 * @param dev UART device struct
 *
 * @return 1 if a Tx or Rx IRQ is pending, 0 otherwise
 */
static int uart_cc2538_irq_is_pending(struct device *dev)
{
	return uart_cc2538_irq_tx_ready(dev) || uart_cc2538_irq_rx_ready(dev);
}

/**
 * @brief Update IRQ status
 *
 * @param dev UART device struct
 *
 * @return always 1
 */
static int uart_cc2538_irq_update(struct device *dev)
{
	return 1;
}

/**
 * @brief Set the callback function pointer for IRQ.
 *
 * @param dev UART device struct
 * @param cb Callback function pointer.
 *
 * @return N/A
 */
static void uart_cc2538_irq_callback_set(struct device *dev,
				      uart_irq_callback_t cb)
{
	struct uart_cc2538_dev_data_t * const dev_data = DEV_DATA(dev);

	dev_data->cb = cb;
}

/**
 * @brief Interrupt service routine.
 *
 * This simply calls the callback function, if one exists.
 *
 * @param arg Argument to ISR.
 *
 * @return N/A
 */
void uart_cc2538_isr(void *arg)
{
	struct device *dev = arg;
	struct uart_cc2538_dev_data_t * const dev_data = DEV_DATA(dev);

	if (dev_data->cb) {
		dev_data->cb(dev);
	}
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */


static struct uart_driver_api uart_cc2538_driver_api = {
	.poll_in = uart_cc2538_poll_in,
	.poll_out = uart_cc2538_poll_out,

	
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	
	.fifo_fill = uart_cc2538_fifo_fill,
	.fifo_read = uart_cc2538_fifo_read,
	.irq_tx_enable = uart_cc2538_irq_tx_enable,
	.irq_tx_disable = uart_cc2538_irq_tx_disable,
	.irq_tx_ready = uart_cc2538_irq_tx_ready,
	.irq_rx_enable = uart_cc2538_irq_rx_enable,
	.irq_rx_disable = uart_cc2538_irq_rx_disable,
	.irq_rx_ready = uart_cc2538_irq_rx_ready,
	.irq_err_enable = uart_cc2538_irq_err_enable,
	.irq_err_disable = uart_cc2538_irq_err_disable,
	.irq_is_pending = uart_cc2538_irq_is_pending,
	.irq_update = uart_cc2538_irq_update,
	.irq_callback_set = uart_cc2538_irq_callback_set,
	
#endif
};

#if CONFIG_UART_CC2538_PORT_0

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void irq_config_func_0(struct device *port);
#endif

static struct uart_device_config uart_cc2538_dev_cfg_0 = {
	.base = (uint8_t *)UART0_ADDR,
	//.sys_clk_freq = CONFIG_UART_ATMEL_cc2538_CLK_FREQ,

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = irq_config_func_0,
#endif
};

static struct uart_cc2538_dev_data_t uart_cc2538_dev_data_0 = {
	.baud_rate = CONFIG_UART_CC2538_PORT_0_BAUD_RATE,
};


DEVICE_AND_API_INIT(uart_cc2538_0, CONFIG_UART_CC2538_PORT_0_NAME, &uart_cc2538_init,
		    &uart_cc2538_dev_data_0, &uart_cc2538_dev_cfg_0,
		    PRIMARY, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &uart_cc2538_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void irq_config_func_0(struct device *dev)
{
	IRQ_CONNECT(IRQ_UART0,
		    CONFIG_UART_CC2538_PORT_0_IRQ_PRI,
		    uart_cc2538_isr, DEVICE_GET(uart_cc2538_0),
		    UART_IRQ_FLAGS);
	irq_enable(IRQ_UART0);
}
#endif

#endif

#if CONFIG_UART_CC2538_PORT_1

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void irq_config_func_1(struct device *port);
#endif

static struct uart_device_config uart_cc2538_dev_cfg_1 = {
	.base = (uint8_t *)UART1_ADDR,
	//.sys_clk_freq = CONFIG_UART_ATMEL_cc2538_CLK_FREQ,

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = irq_config_func_1,
#endif
};
			
static struct uart_cc2538_dev_data_t uart_cc2538_dev_data_1 = {
	.baud_rate = CONFIG_UART_CC2538_PORT_0_BAUD_RATE,
};

DEVICE_AND_API_INIT(uart_cc2538_1, CONFIG_UART_CC2538_PORT_1_NAME, &uart_cc2538_init,
						&uart_cc2538_dev_data_1, &uart_cc2538_dev_cfg_1,
						PRIMARY, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
						&uart_cc2538_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void irq_config_func_1(struct device *dev)
{
	IRQ_CONNECT(IRQ_UART1,
		    CONFIG_UART_CC2538_PORT_1_IRQ_PRI,
		    uart_cc2538_isr, DEVICE_GET(uart_cc2538_1),
		    UART_IRQ_FLAGS);
	irq_enable(IRQ_UART1);
}
#endif

#endif

