/*
 * * Copyright (c) 2016 Thorsten Wilmer
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
 * @brief Driver for the UART of the NXP LPC 1xxx series - tested on LPC1768
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

typedef struct {					/*!< USARTn Structure       */

	union {
		volatile uint32_t  DLL;			/*!< Divisor Latch LSB. Least significant byte of the baud rate divisor value. The full divisor is used to generate a baud rate from the fractional rate divider (DLAB = 1). */
		volatile uint32_t  THR;			/*!< Transmit Holding Register. The next character to be transmitted is written here (DLAB = 0). */
		volatile uint32_t  RBR;			/*!< Receiver Buffer Register. Contains the next received character to be read (DLAB = 0). */
	};

	union {
		volatile  uint32_t IER;			/*!< Interrupt Enable Register. Contains individual interrupt enable bits for the 7 potential UART interrupts (DLAB = 0). */
		volatile uint32_t DLM;			/*!< Divisor Latch MSB. Most significant byte of the baud rate divisor value. The full divisor is used to generate a baud rate from the fractional rate divider (DLAB = 1). */
	};

	union {
		volatile 	  uint32_t FCR;			/*!< FIFO Control Register. Controls UART FIFO usage and modes. */
		volatile uint32_t IIR;			/*!< Interrupt ID Register. Identifies which interrupt(s) are pending. */
	};

	volatile uint32_t LCR;				/*!< Line Control Register. Contains controls for frame formatting and break generation. */
	volatile uint32_t MCR;				/*!< Modem Control Register. Only present on USART ports with full modem support. */
	volatile uint32_t LSR;				/*!< Line Status Register. Contains flags for transmit and receive status, including line errors. */
	volatile uint32_t MSR;				/*!< Modem Status Register. Only present on USART ports with full modem support. */
	volatile uint32_t SCR;				/*!< Scratch Pad Register. Eight-bit temporary storage for software. */
	volatile uint32_t ACR;				/*!< Auto-baud Control Register. Contains controls for the auto-baud feature. */
	volatile uint32_t ICR;				/*!< IrDA control register (not all UARTS) */
	volatile uint32_t FDR;				/*!< Fractional Divider Register. Generates a clock input for the baud rate divider. */
	volatile uint32_t OSR;				/*!< Oversampling Register. Controls the degree of oversampling during each bit time. Only on some UARTS. */
	volatile uint32_t TER1;				/*!< Transmit Enable Register. Turns off USART transmitter for use with software flow control. */
	volatile uint32_t  RESERVED0[3];
	volatile uint32_t HDEN;				/*!< Half-duplex enable Register- only on some UARTs */
	volatile uint32_t RESERVED1[1];
	volatile uint32_t SCICTRL;			/*!< Smart card interface control register- only on some UARTs */

	volatile  uint32_t RS485CTRL;		/*!< RS-485/EIA-485 Control. Contains controls to configure various aspects of RS-485/EIA-485 modes. */
	volatile  uint32_t RS485ADRMATCH;	/*!< RS-485/EIA-485 address match. Contains the address match value for RS-485/EIA-485 mode. */
	volatile uint32_t RS485DLY;			/*!< RS-485/EIA-485 direction control delay. */

	union {
		volatile 	 uint32_t SYNCCTRL;		/*!< Synchronous mode control register. Only on USARTs. */
		volatile   uint32_t FIFOLVL;		/*!< FIFO Level register. Provides the current fill levels of the transmit and receive FIFOs. */
	};

	volatile uint32_t TER2;				/*!< Transmit Enable Register. Only on LPC177X_8X UART4 and LPC18XX/43XX USART0/2/3. */
} _uart;

/* Device data structure */
struct uart_nxp_lpc1xxx_dev_data_t {
	uint32_t baud_rate;	/* Baud rate */
};

#define LPC_UART0_BASE            0x4000C000
#define LPC_UART1_BASE            0x40010000
#define LPC_UART2_BASE            0x40098000
#define LPC_UART3_BASE            0x4009C000

#define LPC_UART0                 ((_uart            *) LPC_UART0_BASE)
#define LPC_UART1                 ((_uart            *) LPC_UART1_BASE)
#define LPC_UART2                 ((_uart            *) LPC_UART2_BASE)
#define LPC_UART3                 ((_uart            *) LPC_UART3_BASE)

/**
 * @brief Macro defines for UART Receive Buffer register
 */
#define UART_RBR_MASKBIT    (0xFF)		        /*!< UART Received Buffer mask bit (8 bits) */

/**
 * @brief Macro defines for UART Divisor Latch LSB register
 */
#define UART_LOAD_DLL(div)  ((div) & 0xFF)		/*!< Macro for loading LSB of divisor */
#define UART_DLL_MASKBIT    (0xFF)	            /*!< Divisor latch LSB bit mask */

/**
 * @brief Macro defines for UART Divisor Latch MSB register
 */
#define UART_LOAD_DLM(div)  (((div) >> 8) & 0xFF)	/*!< Macro for loading MSB of divisors */
#define UART_DLM_MASKBIT    (0xFF)		            /*!< Divisor latch MSB bit mask */

/**
 * @brief Macro defines for UART Interrupt Enable Register
 */
#define UART_IER_RBRINT      (1 << 0)	/*!< RBR Interrupt enable */
#define UART_IER_THREINT     (1 << 1)	/*!< THR Interrupt enable */
#define UART_IER_RLSINT      (1 << 2)	/*!< RX line status interrupt enable */
#define UART_IER_MSINT       (1 << 3)	/*!< Modem status interrupt enable - valid for 11xx, 17xx/40xx UART1, 18xx/43xx UART1  only */
#define UART_IER_CTSINT      (1 << 7)	/*!< CTS signal transition interrupt enable - valid for 17xx/40xx UART1, 18xx/43xx UART1 only */
#define UART_IER_ABEOINT     (1 << 8)	/*!< Enables the end of auto-baud interrupt */
#define UART_IER_ABTOINT     (1 << 9)	/*!< Enables the auto-baud time-out interrupt */
#define UART_IER_BITMASK     (0x307)	/*!< UART interrupt enable register bit mask  - valid for 13xx, 17xx/40xx UART0/2/3, 18xx/43xx UART0/2/3 only*/
#define UART1_IER_BITMASK    (0x30F)	/*!< UART1 interrupt enable register bit mask - valid for 11xx only */
#define UART2_IER_BITMASK    (0x38F)	/*!< UART2 interrupt enable register bit mask - valid for 17xx/40xx UART1, 18xx/43xx UART1 only */

/**
 * @brief Macro defines for UART Interrupt Identification Register
 */
#define UART_IIR_INTSTAT_PEND   (1 << 0)	/*!< Interrupt pending status - Active low */
#define UART_IIR_FIFO_EN        (3 << 6)	/*!< These bits are equivalent to FCR[0] */
#define UART_IIR_ABEO_INT       (1 << 8)	/*!< End of auto-baud interrupt */
#define UART_IIR_ABTO_INT       (1 << 9)	/*!< Auto-baud time-out interrupt */
#define UART_IIR_BITMASK        (0x3CF)		/*!< UART interrupt identification register bit mask */

/* Interrupt ID bit definitions */
#define UART_IIR_INTID_MASK     (7 << 1)	/*!< Interrupt identification: Interrupt ID mask */
#define UART_IIR_INTID_RLS      (3 << 1)	/*!< Interrupt identification: Receive line interrupt */
#define UART_IIR_INTID_RDA      (2 << 1)	/*!< Interrupt identification: Receive data available interrupt */
#define UART_IIR_INTID_CTI      (6 << 1)	/*!< Interrupt identification: Character time-out indicator interrupt */
#define UART_IIR_INTID_THRE     (1 << 1)	/*!< Interrupt identification: THRE interrupt */
#define UART_IIR_INTID_MODEM    (0 << 1)	/*!< Interrupt identification: Modem interrupt */

/**
 * @brief Macro defines for UART FIFO Control Register
 */
#define UART_FCR_FIFO_EN        (1 << 0)	/*!< UART FIFO enable */
#define UART_FCR_RX_RS          (1 << 1)	/*!< UART RX FIFO reset */
#define UART_FCR_TX_RS          (1 << 2)	/*!< UART TX FIFO reset */
#define UART_FCR_DMAMODE_SEL    (1 << 3)	/*!< UART DMA mode selection - valid for 17xx/40xx, 18xx/43xx only */
#define UART_FCR_BITMASK        (0xCF)		/*!< UART FIFO control bit mask */

#define UART_TX_FIFO_SIZE       (16)

/* FIFO trigger level bit definitions */
#define UART_FCR_TRG_LEV0       (0)			/*!< UART FIFO trigger level 0: 1 character */
#define UART_FCR_TRG_LEV1       (1 << 6)	/*!< UART FIFO trigger level 1: 4 character */
#define UART_FCR_TRG_LEV2       (2 << 6)	/*!< UART FIFO trigger level 2: 8 character */
#define UART_FCR_TRG_LEV3       (3 << 6)	/*!< UART FIFO trigger level 3: 14 character */

/**
 * @brief Macro defines for UART Line Control Register
 */
/* UART word length select bit definitions */
#define UART_LCR_WLEN_MASK      (3 << 0)		/*!< UART word length select bit mask */
#define UART_LCR_WLEN5          (0 << 0)		/*!< UART word length select: 5 bit data mode */
#define UART_LCR_WLEN6          (1 << 0)		/*!< UART word length select: 6 bit data mode */
#define UART_LCR_WLEN7          (2 << 0)		/*!< UART word length select: 7 bit data mode */
#define UART_LCR_WLEN8          (3 << 0)		/*!< UART word length select: 8 bit data mode */

/* UART Stop bit select bit definitions */
#define UART_LCR_SBS_MASK       (1 << 2)		/*!< UART stop bit select: bit mask */
#define UART_LCR_SBS_1BIT       (0 << 2)		/*!< UART stop bit select: 1 stop bit */
#define UART_LCR_SBS_2BIT       (1 << 2)		/*!< UART stop bit select: 2 stop bits (in 5 bit data mode, 1.5 stop bits) */

/* UART Parity enable bit definitions */
#define UART_LCR_PARITY_EN      (1 << 3)		/*!< UART Parity Enable */
#define UART_LCR_PARITY_DIS     (0 << 3)		/*!< UART Parity Disable */
#define UART_LCR_PARITY_ODD     (0 << 4)		/*!< UART Parity select: Odd parity */
#define UART_LCR_PARITY_EVEN    (1 << 4)		/*!< UART Parity select: Even parity */
#define UART_LCR_PARITY_F_1     (2 << 4)		/*!< UART Parity select: Forced 1 stick parity */
#define UART_LCR_PARITY_F_0     (3 << 4)		/*!< UART Parity select: Forced 0 stick parity */
#define UART_LCR_BREAK_EN       (1 << 6)		/*!< UART Break transmission enable */
#define UART_LCR_DLAB_EN        (1 << 7)		/*!< UART Divisor Latches Access bit enable */
#define UART_LCR_BITMASK        (0xFF)			/*!< UART line control bit mask */

/**
 * @brief Macro defines for UART Modem Control Register
 */
#define UART_MCR_DTR_CTRL       (1 << 0)		/*!< Source for modem output pin DTR */
#define UART_MCR_RTS_CTRL       (1 << 1)		/*!< Source for modem output pin RTS */
#define UART_MCR_LOOPB_EN       (1 << 4)		/*!< Loop back mode select */
#define UART_MCR_AUTO_RTS_EN    (1 << 6)		/*!< Enable Auto RTS flow-control */
#define UART_MCR_AUTO_CTS_EN    (1 << 7)		/*!< Enable Auto CTS flow-control */
#define UART_MCR_BITMASK        (0xD3)			/*!< UART bit mask value */

/**
 * @brief Macro defines for UART Line Status Register
 */
#define UART_LSR_RDR        (1 << 0)	/*!< Line status: Receive data ready */
#define UART_LSR_OE         (1 << 1)	/*!< Line status: Overrun error */
#define UART_LSR_PE         (1 << 2)	/*!< Line status: Parity error */
#define UART_LSR_FE         (1 << 3)	/*!< Line status: Framing error */
#define UART_LSR_BI         (1 << 4)	/*!< Line status: Break interrupt */
#define UART_LSR_THRE       (1 << 5)	/*!< Line status: Transmit holding register empty */
#define UART_LSR_TEMT       (1 << 6)	/*!< Line status: Transmitter empty */
#define UART_LSR_RXFE       (1 << 7)	/*!< Line status: Error in RX FIFO */
#define UART_LSR_TXFE       (1 << 8)	/*!< Line status: Error in RX FIFO */
#define UART_LSR_BITMASK    (0xFF)		/*!< UART Line status bit mask */
#define UART1_LSR_BITMASK   (0x1FF)		/*!< UART1 Line status bit mask - valid for 11xx, 18xx/43xx UART0/2/3 only */

/**
 * @brief Macro defines for UART Modem Status Register
 */
#define UART_MSR_DELTA_CTS      (1 << 0)	/*!< Modem status: State change of input CTS */
#define UART_MSR_DELTA_DSR      (1 << 1)	/*!< Modem status: State change of input DSR */
#define UART_MSR_LO2HI_RI       (1 << 2)	/*!< Modem status: Low to high transition of input RI */
#define UART_MSR_DELTA_DCD      (1 << 3)	/*!< Modem status: State change of input DCD */
#define UART_MSR_CTS            (1 << 4)	/*!< Modem status: Clear To Send State */
#define UART_MSR_DSR            (1 << 5)	/*!< Modem status: Data Set Ready State */
#define UART_MSR_RI             (1 << 6)	/*!< Modem status: Ring Indicator State */
#define UART_MSR_DCD            (1 << 7)	/*!< Modem status: Data Carrier Detect State */
#define UART_MSR_BITMASK        (0xFF)		/*!< Modem status: MSR register bit-mask value */

/**
 * @brief Macro defines for UART Auto baudrate control register
 */
#define UART_ACR_START              (1 << 0)	/*!< UART Auto-baud start */
#define UART_ACR_MODE               (1 << 1)	/*!< UART Auto baudrate Mode 1 */
#define UART_ACR_AUTO_RESTART       (1 << 2)	/*!< UART Auto baudrate restart */
#define UART_ACR_ABEOINT_CLR        (1 << 8)	/*!< UART End of auto-baud interrupt clear */
#define UART_ACR_ABTOINT_CLR        (1 << 9)	/*!< UART Auto-baud time-out interrupt clear */
#define UART_ACR_BITMASK            (0x307)		/*!< UART Auto Baudrate register bit mask */

/**
 * Autobaud modes
 */
#define UART_ACR_MODE0              (0)	/*!< Auto baudrate Mode 0 */
#define UART_ACR_MODE1              (1)	/*!< Auto baudrate Mode 1 */

/**
 * @brief Macro defines for UART RS485 Control register
 */
#define UART_RS485CTRL_NMM_EN       (1 << 0)	/*!< RS-485/EIA-485 Normal Multi-drop Mode (NMM) is disabled */
#define UART_RS485CTRL_RX_DIS       (1 << 1)	/*!< The receiver is disabled */
#define UART_RS485CTRL_AADEN        (1 << 2)	/*!< Auto Address Detect (AAD) is enabled */
#define UART_RS485CTRL_SEL_DTR      (1 << 3)	/*!< If direction control is enabled (bit DCTRL = 1), pin DTR is
												        used for direction control */
#define UART_RS485CTRL_DCTRL_EN     (1 << 4)	/*!< Enable Auto Direction Control */
#define UART_RS485CTRL_OINV_1       (1 << 5)	/*!< This bit reverses the polarity of the direction
												       control signal on the RTS (or DTR) pin. The direction control pin
												       will be driven to logic "1" when the transmitter has data to be sent */
#define UART_RS485CTRL_BITMASK      (0x3F)		/*!< RS485 control bit-mask value */

/**
 * @brief Macro defines for UART IrDA Control Register - valid for 11xx, 17xx/40xx UART0/2/3, 18xx/43xx UART3 only
 */
#define UART_ICR_IRDAEN         (1 << 0)			/*!< IrDA mode enable */
#define UART_ICR_IRDAINV        (1 << 1)			/*!< IrDA serial input inverted */
#define UART_ICR_FIXPULSE_EN    (1 << 2)			/*!< IrDA fixed pulse width mode */
#define UART_ICR_PULSEDIV(n)    ((n & 0x07) << 3)	/*!< PulseDiv - Configures the pulse when FixPulseEn = 1 */
#define UART_ICR_BITMASK        (0x3F)				/*!< UART IRDA bit mask */

/**
 * @brief Macro defines for UART half duplex register - ????
 */
#define UART_HDEN_HDEN          ((1 << 0))			/*!< enable half-duplex mode*/

/**
 * @brief Macro defines for UART Smart card interface Control Register - valid for 11xx, 18xx/43xx UART0/2/3 only
 */
#define UART_SCICTRL_SCIEN        (1 << 0)			/*!< enable asynchronous half-duplex smart card interface*/
#define UART_SCICTRL_NACKDIS      (1 << 1)			/*!< NACK response is inhibited*/
#define UART_SCICTRL_PROTSEL_T1   (1 << 2)			/*!< ISO7816-3 protocol T1 is selected*/
#define UART_SCICTRL_TXRETRY(n)   ((n & 0x07) << 5)	/*!< number of retransmission*/
#define UART_SCICTRL_GUARDTIME(n) ((n & 0xFF) << 8)	/*!< Extra guard time*/

/**
 * @brief Macro defines for UART Fractional Divider Register
 */
#define UART_FDR_DIVADDVAL(n)   (n & 0x0F)			/*!< Baud-rate generation pre-scaler divisor */
#define UART_FDR_MULVAL(n)      ((n << 4) & 0xF0)	/*!< Baud-rate pre-scaler multiplier value */
#define UART_FDR_BITMASK        (0xFF)				/*!< UART Fractional Divider register bit mask */

/**
 * @brief Macro defines for UART Tx Enable Register
 */
#define UART_TER1_TXEN      (1 << 7)		/*!< Transmit enable bit  - valid for 11xx, 13xx, 17xx/40xx only */
#define UART_TER2_TXEN      (1 << 0)		/*!< Transmit enable bit  - valid for 18xx/43xx only */

/**
 * @brief Macro defines for UART Synchronous Control Register - 11xx, 18xx/43xx UART0/2/3 only
 */
#define UART_SYNCCTRL_SYNC             (1 << 0)			/*!< enable synchronous mode*/
#define UART_SYNCCTRL_CSRC_MASTER      (1 << 1)  		/*!< synchronous master mode*/
#define UART_SYNCCTRL_FES              (1 << 2)			/*!< sample on falling edge*/
#define UART_SYNCCTRL_TSBYPASS         (1 << 3)			/*!< to be defined*/
#define UART_SYNCCTRL_CSCEN            (1 << 4)			/*!< Continuous running clock enable (master mode only)*/
#define UART_SYNCCTRL_STARTSTOPDISABLE (1 << 5)	        /*!< Do not send start/stop bit*/
#define UART_SYNCCTRL_CCCLR            (1 << 6)			/*!< stop continuous clock*/

/* convenience defines */
#define DEV_CFG(dev) \
	((struct uart_device_config * const)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct uart_nxp_lpc1xxx_dev_data_t * const)(dev)->driver_data)
#define UART_STRUCT(dev) \
	((volatile struct _uart *)(DEV_CFG(dev))->base)

static struct uart_device_config uart_lpc_dev_cfg_0 = {
#if CONFIG_UART0_NXP_NUMBER == 0
	.base = (uint8_t *)LPC_UART0,
#elif	CONFIG_UART0_NXP_NUMBER == 1
	.base = (uint8_t *)LPC_UART1,
#elif	CONFIG_UART0_NXP_NUMBER == 2
	.base = (uint8_t *)LPC_UART2,
#elif	CONFIG_UART0_NXP_NUMBER == 3
	.base = (uint8_t *)LPC_UART3,
#else
#error "Unsupported UART Number"
#endif
	.sys_clk_freq = CONFIG_UART_NXP_LPC1XXX_CLK_FREQ,
};

static struct uart_nxp_lpc1xxx_dev_data_t uart_lpc_dev_data_0 = {
	.baud_rate = CONFIG_UART_NXP_LPC1XXX_BAUD_RATE,
};


static int uart_lpc_poll_in(struct device *dev, unsigned char *c)
{
	volatile struct _uart *uart = UART_STRUCT(dev);


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
static unsigned char uart_lpc_poll_out(struct device *dev,
					     unsigned char c)
{
	volatile  _uart *uart = UART_STRUCT(dev);


	while (((uart->LSR) & UART_LSR_THRE) == 0)
		{}
	uart->THR = (uint32_t) c;
}


static struct uart_driver_api uart_lpc_driver_api = {
	.poll_in = uart_lpc_poll_in,
	.poll_out = uart_lpc_poll_out,
};


static soc_clocksignals_t LPC_UART_GetClockIndex(_uart *pUART)
{
	soc_clocksignals_t clkUART;

	if (pUART == LPC_UART1) {
		clkUART = SOC_SYSCTL_CLOCK_UART1;
	}
	else if (pUART == LPC_UART2) {
		clkUART = SOC_SYSCTL_CLOCK_UART2;
	}
	else if (pUART == LPC_UART3) {
		clkUART = SOC_SYSCTL_CLOCK_UART3;
	}
#if defined(CHIP_LPC177X_8X) || defined(CHIP_LPC40XX)
	else if (pUART == LPC_UART4) {
		clkUART = SOC_SYSCTL_CLOCK_UART4;
	}
#endif
	else {
		clkUART = SOC_SYSCTL_CLOCK_UART0;
	}
	clkUART = SOC_SYSCTL_CLOCK_UART3;
	return clkUART;
}


static soc_sysctl_pclk_t Chip_UART_GetClkIndex(_uart *pUART)
{
	soc_sysctl_pclk_t clkUART;

	if (pUART == LPC_UART1) {
		clkUART = SOC_SYSCTL_PCLK_UART1;
	}
	else if (pUART == LPC_UART2) {
		clkUART = SOC_SYSCTL_PCLK_UART2;
	}
	else if (pUART == LPC_UART3) {
		clkUART = SOC_SYSCTL_PCLK_UART3;
	}
	else {
		clkUART = SOC_SYSCTL_PCLK_UART0;
	}

	return clkUART;
}

/**
 * @brief	Setup the UART FIFOs
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	fcr		: FIFO control register setup OR'ed flags
 * @return	Nothing
 * @note	Use OR'ed value of UART_FCR_* definitions with this function
 *			to select specific options. For example, to enable the FIFOs
 *			with a RX trip level of 8 characters, use something like
 *			(UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2)
 */
static  inline  void Chip_UART_SetupFIFOS(_uart *pUART, uint32_t fcr)
{
	pUART->FCR = fcr;
}

/**
 * @brief	Configure data width, parity and stop bits
 * @param	pUART	: Pointer to selected pUART peripheral
 * @param	config	: UART configuration, OR'ed values of UART_LCR_* defines
 * @return	Nothing
 * @note	Select OR'ed config options for the UART from the UART_LCR_*
 *			definitions. For example, a configuration of 8 data bits, 1
 *			stop bit, and even (enabled) parity would be
 *			(UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_EN | UART_LCR_PARITY_EVEN)
 */
static inline void Chip_UART_ConfigData(_uart *pUART, uint32_t config)
{
	pUART->LCR = config;
}

/* Determines and sets best dividers to get a target bit rate */
uint32_t Chip_UART_SetBaud(volatile _uart *pUART, uint32_t baudrate)
{
	uint32_t div, divh, divl, clkin;

	/* Determine UART clock in rate without FDR */

	clkin = Soc_Clock_GetPeripheralClockRate(Chip_UART_GetClkIndex(pUART));

	div = clkin / (baudrate * 16);

	/* High and low halves of the divider */
	divh = div / 256;
	divl = div - (divh * 256);
	pUART->LCR |= UART_LCR_DLAB_EN;
	pUART->DLL = (uint32_t) divl;
	pUART->DLM = (uint32_t) divh;

	pUART->LCR &= ~UART_LCR_DLAB_EN;

	/* Fractional FDR already setup for 1 in UART init */

	return clkin / div;
}

/**
* @brief Initialize UART channel
*
* This routine is called to reset the chip in a quiescent state.
* It is assumed that this function is called only once per UART.
*
* @param dev UART device struct
*
* @return DEV_OK
*/
static int uart_lpc_init(struct device *dev)
{
	volatile  _uart *uart = UART_STRUCT(dev);
	uint32_t tmp;


	ARG_UNUSED(tmp);

	Soc_Clock_EnablePeriphClock(LPC_UART_GetClockIndex(uart));
	Chip_UART_SetupFIFOS(uart, (UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS));

    /* Disable interrupts */
	uart->IER = 0;
	/* Set LCR to default state */
	uart->LCR = 0;
	/* Set ACR to default state */
	uart->ACR = 0;
    /* Set RS485 control to default state */
	uart->RS485CTRL = 0;
	/* Set RS485 delay timer to default state */
	uart->RS485DLY = 0;
	/* Set RS485 addr match to default state */
	uart->RS485ADRMATCH = 0;

    /* Clear MCR */
    if (uart == LPC_UART1) {
		/* Set Modem Control to default state */
    	uart->MCR = 0;
		/*Dummy Reading to Clear Status */
		tmp = uart->MSR;
	}

	/* Default 8N1, with DLAB disabled */
	Chip_UART_ConfigData(uart, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS));

	/* Disable fractional divider */
	uart->FDR = 0x10;

	dev->driver_api = &uart_lpc_driver_api;

	Chip_UART_SetBaud(uart, 115200);
}




DEVICE_INIT(uart_lpc1xxx_0, CONFIG_UART_NXP_LPC1XXX_NAME, &uart_lpc_init,
			&uart_lpc_dev_data_0, &uart_lpc_dev_cfg_0,
			PRIMARY, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
