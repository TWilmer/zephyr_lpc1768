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
 * @file SoC configuration macros for the NXP_LPC176X family processors.
 */

/*
 * @brief Basic CMSIS include file for LPC175x/6x
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#ifndef _NXP_LPC176X_SOC_H_
#define _NXP_LPC176X_SOC_H_




#ifndef _ASMLANGUAGE

#include <device.h>
#include <misc/util.h>
#include <drivers/rand32.h>

#include "soc_registers.h"


typedef enum  {
#if defined(CHIP_LPC175X_6X)
	SOC_SYSCTL_CLOCK_RSVD0,
#else
	SOC_SYSCTL_CLOCK_LCD,					/*!< LCD clock */
#endif
	SOC_SYSCTL_CLOCK_TIMER0,			/*!< Timer 0 clock */
	SOC_SYSCTL_CLOCK_TIMER1,			/*!< Timer 1 clock */
	SOC_SYSCTL_CLOCK_UART0,				/*!< UART 0 clock */
	SOC_SYSCTL_CLOCK_UART1,				/*!< UART 1 clock */
#if defined(CHIP_LPC175X_6X)
	SOC_SYSCTL_CLOCK_RSVD5,
#else
	SSOC_YSCTL_CLOCK_PWM0,				/*!< PWM0 clock */
#endif
	SOC_SYSCTL_CLOCK_PWM1,				/*!< PWM1 clock */
	SOC_SYSCTL_CLOCK_I2C0,				/*!< I2C0 clock */
#if defined(CHIP_LPC175X_6X)
	SOC_SYSCTL_CLOCK_SPI,					/*!< SPI clock */
#else
	SOC_SYSCTL_CLOCK_UART4,				/*!< UART 4 clock */
#endif
	SOC_SYSCTL_CLOCK_RTC,					/*!< RTC clock */
	SOC_SYSCTL_CLOCK_SSP1,				/*!< SSP1 clock */
#if defined(CHIP_LPC175X_6X)
	SOC_SYSCTL_CLOCK_RSVD11,
#else
	SOC_SYSCTL_CLOCK_EMC,					/*!< EMC clock */
#endif
	SOC_SYSCTL_CLOCK_ADC,					/*!< ADC clock */
	SOC_SYSCTL_CLOCK_CAN1,				/*!< CAN1 clock */
	SOC_SYSCTL_CLOCK_CAN2,				/*!< CAN2 clock */
	SOC_SYSCTL_CLOCK_GPIO,				/*!< GPIO clock */
#if defined(CHIP_LPC175X_6X)
	SOC_SYSCTL_CLOCK_RIT,				/*!< RIT clock */
#else
	SOC_SYSCTL_CLOCK_SPIFI,				/*!< SPIFI clock */
#endif
	SOC_SYSCTL_CLOCK_MCPWM,				/*!< MCPWM clock */
	SOC_SYSCTL_CLOCK_QEI,					/*!< QEI clock */
	SOC_SYSCTL_CLOCK_I2C1,				/*!< I2C1 clock */
#if defined(CHIP_LPC175X_6X)
	SOC_SYSCTL_CLOCK_RSVD20,
#else
	SOC_SYSCTL_CLOCK_SSP2,				/*!< SSP2 clock */
#endif
	SOC_SYSCTL_CLOCK_SSP0,				/*!< SSP0 clock */
	SOC_SYSCTL_CLOCK_TIMER2,			/*!< Timer 2 clock */
	SOC_SYSCTL_CLOCK_TIMER3,			/*!< Timer 3 clock */
	SOC_SYSCTL_CLOCK_UART2,				/*!< UART 2 clock */
	SOC_SYSCTL_CLOCK_UART3,				/*!< UART 3 clock */
	SOC_SYSCTL_CLOCK_I2C2,				/*!< I2C2 clock */
	SOC_SYSCTL_CLOCK_I2S,					/*!< I2S clock */
#if defined(CHIP_LPC175X_6X)
	SOC_SYSCTL_CLOCK_RSVD28,
#else
	SOC_SYSCTL_CLOCK_SDC,				/*!< SD Card interface clock */
#endif
	SOC_SYSCTL_CLOCK_GPDMA,				/*!< GP DMA clock */
	SOC_SYSCTL_CLOCK_ENET,				/*!< EMAC/Ethernet clock */
	SOC_SYSCTL_CLOCK_USB,					/*!< USB clock */
	SOC_SYSCTL_CLOCK_RSVD32,
	SOC_SYSCTL_CLOCK_RSVD33,
	SOC_SYSCTL_CLOCK_RSVD34,
#if defined(CHIP_LPC40XX)
	SOC_SYSCTL_CLOCK_CMP,				/*!< Comparator clock (PCONP1) */
#else
	SOC_SYSCTL_CLOCK_RSVD35,
#endif
} soc_clocksignals_t;



/**
 * Clock and power peripheral clock divider rates used with the
 * Clock_CLKDIVSEL_T clock types (devices only)
 */
typedef enum {
	SOC_SYSCTL_CLKDIV_4,			/*!< Divider by 4 */
	SOC_SYSCTL_CLKDIV_1,			/*!< Divider by 1 */
	SOC_SYSCTL_CLKDIV_2,			/*!< Divider by 2 */
	SOC_SYSCTL_CLKDIV_8,			/*!< Divider by 8, not for use with CAN */
	SOC_SYSCTL_CLKDIV_6_CCAN = SOC_SYSCTL_CLKDIV_8	/*!< Divider by 6, CAN only */
} soc_sysctl_clkdiv_t ;

/**
 * Peripheral clock selection for LPC175x/6x
 * This is a list of clocks that can be divided on the 175x/6x
 */
typedef enum {
	SOC_SYSCTL_PCLK_WDT,		/*!< Watchdog divider */
	SOC_SYSCTL_PCLK_TIMER0,	/*!< Timer 0 divider */
	SOC_SYSCTL_PCLK_TIMER1,	/*!< Timer 1 divider */
	SOC_SYSCTL_PCLK_UART0,	/*!< UART 0 divider */
	SOC_SYSCTL_PCLK_UART1,	/*!< UART 1 divider */
	SOC_SYSCTL_PCLK_RSVD5,
	SOC_SYSCTL_PCLK_PWM1,		/*!< PWM 1 divider */
	SOC_SYSCTL_PCLK_I2C0,		/*!< I2C 0 divider */
	SOC_SYSCTL_PCLK_SPI,		/*!< SPI divider */
	SOC_SYSCTL_PCLK_RSVD9,
	SOC_SYSCTL_PCLK_SSP1,		/*!< SSP 1 divider */
	SOC_SYSCTL_PCLK_DAC,		/*!< DAC divider */
	SOC_SYSCTL_PCLK_ADC,		/*!< ADC divider */
	SOC_SYSCTL_PCLK_CAN1,		/*!< CAN 1 divider */
	SOC_SYSCTL_PCLK_CAN2,		/*!< CAN 2 divider */
	SOC_SYSCTL_PCLK_ACF,		/*!< ACF divider */
	SOC_SYSCTL_PCLK_QEI,		/*!< QEI divider */
	SOC_SYSCTL_PCLK_GPIOINT,	/*!< GPIOINT divider */
	SOC_SYSCTL_PCLK_PCB,		/*!< PCB divider */
	SOC_SYSCTL_PCLK_I2C1,		/*!< I2C 1 divider */
	SOC_SYSCTL_PCLK_RSVD20,
	SOC_SYSCTL_PCLK_SSP0,		/*!< SSP 0 divider */
	SOC_SYSCTL_PCLK_TIMER2,	/*!< Timer 2 divider */
	SOC_SYSCTL_PCLK_TIMER3,	/*!< Timer 3 divider */
	SOC_SYSCTL_PCLK_UART2,	/*!< UART 2 divider */
	SOC_SYSCTL_PCLK_UART3,	/*!< UART 3 divider */
	SOC_SYSCTL_PCLK_I2C2,		/*!< I2C 2 divider */
	SOC_SYSCTL_PCLK_I2S,		/*!< I2S divider */
	SOC_SYSCTL_PCLK_RSVD28,
	SOC_SYSCTL_PCLK_RIT,		/*!< Repetitive timer divider */
	SOC_SYSCTL_PCLK_SYSCON,	/*!< SYSCON divider */
	SOC_SYSCTL_PCLK_MCPWM		/*!< Motor control PWM divider */
} soc_sysctl_pclk_t ;

extern void Chip_SystemInit(void);
extern uint32_t Soc_Clock_GetPeripheralClockRate(soc_sysctl_pclk_t clk);
extern void Soc_Clock_EnablePeriphClock(soc_clocksignals_t clock);

#endif /* !_ASMLANGUAGE */

#endif /* _ATMEL_SAM3_SOC_H_ */
