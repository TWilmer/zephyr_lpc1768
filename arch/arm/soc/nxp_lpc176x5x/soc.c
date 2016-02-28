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

#ifdef CONFIG_RUNTIME_NMI
extern void _NmiInit(void);
#define NMI_INIT() _NmiInit()
#else
#define NMI_INIT()
#endif

#define CORE_M3
#define CHIP_LPC175X_6X

//#include "adc_17xx_40xx.c"
//#include "can_17xx_40xx.c"
//#include "chip_17xx_40xx.c"
#include "clock_17xx_40xx.c"
//#include "cmp_17xx_40xx.c"
//#include "crc_17xx_40xx.c"
//#include "dac_17xx_40xx.c"
//#include "eeprom_17xx_40xx.c"
//#include "emc_17xx_40xx.c"
//#include "enet_17xx_40xx.c"
//#include "gpdma_17xx_40xx.c"
//#include "gpio_17xx_40xx.c"
//#include "gpioint_17xx_40xx.c"
//#include "i2c_17xx_40xx.c"
//#include "i2s_17xx_40xx.c"
//#include "iocon_17xx_40xx.c"
//#include "lcd_17xx_40xx.c"
//#include "pmu_17xx_40xx.c"
//#include "ritimer_17xx_40xx.c"
//#include "rtc_17xx_40xx.c"
//#include "sdc_17xx_40xx.c"
//#include "sdmmc_17xx_40xx.c"
//#include "spi_17xx_40xx.c"
//#include "ssp_17xx_40xx.c"
//#include "stopwatch_17xx_40xx.c"
//#include "sysctl_17xx_40xx.c"
#include "sysinit_17xx_40xx.c"
//#include "timer_17xx_40xx.c"
//#include "uart_17xx_40xx.c"
//#include "wwdt_17xx_40xx.c"



/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */
static int nxp_lpc176x5x_init(struct device *arg)
{
	uint32_t key;


	ARG_UNUSED(arg);


	Chip_SystemInit();

	/* Install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */
	NMI_INIT();

	irq_unlock(key);

	return 0;
}


extern void Soc_Clock_EnablePeriphClock(soc_clocksignals_t clock)
{
	LPC_SYSCTL->PCONP |= (1 << clock);
}
uint32_t Soc_Chip_Clock_GetSystemClockRate(void)
{
	return Chip_Clock_GetMainClockRate() / Chip_Clock_GetCPUClockDiv();
}
/* Gets a clock divider for a peripheral */
uint32_t Soc_Chip_Clock_GetPCLKDiv(soc_sysctl_pclk_t clk)
{
	uint32_t div = 1, bitIndex, regIndex = ((uint32_t) clk) * 2;

	/* Get register array index and clock index into the register */
	bitIndex = regIndex % 32;
	regIndex = regIndex / 32;

	/* Mask and update register */
	div = LPC_SYSCTL->PCLKSEL[regIndex];
	div = (div >> bitIndex) & 0x3;
	if (div == SOC_SYSCTL_CLKDIV_4) {
		div = 4;
	}
	else if (div == SOC_SYSCTL_CLKDIV_1) {
		div = 1;
	}
	else if (div == SOC_SYSCTL_CLKDIV_2) {
		div = 2;
	}
	else {
		/* Special case for CAN clock divider */
		if ((clk == SOC_SYSCTL_PCLK_CAN1) || (clk == SOC_SYSCTL_PCLK_CAN2) || (clk == SOC_SYSCTL_PCLK_ACF)) {
			div = 6;
		}
		else {
			div = 8;
		}
	}

	return div;
}


uint32_t Soc_Clock_GetPeripheralClockRate(soc_sysctl_pclk_t clk) {
	/* 175x/6x clock is derived from CPU clock with CPU divider */
	return Soc_Chip_Clock_GetSystemClockRate() / Soc_Chip_Clock_GetPCLKDiv(clk);
}

SYS_INIT(nxp_lpc176x5x_init, PRIMARY, 0);
