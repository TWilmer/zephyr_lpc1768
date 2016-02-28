/*
 * Copyright (c) 2011-2015, Wind River Systems, Inc.
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

#include <nanokernel.h>
#include "board.h"
#include <uart.h>
#include <device.h>
#include <init.h>
#include <pinmux.h>
#include <soc.h>

#define LPC_GPIO0_BASE            0x2009C000
#define LPC_GPIO1_BASE            0x2009C020
#define LPC_GPIO2_BASE            0x2009C040
#define LPC_GPIO3_BASE            0x2009C060
#define LPC_GPIO4_BASE            0x2009C080
typedef struct {				/* GPIO_PORT Structure */
	volatile uint32_t DIR;			/*!< Offset 0x0000: GPIO Port Direction control register */
	uint32_t RESERVED0[3];
	volatile uint32_t MASK;			/*!< Offset 0x0010: GPIO Mask register */
	volatile uint32_t PIN;			/*!< Offset 0x0014: Pin value register using FIOMASK */
	volatile uint32_t SET;			/*!< Offset 0x0018: Output Set register using FIOMASK */
	volatile  uint32_t CLR;			/*!< Offset 0x001C: Output Clear register using FIOMASK */
} _gpio_t;

void setLed1()
{
	_gpio_t *gpio0=(_gpio_t*)LPC_GPIO0_BASE;

	gpio0->DIR|=1<<23;
	gpio0->SET|=1<<23;

}

#define IOCON_REG_INDEX(port, pin)		(2 * port + (pin / 16))
/* Bit position calculation in PINSEL and PINMODE register.*/
#define IOCON_BIT_INDEX(pin)			((pin % 16) * 2)

#define LPC_IOCON_BASE            0x4002C000


/**
 * @brief Array of IOCON pin definitions passed to Chip_IOCON_SetPinMuxing() must be in this format
 */
typedef struct {
	uint32_t pingrp:3;		/* Pin group */
	uint32_t pinnum:5;		/* Pin number */
	uint32_t modefunc:24;	/* Function and mode. */
} pinmux_grp_t;


pinmux_grp_t muxConfiguration[]={
		{0,25,3}, // TX3
		{0,26,3}, // TX3
		{0,23,0}, // TX3
};

typedef struct {

	volatile	 uint32_t PINSEL[11];
	volatile	uint32_t RESERVED0[5];
	volatile uint32_t PINMODE[10];
	volatile uint32_t PINMODE_OD[5];
	volatile uint32_t I2CPADCFG;

} _iocon_t;

struct pinmux_config {
	volatile _iocon_t *pinumux_base;
};

struct pinmux_config board_pmux = {
		.pinumux_base = (volatile  _iocon_t *)LPC_IOCON_BASE,
};
/* convenience defines */
#define DEV_CFG(dev) \
	((struct pinmux_config * const)(dev)->config->config_info)
#define IOCON_STRUCT(dev) \
	((volatile struct _iocon_t *)(DEV_CFG(dev))->pinumux_base)



static uint32_t pinmux_dev_set(struct device *dev, uint32_t port_pin, uint8_t modefunc)
{
	volatile  _iocon_t *iocon = IOCON_STRUCT(dev);




	uint32_t mode=  modefunc >> 2;
	uint8_t func= modefunc & 3;
	uint8_t port=port_pin >> 8;
	uint8_t pin=port_pin & 0xFF;

	uint8_t reg, bitPos;
	uint32_t temp;

	bitPos =  IOCON_BIT_INDEX(pin);
	reg = IOCON_REG_INDEX(port,pin);

	temp = iocon->PINSEL[reg] & ~(0x03UL << bitPos);
	iocon->PINSEL[reg] = temp | (func << bitPos);

	temp = iocon->PINMODE[reg] & ~(0x03UL << bitPos);
	iocon->PINMODE[reg] = temp | (mode << bitPos);


	return DEV_NOT_CONFIG;
}
/* Set all I/O Control pin muxing */
static void pinmux_dev_setMuxing(struct device *dev, const pinmux_grp_t* pinArray, uint32_t arrayLength)
{
	uint32_t ix;

	for (ix = 0; ix < arrayLength; ix++ ) {
		pinmux_dev_set(dev, pinArray[ix].pingrp <<8 | pinArray[ix].pinnum, pinArray[ix].modefunc);
	}
}
static uint32_t pinmux_dev_get(struct device *dev, uint32_t pin, uint8_t *func)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pin);
	ARG_UNUSED(func);



	return DEV_OK;
}



static uint32_t pinmux_dev_pullup(struct device *dev,
				  uint32_t pin,
				  uint8_t func)
{
	struct pinmux_config * const pmux = dev->config->config_info;



	return DEV_OK;
}
static uint32_t pinmux_dev_input(struct device *dev, uint32_t pin, uint8_t func)
{
	struct pinmux_config * const pmux = dev->config->config_info;


	return DEV_OK;
}

static struct pinmux_driver_api api_funcs = {
	.set = pinmux_dev_set,
	.get = pinmux_dev_get,
	.pullup = pinmux_dev_pullup,
	.input = pinmux_dev_input
};

 const uint32_t OscRateIn=4000000;

/**
 * @brief	RTC oscillator rate
 * This value is defined externally to the chip layer and contains
 * the value in Hz for the RTC oscillator for the board. This is
 * usually 32KHz (32768). If not using the RTC, this rate can be 0.
 */
 const uint32_t RTCOscRateIn=0;

int pinmux_initialize(struct device *port)
{
	volatile int i=0;
	setLed1();
	struct pinmux_config * const pmux = port->config->config_info;

	port->driver_api = &api_funcs;


	pinmux_dev_setMuxing(port,muxConfiguration, sizeof(muxConfiguration)/sizeof(pinmux_grp_t) );

	return DEV_OK;
}

DEVICE_INIT(pmux, PINMUX_NAME, &pinmux_initialize,
			NULL, &board_pmux,
			SECONDARY, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
