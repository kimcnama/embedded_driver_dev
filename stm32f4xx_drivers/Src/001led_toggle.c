/*
 * 001led_toggle.c
 *
 *  Created on: 30 Nov 2022
 *      Author: kieranmc
 */

#include "stm32f407xx.h"

void delay(void) {
	for (uint32_t i = 0; i < 500000; ++i);
}

int main(void) {

	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PingConfig.GPIO_PinNumber = GPIO_PIN_NO_12;

	GpioLed.GPIO_PingConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PingConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GpioLed.GPIO_PingConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PingConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PingConfig.GPIO_PinAltFunMode = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	while (1) {
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}

	return 0;
}
