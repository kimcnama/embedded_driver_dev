/*
 * 002led_button.c
 *
 *  Created on: 4 Dec 2022
 *      Author: kieranmc
 */


#include "stm32f407xx.h"

#define HIGH		ENABLE
#define BTN_PRESSED	HIGH

void delay(void) {
	for (uint32_t i = 0; i < 500000 / 2; ++i);
}

int main(void) {

	GPIO_Handle_t GpioLed, GPIObtn;

	GpioLed.pGPIOx = GPIOD;

	GpioLed.GPIO_PingConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PingConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PingConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PingConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PingConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PingConfig.GPIO_PinAltFunMode = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	GPIObtn.pGPIOx = GPIOA;
	GPIObtn.GPIO_PingConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIObtn.GPIO_PingConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIObtn.GPIO_PingConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIObtn.GPIO_PingConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIObtn);

	while (1) {
		if (GPIO_ReadFromInputPin(GPIObtn.pGPIOx, GPIO_PIN_NO_0) == BTN_PRESSED) {
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}
	}

	return 0;
}
