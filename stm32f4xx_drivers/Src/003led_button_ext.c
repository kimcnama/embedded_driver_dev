/*
 * 003led_button_ext.c
 *
 *  Created on: 7 Dec 2022
 *      Author: kieranmc
 */


#include "stm32f407xx.h"

#define HIGH		ENABLE
#define LOW			0
#define BTN_PRESSED	LOW

void delay(void) {
	for (uint32_t i = 0; i < 500000 / 2; ++i);
}

int main(void) {

	GPIO_Handle_t GpioLed, GPIObtn;

	GpioLed.pGPIOx = GPIOA;

	GpioLed.GPIO_PingConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PingConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PingConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PingConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PingConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PingConfig.GPIO_PinAltFunMode = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	GPIObtn.pGPIOx = GPIOB;
	GPIObtn.GPIO_PingConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIObtn.GPIO_PingConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIObtn.GPIO_PingConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIObtn.GPIO_PingConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GPIObtn);

	while (1) {
		if (GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == BTN_PRESSED) {
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
		}
	}

	return 0;
}
