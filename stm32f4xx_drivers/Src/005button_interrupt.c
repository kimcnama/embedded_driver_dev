/*
 * 005button_interrupt.c
 *
 *  Created on: 14 Dec 2022
 *      Author: kieranmc
 */

#include <string.h>
#include "stm32f407xx.h"

#define HIGH		ENABLE
#define LOW			0
#define BTN_PRESSED	LOW

void delay(void) {
	for (uint32_t i = 0; i < 500000 / 2; ++i);
}

int main(void) {

	GPIO_Handle_t GpioLed, GPIObtn;
	// init to 0 so now un-intended configuration done
	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GPIObtn, 0, sizeof(GPIObtn));

	GpioLed.pGPIOx = GPIOD;

	GpioLed.GPIO_PingConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PingConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PingConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PingConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PingConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PingConfig.GPIO_PinAltFunMode = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOD, ENABLE);
 	GPIO_Init(&GpioLed);

	GPIObtn.pGPIOx = GPIOD;
	GPIObtn.GPIO_PingConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIObtn.GPIO_PingConfig.GPIO_PinMode = GPIO_MODE_IT_FT; // interrupt mode falling edge
	GPIObtn.GPIO_PingConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIObtn.GPIO_PingConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIObtn);

	// IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	while (1);

	return 0;
}

void EXTI9_5_IRQHandler(void) {
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
