/*
 * 006spi_testing.c
 *
 *  Created on: 27 Dec 2022
 *      Author: kieranmc
 */

/* Pin configs:
 * PB14 -> SPI2_MISO
 * PB15 -> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 -> SPI2_NSS
 * ALT function mode: 5
 */

#include <string.h>
#include "stm32f407xx.h"

void SPI2_GPIOInits(void) {
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PingConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PingConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PingConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PingConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PingConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK
	SPIPins.GPIO_PingConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PingConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	// MISO
	SPIPins.GPIO_PingConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	// NSS
	SPIPins.GPIO_PingConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void) {
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;

	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; // 8 MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LO;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LO;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_SSM_EN;

	SPI_Init(&SPI2Handle);
}


int main(void) {

	char user_data[] = "Hello world";

	// Init GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// Init SPI2 peripheral parameters
	SPI2_Inits();

	// makes NSS signal internally high and avoids error
	SPI_SSIConfig(SPI2, ENABLE);

	// enable SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	// send data
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	return 0;
}
