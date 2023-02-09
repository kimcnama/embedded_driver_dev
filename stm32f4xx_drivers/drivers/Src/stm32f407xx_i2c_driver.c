/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 21 Jan 2023
 *      Author: kieranmc
 */

#include "stm32f407xx.h"

uint16_t AHB_PreScalar[8] = {2,4,8,16,128,256,512};
uint8_t APB1_PreScalar[4] = {2,4,8,16};

/**************************************************
 * @fn			- I2C_PeripheralControl
 *
 * @ brief		- This function enables or disables peripheral clock for a given I2C port
 *
 * @param[in]	- Base address of I2C peripheral
 * @param[in]	- enable or disable peripheral clock
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_PeripheralControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/**************************************************
 * @fn			- I2C_PeriClockControl
 *
 * @ brief		- This function enables or disables peripheral clock for a given I2C port
 *
 * @param[in]	- Base address of I2C peripheral
 * @param[in]	- enable or disable peripheral clock
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_PeriClockControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}
	} else {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_DI();
		}
	}
}

uint32_t RCC_GetPLLOutputClk() {
	// Not implementing this func in this course
	return 0;
}

uint32_t RCC_GetPCLK1Value() {
	uint32_t pc1k1, sysclk;
	uint8_t clksrc, temp, ahbp, apb1p;
	clksrc = (RCC->CFGR >> 2) & 0x3;

	if (clksrc == 0) {
		// system clock us HSI (16 MHz)
		sysclk = 16000000;
	} else if (clksrc == 1) {
		// system clock us HSE (8 MHz)
		sysclk = 8000000;
	} else if (clksrc == 2) {
		// sys clock decided by PLL and must be calculated
		sysclk = RCC_GetPLLOutputClk();
	}

	// for AHB
	temp = (RCC->CFGR >> 4) & 0xF;

	if (temp < 8) {
		ahbp = 1;
	} else {
		ahbp = AHB_PreScalar[temp - 8];
	}

	// for APB1
	temp = (RCC->CFGR >> 10) & 0x7;
	if (temp < 4) {
		apb1p = 1;
	} else {
		apb1p = APB1_PreScalar[temp - 4];
	}

	pclk1 = (sysclk / ahbp) / apb1p;

	return pclk1;
}

/**************************************************
 * @fn			- I2C_Init
 *
 * @ brief		- Configure the gpio register according to the config structure
 *
 * @param[in]	- configuration structure
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_Init(I2C_Handle_t* pI2CHandle) {
	unit32_t tempreg = 0;

	// 2. Configure speed of serial clock
	// configure the FREQ field of CR2
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	// 3. Configure the device address (applicable when device is slave) 7-bit
	tempreg = pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= 1 << 14;
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// 4. Enable the acking
	// ack control bit
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_AckControl << 10);
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// 1. Configure mode (standard or fast)
	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		// sandard mode
		ccr_value = RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed );
		tempreg |= (ccr_value & 0xFFF);
	} else {
		// fast mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

		if (I2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			ccr_value = RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed );
		} else {
			ccr_value = RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed );
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	// 5. Configure the rise time for I2C pins
}




