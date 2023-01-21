/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 21 Jan 2023
 *      Author: kieranmc
 */

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
		if (pSPIx == I2C1) {
			I2C1_PCLK_EN();
		} else if (pSPIx == I2C2) {
			I2C2_PCLK_EN();
		} else if (pSPIx == I2C3) {
			I2C3_PCLK_EN();
		}
	} else {
		if (pSPIx == I2C1) {
			I2C1_PCLK_DI();
		} else if (pSPIx == I2C2) {
			I2C2_PCLK_DI();
		} else if (pSPIx == I2C3) {
			I2C3_PCLK_DI();
		}
	}
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
	// 1. Configure mode (standard or fast)
	// 2. Configure speed of serial clock
	// 3. Configure the device address (applicable when device is slave)
	// 4. Enable the acking
	// 5. Configure the rise time for I2C pins
}

