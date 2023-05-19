/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 21 Jan 2023
 *      Author: kieranmc
 */

#include "stm32f407xx.h"

uint16_t AHB_PreScalar[8] = {2,4,8,16,128,256,512};
uint8_t APB1_PreScalar[4] = {2,4,8,16};

static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t* pI2CHandle);
static void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx);

static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx) {
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint32_t FlagName) {
	if (pI2Cx->SR1 & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr) {
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr) {
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t* pI2CHandle) {

	uint32_t dummyRead;
	if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			if (pI2CHandle->RxSize == 1) {
				// first disable acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				// clear ADDR flag by read SR1 and SR2
				 dummyRead = pI2CHandle->pI2Cx->SR1;
				 dummyRead = pI2CHandle->pI2Cx->SR2;
				 (void)dummyRead; // suppress unused warnings
			}
		}
	} else {
		// device in slave mode
		// clear ADDR flag by read SR1 and SR2
		 dummyRead = pI2CHandle->pI2Cx->SR1;
		 dummyRead = pI2CHandle->pI2Cx->SR2;
		 (void)dummyRead; // suppress unused warnings
	}
}

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
	uint32_t pclk1, sysclk;
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
	uint32_t tempreg = 0;
	uint8_t trise;

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

		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			ccr_value = RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed );
		} else {
			ccr_value = RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed );
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	// 5. Configure the rise time for I2C pins
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		// sandard mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	} else {
		// mode is fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

void I2C_MasterSendData(I2C_Handle_t* pI2CHandle, uint8_t* pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {
	// 1. Generate the START confition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. confirm that start generation is completed by checking the SB flag in the SR1
	// Note: until SB is cleared SCL will be stretched (pulled to LOW)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));

	// 3. Send the address of the slave with r/nw but set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));

	// 5. clear the ADDR flag according to its software sequence
	// Note: until ADDR is cleared, SCL will be stretched low
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	// 6. Send data until Len becomes 0
	while (Len > 0) {
		// wait until TXE is set
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	// 7. When Len becomes zero wait for TXE=1 and BTF=1 before generating STOP condition
	// Note: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin
	// when BTF=1 SCL will be stretched low
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG));
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG));

	// 8. Generate stop condition
	if (Sr == I2C_NO_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

/**************************************************
 * @fn			- I2C_MasterReceiveData
 *
 * @ brief		- Configure
 *
 * @param[in]	- configuration structure
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_MasterReceiveData(I2C_Handle_t* pI2CHandle, uint8_t* pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {
	// 1. Generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm start sequence success by checking SB flag in SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));

	// 3, Send the address of slave w/nw bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. Wait until address phase is complete by checking if the ADDR flag in the SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));

	// read only 1 byte
	if (Len == 1) {
		// disable acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG));
		if (Sr == I2C_NO_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	if (Len > 1) {
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		// read until len 0
		for (uint32_t i = Len; i > 0; i--) {
			// wait for rxne to become 1
			while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG));
			if (i == 2) {
				// for last 2 bytes
				// clear ack bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
				// generate stop condition
				if (Sr == I2C_NO_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			// read data into buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
		}
	}

	// re-enable acking to leave as it was before func call
	if (pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE) {
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

/**************************************************
 * @fn			- I2C_MasterSendDataIT
 *
 * @ brief		- Configure
 *
 * @param[in]	- configuration structure
 *
 * @return		- none
 *
 * @Note		- none
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t* pI2CHandle, uint8_t* pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {
	uint8_t busystate = pI2CHandle->TxRxState;

	if ( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX) ) {
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// 1. Generate start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Enable IT Buffen control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		// Enable ITEVFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		// Enable I2C_CR2_ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return busystate;
}

/**************************************************
 * @fn			- I2C_MasterReceiveDataIT
 *
 * @ brief		- Configure
 *
 * @param[in]	- configuration structure
 *
 * @return		- none
 *
 * @Note		- none
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t* pI2CHandle, uint8_t* pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {

}

/**************************************************
 * @fn			- I2C_IRQInterruptConfig
 *
 * @ brief		- Configure
 *
 * @param[in]	- configuration structure
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {

}

/**************************************************
 * @fn			- I2C_IRQPriorityConfig
 *
 * @ brief		- Configure
 *
 * @param[in]	- configuration structure
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {

}

/**************************************************
 * @fn			- I2C_EV_IRQHandling
 *
 * @ brief		- Configure
 *
 * @param[in]	- configuration structure
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_EV_IRQHandling(I2C_Handle_t* pI2CHandle) {
	// Handle for SB event interrupt

	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN );
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN );
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB );

	// Handle for interrupt generated by SB event
	if (temp1 && temp3) {
		// SB flag is set
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		} else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR );
	if (temp1 && temp3) {
		// Addr flag is set
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF );
	if (temp1 && temp3) {
		// BTF flag is set
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)) {
				if (pI2CHandle->TxLen == 0){
				// 1. generate stop condition
				if (pI2CHandle->Sr == I2C_NO_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

				// 2. reset all member elements
				I2C_CloseSendData();

				// 3. notify application
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
			}
		}
		} else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			;
		}
}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF );
	if (temp1 && temp3) {
		// STOPF flag is set
		// Clear stopf flag, write to CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0000;
		// Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE );
	if (temp1 && temp2 && temp3) {
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
			// TXE flag is set
			// We have to do the data transmission
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
				if (pI2CHandle->TxLen > 0) {
					pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
					pI2CHandle->TxLen--;
					pI2CHandle->pTxBuffer++;
				}
			}
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE );
	if (temp1 && temp2 && temp3) {
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
			// RXNE flag is set
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
				if (pI2CHandle->RxSize == 1) {
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
					pI2CHandle->RxLen--;
				}
				if (pI2CHandle->RxSize > 1) {
					if (pI2CHandle->RxLen == 2) {
						I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
					}
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
					pI2CHandle->RxLen--;
					pI2CHandle->pRxBuffer++;
				}
			}
			if (pI2CHandle->RxSize == 0) {
				// close I2C data reception and notify app
				// 1. generate stop condition
				if (pI2CHandle->Sr == I2C_NO_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

				// 2. reset all member elements
				I2C_CloseReceiveData();

				// 3. notify application
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
			}
		}
	}
}

/**************************************************
 * @fn			- I2C_EV_IRQHandling
 *
 * @ brief		- Configure
 *
 * @param[in]	- configuration structure
 *
 * @return		- none
 *
 * @Note		- none
 */
void I2C_ER_IRQHandling(I2C_Handle_t* pI2CHandle) {

}

void I2C_ManageAcking(I2C_RegDef_t* pI2Cx, uint8_t EnorDi) {
	if (EnorDi == I2C_ACK_ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}


