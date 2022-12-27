/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 26 Dec 2022
 *      Author: kieranmc
 */

#include "stm32f407xx.h"

/**************************************************
 * @fn			- SPI_PeriClockControl
 *
 * @ brief		- This function enables or disables peripheral clock for a given SPI port
 *
 * @param[in]	- Base address of spi peripheral
 * @param[in]	- enable or disable peripheral clock
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_PeriClockControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_EN();
		}
	} else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_DI();
		}
	}
}

/**************************************************
 * @fn			- SPI_Init
 *
 * @ brief		- Configure the gpio register according to the config structure
 *
 * @param[in]	- configuration structure
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_Init(SPI_Handle_t* pSPIHandle) {
	// configure SPI_CR1 register
	uint32_t tempreg = 0;

	// 1. configure device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		// bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		// enable bidi mode
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		// bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		// RXONLY bit should be set
		tempreg &= ~(1 << SPI_CR1_RXONLY);
	}

	// 3. configure spi serial clock speed (baud rate)
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	// 4. configure DFF
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	// 5. configure CPOL
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	// 5. configure CPHA
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/**************************************************
 * @fn			- SPI_DeInit
 *
 * @ brief		- Configure the gpio register according to the config structure
 *
 * @param[in]	- configuration structure
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_DeInit(SPI_RegDef_t* pSPIx) {
	if (pSPIx == SPI1) {
		return;
	} else if (pSPIx == SPI2) {
		return;
	} else if (pSPIx == SPI3) {
		return;
	} else if (pSPIx == SPI4) {
		return;
	}
}

/**************************************************
 * @fn			- SPI_GetFlagStatus
 *
 * @ brief		- Get status flag from status register
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx, uint32_t FlagName) {
	if (pSPIx->SR & FlagName) {
		return FLAG_SET;
	}
}

/*
 * Init / De-Init
 */
/**************************************************
 * @fn			- SPI_SendData
 *
 * @ brief		- Blocking API to send data. Will wait until all bytes transmitted
 *
 * @param[in]	- SPI register definition
 * @param[in]	- pointer to buffer
 * @param[in]	- length of data to tx
 *
 * @return		- none
 *
 * @Note		- This is a blocking call
 */
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t *pTxBuffer, uint32_t Len) {
	while (Len > 0) {
		// 1. wait until TXE is set (tx register is free)
		while ( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );

		// 2. check DFF bit in CR1
		if ( pSPIx->SR & (1 << SPI_CR1_DFF) ) {
			// 16 bit DFF format
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			--Len;
			--Len;
			(uint16_t*)pTxBuffer++;
		} else {
			// 8 bit DFF format
			pSPIx->DR = *pTxBuffer;
			--Len;
			pTxBuffer++;
		}
	}
}

/**************************************************
 * @fn			- GPIO_Init
 *
 * @ brief		- Configure the gpio register according to the config structure
 *
 * @param[in]	- configuration structure
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t *pTxBuffer, uint32_t Len) {

}


/**************************************************
 * @fn			- SPI_IRQInterruptConfig
 *
 * @ brief		- Function to configure the interrupt registers for the Arm Cortex processor
 *
 * @param[in]	- Interrupt number
 * @param[in]	- Enable or Disable
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {

}

/**************************************************
 * @fn			- SPI_IRQPriorityConfig
 *
 * @ brief		- Set priority level of given interrupt
 *
 * @param[in]	- Interrupt number
 * @param[in]	- Interrupt priority
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {

}

/**************************************************
 * @fn			- SPI_IRQHandling
 *
 * @ brief		- Configure the gpio register according to the config structure
 *
 * @param[in]	- configuration structure
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_IRQHandling(SPI_Handle_t* pSPIHandle) {

}

