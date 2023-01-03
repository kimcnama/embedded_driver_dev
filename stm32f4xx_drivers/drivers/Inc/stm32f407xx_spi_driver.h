/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: 26 Dec 2022
 *      Author: kieranmc
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for SPIx peripheral
 */

typedef struct {
	uint8_t SPI_DeviceMode;		// input / output / duplex
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed; 		// Clock speed
	uint8_t SPI_DFF; 			// Data frame format
	uint8_t SPI_CPOL;			// idle state is when clock is high or lo?
	uint8_t SPI_CPHA;			// rising edge or falling edge sample data lines?
	uint8_t SPI_SSM;			// software / hardware
}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */

typedef struct {
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
}SPI_Handle_t;

/*
 * SPI Device mode MACROS
 */
#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0

/*
 * SPI Bus config MACROS
 */
#define SPI_BUS_CONFIG_FD				1	// Full duplex
#define SPI_BUS_CONFIG_HD				2	// Half duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3	// Simplex rx
//#define SPI_BUS_CONFIG_SIMPLEX_TXONLY	4	// Simplex tx - not required just remove rx line

/*
 * SPI clock speeds. Divides peripheral clock by constant
 */
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

/*
 * SPI DFF - data frame format
 */
#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1

/*
 * SPI CPOL
 */
#define SPI_CPOL_HIGH					1
#define SPI_CPOL_LO						0

/*
 * SPI CPHA
 */
#define SPI_CPHA_HIGH					1
#define SPI_CPHA_LO						0

/*
 * SPI SSM - software slave management
 */
#define SPI_SSM_EN						1
#define SPI_SSM_DI						0 	// HW mode

/*
 * SPI related status flag definitions
 */

#define SPI_RXNE_FLAG			(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG			(1 << SPI_SR_TXE)
#define SPI_CHSIDE_FLAG			(1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG			(1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG			(1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG			(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG			(1 << SPI_SR_OVR)
#define SPI_BUSY_FLAG			(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG			(1 << SPI_SR_FRE)

/******************************************************************************
 *
 * APIs supported by this driver
 *
 ******************************************************************************/

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi);

/*
 * Init / De-Init
 */
void SPI_Init(SPI_Handle_t* pSPIHandle);
void SPI_DeInit(SPI_RegDef_t* pSPIx);


/*
 * Data send and data receive
 */
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t *pTxBuffer, uint32_t Len);

/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t* pSPIHandle);

/*
 * Other peripheral control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t* pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t* pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx, uint32_t FlagName);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
