/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: 21 Jan 2023
 *      Author: kieranmc
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */

typedef struct {
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_AckControl;
	uint8_t I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */

typedef struct {
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM	100000 // Speed standard mode
#define I2C_SCL_SPEED_FM4K	400000 // Speed fast mode 4 kHz
#define I2C_SCL_SPEED_FM2K	200000 // Speed fast mode 2 kHz

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1


/*I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG)
 * I2C status flags
 */
#define I2C_SB_FLAG				(1 << I2C_SR1_SB)
#define I2C_ADDR_FLAG			(1 << I2C_SR1_ADDR)
#define I2C_BTF_FLAG			(1 << I2C_SR1_BTF)
#define I2C_STOPF_FLAG			(1 << I2C_SR1_STOPF)
#define I2C_RXNE_FLAG			(1 << I2C_SR1_RXNE)
#define I2C_TXE_FLAG			(1 << I2C_SR1_TXE)
#define I2C_BERR_FLAG			(1 << I2C_SR1_BERR)
#define I2C_AF_FLAG				(1 << I2C_SR1_AF)
#define I2C_OVR_FLAG			(1 << I2C_SR1_OVR)
#define I2C_TIMEOUT_FLAG		(1 << I2C_SR1_TIMEOUT)

/*
 * Init / De-Init
 */
void I2C_Init(I2C_Handle_t* pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data send receive
 */
void I2C_MasterSendData(I2C_Handle_t* pI2CHandle, uint8_t* pTxBuffer, uint32_t Len, uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t* pI2CHandle, uint8_t* pRxBuffer, uint32_t Len, uint8_t SlaveAddr);

uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint32_t FlagName);

void I2C_ManageAcking(I2C_RegDef_t* pI2Cx, uint8_t EnorDi);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
