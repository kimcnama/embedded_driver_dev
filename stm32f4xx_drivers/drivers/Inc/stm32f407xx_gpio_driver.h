/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: 28 Nov 2022
 *      Author: kieranmc
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

typedef struct {
	uint8_t GPIO_PinNumber;				/* possible values from @GPIO_PIN_NUMS */
	uint8_t GPIO_PinMode;				/* possible values from @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;				/* possible values from @GPIO_PIN_SPEED */
	uint8_t GPIO_PinPuPdControl;		/* possible values from @GPIO_PIN_PUPD */
	uint8_t GPIO_PinOPType;				/* possible values from @GPIO_PIN_OPTYPE */
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/*
 * This is the handle structure for a GPIO pin
 */
typedef struct {
	GPIO_RegDef_t* pGPIOx; 				/* Base address to which the GPIO port pin belongs */
	GPIO_PinConfig_t GPIO_PingConfig;	/* This holds GPIO ping configuration settings */
} GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMS
 * Pin numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * @GPIO_PIN_MODES
 * Input modes for GPIO ping
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4		// For interrupt handling (input), can signal interrupt to processor when falling edge
#define GPIO_MODE_IT_RT		5		// For interrupt handling (input), can signal interrupt to processor when rising edge
#define GPIO_MODE_IT_RFT	6		// For interrupt handling (input), can signal interrupt to processor when rising & falling edge trigger

/*
 * @GPIO_PIN_OPTYPE
 * Output types for GPIO
 */
#define GPIO_OP_TYPE_PP		0		// Push/pull output type
#define GPIO_OP_TYPE_OD		1		// Output type open-drain

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * @GPIO_PIN_PUPD
 * GPIO pull up / pull down configuration macros
 */
#define GPIO_NO_PUPD		0		// No pull up / pull down
#define GPIO_PIN_PU			1		// pull up
#define GPIO_PIN_PD			2		// pull down

/******************************************
 * API Prototypes supported by this driver
 ******************************************
 */

/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi);

/*
 * Init / De-Init
 */
void GPIO_Init(GPIO_Handle_t* pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
