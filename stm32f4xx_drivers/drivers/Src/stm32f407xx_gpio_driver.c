/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: 28 Nov 2022
 *      Author: kieranmc
 */
#include "stm32f407xx_gpio_driver.h"

/*
 * Peripheral Clock Setup
 */
/**************************************************
 * @fn			- GPIO_PeriClockControl
 *
 * @ brief		- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]	- The base address of the gpio peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {

		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI) {
			GPIOH_PCLK_EN();
		}

	} else {

		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		} else if (pGPIOx == GPIOI) {
			GPIOH_PCLK_DI();
		}
	}
}

/*
 * Init / De-Init
 */
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
void GPIO_Init(GPIO_Handle_t* pGPIOHandle) {
	uint32_t temp = 0; // temp register

	// enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. Configure mode of gpio pin
	if (pGPIOHandle->GPIO_PingConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		// The non-interrupt modes
		temp = (pGPIOHandle->GPIO_PingConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PingConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PingConfig.GPIO_PinNumber)); // clear
		pGPIOHandle->pGPIOx->MODER |= temp; // set
	} else {
		// Configure for external interrupt handling
		if (pGPIOHandle->GPIO_PingConfig.GPIO_PinMode == GPIO_MODE_IT_FT ) {
			// 1. configure FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PingConfig.GPIO_PinNumber);
			// Clear the corresponding RTSR bit in case it was active
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PingConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PingConfig.GPIO_PinMode == GPIO_MODE_IT_RT ) {
			// 1. configure RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PingConfig.GPIO_PinNumber);
			// Clear the corresponding RTSR bit in case it was active
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PingConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PingConfig.GPIO_PinMode == GPIO_MODE_IT_RFT ) {
			// 1. configure both FTSR and RTSR for both rising and falling edge
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PingConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PingConfig.GPIO_PinNumber);
		}

		// 2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PingConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PingConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = (portcode << (temp2 * 4));

		// 3. enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PingConfig.GPIO_PinNumber);
	}

	// 2. configure speed
	temp = 0;
	temp = (pGPIOHandle->GPIO_PingConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PingConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PingConfig.GPIO_PinNumber)); // clear
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	// 3. configure pupd (pull-up / pull-down) settings
	temp = 0;
	temp = (pGPIOHandle->GPIO_PingConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PingConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PingConfig.GPIO_PinNumber)); // clear
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	// 4. configure the optype (output type)
	temp = 0;
	temp = (pGPIOHandle->GPIO_PingConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PingConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PingConfig.GPIO_PinNumber); // clear
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	// 5. configure the alt functionality
	if (pGPIOHandle->GPIO_PingConfig.GPIO_PinAltFunMode == GPIO_MODE_ALTFN) {
		temp = 0;
		uint8_t AfRegInd = pGPIOHandle->GPIO_PingConfig.GPIO_PinNumber / 8; // Index 0 if pins 0 - 7; Index 1 if pins 8 - 15
		temp = (pGPIOHandle->GPIO_PingConfig.GPIO_PinAltFunMode << ((pGPIOHandle->GPIO_PingConfig.GPIO_PinNumber % 8) * 4));
		pGPIOHandle->pGPIOx->AFR[AfRegInd] &= ~(0xF << ((pGPIOHandle->GPIO_PingConfig.GPIO_PinNumber % 8) * 4)); // clear
		pGPIOHandle->pGPIOx->AFR[AfRegInd] |= temp;
	}
}

/**************************************************
 * @fn			- GPIO_DeInit
 *
 * @ brief		- Reset the pin for the appropriate GPIO
 *
 * @param[in]	- GPIO configuration structure
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	} else if (pGPIOx == GPIOI) {
		GPIOH_REG_RESET();
	}
}

/*
 * Data read and write
 */
/**************************************************
 * @fn			- GPIO_ReadFromInputPin
 *
 * @ brief		- return appropriate data bit from input register
 *
 * @param[in]	- GPIO register structure
 * @param[in]	- Pin number of GPIO port of interest
 *
 * @return		- 0 or 1. data bit of input pin
 *
 * @Note		- none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber) {
	uint8_t value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/**************************************************
 * @fn			- GPIO_ReadFromInputPort
 *
 * @ brief		- Return state of entire input port
 *
 * @param[in]	- GPIO register structure
 *
 * @return		- 16 bits of input port
 *
 * @Note		- none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx) {
	uint16_t value = (uint16_t)pGPIOx->IDR;
	return value;
}

/**************************************************
 * @fn			- GPIO_WriteToOutputPin
 *
 * @ brief		- Write bit to GPIO pin in output mode
 *
 * @param[in]	- GPIO register structure
 * @param[in]	- pin number to write out to
 * @param[in]	- value to write to pin (0 or 1)
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value) {
	if (Value == GPIO_PIN_SET) {
		// Write 1 to output data register at bit field corresponding pin number
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		// Write 0 to output data register at bit field corresponding pin number
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/**************************************************
 * @fn			- GPIO_WriteToOutputPort
 *
 * @ brief		- Overwrite value of output register port
 *
 * @param[in]	- GPIO register structure
 * @param[in]	- value to overwrite register with
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t Value) {
	pGPIOx->ODR = Value;
}

/**************************************************
 * @fn			- GPIO_ToggleOutputPin
 *
 * @ brief		- Flip bit of output bit for a given pin number
 *
 * @param[in]	- GPIO register structure
 * @param[in]	- Pin number
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber) {
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR handling
 */
/**************************************************
 * @fn			- GPIO_IRQITConfig
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
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (IRQNumber <= 31) {
			// Program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			// Program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			// Program ISER2 register
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	} else {
		if (IRQNumber <= 31) {
			// Program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			// Program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			// Program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
}

/**************************************************
 * @fn			- GPIO_IRQPriorityConfig
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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	// 1. Find out IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = ( 8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/**************************************************
 * @fn			- GPIO_IRQHandling
 *
 * @ brief		-
 *
 * @param[in]	-
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 */
void GPIO_IRQHandling(uint8_t PinNumber) {
	if (EXTI->PR & (1 << PinNumber)) {
		// clear
		EXTI->PR |= (1 << PinNumber);
	}
}

