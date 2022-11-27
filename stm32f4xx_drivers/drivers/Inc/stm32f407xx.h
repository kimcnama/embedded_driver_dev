/*
 * stm32f407xx.h
 *
 *  Created on: Nov 24, 2022
 *      Author: kieranmc
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile

/*
 * Base addresses of Flash and SRAM *
 */

#define FLASH_BASEADDR			0x08000000U /* Start of flash memory */
#define SRAM1_BASEADDR			0x20000000U /* Start of SRAM1 */
#define SRAM2_BASEADDR			0x20001C00U /* 112 KB after SRAM1 */
#define ROM						0x1FFF0000U
#define SRAM					SRAM1_BASEADDR

/*
 * AHBx and APBx bus peripheral base addresses
 */

#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASE
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

/*
 * AHB1 Bus peripherals: GPIO base addresses
 */

#define GPIOA_BASEADDR			( AHB1PERIPH_BASE + 0x0000 )
#define GPIOB_BASEADDR			( AHB1PERIPH_BASE + 0x0400 )
#define GPIOC_BASEADDR			( AHB1PERIPH_BASE + 0x0800 )
#define GPIOD_BASEADDR			( AHB1PERIPH_BASE + 0x0C00 )
#define GPIOE_BASEADDR			( AHB1PERIPH_BASE + 0x1000 )
#define GPIOF_BASEADDR			( AHB1PERIPH_BASE + 0x1400 )
#define GPIOG_BASEADDR			( AHB1PERIPH_BASE + 0x1800 )
#define GPIOH_BASEADDR			( AHB1PERIPH_BASE + 0x1C00 )
#define RCC_BASEADDR			( AHB1PERIPH_BASE + 0x3800 )

/*
 * APB1 Bus peripherals: I2C, SPI, UART
 */

#define I2C1_BASEADDR			( APB1PERIPH_BASE + 0x5400 )
#define I2C2_BASEADDR			( APB1PERIPH_BASE + 0x5800 )
#define I2C3_BASEADDR			( APB1PERIPH_BASE + 0x5C00 )
#define SPI2_BASEADDR			( APB1PERIPH_BASE + 0x3800 )
#define SPI3_BASEADDR			( APB1PERIPH_BASE + 0x3C00 )
#define USART2_BASEADDR			( APB1PERIPH_BASE + 0x4400 )
#define USART3_BASEADDR			( APB1PERIPH_BASE + 0x4800 )
#define UART4_BASEADDR			( APB1PERIPH_BASE + 0x4C00 )
#define UART5_BASEADDR			( APB1PERIPH_BASE + 0x5000 )

/*
 * APB2 Bus peripherals: SPI, USART, SYSCFG and EXTI
 */

#define SPI1_BASEADDR			( APB2PERIPH_BASE + 0x3000 )
#define SPI4_BASEADDR			( APB2PERIPH_BASE + 0x3400 )
#define USART1_BASEADDR			( APB2PERIPH_BASE + 0x1000 )
#define USART6_BASEADDR			( APB2PERIPH_BASE + 0x1400 )
#define EXTI_BASEADDR			( APB2PERIPH_BASE + 0x3C00 )
#define SYSCFG_BASEADDR			( APB2PERIPH_BASE + 0x3800 )

/*
 **********************************************************
 * Peripheral register definition structures
 **********************************************************
 */
/*
 * GPIO Configuration Register Structure
 */
typedef struct {
	__vo uint32_t MODER;		// port mode register				Address offset: 0x00
	__vo uint32_t OTYPER;		// port output type register		Address offset: 0x04
	__vo uint32_t OSPEEDR;		// port output speed register		Address offset: 0x08
	__vo uint32_t PUPDR;		// port pull-up/pull-down register	Address offset: 0x0C
	__vo uint32_t IDR;			// port input data register			Address offset: 0x10
	__vo uint32_t ODR;			// port output data register		Address offset: 0x14
	__vo uint32_t BSRR;			// port bit set/reset register		Address offset: 0x18
	__vo uint32_t LCKR;			// port configuration lock register	Address offset: 0x1C
	__vo uint32_t AFR[2]; 		// 2 registers 1 for AFR (Alternate function Register) 1. Low 2. High
} GPIO_RegDef_t;

typedef struct {
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
} RCC_RegDef_t;

/*
 * GPIO Peripheral Registers
 */

#define GPIOA					((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB					((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF					((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG					((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH					((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC						((RCC_RegDef_t*) RCC_BASEADDR)

/*
 * Clock enable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()		( RCC->AHB1ENR | = (1 << 0) )
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR | = (1 << 1) )
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR | = (1 << 2) )
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR | = (1 << 3) )
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR | = (1 << 4) )
#define GPIOF_PCLK_EN()		( RCC->AHB1ENR | = (1 << 5) )
#define GPIOG_PCLK_EN()		( RCC->AHB1ENR | = (1 << 6) )
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR | = (1 << 7) )

/*
 * Clock enable macros for I2Cx peripherals
 */
#define IC1_PCLK_EN()		( RCC->APB1ENR | = (1 << 21) )

/*
 * Clock enable macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()		( RCC->APB2ENR | = (1 << 12) )

/*
 * Clock enable macros for USARTx peripherals
 */
#define USART1_PCLK_EN()		( RCC->APB2ENR | = (1 << 4) )

/*
 * Clock enable macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()		( RCC->APB2ENR | = (1 << 14) )

// TODO: Create clock disable macros

#endif /* INC_STM32F407XX_H_ */
