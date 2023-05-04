/*
 * dac.c
 *
 *  Created on: May 2, 2023
 *      Author: macke
 */

#include "stm32l4a6xx.h"
#include "stm32l4xx_hal.h"
#include "dac.h"

#define DAC_PORT GPIOE
#define SPI_CS GPIO_PIN_12	 // CS
#define SPI_SCK GPIO_PIN_13  // CLK
//#define SPI_PICO GPIO_PIN_14 // MISO
#define SPI_COPI GPIO_PIN_15 // SDI
#define VREF 3300
#define WRITE_MSK 0x3000


void SPI_init(void) {
   // SPI config as specified @ STM32L4 RM p.1459
   // called by or with DAC_init()
   // build control registers CR1 & CR2 for SPI control of peripheral DAC
   // assumes no active SPI xmits & no recv data in process (BSY=0)
   // CR1 (reset value = 0x0000)
   SPI1->CR1 &= ~( SPI_CR1_SPE );             	// disable SPI for config
   SPI1->CR1 &= ~( SPI_CR1_RXONLY );          	// recv-only OFF
   SPI1->CR1 &= ~( SPI_CR1_LSBFIRST );        	// data bit order MSb:LSb
   SPI1->CR1 &= ~( SPI_CR1_CPOL | SPI_CR1_CPHA ); // SCLK polarity:phase = 0:0
   SPI1->CR1 |=	 SPI_CR1_MSTR;              	// MCU is SPI controller
   // CR2 (reset value = 0x0700 : 8b data)
   SPI1->CR2 &= ~( SPI_CR2_TXEIE | SPI_CR2_RXNEIE ); // disable FIFO intrpts
   SPI1->CR2 &= ~( SPI_CR2_FRF);              	// Moto frame format
   SPI1->CR2 |=	 SPI_CR2_NSSP;              	// auto-generate NSS pulse
   SPI1->CR2 |=	 SPI_CR2_DS;                	// 16-bit data
   SPI1->CR2 |=	 SPI_CR2_SSOE;              	// enable SS output
   // CR1
   SPI1->CR1 |=	 SPI_CR1_SPE;               	// re-enable SPI for ops
}


// initialize the SPI peripheral to communicate with the DAC
void DAC_init() {
	// enable clock for GPIOE & SPI1
	RCC-> AHB2ENR |= (RCC_AHB2ENR_GPIOEEN);               // GPIOE: DAC NSS/SCK/SDO
	RCC-> APB2ENR |= (RCC_APB2ENR_SPI1EN);                // SPI1 port

	/* USER ADD GPIO configuration of MODER/PUPDR/OTYPER/OSPEEDR registers HERE : 2-7 */
	DAC_PORT->MODER   &= ~(GPIO_MODER_MODE12 | GPIO_MODER_MODE13 | GPIO_MODER_MODE15);
	DAC_PORT->MODER   |=  (GPIO_MODER_MODE12 | GPIO_MODER_MODE13_1 | GPIO_MODER_MODE15_1);
	DAC_PORT->OTYPER  &= ~(GPIO_MODER_MODE12 | GPIO_OTYPER_OT13 | GPIO_OTYPER_OT15);
	DAC_PORT->PUPDR   &= ~(GPIO_MODER_MODE12 | GPIO_PUPDR_PUPD13 | GPIO_PUPDR_PUPD15);
	DAC_PORT->OSPEEDR |= ((3 << GPIO_OSPEEDR_OSPEED12_Pos) |
	 (3 << GPIO_OSPEEDR_OSPEED13_Pos) |
	 (3 << GPIO_OSPEEDR_OSPEED15_Pos));

	// 12 is CS, 13 is SCK, 15 is SDI
	DAC_PORT->AFR[1] &= ~((0x000F << GPIO_AFRH_AFSEL12_Pos));	// clear nibble for bit 12
	DAC_PORT->AFR[1] |=  ((0x0005 << GPIO_AFRH_AFSEL12_Pos));	// set b12 AF to SPI1 (fcn 5)
	DAC_PORT->AFR[1] &= ~((0x000F << GPIO_AFRH_AFSEL13_Pos));	// clear nibble for bit 13 AF
	DAC_PORT->AFR[1] |=  ((0x0005 << GPIO_AFRH_AFSEL13_Pos));	// set b13 AF to SPI1 (fcn 5)
	DAC_PORT->AFR[1] &= ~((0x000F << GPIO_AFRH_AFSEL15_Pos));	// clear nibble for bit 15 AF
	DAC_PORT->AFR[1] |=  ((0x0005 << GPIO_AFRH_AFSEL15_Pos));	// set b15 AF to SPI1 (fcn 5)

	SPI_init();
}

// write a 12-bit value to the DAC
void DAC_write(uint16_t converted_voltage) {
	// bit 15 = 0, bit 14 = 1, bit 13 = 1, bit 12 = 1, bit 11-0 from DAC_volt_conv
	// stitch all the bits together to make dac_value, clear the upper four bits
	converted_voltage = 0xFFF & converted_voltage;
	uint16_t dac_value = WRITE_MSK | converted_voltage;
	// look for when data flag goes high, then set data register to dac value
	if (SPI1->SR & SPI_SR_TXE) {
		SPI1->DR = dac_value;
	}
}

// convert a voltage value into a 12-bit value to control the DAC
uint16_t DAC_volt_conv(uint16_t voltageMV) {
	if (voltageMV >= VREF) {
		return 4096-1;
	}
	return 4095*voltageMV/VREF;
}

