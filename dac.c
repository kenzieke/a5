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
#define CS GPIO_ODR_OD2
#define SCK GPIO_ODR_OD13 // GPIOE Pin 13
#define SDI GPIO_ODR_OD4 // 
//#define LDAC GPIO_ODR_OD5
//#define VREF_PIN GPIO_ODR_OD6
#define VREF 3300
#define WRITE_MSK 0111000000000000
//#define VOUT GPIO_ODR_OD7


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
	DAC_PORT->MODER   &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3 | GPIO_MODER_MODE4);
	DAC_PORT->MODER   |=  (GPIO_MODER_MODE2_0 | GPIO_MODER_MODE3_0 | GPIO_MODER_MODE4_0);
	DAC_PORT->OTYPER  &= ~(GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3 | GPIO_OTYPER_OT4);
	DAC_PORT->PUPDR   &= ~(GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3 | GPIO_PUPDR_PUPD4);
	DAC_PORT->OSPEEDR |= ((3 << GPIO_OSPEEDR_OSPEED2_Pos) |
	 (3 << GPIO_OSPEEDR_OSPEED3_Pos) |
	 (3 << GPIO_OSPEEDR_OSPEED4_Pos));

	// configure AFR for SPI1 function (1 of 3 SPI bits shown here)
	// make sure using pin designated for spi signal; 13, 14, 15
	DAC_PORT->AFR[0] &= ~((0x000F << GPIO_AFRL_AFSEL7_Pos));	// clear nibble for bit 7 AF
	DAC_PORT->AFR[0] |=  ((0x0005 << GPIO_AFRL_AFSEL7_Pos));	// set b7 AF to SPI1 (fcn 5)
	DAC_PORT->AFR[1] &= ~((0x000F << GPIO_AFRL_AFSEL7_Pos));	// clear nibble for bit 7 AF
	DAC_PORT->AFR[1] |=  ((0x0005 << GPIO_AFRL_AFSEL7_Pos));	// set b7 AF to SPI1 (fcn 5)
	DAC_PORT->AFR[2] &= ~((0x000F << GPIO_AFRL_AFSEL7_Pos));	// clear nibble for bit 7 AF
	DAC_PORT->AFR[2] |=  ((0x0005 << GPIO_AFRL_AFSEL7_Pos));	// set b7 AF to SPI1 (fcn 5)

	SPI_init();
}

// write a 12-bit value to the DAC
void DAC_write(uint16_t converted_voltage) {
	// set cs low
	DAC_PORT->BRR = CS;
	// bit 15 = 0, bit 14 = 1, bit 13 = 1, bit 12 = 1, bit 11-0 from DAC_volt_conv
	// stitch all the bits together to make dac_value
	uint16_t dac_value = WRITE_MSK | converted_voltage;
	// look for when data flag goes high?
	// if SPI1->SR & SPI->TXR
	// SPI1->DR = dac_value
}

// convert a voltage value into a 12-bit value to control the DAC
uint16_t DAC_volt_conv(uint16_t voltageMV) {
	if (voltageMV >= VREF) {
		return 4096-1;
	}
	return 4095*voltageMV/VREF;
}

