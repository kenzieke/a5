/*
 * dac.h
 *
 *  Created on: May 2, 2023
 *      Author: macke
 */

#ifndef SRC_DAC_H_
#define SRC_DAC_H_

void SPI_init();
void DAC_init();
void DAC_write();
uint16_t DAC_volt_conv(uint16_t voltageMV);

#endif /* SRC_DAC_H_ */
