/*
 * pulse_counter.c
 *
 *  Created on: Nov 10, 2025
 *      Author: feder
 */

#include "pulse_counter.h"


void  counter_init(pulse_t *counter,SPI_HandleTypeDef *spi,GPIO_TypeDef  *cs_gpio,uint16_t cs_pin){
	if(counter->lock == 1)
		HAL_Delay(1);
	counter->lock = 1;
	counter->spi = spi;
	counter->cs_gpio = cs_gpio;
	counter->cs_pin = cs_pin;
	HAL_GPIO_WritePin(counter->cs_gpio,counter->cs_pin,GPIO_PIN_SET);
	HAL_Delay(100);

}
bool  counter_readPulse(pulse_t *counter,uint16_t *readCount, uint8_t ch){

	uint8_t tx_data[1]={0};
	uint8_t rx_data[2]={0};
	*readCount=0;
	if ((ch<0)||(ch>3))
		return -1;
	tx_data[0]=0xA1+ch;
	HAL_GPIO_WritePin(counter->cs_gpio, counter->cs_pin, GPIO_PIN_RESET);


	if (HAL_SPI_Transmit(counter->spi, tx_data, 1, HAL_MAX_DELAY) == HAL_OK) {
		//HAL_Delay(10);
		for(volatile uint32_t ll=0; ll<7500; ll++);
		if(HAL_SPI_Receive(counter->spi, rx_data, 2, HAL_MAX_DELAY) == HAL_OK) {
			*readCount=((uint16_t)rx_data[0] << 8) | (uint16_t)rx_data[1];
		}
	}
	HAL_GPIO_WritePin(counter->cs_gpio, counter->cs_pin, GPIO_PIN_SET);
	return 1;

}
