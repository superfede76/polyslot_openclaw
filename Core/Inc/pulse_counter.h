/*
 * pulse_counter.h
 *
 *  Created on: Nov 10, 2025
 *      Author: feder
 */

#ifndef INC_PULSE_COUNTER_H_
#define INC_PULSE_COUNTER_H_

#include "main.h"
#include <stdbool.h>


typedef struct
{
	GPIO_TypeDef      *cs_gpio;
	uint16_t          cs_pin;
	SPI_HandleTypeDef *spi;
	uint8_t			channel;
	uint8_t          lock;
	uint8_t				position;

}pulse_t;

void  counter_init(pulse_t *counter,SPI_HandleTypeDef *spi,GPIO_TypeDef  *cs_gpio,uint16_t cs_pin);
bool  counter_readPulse(pulse_t *counter,uint16_t *readCount, uint8_t ch);


#endif /* INC_PULSE_COUNTER_H_ */
