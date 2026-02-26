/*
 * PS_mixedIO.h
 *
 *  Created on: Jul 10, 2025
 *      Author: feder
 */

#ifndef INC_DIGITALIO_H_
#define INC_DIGITALIO_H_


#include "stm32f1xx_hal.h"  // o altro in base alla tua MCU
#include "stdbool.h"
#include "MCP23S08.h"



uint8_t digitalIO_Init(DigitalIO_t *digitalIO,SPI_HandleTypeDef *spi,GPIO_TypeDef  *cs_gpio,uint16_t cs_pin, digital_board board);
uint8_t digitalIO_Read(DigitalIO_t *digitalIO, uint8_t pin);
uint8_t digitalIO_Write(DigitalIO_t *digitalIO,uint8_t output_pin,bool value);

#endif /* INC_DIGITALIO_H_ */
