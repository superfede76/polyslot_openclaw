/*
 * MCP23S08.h
 *
 *  Created on: Jul 8, 2025
 *      Author: feder
 */

#ifndef MCP23S08_H
#define MCP23S08_H

#include "stm32f1xx_hal.h"  // o altro in base alla tua MCU
#include "stdbool.h"


#define ON  1
#define OFF 0
#define MAX_MCP_CHIPS 8
#define OUT_MASK 	0x00
#define IN_MASK 	0xFF
#define MIX_MASK	0x0F


typedef enum {
	OUTPUT,
	INPUT,
	MIXED
} digital_board;



typedef struct
{
	GPIO_TypeDef      *cs_gpio;
	uint16_t          cs_pin;
	SPI_HandleTypeDef *spi;
	uint8_t           lock;
	digital_board     board;
	uint8_t           position;
}DigitalIO_t  ;


void MCP23S08_Init(DigitalIO_t *digitalIO, uint8_t iodir);
uint8_t MCP23S08_WritePin(DigitalIO_t *digitalIO, uint8_t output_mask, uint8_t output_pin, bool output_value);
uint8_t MCP23S08_ReadPin(DigitalIO_t *digitalIO, uint8_t input_pin);
uint8_t MCP23S08_ReadPort(DigitalIO_t *digitalIO);
uint8_t MCP23S08_WritePort(DigitalIO_t *digitalIO, uint8_t output_mask, uint8_t output_value);



#endif
