/*
 * PS_mixedIO.c
 *
 *  Created on: Jul 10, 2025
 *      Author: feder
 */
#include "digitalIO.h"


#define MIXED_MASK 0xF0		// 0 = OUTPUT, 1 = INPUT
#define INPUT_MASK 0xFF 	// 0 = OUTPUT, 1 = INPUT
#define OUTPUT_MASK 0x00	// 0 = OUTPUT, 1 = INPUT


#define OPCODE_WRITE 0x40
#define OPCODE_READ  0x41

#define IODIR_REG    0x00
#define GPIO_REG     0x09
#define OLAT_REG     0x0A



uint8_t digitalIO_Init(DigitalIO_t *digitalIO,SPI_HandleTypeDef *spi,GPIO_TypeDef  *cs_gpio,uint16_t cs_pin, digital_board board){
	if(digitalIO->lock == 1)
		HAL_Delay(1);
	digitalIO->lock = 1;
	digitalIO->spi = spi;
	digitalIO->cs_gpio = cs_gpio;
	digitalIO->cs_pin = cs_pin;
	digitalIO->board = board;
	uint8_t mask;
	HAL_GPIO_WritePin(digitalIO->cs_gpio,digitalIO->cs_pin,GPIO_PIN_SET);
	HAL_Delay(100);
	switch (board){
	case INPUT:
		mask = INPUT_MASK;
		break;
	case OUTPUT:
		mask = OUTPUT_MASK;
		break;
	case MIXED:
		mask = MIXED_MASK;
		break;
	default:
		return -1;
		break;

	}
	//	HAL_Delay(1);
	for(volatile uint32_t ll=0; ll<750; ll++);
	MCP23S08_Init(digitalIO, mask);  //0 = output
	if (board==MIXED){ //necessario perchè sulla scheda mixed manca ULN
		HAL_GPIO_WritePin(digitalIO->cs_gpio, digitalIO->cs_pin, GPIO_PIN_RESET);
		uint8_t txBuf[3] = {OPCODE_WRITE, OLAT_REG, 0x0F}; //IODIR: 0 = output //MIXED
		HAL_SPI_Transmit(digitalIO->spi, txBuf, 3, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(digitalIO->cs_gpio, digitalIO->cs_pin, GPIO_PIN_SET);

	}
	return 0;
}
uint8_t digitalIO_Read(DigitalIO_t *digitalIO, uint8_t pin){

	if ((pin >= 8)||(pin < 0))
		return -1;
	//	HAL_Delay(1);
	for(volatile uint32_t ll=0; ll<750; ll++);
	if(digitalIO->board==OUTPUT){
		pin =  7 - pin ; //i led della scheda output sono posizionati al contrario //OUTPUT
		return !MCP23S08_ReadPin( digitalIO, pin);	 //OUTPUT
	}
	return MCP23S08_ReadPin(digitalIO, pin);
}


uint8_t digitalIO_Write(DigitalIO_t *digitalIO,uint8_t output_pin,bool value){

	uint8_t mask;

	if (output_pin < 0)
		return -1;

	if((digitalIO->board != MIXED)&&(digitalIO->board != OUTPUT))
		return -1;

	if(digitalIO->board==MIXED){
		if(output_pin>=4)
			return -1;
		value = ! value; //sul circuito manca ULN per le uscite //mixed
		mask = MIXED_MASK;
	}

	if(digitalIO->board==OUTPUT){
		if(output_pin>=8)
			return -1;
		output_pin =  7 - output_pin ; //i led della scheda output sono posizionati al contrario //OUTPUT
		mask = OUTPUT_MASK;

	}

	MCP23S08_WritePin(digitalIO, mask, output_pin, value); //mixed

	return 0;
}

