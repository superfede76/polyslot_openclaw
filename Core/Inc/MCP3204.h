/*
 * MCP3204.h
 *
 *  Created on: Jul 10, 2025
 *      Author: feder
 */

#ifndef INC_MCP3204_H_
#define INC_MCP3204_H_

#include "stm32f1xx_hal.h" // Adatta alla tua famiglia STM32


typedef struct
{
  GPIO_TypeDef      *cs_gpio;
  uint16_t          cs_pin;
  SPI_HandleTypeDef *spi;
  uint8_t           lock;
  uint8_t 			position;

}Mcp3204_t;


#define VREF 2048

void  Mcp3204_init(Mcp3204_t *mcp3204,SPI_HandleTypeDef *spi,GPIO_TypeDef  *cs_gpio,uint16_t cs_pin);
uint16_t MCP3204_ReadChannel(Mcp3204_t *mcp3204, uint8_t channel);
float MCP3204_ConvertToMA(uint16_t adcValue, float vref);

#endif /* INC_MCP3204_H_ */
