/*
 * MCP3204.c
 *
 *  Created on: Jul 10, 2025
 *      Author: feder
 */


#include "mcp3204.h"
#include "main.h"
#include "polyslot.h"

#define ADC_DATA_RESOL    4095

extern SLOT_CS slot_cs_array[MAX_CS];


/*
extern void CS_AllHigh(void);
extern void CS_LOW(uint8_t chip_index);
extern void CS_HIGH(uint8_t chip_index);
*/
float res=0;

void  Mcp3204_init(Mcp3204_t *mcp3204,SPI_HandleTypeDef *spi,GPIO_TypeDef  *cs_gpio,uint16_t cs_pin)
{
  if(mcp3204->lock == 1)
    HAL_Delay(1);
  mcp3204->lock = 1;
  mcp3204->spi = spi;
  mcp3204->cs_gpio = cs_gpio;
  mcp3204->cs_pin = cs_pin;
  HAL_GPIO_WritePin(mcp3204->cs_gpio,mcp3204->cs_pin,GPIO_PIN_SET);
  HAL_Delay(100);
}


uint16_t MCP3204_ReadChannel(Mcp3204_t *mcp3204, uint8_t channel) {

	uint8_t tx_data[3];
	uint8_t rx_data[3]={0};
	uint16_t adc_value=0;



	tx_data[0]= 0x06|((channel >> 2) & 0x01); // Start + SGL + D2;
	tx_data[1]= (channel & 0x03) << 6;   // D1 D0 nei 2 bit alti
	tx_data[2]=0;// Dummy per ricevere dati


	HAL_GPIO_WritePin(mcp3204->cs_gpio, mcp3204->cs_pin, GPIO_PIN_RESET);
	for(int j=0;j<10000;j++);

	// Transmit and receive data over SPI
	if (HAL_SPI_TransmitReceive(mcp3204->spi, tx_data, rx_data, 3, HAL_MAX_DELAY) == HAL_OK) {


		adc_value = ((rx_data[1] & 0x0F) << 8) | rx_data[2];
		res=adc_value;

		res*=0.0005;


		HAL_GPIO_WritePin(mcp3204->cs_gpio, mcp3204->cs_pin, GPIO_PIN_SET);
	}

	return adc_value;
}

float MCP3204_ConvertToMA(uint16_t adcValue, float vref) {
	return ((float)adcValue * vref) / 4096.0f/100; //il diviso 100 provine dal fatto che la resistenza di shunt è di 5 ohm e l'amplificazione del INA e di 20V/V
}
