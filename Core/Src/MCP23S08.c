/*
 * MCP23S08.c
 *
 *  Created on: Jul 8, 2025
 *      Author: feder
 */


#include "mcp23s08.h"
#include "main.h"

// Opcodes SPI
#define OPCODE_WRITE 0x40
#define OPCODE_READ  0x41

// Indirizzi registri MCP23S08
#define IODIR_REG    0x00
#define GPIO_REG     0x09
#define OLAT_REG     0x0A


void MCP23S08_Init(DigitalIO_t *digitalIO, uint8_t iodir) {
	HAL_Delay(1);
	HAL_GPIO_WritePin(digitalIO->cs_gpio, digitalIO->cs_pin, GPIO_PIN_RESET);
	uint8_t txBuf[3] = {OPCODE_WRITE, IODIR_REG, iodir}; //IODIR: 0 = output
	HAL_SPI_Transmit(digitalIO->spi, txBuf, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(digitalIO->cs_gpio, digitalIO->cs_pin, GPIO_PIN_SET);
}
/*
void MCP23S08_SetIODIR(uint8_t chip_index, uint8_t iodir) {
    CS_LOW(chip_index);
    uint8_t txBuf[3] = {OPCODE_WRITE, IODIR_REG, iodir};
    HAL_SPI_Transmit(&hspi1, txBuf, 3, HAL_MAX_DELAY);
    CS_HIGH(chip_index);
}
 */
/*
void MCP23S08_WriteGPIO(uint8_t chip_index, uint8_t value) {
    CS_AllHigh(); // Deseleziona tutti prima
    CS_LOW(chip_index); // Seleziona solo il chip desiderato

    uint8_t txBuf[3] = {OPCODE_WRITE, OLAT_REG, value};
    HAL_SPI_Transmit(&hspi1, txBuf, 3, HAL_MAX_DELAY);

    CS_HIGH(chip_index); // Deseleziona il chip
}
 */
uint8_t MCP23S08_WritePin(DigitalIO_t *digitalIO, uint8_t output_mask, uint8_t output_pin, bool output_value) {
	//	HAL_Delay(1);

	uint8_t txBuf[3];
	uint8_t rxBuf[1];
	for(int j=0;j<10000;j++);

	HAL_GPIO_WritePin(digitalIO->cs_gpio, digitalIO->cs_pin, GPIO_PIN_RESET);

	if (!(output_mask & (1 << output_pin))){
		txBuf[0] = OPCODE_READ;
		txBuf[1] = GPIO_REG;
		HAL_SPI_Transmit(digitalIO->spi, txBuf, 2, HAL_MAX_DELAY);

		HAL_SPI_Receive(digitalIO->spi, rxBuf, 1, HAL_MAX_DELAY);

		HAL_GPIO_WritePin(digitalIO->cs_gpio, digitalIO->cs_pin, GPIO_PIN_SET);
		//    HAL_Delay(1);
		HAL_GPIO_WritePin(digitalIO->cs_gpio, digitalIO->cs_pin, GPIO_PIN_RESET);

		if (output_value) //acceso
			rxBuf[0] |= (1 << output_pin); //imposta

		else
			rxBuf[0] &= ~(1 << output_pin);


		// 2. Scrivi sui pin in output (OLAT)
		txBuf[0] = OPCODE_WRITE;
		txBuf[1] = OLAT_REG;
		txBuf[2] = rxBuf[0] & ~output_mask; // Scrivi solo su quelli output
		HAL_SPI_Transmit(digitalIO->spi, txBuf, 3, HAL_MAX_DELAY);

		HAL_GPIO_WritePin(digitalIO->cs_gpio, digitalIO->cs_pin, GPIO_PIN_SET);
		return 0;
	}
	return -1;

}

uint8_t MCP23S08_ReadPin(DigitalIO_t *digitalIO, uint8_t input_pin) {
	//	HAL_Delay(1);
	for(int j=0;j<10000;j++);
	HAL_GPIO_WritePin(digitalIO->cs_gpio, digitalIO->cs_pin, GPIO_PIN_RESET);

	uint8_t txBuf[3];

	// Leggi lo stato dei GPIO
	uint8_t rxBuf[1];
	txBuf[0] = OPCODE_READ;
	txBuf[1] = GPIO_REG;
	HAL_SPI_Transmit(digitalIO->spi, txBuf, 2, HAL_MAX_DELAY);
	HAL_SPI_Receive(digitalIO->spi, rxBuf, 1, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(digitalIO->cs_gpio, digitalIO->cs_pin, GPIO_PIN_SET);

	//Estrai lo stato del singolo pin
	return !((rxBuf[0]>> input_pin) & 0x01);
}

uint8_t MCP23S08_WritePort(DigitalIO_t *digitalIO, uint8_t output_mask, uint8_t output_value) {
	HAL_Delay(1);
	HAL_GPIO_WritePin(digitalIO->cs_gpio, digitalIO->cs_pin, GPIO_PIN_RESET);

	uint8_t txBuf[3];

	uint8_t rxBuf[1];
	//    if (!(output_mask & (1 << output_pin))){
	txBuf[0] = OPCODE_READ;
	txBuf[1] = GPIO_REG;
	HAL_SPI_Transmit(digitalIO->spi, txBuf, 2, HAL_MAX_DELAY);

	HAL_SPI_Receive(digitalIO->spi, rxBuf, 1, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(digitalIO->cs_gpio, digitalIO->cs_pin, GPIO_PIN_SET);
	//    HAL_Delay(1);
	HAL_GPIO_WritePin(digitalIO->cs_gpio, digitalIO->cs_pin, GPIO_PIN_RESET);

	rxBuf[0] = output_value; //imposta

	// 2. Scrivi sui pin in output (OLAT)
	txBuf[0] = OPCODE_WRITE;
	txBuf[1] = OLAT_REG;
	txBuf[2] = rxBuf[0] & ~output_mask; // Scrivi solo su quelli output
	HAL_SPI_Transmit(digitalIO->spi, txBuf, 3, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(digitalIO->cs_gpio, digitalIO->cs_pin, GPIO_PIN_SET);
	return 0;
	//    }
	//return -1;

}
