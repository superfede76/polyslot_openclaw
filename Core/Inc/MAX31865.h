#ifndef _MAX31865_H
#define _MAX31865_H


#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

#define MAX31865_2WIRE 	2
#define MAX31865_3WIRE 	3
#define MAX31865_4WIRE 	4
#define FILT50HZ 		50
#define FILT60HZ 		60

typedef struct
{
	GPIO_TypeDef      *cs_gpio;
	uint16_t          cs_pin;
	SPI_HandleTypeDef *spi;
	uint8_t			channel;
	uint8_t           lock;
	uint8_t			position;

}Max31865_t;

void  Max31865_init(Max31865_t *max31865,SPI_HandleTypeDef *spi,GPIO_TypeDef  *cs_gpio,uint16_t cs_pin,uint8_t ch,uint8_t  numwires, uint8_t filterHz);
bool  Max31865_readTempC(Max31865_t *max31865,float *readTemp);
//bool  Max31865_readTempF(Max31865_t *max31865,float *readTemp);
float Max31865_Filter(float	newInput, float	lastOutput, float efectiveFactor);

#ifdef __cplusplus
}
#endif



#endif
