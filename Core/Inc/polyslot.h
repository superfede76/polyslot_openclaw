/*
 * polyslot.h
 *
 *  Created on: Nov 5, 2025
 *      Author: feder
 */

#ifndef INC_POLYSLOT_H_
#define INC_POLYSLOT_H_

#include "stm32f1xx_hal.h"  // o altro in base alla tua MCU

#define MAX_CS 8
#define MAX_DEMUX 4

#define SLOT_A 	0
#define SLOT_B 	1
#define SLOT_C 	2
#define SLOT_D 	3
#define SLOT_E 	4
#define SLOT_F 	5
#define SLOT_G 	6
#define SLOT_H 	7

enum slot{
	//ID3,ID2,ID1,ID0 0=Diodo 1=open
	cassandra=7, 		//0111 	//poi
	powermeter_board=8,		//1000  //poi
	analog_board=9,		//1001
	pt100_board=10,		//1010
	pulsecounter=11,	//1011
	mixedIO=12,			//1100 	//ok
	output=13,			//1101	//ok
	input=14,			//1110	//ok
	none=15				//1111	//ok
};

typedef struct {
	GPIO_TypeDef* NSS_Port;
	uint16_t NSS_Pin;
} SLOT_CS;

#endif /* INC_POLYSLOT_H_ */
