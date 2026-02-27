/*
 * modbusSlave.h
 *
 *  Created on: Oct 27, 2022
 *      Author: controllerstech.com
 */

#ifndef INC_MODBUSSLAVE_H_
#define INC_MODBUSSLAVE_H_

#include "modbus_crc.h"
#include "stm32f1xx_hal.h"

#define SLAVE_ID_DEFAULT 7
#define MODBUS_MIN_SLAVE_ID 1
#define MODBUS_MAX_SLAVE_ID 247

/* Registro ausiliario Modbus in EEPROM per configurare lo slave ID.
 * Indice 0 => indirizzo Modbus 40001
 */
#define MB_AUX_REGIDX_SLAVE_ID 0

/* Slot type esposti in RAM (stato live rilevato a boot), read-only lato master */
#define RAM_SLOT_TYPE_START_ADDR 40002
#define RAM_SLOT_TYPE_COUNT 8
#define RAM_SLOT_TYPE_END_ADDR (RAM_SLOT_TYPE_START_ADDR + RAM_SLOT_TYPE_COUNT - 1)


#define MAX_MAX_CHIPS 8

#define BIT_PER_SLOT 8
#define HALF_BIT_PER_SLOT 4

#define REG_PER_SLOT      4
#define NUM_SLOT 8


/* --- INPUT REGISTERS --- */

/* Zona base: 4 registri per slot = 32 registri totali
 * 30001..30032
 *  - analog_board
 *  - pt100_board
 *  - pulsecounter
 */
#define INPUTREG_ANALOG_COUNT   (NUM_SLOT * REG_PER_SLOT) // 32
/* Zona powermeter estesa:
 * diamo 32 registri per slot powermeter
 *  - per 8 slot => 256 registri
 *  - indirizzi Modbus: da 30033 in poi
 */


// zona estesa powermeter
#define PM_REGS_PER_SLOT    64      // 64 registri per ogni powermeter
#define PM_BASE_ADDR        30097   // inizia dai registri "disponibili"
#define PM_TOTAL_REGS       (PM_REGS_PER_SLOT * NUM_SLOT)
// indirizzo massimo gestito dai powermeter
#define PM_INPUTREG_FIRST       PM_BASE_ADDR
#define PM_INPUTREG_LAST        (PM_BASE_ADDR + PM_TOTAL_REGS - 1)

/* Zona powermeter estesa: NUM_SLOT * 64 registri */
#define INPUTREG_PM_COUNT       (NUM_SLOT * PM_REGS_PER_SLOT)

/* Totale registri di input disponibili */
#define TOTAL_INPUT_REGS        (INPUTREG_ANALOG_COUNT + INPUTREG_PM_COUNT)

/* Alias leggibile per la sola parte analog/pt100/pulse */
#define BASE_INPUT_REGS         INPUTREG_ANALOG_COUNT

#define COIL_START_ADDR			1
#define CONTACT_START_ADDR	10001
#define INPUTREG_START_ADDR	30001
#define HOLDING_START_ADDR 	40001
#define EEPROM_HOLDING_REGS   1  /* 40001: slave-id */
#define RAM_HOLDING_START_ADDR (HOLDING_START_ADDR + EEPROM_HOLDING_REGS)

/* Base address dei registri powermeter */
#define INPUTREG_PM_START_ADDR  PM_BASE_ADDR

#define TOTAL_COILS 		(NUM_SLOT * BIT_PER_SLOT)
#define TOTAL_CONTACTS 		(NUM_SLOT * BIT_PER_SLOT)
//#define TOTAL_INPUT_REGS	(NUM_SLOT * REG_PER_SLOT)
#define TOTAL_HOLDING_REGS 	128

/* RAM holding registers per acquisizione Cassandra (non persistenti in EEPROM)
 * 40010..40128
 */
#define CASS_HREG_ADC_SELECT      40010
#define CASS_HREG_CHANNEL         40011
#define CASS_HREG_FREQ_HZ         40012
#define CASS_HREG_NUM_SAMPLES     40013
#define CASS_HREG_STATUS          40014
#define CASS_HREG_SAMPLE_OFFSET   40015
#define CASS_HREG_SAMPLES_START   40016
#define CASS_HREG_SAMPLES_END     40128



/*
#define BASE_INPUTREG_SLOTS      (NUM_SLOT * BASE_INPUTREG_PER_SLOT)   // 32
#define PM_EXT_TOTAL_REGS        (NUM_SLOT * PM_EXT_REGS_PER_SLOT)     // 256
#define TOTAL_INPUT_REGS         (BASE_INPUTREG_SLOTS + PM_EXT_TOTAL_REGS) // 288
#define INPUTREG_PM_EXT_START    (INPUTREG_START_ADDR + BASE_INPUTREG_SLOTS) // 30033
*/
#define ILLEGAL_FUNCTION       0x01
#define ILLEGAL_DATA_ADDRESS   0x02
#define ILLEGAL_DATA_VALUE     0x03



uint8_t readHoldingRegs (void);
uint8_t readInputRegs (void);
uint8_t readCoils (void);
uint8_t readInputs (void);

uint8_t writeSingleReg (void);
uint8_t writeHoldingRegs (void);
uint8_t writeSingleCoil (void);
uint8_t writeMultiCoils (void);

void modbusException (uint8_t exceptioncode);

uint8_t Modbus_GetSlaveId(void);
void Modbus_LoadSlaveIdFromEeprom(void);
void Modbus_SaveSlotTypesToEeprom(const uint8_t slotTypes[NUM_SLOT]);
void Modbus_ProcessBackground(void);

#endif /* INC_MODBUSSLAVE_H_ */
