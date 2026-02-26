#ifndef INC_POWERMETER_IF_H_
#define INC_POWERMETER_IF_H_

#include "stm32f1xx_hal.h"
#include "polyslot.h"
#include <stdbool.h>

#define PM_MAX_REGS        64
#define PM_CMD_READ_REGS   0x01   // comando lettura registri verso STM32H503

/*
 * Legge numRegs registri 16-bit dal modulo powermeter nello slot indicato.
 *  slot     : 0..7 (SLOT_A..SLOT_H)
 *  firstReg : indice primo registro (0..63)
 *  numRegs  : quanti registri leggere (>=1)
 *  dest     : buffer uint16_t dove scrivere i dati
 *
 * Ritorna true se OK, false in caso di errore SPI o parametri invalidi.
 */
bool Powermeter_ReadRegs(uint8_t slot, uint8_t firstReg, uint8_t numRegs, uint16_t *dest);

#endif /* INC_POWERMETER_IF_H_ */
