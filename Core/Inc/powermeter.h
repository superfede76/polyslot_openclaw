/*
 * powermeter.h
 *
 *  Created on: Nov 20, 2025
 *      Author: feder
 */

#ifndef INC_POWERMETER_H_
#define INC_POWERMETER_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>

/*
 * Struttura di gestione della scheda powermeter nello slot.
 * Lo STM32H503 sulla scheda gestisce STPM33/34 e si presenta qui
 * come semplice slave SPI che espone registri 16 bit.
 */
typedef struct
{
    GPIO_TypeDef      *cs_gpio;
    uint16_t           cs_pin;
    SPI_HandleTypeDef *spi;
    uint8_t            lock;
    uint8_t            slot_index;   // 0..7
} powermeter_t;

/* Mappa registri interni della scheda powermeter (indice 0..31)
 * Questi sono indici "logici" che tu implementerai lato STM32H503.
 * Tutti i valori sono 16 bit, con i fattori di scala che decidi tu.
 */
#define PM_REG_VRMS_R          0   // V RMS fase R (es. V*10)
#define PM_REG_IRMS_R          1   // I RMS fase R (es. A*100)
#define PM_REG_VRMS_S          2
#define PM_REG_IRMS_S          3
#define PM_REG_VRMS_T          4
#define PM_REG_IRMS_T          5
#define PM_REG_IRMS_N          6   // I RMS neutro (se presente)
#define PM_REG_FREQ            7   // Frequenza di rete (es. Hz*100)

#define PM_REG_P_TOTAL         8   // Potenza attiva totale (W o W*10)
#define PM_REG_Q_TOTAL         9   // Potenza reattiva totale
#define PM_REG_S_TOTAL         10  // Potenza apparente totale
#define PM_REG_PF_TOTAL        11  // Fattore di potenza totale (es. *1000)

/* Energie 32 bit: HI/LO */
#define PM_REG_E_ACTIVE_HI     12
#define PM_REG_E_ACTIVE_LO     13
#define PM_REG_E_REACTIVE_HI   14
#define PM_REG_E_REACTIVE_LO   15

/* Alcuni registri di fase addizionali (ad es. potenze per fase) */
#define PM_REG_PHASE_P_R       16
#define PM_REG_PHASE_P_S       17
#define PM_REG_PHASE_P_T       18
#define PM_REG_PHASE_Q_R       19
#define PM_REG_PHASE_Q_S       20
#define PM_REG_PHASE_Q_T       21
#define PM_REG_PHASE_PF_R      22
#define PM_REG_PHASE_PF_S      23
#define PM_REG_PHASE_PF_T      24

/* 25..31 riservati per future espansioni */
#define PM_REG_RESERVED_25     25
#define PM_REG_RESERVED_26     26
#define PM_REG_RESERVED_27     27
#define PM_REG_RESERVED_28     28
#define PM_REG_RESERVED_29     29
#define PM_REG_RESERVED_30     30
#define PM_REG_RESERVED_31     31

void powermeter_init(powermeter_t *pm,
                     SPI_HandleTypeDef *spi,
                     GPIO_TypeDef *cs_gpio,
                     uint16_t cs_pin,
                     uint8_t slot_index);

/* Legge un registro 16 bit dalla scheda powermeter nello slot */
bool powermeter_read_reg(powermeter_t *pm, uint8_t reg_index, uint16_t *value);


#endif /* INC_POWERMETER_H_ */
