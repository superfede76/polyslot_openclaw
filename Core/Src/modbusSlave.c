/*
 * modbusSlave.c
 *
 *  Created on: Oct 27, 2022
 *  Regenerato: Nov 2025
 */

#include "modbusSlave.h"
#include "string.h"
#include <math.h>
#include <limits.h>
#include <stdint.h>

#include "digitalIO.h"
#include "MAX31865.h"
#include "MCP3204.h"
#include "pulse_counter.h"
#include "polyslot.h"
#include "powermeter.h"
#include "powermeter_if.h"


#define EEPROM_ADDRESS  (0x50 << 1)

/* -------------------- extern dal main -------------------- */

extern uint8_t RxData[256];
extern uint8_t TxData[256];

extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef  hi2c1;

extern SPI_HandleTypeDef hspi1;
extern SLOT_CS slot_cs_array[MAX_CS];


/* mappe logiche */
extern bool coils_table[TOTAL_COILS];
extern bool contact_table[TOTAL_CONTACTS];
extern bool inputReg_table[TOTAL_INPUT_REGS];

/* HW per slot */
extern DigitalIO_t  digitalIO[MAX_CS];
extern Max31865_t   pt100[MAX_CS][MAX_DEMUX];
extern Mcp3204_t    analog[MAX_CS];
extern pulse_t      counter[MAX_CS];
extern powermeter_t powermeter[MAX_CS];

extern enum slot installed_slot[NUM_SLOT];

static uint8_t g_modbus_slave_id = SLAVE_ID_DEFAULT;

#define CASS_STATUS_IDLE   0
#define CASS_STATUS_BUSY   1
#define CASS_STATUS_DONE   2
#define CASS_STATUS_ERROR  3
#define CASS_MAX_SAMPLES   1024

static uint16_t g_cass_samples[CASS_MAX_SAMPLES];
static uint16_t g_cass_adc_select = 0;
static uint16_t g_cass_channel = 0;
static uint16_t g_cass_freq_hz = 1;
static uint16_t g_cass_num_samples = 1;
static uint16_t g_cass_sample_offset = 0;
static uint16_t g_cass_last_count = 0;
static uint16_t g_cass_status = CASS_STATUS_IDLE;
static volatile uint8_t g_cass_run_pending = 0;

static bool cassandra_run_acquisition(void);

static bool is_valid_slave_id(uint16_t id)
{
	return (id >= MODBUS_MIN_SLAVE_ID) && (id <= MODBUS_MAX_SLAVE_ID);
}

uint8_t Modbus_GetSlaveId(void)
{
	return g_modbus_slave_id;
}

void Modbus_LoadSlaveIdFromEeprom(void)
{
	uint8_t raw[2] = {0};
	if (HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS, MB_AUX_REGIDX_SLAVE_ID * 2,
			I2C_MEMADD_SIZE_8BIT, raw, 2, HAL_MAX_DELAY) != HAL_OK)
	{
		g_modbus_slave_id = SLAVE_ID_DEFAULT;
		return;
	}

	uint16_t configured = ((uint16_t)raw[0] << 8) | raw[1];
	if (is_valid_slave_id(configured)) {
		g_modbus_slave_id = (uint8_t)configured;
	} else {
		g_modbus_slave_id = SLAVE_ID_DEFAULT;
	}
}

void Modbus_SaveSlotTypesToEeprom(const uint8_t slotTypes[NUM_SLOT])
{
	if (slotTypes == NULL) return;

	/*
	 * Evita write multi-byte unico: su molte EEPROM I2C i write che attraversano
	 * il page-boundary fanno wrap e possono corrompere indirizzi precedenti
	 * (incluso il registro slave-id a byte 0..1).
	 *
	 * Scriviamo 1 registro (2 byte) alla volta, solo nel range 40002..40009.
	 */
	for (uint8_t i = 0; i < MB_AUX_SLOT_TYPE_COUNT; i++) {
		uint8_t raw[2] = {0x00, slotTypes[i]};
		uint16_t byteAddr = (uint16_t)(MB_AUX_REGIDX_SLOT_TYPE_BASE + i) * 2;
		(void)HAL_I2C_Mem_Write(&hi2c1,
				EEPROM_ADDRESS,
				byteAddr,
				I2C_MEMADD_SIZE_8BIT,
				raw,
				sizeof(raw),
				HAL_MAX_DELAY);
	}
}

void Modbus_ProcessBackground(void)
{
	if (!g_cass_run_pending) return;
	g_cass_run_pending = 0;

	if (!cassandra_run_acquisition()) {
		g_cass_status = CASS_STATUS_ERROR;
	}
}

static bool cass_holding_is_ram(uint16_t addr)
{
	return (addr >= CASS_HREG_ADC_SELECT) && (addr <= CASS_HREG_SAMPLES_END);
}

static int8_t cassandra_find_slot(void)
{
	for (uint8_t i = 0; i < NUM_SLOT; i++) {
		if (installed_slot[i] == cassandra) return (int8_t)i;
	}
	return -1;
}

static void cassandra_select_adc(uint8_t adc)
{
	// 00->ADC0, 01->ADC1, 10->ADC2, 11 non usato
	HAL_GPIO_WritePin(DEMUX_A_GPIO_Port, DEMUX_A_Pin, (adc & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DEMUX_B_GPIO_Port, DEMUX_B_Pin, (adc & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint16_t cassandra_read_sample_raw(uint8_t slot, uint8_t adc, uint8_t ch)
{
	uint8_t tx[2] = {0};
	uint8_t rx[2] = {0};
	// Comando canale: CH0/CH1 (0/1)
	tx[0] = (uint8_t)((ch & 0x01) << 3);
	tx[1] = 0x00;

	cassandra_select_adc(adc);
	HAL_GPIO_WritePin(slot_cs_array[slot].NSS_Port, slot_cs_array[slot].NSS_Pin, GPIO_PIN_RESET);
	if (HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY) != HAL_OK) {
		HAL_GPIO_WritePin(slot_cs_array[slot].NSS_Port, slot_cs_array[slot].NSS_Pin, GPIO_PIN_SET);
		return 0;
	}
	HAL_GPIO_WritePin(slot_cs_array[slot].NSS_Port, slot_cs_array[slot].NSS_Pin, GPIO_PIN_SET);

	return (uint16_t)(((uint16_t)(rx[0] & 0x0F) << 8) | rx[1]); // 12-bit raw
}

static bool cassandra_run_acquisition(void)
{
	if (g_cass_adc_select > 2) return false;
	if (g_cass_channel > 1) return false;
	if (g_cass_freq_hz == 0) return false;
	if ((g_cass_num_samples == 0) || (g_cass_num_samples > CASS_MAX_SAMPLES)) return false;

	int8_t slot = cassandra_find_slot();
	if (slot < 0) return false;

	g_cass_status = CASS_STATUS_BUSY;
	uint32_t period_ms = 1000u / g_cass_freq_hz;
	for (uint16_t i = 0; i < g_cass_num_samples; i++) {
		g_cass_samples[i] = cassandra_read_sample_raw((uint8_t)slot, (uint8_t)g_cass_adc_select, (uint8_t)g_cass_channel);
		if (period_ms > 0) HAL_Delay(period_ms);
	}
	g_cass_last_count = g_cass_num_samples;
	if (g_cass_sample_offset >= g_cass_last_count) g_cass_sample_offset = 0;
	g_cass_status = CASS_STATUS_DONE;
	return true;
}

static uint16_t cassandra_ram_holding_read(uint16_t addr)
{
	switch (addr) {
	case CASS_HREG_ADC_SELECT: return g_cass_adc_select;
	case CASS_HREG_CHANNEL: return g_cass_channel;
	case CASS_HREG_FREQ_HZ: return g_cass_freq_hz;
	case CASS_HREG_NUM_SAMPLES: return g_cass_num_samples;
	case CASS_HREG_STATUS: return g_cass_status;
	case CASS_HREG_SAMPLE_OFFSET: return g_cass_sample_offset;
	default:
		if ((addr >= CASS_HREG_SAMPLES_START) && (addr <= CASS_HREG_SAMPLES_END)) {
			uint16_t rel = (uint16_t)(addr - CASS_HREG_SAMPLES_START);
			uint16_t idx = (uint16_t)(g_cass_sample_offset + rel);
			if (idx < g_cass_last_count) return g_cass_samples[idx];
			return 0;
		}
		return 0;
	}
}

/* -------------------- Utility Modbus -------------------- */

void sendData (uint8_t *data, int size)
{
	uint16_t crc = crc16(data, size);
	data[size]   = crc & 0xFF;         // CRC LOW
	data[size+1] = (crc >> 8) & 0xFF;  // CRC HIGH

	HAL_UART_Transmit(&huart1, data, size+2, 1000);
}

void modbusException (uint8_t exceptioncode)
{
	//| SLAVE_ID | FUNCTION_CODE | Exception code | CRC |
	TxData[0] = RxData[0];          // slave ID
	TxData[1] = RxData[1] | 0x80;   // set MSB
	TxData[2] = exceptioncode;
	sendData(TxData, 3);
}

/* =========================================================
 *                  FUNZIONI DI SUPPORTO
 * ========================================================= */

/* Converte temperatura PT100 (float °C) in int16_t (°C * 100)
 * e la restituisce come uint16_t (two's complement).
 * In caso di errore ritorna 0x8000 come sentinel.
 */
static uint16_t pt100_to_int16_scaled(Max31865_t *dev)
{
	float tempC;
	bool ok = Max31865_readTempC(dev, &tempC);
	if (!ok) {
		return (uint16_t)0x8000;   // sentinel errore
	}

	// scala di 100 con arrotondamento
	float scaledF = tempC * 100.0f;
	int32_t scaled = (int32_t)((scaledF >= 0.0f) ? (scaledF + 0.5f) : (scaledF - 0.5f));

	if (scaled > INT16_MAX) scaled = INT16_MAX;
	if (scaled < INT16_MIN) scaled = INT16_MIN;

	int16_t sval = (int16_t)scaled;
	return (uint16_t)sval;   // reinterpretazione two's complement
}

/* Converte corrente in mA (float) in uint16_t (mA * 100).
 * Esempio 4.00mA -> 400, 20.00mA -> 2000
 */
static uint16_t current_to_uint16_scaled(float mA)
{
	if (mA < 0.0f) mA = 0.0f;
	float scaledF = mA * 100.0f;
	uint32_t scaled = (uint32_t)(scaledF + 0.5f);
	if (scaled > 0xFFFFu) scaled = 0xFFFFu;
	return (uint16_t)scaled;
}

/* Ritorna il valore dell’input register indicizzato
 * index = 0 -> 30001 (slot0 ch0)
 * index = 1 -> 30002 (slot0 ch1)
 * ...
 */
static uint16_t get_input_register_value_by_index(uint16_t index)
{

	// Parte base: 4 registri per slot (30001..30032)
	if (index < INPUTREG_ANALOG_COUNT)
	{
        uint16_t slot = index / REG_PER_SLOT; // 0..7
        uint16_t ch   = index % REG_PER_SLOT; // 0..3

		if (slot >= NUM_SLOT) {
			return 0;
		}

		switch (installed_slot[slot]) {

		case analog_board:
		{
			// Analog 4–20mA -> valore mA * 100
			if (analog[slot].spi == NULL || analog[slot].cs_gpio == NULL)
				return 0;

			uint16_t adc = MCP3204_ReadChannel(&analog[slot], (uint8_t)ch);
			float current_mA = MCP3204_ConvertToMA(adc, (float)VREF);
			return current_to_uint16_scaled(current_mA);
		}

		case pt100_board:
		{
			// Temperatura in °C * 100
			if (pt100[slot][ch].spi == NULL || pt100[slot][ch].cs_gpio == NULL)
				return (uint16_t)0x8000;

			return pt100_to_int16_scaled(&pt100[slot][ch]);
		}

		case pulsecounter:
		{
			if (counter[slot].spi == NULL || counter[slot].cs_gpio == NULL)
				return 0;

			uint16_t cnt = 0;
			bool ok = counter_readPulse(&counter[slot], &cnt, (uint8_t)ch);
			if (!ok) return 0;
			return cnt;   // valore grezzo 0..65535
		}
		default:
			// slot senza registri analogici
			return 0;
		}
	}
    /* Zona powermeter */
    uint16_t pm_index = index - INPUTREG_ANALOG_COUNT;
    uint16_t slot     = pm_index / PM_REGS_PER_SLOT;  // slot 0..7
    uint16_t reg      = pm_index % PM_REGS_PER_SLOT;  // 0..31

    if (slot >= NUM_SLOT) return 0;
    if (installed_slot[slot] != powermeter_board) return 0;

    uint16_t value = 0;
    if (!powermeter_read_reg(&powermeter[slot], (uint8_t)reg, &value))
        return 0;

    return value;
}

/* =========================================================
 *                      FUNZIONE 0x01
 *                    READ COILS (1 bit)
 * ========================================================= */

uint8_t readCoils (void)
{
	uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);
	uint16_t numCoils  = ((RxData[4] << 8) | RxData[5]);

	if ((numCoils < 1) || (numCoils > TOTAL_COILS)) {
		modbusException(ILLEGAL_DATA_VALUE);
		return 0;
	}

	uint16_t endAddr = startAddr + numCoils - 1;

	if ((startAddr < COIL_START_ADDR) ||
			(endAddr   > COIL_START_ADDR + TOTAL_COILS - 1))
	{
		modbusException(ILLEGAL_DATA_ADDRESS);
		return 0;
	}

	// Indici 0-based nella tabella coil
	uint16_t startIdx = startAddr - COIL_START_ADDR;
	uint16_t endIdx   = endAddr   - COIL_START_ADDR;

	// Verifica che tutte le coil richieste esistano realmente
	for (uint16_t k = startIdx; k <= endIdx; k++) {
		if (!coils_table[k]) {
			modbusException(ILLEGAL_DATA_ADDRESS);
			return 0;
		}
	}

	uint8_t byteCount = (uint8_t)((numCoils + 7) / 8);

	TxData[0] = Modbus_GetSlaveId();
	TxData[1] = RxData[1];   // function code
	TxData[2] = byteCount;

	int outIdx = 3;
	uint8_t currentByte = 0;
	uint8_t bitPos = 0;

	for (uint16_t idx = startIdx; idx <= endIdx; idx++) {
		uint16_t slot = idx / BIT_PER_SLOT;
		uint16_t pin  = idx % BIT_PER_SLOT;

		uint8_t val = digitalIO_Read(&digitalIO[slot], (uint8_t)pin) & 0x01;

		currentByte |= (val << bitPos);
		bitPos++;

		if (bitPos == 8) {
			TxData[outIdx++] = currentByte;
			currentByte = 0;
			bitPos = 0;
		}
	}

	if (bitPos > 0) {
		TxData[outIdx++] = currentByte;
	}

	sendData(TxData, outIdx);
	return 1;
}

/* =========================================================
 *                      FUNZIONE 0x02
 *                READ DISCRETE INPUTS (1 bit)
 * ========================================================= */

uint8_t readInputs (void)
{
	uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);
	uint16_t numInputs = ((RxData[4] << 8) | RxData[5]);

	if ((numInputs < 1) || (numInputs > TOTAL_CONTACTS)) {
		modbusException(ILLEGAL_DATA_VALUE);
		return 0;
	}

	uint16_t endAddr = startAddr + numInputs - 1;

	if ((startAddr < CONTACT_START_ADDR) ||
			(endAddr   > CONTACT_START_ADDR + TOTAL_CONTACTS - 1))
	{
		modbusException(ILLEGAL_DATA_ADDRESS);
		return 0;
	}

	uint16_t startIdx = startAddr - CONTACT_START_ADDR;
	uint16_t endIdx   = endAddr   - CONTACT_START_ADDR;

	// Verifica mappa dei contatti
	for (uint16_t k = startIdx; k <= endIdx; k++) {
		if (!contact_table[k]) {
			modbusException(ILLEGAL_DATA_ADDRESS);
			return 0;
		}
	}

	uint8_t byteCount = (uint8_t)((numInputs + 7) / 8);

	TxData[0] = Modbus_GetSlaveId();
	TxData[1] = RxData[1];  // function code
	TxData[2] = byteCount;

	int outIdx = 3;
	uint8_t currentByte = 0;
	uint8_t bitPos = 0;

	for (uint16_t idx = startIdx; idx <= endIdx; idx++) {
		uint16_t slot = idx / BIT_PER_SLOT;
		uint16_t pin  = idx % BIT_PER_SLOT;

		uint8_t val = digitalIO_Read(&digitalIO[slot], (uint8_t)pin) & 0x01;

		currentByte |= (val << bitPos);
		bitPos++;

		if (bitPos == 8) {
			TxData[outIdx++] = currentByte;
			currentByte = 0;
			bitPos = 0;
		}
	}

	if (bitPos > 0) {
		TxData[outIdx++] = currentByte;
	}

	sendData(TxData, outIdx);
	return 1;
}

/* =========================================================
 *                      FUNZIONE 0x03
 *             READ HOLDING REGISTERS (EEPROM)
 * ========================================================= */

uint8_t readHoldingRegs (void)
{
	uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);
	uint16_t numRegs   = ((RxData[4] << 8) | RxData[5]);

	if ((numRegs < 1) || (numRegs > TOTAL_HOLDING_REGS)) {
		modbusException(ILLEGAL_DATA_VALUE);
		return 0;
	}

	uint16_t endAddr = startAddr + numRegs - 1;
	if ((startAddr < HOLDING_START_ADDR) ||
			(endAddr   > HOLDING_START_ADDR + TOTAL_HOLDING_REGS - 1))
	{
		modbusException(ILLEGAL_DATA_ADDRESS);
		return 0;
	}

	uint16_t regIndex  = startAddr - HOLDING_START_ADDR;  // 0-based
	uint16_t byteAddr  = regIndex * 2;                    // 2 byte per registro
	uint16_t byteCount = numRegs * 2;

	TxData[0] = Modbus_GetSlaveId();
	TxData[1] = RxData[1];
	TxData[2] = (uint8_t)byteCount;
	int outIdx = 3;

	if (cass_holding_is_ram(startAddr) || cass_holding_is_ram(endAddr)) {
		if (!(cass_holding_is_ram(startAddr) && cass_holding_is_ram(endAddr))) {
			modbusException(ILLEGAL_DATA_ADDRESS); // no range misti RAM/EEPROM
			return 0;
		}
		for (uint16_t a = startAddr; a <= endAddr; a++) {
			uint16_t v = cassandra_ram_holding_read(a);
			TxData[outIdx++] = (uint8_t)(v >> 8);
			TxData[outIdx++] = (uint8_t)(v & 0xFF);
		}
		sendData(TxData, outIdx);
		return 1;
	}

	uint8_t buffer[256];
	if (HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS, byteAddr, I2C_MEMADD_SIZE_8BIT,
			buffer, byteCount, HAL_MAX_DELAY) != HAL_OK)
	{
		modbusException(ILLEGAL_FUNCTION);
		return 0;
	}

	for (uint16_t i = 0; i < byteCount; i++) {
		TxData[outIdx++] = buffer[i];
	}

	/*
	 * Coerenza runtime: il registro 40001 (slave-id) deve riflettere
	 * sempre l'ID realmente in uso sul bus, anche se EEPROM è stale.
	 */
	if ((regIndex <= MB_AUX_REGIDX_SLAVE_ID) && ((regIndex + numRegs - 1) >= MB_AUX_REGIDX_SLAVE_ID)) {
		uint16_t off = (MB_AUX_REGIDX_SLAVE_ID - regIndex) * 2;
		TxData[3 + off] = 0x00;
		TxData[3 + off + 1] = Modbus_GetSlaveId();
	}

	/*
	 * I registri 40002..40009 (slot type) sono esposti come stato LIVE
	 * dal runtime (installed_slot), non da EEPROM, così riflettono sempre
	 * il rilevamento effettivo post-boot.
	 */
	for (uint16_t i = 0; i < numRegs; i++) {
		uint16_t idx = regIndex + i;
		if ((idx >= MB_AUX_REGIDX_SLOT_TYPE_BASE) && (idx <= MB_AUX_REGIDX_SLOT_TYPE_LAST)) {
			uint16_t off = i * 2;
			uint8_t slotIx = (uint8_t)(idx - MB_AUX_REGIDX_SLOT_TYPE_BASE);
			uint16_t v = (uint16_t)installed_slot[slotIx];
			TxData[3 + off] = (uint8_t)(v >> 8);
			TxData[3 + off + 1] = (uint8_t)(v & 0xFF);
		}
	}

	sendData(TxData, outIdx);
	return 1;
}



/* =========================================================
 *                      FUNZIONE 0x04
 *                 READ INPUT REGISTERS (RAW)
 * ========================================================= */

uint8_t readInputRegs (void)
{
    uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);
    uint16_t numRegs   = ((RxData[4] << 8) | RxData[5]);

    /* Check di base */
    if ((numRegs < 1) || (numRegs > TOTAL_INPUT_REGS)) {
        modbusException(ILLEGAL_DATA_VALUE);
        return 0;
    }

    uint16_t endAddr = startAddr + numRegs - 1;

    if ((startAddr < INPUTREG_START_ADDR) ||
        (endAddr   > INPUTREG_START_ADDR + TOTAL_INPUT_REGS - 1))
    {
        modbusException(ILLEGAL_DATA_ADDRESS);
        return 0;
    }

    /* Offset 0-based interno */
    uint16_t startIndex = startAddr - INPUTREG_START_ADDR;
    uint16_t endIndex   = endAddr   - INPUTREG_START_ADDR;

    /* === Caso 1: registri 30001..30032 (analog / pt100 / pulse counter) === */
    if (endAddr <= (INPUTREG_START_ADDR + INPUTREG_ANALOG_COUNT - 1))   // <= 30032
    {
        /* Controllo mappa inputReg_table */
        for (uint16_t k = startIndex; k <= endIndex; k++) {
            if (!inputReg_table[k]) {
                modbusException(ILLEGAL_DATA_ADDRESS);
                return 0;
            }
        }

        TxData[0] = Modbus_GetSlaveId();
        TxData[1] = RxData[1];
        TxData[2] = (uint8_t)(numRegs * 2);
        int indx  = 3;

        for (uint16_t idx = startIndex; idx <= endIndex; idx++) {
            uint16_t value = get_input_register_value_by_index(idx);
            TxData[indx++] = (uint8_t)(value >> 8);
            TxData[indx++] = (uint8_t)(value & 0xFF);
        }

        sendData(TxData, indx);
        return 1;
    }

    /* === Caso 2: registri powermeter (da PM_BASE_ADDR = 30097 in poi) === */
    if (startAddr >= PM_BASE_ADDR)
    {
        /* Mapping:
         *  PM_BASE_ADDR..PM_BASE_ADDR+63   -> slot 0 (reg 0..63)
         *  PM_BASE_ADDR+64..PM_BASE_ADDR+127 -> slot 1
         *  ecc.
         */
        uint16_t pmOffset = (uint16_t)(startAddr - PM_BASE_ADDR); // 0-based nella zona powermeter

        uint8_t slot     = (uint8_t)(pmOffset / PM_REGS_PER_SLOT);  // 0..7
        uint8_t firstReg = (uint8_t)(pmOffset % PM_REGS_PER_SLOT);  // 0..63

        if (slot >= NUM_SLOT) {
            modbusException(ILLEGAL_DATA_ADDRESS);
            return 0;
        }

        if (installed_slot[slot] != powermeter_board) {
            modbusException(ILLEGAL_DATA_ADDRESS);
            return 0;
        }

        /* Non sforare i 64 registri del modulo */
        if ((firstReg + numRegs) > PM_MAX_REGS) {
            modbusException(ILLEGAL_DATA_ADDRESS);
            return 0;
        }

        uint16_t tmp[PM_MAX_REGS];

        if (!Powermeter_ReadRegs(slot, firstReg, (uint8_t)numRegs, tmp)) {
            modbusException(ILLEGAL_FUNCTION);
            return 0;
        }

        TxData[0] = Modbus_GetSlaveId();
        TxData[1] = RxData[1];
        TxData[2] = (uint8_t)(numRegs * 2);
        int indx  = 3;

        for (uint16_t i = 0; i < numRegs; i++) {
            TxData[indx++] = (uint8_t)(tmp[i] >> 8);
            TxData[indx++] = (uint8_t)(tmp[i] & 0xFF);
        }

        sendData(TxData, indx);
        return 1;
    }

    /* Se si cade qui, range misti o buchi (es. 30020..30100) => non supportato */
    modbusException(ILLEGAL_DATA_ADDRESS);
    return 0;
}


/*
uint8_t readInputRegs (void)
{
	uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);
	uint16_t numRegs   = ((RxData[4] << 8) | RxData[5]);

	if ((numRegs < 1) || (numRegs > TOTAL_INPUT_REGS)) {
		modbusException(ILLEGAL_DATA_VALUE);
		return 0;
	}

	uint16_t endAddr = startAddr + numRegs - 1;

	if ((startAddr < INPUTREG_START_ADDR) ||
			(endAddr   > INPUTREG_START_ADDR + TOTAL_INPUT_REGS - 1))
	{
		modbusException(ILLEGAL_DATA_ADDRESS);
		return 0;
	}

	uint16_t startIndex = startAddr - INPUTREG_START_ADDR;
	uint16_t endIndex   = endAddr   - INPUTREG_START_ADDR;

	// Verifica mappa inputReg_table
	for (uint16_t k = startIndex; k <= endIndex; k++) {
		if (!inputReg_table[k]) {
			modbusException(ILLEGAL_DATA_ADDRESS);
			return 0;
		}
	}

	TxData[0] = Modbus_GetSlaveId();
	TxData[1] = RxData[1];
	TxData[2] = (uint8_t)(numRegs * 2);   // byte count

	int outIdx = 3;

	for (uint16_t idx = startIndex; idx <= endIndex; idx++) {
		uint16_t value = get_input_register_value_by_index(idx);
		//        TxData[outIdx++] = (uint8_t)((value >> 8) & 0xFF);
		TxData[outIdx++] = (uint8_t)( value       & 0xFF);
		TxData[outIdx++] = (uint8_t)((value >> 8) & 0xFF);

	}

	sendData(TxData, outIdx);
	return 1;
}
*/

/* =========================================================
 *                      FUNZIONE 0x05
 *                  WRITE SINGLE COIL (1 bit)
 * ========================================================= */

uint8_t writeSingleCoil (void)
{
	uint16_t addr = ((RxData[2] << 8) | RxData[3]);

	if ((addr < COIL_START_ADDR) ||
			(addr > COIL_START_ADDR + TOTAL_COILS - 1))
	{
		modbusException(ILLEGAL_DATA_ADDRESS);
		return 0;
	}

	uint16_t idx = addr - COIL_START_ADDR;

	if (!coils_table[idx]) {
		modbusException(ILLEGAL_DATA_ADDRESS);
		return 0;
	}

	uint16_t slot = idx / BIT_PER_SLOT;
	uint16_t pin  = idx % BIT_PER_SLOT;

	bool value;

	if (RxData[4] == 0xFF && RxData[5] == 0x00) {
		value = true;
	} else if (RxData[4] == 0x00 && RxData[5] == 0x00) {
		value = false;
	} else {
		modbusException(ILLEGAL_DATA_VALUE);
		return 0;
	}

	digitalIO_Write(&digitalIO[slot], (uint8_t)pin, value);

	// Echo
	TxData[0] = Modbus_GetSlaveId();
	TxData[1] = RxData[1];
	TxData[2] = RxData[2];
	TxData[3] = RxData[3];
	TxData[4] = RxData[4];
	TxData[5] = RxData[5];

	sendData(TxData, 6);
	return 1;
}

/* =========================================================
 *                      FUNZIONE 0x06
 *              WRITE SINGLE HOLDING REGISTER
 * ========================================================= */

uint8_t writeSingleReg (void)
{
	uint16_t addr = ((RxData[2] << 8) | RxData[3]);

	if ((addr < HOLDING_START_ADDR) ||
			(addr > HOLDING_START_ADDR + TOTAL_HOLDING_REGS - 1))
	{
		modbusException(ILLEGAL_DATA_ADDRESS);
		return 0;
	}

	uint16_t regIndex = addr - HOLDING_START_ADDR;
	uint16_t byteAddr = regIndex * 2;

	if (cass_holding_is_ram(addr)) {
		uint16_t value = ((uint16_t)RxData[4] << 8) | RxData[5];
		switch (addr) {
		case CASS_HREG_ADC_SELECT: g_cass_adc_select = value; break;
		case CASS_HREG_CHANNEL: g_cass_channel = value; break;
		case CASS_HREG_FREQ_HZ: g_cass_freq_hz = value; break;
		case CASS_HREG_NUM_SAMPLES:
			g_cass_num_samples = value;
			g_cass_status = CASS_STATUS_BUSY;
			g_cass_run_pending = 1;
			break;
		case CASS_HREG_SAMPLE_OFFSET: g_cass_sample_offset = value; break;
		default:
			modbusException(ILLEGAL_DATA_ADDRESS);
			return 0;
		}

		TxData[0] = Modbus_GetSlaveId();
		TxData[1] = RxData[1];
		TxData[2] = RxData[2];
		TxData[3] = RxData[3];
		TxData[4] = RxData[4];
		TxData[5] = RxData[5];
		sendData(TxData, 6);
		return 1;
	}

	if ((regIndex >= MB_AUX_REGIDX_SLOT_TYPE_BASE) && (regIndex <= MB_AUX_REGIDX_SLOT_TYPE_LAST)) {
		modbusException(ILLEGAL_DATA_ADDRESS); // registri slot type in sola lettura lato master
		return 0;
	}

	uint8_t buffer[2];
	buffer[0] = RxData[4];
	buffer[1] = RxData[5];

	uint16_t newSlaveId = ((uint16_t)buffer[0] << 8) | buffer[1];
	bool changeSlaveId = (regIndex == MB_AUX_REGIDX_SLAVE_ID);
	if (changeSlaveId && !is_valid_slave_id(newSlaveId)) {
		modbusException(ILLEGAL_DATA_VALUE);
		return 0;
	}

	if (HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, byteAddr, I2C_MEMADD_SIZE_8BIT,
			buffer, 2, HAL_MAX_DELAY) != HAL_OK)
	{
		modbusException(ILLEGAL_FUNCTION);
		return 0;
	}

	// Echo con ID della richiesta (prima di eventuale cambio ID runtime)
	TxData[0] = RxData[0];
	TxData[1] = RxData[1];
	TxData[2] = RxData[2];
	TxData[3] = RxData[3];
	TxData[4] = RxData[4];
	TxData[5] = RxData[5];

	sendData(TxData, 6);

	if (changeSlaveId) {
		g_modbus_slave_id = (uint8_t)newSlaveId;
	}

	return 1;
}

/* =========================================================
 *                      FUNZIONE 0x0F
 *                 WRITE MULTIPLE COILS (1 bit)
 * ========================================================= */

uint8_t writeMultiCoils (void)
{
	uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);
	uint16_t numCoils  = ((RxData[4] << 8) | RxData[5]);

	if ((numCoils < 1) || (numCoils > TOTAL_COILS)) {
		modbusException(ILLEGAL_DATA_VALUE);
		return 0;
	}

	uint16_t endAddr = startAddr + numCoils - 1;

	if ((startAddr < COIL_START_ADDR) ||
			(endAddr   > COIL_START_ADDR + TOTAL_COILS - 1))
	{
		modbusException(ILLEGAL_DATA_ADDRESS);
		return 0;
	}

	uint16_t startIdx = startAddr - COIL_START_ADDR;
	uint16_t endIdx   = endAddr   - COIL_START_ADDR;

	// verifica mappa coil
	for (uint16_t k = startIdx; k <= endIdx; k++) {
		if (!coils_table[k]) {
			modbusException(ILLEGAL_DATA_ADDRESS);
			return 0;
		}
	}

	uint8_t  byteCount   = RxData[6];
	uint16_t expectedBC  = (uint16_t)((numCoils + 7) / 8);
	if (byteCount != expectedBC) {
		modbusException(ILLEGAL_DATA_VALUE);
		return 0;
	}

	int dataByteIndex = 7;
	uint8_t bitPos = 0;

	for (uint16_t idx = startIdx; idx <= endIdx; idx++) {
		uint16_t slot = idx / BIT_PER_SLOT;
		uint16_t pin  = idx % BIT_PER_SLOT;

		uint8_t rawBit = (RxData[dataByteIndex] >> bitPos) & 0x01;
		bool value = (rawBit != 0);

		digitalIO_Write(&digitalIO[slot], (uint8_t)pin, value);

		bitPos++;
		if (bitPos == 8) {
			bitPos = 0;
			dataByteIndex++;
		}
	}

	// risposta: echo indirizzo e quantità
	TxData[0] = Modbus_GetSlaveId();
	TxData[1] = RxData[1];
	TxData[2] = RxData[2];
	TxData[3] = RxData[3];
	TxData[4] = RxData[4];
	TxData[5] = RxData[5];

	sendData(TxData, 6);
	return 1;
}

/* =========================================================
 *                      FUNZIONE 0x10
 *           WRITE MULTIPLE HOLDING REGISTERS (EEPROM)
 * ========================================================= */

uint8_t writeHoldingRegs (void)
{
	uint16_t startAddr = ((RxData[2] << 8) | RxData[3]);
	uint16_t numRegs   = ((RxData[4] << 8) | RxData[5]);

	if ((numRegs < 1) || (numRegs > TOTAL_HOLDING_REGS)) {
		modbusException(ILLEGAL_DATA_VALUE);
		return 0;
	}

	uint16_t endAddr = startAddr + numRegs - 1;
	if ((startAddr < HOLDING_START_ADDR) ||
			(endAddr   > HOLDING_START_ADDR + TOTAL_HOLDING_REGS - 1))
	{
		modbusException(ILLEGAL_DATA_ADDRESS);
		return 0;
	}

	uint8_t  byteCount  = RxData[6];
	uint16_t expectedBC = numRegs * 2;
	if (byteCount != expectedBC) {
		modbusException(ILLEGAL_DATA_VALUE);
		return 0;
	}

	uint16_t regIndex = startAddr - HOLDING_START_ADDR;
	uint16_t byteAddr = regIndex * 2;

	if (cass_holding_is_ram(startAddr) || cass_holding_is_ram(endAddr)) {
		if (!(cass_holding_is_ram(startAddr) && cass_holding_is_ram(endAddr))) {
			modbusException(ILLEGAL_DATA_ADDRESS); // no range misti RAM/EEPROM
			return 0;
		}
		int dataIdxRam = 7;
		for (uint16_t a = startAddr; a <= endAddr; a++) {
			uint16_t value = ((uint16_t)RxData[dataIdxRam] << 8) | RxData[dataIdxRam + 1];
			dataIdxRam += 2;
			switch (a) {
			case CASS_HREG_ADC_SELECT: g_cass_adc_select = value; break;
			case CASS_HREG_CHANNEL: g_cass_channel = value; break;
			case CASS_HREG_FREQ_HZ: g_cass_freq_hz = value; break;
			case CASS_HREG_NUM_SAMPLES: g_cass_num_samples = value; break;
			case CASS_HREG_SAMPLE_OFFSET: g_cass_sample_offset = value; break;
			default:
				modbusException(ILLEGAL_DATA_ADDRESS);
				return 0;
			}
		}
		if ((startAddr <= CASS_HREG_NUM_SAMPLES) && (endAddr >= CASS_HREG_NUM_SAMPLES)) {
			g_cass_status = CASS_STATUS_BUSY;
			g_cass_run_pending = 1;
		}

		TxData[0] = Modbus_GetSlaveId();
		TxData[1] = RxData[1];
		TxData[2] = RxData[2];
		TxData[3] = RxData[3];
		TxData[4] = RxData[4];
		TxData[5] = RxData[5];
		sendData(TxData, 6);
		return 1;
	}

	if ((regIndex <= MB_AUX_REGIDX_SLOT_TYPE_LAST) && ((regIndex + numRegs - 1) >= MB_AUX_REGIDX_SLOT_TYPE_BASE)) {
		modbusException(ILLEGAL_DATA_ADDRESS); // blocco registri slot type in sola lettura lato master
		return 0;
	}

	uint8_t buffer[256];
	int dataIdx = 7;

	for (uint16_t i = 0; i < byteCount; i++) {
		buffer[i] = RxData[dataIdx++];
	}

	bool changeSlaveId = false;
	uint16_t newSlaveId = 0;
	if ((MB_AUX_REGIDX_SLAVE_ID >= regIndex) && (MB_AUX_REGIDX_SLAVE_ID < (regIndex + numRegs))) {
		uint16_t off = (MB_AUX_REGIDX_SLAVE_ID - regIndex) * 2;
		newSlaveId = ((uint16_t)buffer[off] << 8) | buffer[off + 1];
		if (!is_valid_slave_id(newSlaveId)) {
			modbusException(ILLEGAL_DATA_VALUE);
			return 0;
		}
		changeSlaveId = true;
	}

	if (HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, byteAddr, I2C_MEMADD_SIZE_8BIT,
			buffer, byteCount, HAL_MAX_DELAY) != HAL_OK)
	{
		modbusException(ILLEGAL_FUNCTION);
		return 0;
	}

	// risposta: echo start address e numero registri (ID della richiesta)
	TxData[0] = RxData[0];
	TxData[1] = RxData[1];
	TxData[2] = RxData[2];
	TxData[3] = RxData[3];
	TxData[4] = RxData[4];
	TxData[5] = RxData[5];

	sendData(TxData, 6);

	if (changeSlaveId) {
		g_modbus_slave_id = (uint8_t)newSlaveId;
	}

	return 1;
}
