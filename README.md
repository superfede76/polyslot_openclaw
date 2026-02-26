# polyslot_openclaw

Firmware STM32 (CubeIDE) per carrier **Polyslot** con 8 slot modulari.
La MCU espone una interfaccia **Modbus RTU slave** su RS485 e traduce le richieste verso i moduli slot via SPI.

## Panoramica architettura

- **MCU**: STM32F103 (progetto CubeIDE)
- **Comunicazione campo**: UART1 RS485, Modbus RTU slave
- **Bus moduli**: SPI1 + chip-select per slot A..H
- **Autodetect moduli**: pin ID0..ID3 per ciascuno slot

Flusso logico:

1. Boot MCU e init periferiche
2. `slot_poll()` rileva il tipo modulo in ogni slot
3. Popola le mappe logiche Modbus (coil/contact/input reg validi)
4. In callback UART (`HAL_UARTEx_RxEventCallback`) gestisce le function code Modbus

---

## Tipi modulo rilevati (ID slot)

Da `polyslot.h`:

- `7`  -> `cassandra`
- `8`  -> `powermeter_board`
- `9`  -> `analog_board` (MCP3204, 4 canali)
- `10` -> `pt100_board` (MAX31865, 4 canali via DEMUX)
- `11` -> `pulsecounter` (4 canali)
- `12` -> `mixedIO` (4 DO + 4 DI)
- `13` -> `output` (8 DO)
- `14` -> `input` (8 DI)
- `15` -> `none`

Slot index: A=0, B=1, ... H=7.

---

## Function code Modbus supportate

- `0x01` Read Coils
- `0x02` Read Discrete Inputs
- `0x03` Read Holding Registers (EEPROM I2C)
- `0x04` Read Input Registers
- `0x05` Write Single Coil
- `0x06` Write Single Holding Register
- `0x0F` Write Multiple Coils
- `0x10` Write Multiple Holding Registers

Slave ID attuale: **7** (`modbusSlave.h`).

---

## Mappa Modbus (schema pratico)

## 1) Coils (0x01 / 0x05 / 0x0F)

- Base address: **00001**
- Totale: **64 bit** (8 slot × 8 bit)
- Mapping:
  - Slot `s` (0..7), bit `b` (0..7)
  - Address = `1 + s*8 + b`

Validità coil dipende dal modulo installato nello slot:

- `output`: 8 coil valide
- `mixedIO`: prime 4 coil valide (bit 0..3)
- altri moduli: coil non valide (eccezione `ILLEGAL_DATA_ADDRESS`)

## 2) Discrete Inputs / Contacts (0x02)

- Base address: **10001**
- Totale: **64 bit**
- Mapping:
  - Slot `s`, bit `b`
  - Address = `10001 + s*8 + b`

Validità contatti:

- `input`: 8 input validi
- `mixedIO`: ultimi 4 input validi (bit 4..7)
- altri moduli: non validi

## 3) Input Registers (0x04)

### Zona base (slot analog/PT100/pulse)

- Range: **30001..30032**
- 4 registri per slot (`REG_PER_SLOT = 4`)
- Mapping:
  - Slot `s`, canale `ch` (0..3)
  - Address = `30001 + s*4 + ch`

Interpretazione valore:

- `analog_board`: corrente 4-20 mA scalata a **mA × 100**
  - es. 12.34 mA -> 1234
- `pt100_board`: temperatura scalata a **°C × 100** su int16 (two's complement)
  - errore -> `0x8000`
- `pulsecounter`: conteggio grezzo uint16

### Zona powermeter estesa

- Range: **30097..30608**
- 64 registri per slot powermeter
- Mapping:
  - Slot `s`, registro `r` (0..63)
  - Address = `30097 + s*64 + r`

Nota importante implementativa:

- Le letture 0x04 in zona powermeter devono restare **dentro lo stesso slot** (non attraversare blocchi da 64 registri).
- Range intermedio **30033..30096** non usato (richiesta -> `ILLEGAL_DATA_ADDRESS`).

## 4) Holding Registers (0x03 / 0x06 / 0x10)

- Base address: **40001**
- Totale: **128 registri**
- Backend: EEPROM I2C (`0x50 << 1`), 2 byte per registro

---

## Schema slot rapido (cheat-sheet)

Per slot `s` (A=0..H=7):

- Coil slot: `00001 + s*8 .. 00008 + s*8`
- Contact slot: `10001 + s*8 .. 10008 + s*8`
- InputReg base slot: `30001 + s*4 .. 30004 + s*4`
- InputReg powermeter slot: `30097 + s*64 .. 30160 + s*64`

Esempi:

- Slot A (`s=0`)
  - coil: 1..8
  - contacts: 10001..10008
  - input base: 30001..30004
  - powermeter: 30097..30160

- Slot B (`s=1`)
  - coil: 9..16
  - contacts: 10009..10016
  - input base: 30005..30008
  - powermeter: 30161..30224

---

## Note operative

- `main()` è event-driven: loop principale vuoto, logica su callback UART.
- Inizializzazione slot e mappe avviene una volta al boot (`slot_poll()`).
- Per test Modbus, evitare richieste che mescolano zona base e zona powermeter nello stesso frame 0x04.

---

## File principali

- `Core/Src/main.c` -> init MCU, slot poll, dispatcher callback UART
- `Core/Src/modbusSlave.c` -> protocollo Modbus e mapping registri/bit
- `Core/Src/digitalIO.c` -> gestione moduli digitali (MCP23S08)
- `Core/Src/MCP3204.c` -> ingressi analogici 4 canali
- `Core/Src/MAX31865.c` -> PT100
- `Core/Src/pulse_counter.c` -> contatori impulso
- `Core/Src/powermeter*.c` -> interfaccia powermeter
