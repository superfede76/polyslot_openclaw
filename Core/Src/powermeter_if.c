#include "powermeter_if.h"

extern SPI_HandleTypeDef hspi1;
extern SLOT_CS slot_cs_array[MAX_CS];

bool Powermeter_ReadRegs(uint8_t slot, uint8_t firstReg, uint8_t numRegs, uint16_t *dest)
{
    if (dest == NULL) return false;
    if (slot >= MAX_CS) return false;
    if (numRegs == 0) return false;
    if (firstReg >= PM_MAX_REGS) return false;
    if ((firstReg + numRegs) > PM_MAX_REGS) return false;

    uint8_t txHeader[3];
    txHeader[0] = PM_CMD_READ_REGS;
    txHeader[1] = firstReg;
    txHeader[2] = numRegs;

    // Seleziona lo slot (chip select dello slot X)
    HAL_GPIO_WritePin(slot_cs_array[slot].NSS_Port,
                      slot_cs_array[slot].NSS_Pin,
                      GPIO_PIN_RESET);

    // Trasmetti il comando (3 byte)
    if (HAL_SPI_Transmit(&hspi1, txHeader, 3, 1000) != HAL_OK)
    {
        HAL_GPIO_WritePin(slot_cs_array[slot].NSS_Port,
                          slot_cs_array[slot].NSS_Pin,
                          GPIO_PIN_SET);
        return false;
    }

    // Ora riceviamo numRegs * 2 byte (MSB, LSB ...), mandando dummy 0xFF
    uint8_t rxBuf[PM_MAX_REGS * 2];
    uint8_t txDummy[PM_MAX_REGS * 2];
    uint16_t totalBytes = (uint16_t)(numRegs * 2);

    for (uint16_t i = 0; i < totalBytes; i++)
        txDummy[i] = 0xFF;

    if (HAL_SPI_TransmitReceive(&hspi1,
                                txDummy,
                                rxBuf,
                                totalBytes,
                                1000) != HAL_OK)
    {
        HAL_GPIO_WritePin(slot_cs_array[slot].NSS_Port,
                          slot_cs_array[slot].NSS_Pin,
                          GPIO_PIN_SET);
        return false;
    }

    // Rilascia lo slot
    HAL_GPIO_WritePin(slot_cs_array[slot].NSS_Port,
                      slot_cs_array[slot].NSS_Pin,
                      GPIO_PIN_SET);

    // Converte i byte ricevuti in uint16_t (MSB,LSB)
    for (uint8_t i = 0; i < numRegs; i++)
    {
        dest[i] = ((uint16_t)rxBuf[i * 2] << 8) |
                  ((uint16_t)rxBuf[i * 2 + 1]);
    }

    return true;
}
