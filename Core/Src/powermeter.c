#include "powermeter.h"
#include "powermeter_if.h"   // per Powermeter_ReadRegs()

void powermeter_init(powermeter_t *pm,
                     SPI_HandleTypeDef *spi,
                     GPIO_TypeDef *cs_gpio,
                     uint16_t cs_pin,
                     uint8_t slot_index)
{
    if (pm->lock == 1)
        HAL_Delay(1);

    pm->lock       = 1;
    pm->spi        = spi;
    pm->cs_gpio    = cs_gpio;
    pm->cs_pin     = cs_pin;
    pm->slot_index = slot_index;

    HAL_GPIO_WritePin(pm->cs_gpio, pm->cs_pin, GPIO_PIN_SET);
    HAL_Delay(10);

    pm->lock = 0;
}

/*
 * PROTOCOLLO SPI CON STM32H503 (implementato in Powermeter_ReadRegs()):
 *  - master (F103) manda 3 byte: [ PM_CMD_READ_REGS (0x01), firstReg, numRegs ]
 *  - poi, con lo stesso CS basso, continua l'SPI con 2*numRegs byte dummy su MOSI
 *  - lo slave (H503) risponde su MISO con 2*numRegs byte dati (MSB,LSB)
 */
bool powermeter_read_reg(powermeter_t *pm, uint8_t reg_index, uint16_t *value)
{
    if (pm == NULL || value == NULL)
        return false;

    uint16_t tmp;
    if (!Powermeter_ReadRegs(pm->slot_index, reg_index, 1, &tmp))
        return false;

    *value = tmp;
    return true;
}
