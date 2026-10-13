#include "ms5611.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static SPI_HandleTypeDef spi;
static GPIO_TypeDef gpio;
static uint16_t prom[8] = { 0U, 40127U, 36924U, 23317U, 23282U, 33464U, 28312U, 0U };
static uint32_t d1 = 9085466U, d2 = 8569150U;
static GPIO_PinState cs = GPIO_PIN_SET;
static HAL_StatusTypeDef injected_error = HAL_OK;
static uint8_t pending_command;
static uint32_t elapsed_ms;
static unsigned int transfers;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    assert(port == &gpio && pin == 1U);
    cs = state;
}

void HAL_Delay(uint32_t milliseconds)
{
    assert(cs == GPIO_PIN_SET);
    elapsed_ms += milliseconds;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *bus, uint8_t *tx,
                                         uint8_t *rx, uint16_t size, uint32_t timeout)
{
    assert(bus == &spi && cs == GPIO_PIN_RESET && timeout == 100U);
    ++transfers;
    if (injected_error != HAL_OK) {
        return injected_error;
    }
    memset(rx, 0, size);
    if (tx[0] == 0x1EU) {
        assert(size == 1U);
        pending_command = 0U;
        elapsed_ms = 0U;
    } else if (tx[0] >= 0xA0U && tx[0] <= 0xAEU) {
        assert(size == 3U && elapsed_ms >= 3U);
        uint16_t word = prom[(tx[0] - 0xA0U) / 2U];
        rx[1] = (uint8_t)(word >> 8);
        rx[2] = (uint8_t)word;
    } else if ((tx[0] & 0xF0U) == 0x40U || (tx[0] & 0xF0U) == 0x50U) {
        assert(size == 1U);
        pending_command = tx[0];
        elapsed_ms = 0U;
    } else {
        static const uint8_t minimum_wait_ms[5] = { 1U, 2U, 3U, 5U, 10U };
        assert(tx[0] == 0U && size == 4U && pending_command != 0U);
        assert(elapsed_ms >= minimum_wait_ms[(pending_command & 0x0FU) / 2U]);
        uint32_t value = (pending_command & 0xF0U) == 0x40U ? d1 : d2;
        rx[1] = (uint8_t)(value >> 16);
        rx[2] = (uint8_t)(value >> 8);
        rx[3] = (uint8_t)value;
        pending_command = 0U;
    }
    return HAL_OK;
}

int main(void)
{
    MS5611_t dev;
    MS5611_Data_t data;
    MS5611_AssignSPI(&dev, &spi, &gpio, 1U);
    assert(MS5611_Read(&dev, &data) == MS5611_ERROR_NOT_INITIALIZED);
    /* Manufacturer's AN520 CRC example: {0x3132..0x4142, 0x4344, 0x4500} => 0xB. */
    const uint16_t crc_example[8] = { 0x3132, 0x3334, 0x3536, 0x3738,
                                     0x3940, 0x4142, 0x4344, 0x4500 };
    assert(MS5611_CalculateCRC4(crc_example) == 0xBU);
    prom[7] = MS5611_CalculateCRC4(prom);
    uint16_t original[8];
    memcpy(original, prom, sizeof(prom));
    assert(MS5611_Init(&dev) == MS5611_OK);
    assert(memcmp(original, prom, sizeof(prom)) == 0);
    assert(MS5611_Read(&dev, &data) == MS5611_OK);
    assert(data.pressure_centi_mbar == 100009 && data.temperature_centi_c == 2007);
    assert(data.pressure_mbar > 1000.08f && data.pressure_mbar < 1000.10f);
    assert(data.temperature_c > 20.06f && data.temperature_c < 20.08f);

    /* Independent cold-temperature reference, including the <-15 C branch. */
    assert(MS5611_Compensate(prom, d1, 7000000U, true, &data) == MS5611_OK);
    assert(data.pressure_centi_mbar == 85696 && data.temperature_centi_c == -4430);
    assert(MS5611_Compensate(prom, d1, 8000000U, true, &data) == MS5611_OK);
    assert(data.pressure_centi_mbar == 95989 && data.temperature_centi_c == -61);
    assert(MS5611_Compensate(prom, d1, 8000000U, false, &data) == MS5611_OK);
    assert(data.temperature_centi_c == 88);

    const MS5611_Osr_t osrs[] = { MS5611_OSR_256, MS5611_OSR_512, MS5611_OSR_1024,
                                 MS5611_OSR_2048, MS5611_OSR_4096 };
    for (unsigned int i = 0U; i < 5U; ++i) {
        dev.config.pressure_osr = osrs[i];
        dev.config.temperature_osr = osrs[4U - i];
        assert(MS5611_Read(&dev, &data) == MS5611_OK);
    }
    MS5611_Data_t before = data;
    injected_error = HAL_TIMEOUT;
    assert(MS5611_Read(&dev, &data) == MS5611_ERROR_SPI);
    assert(dev.last_hal_status == HAL_TIMEOUT && cs == GPIO_PIN_SET);
    assert(memcmp(&data, &before, sizeof(data)) == 0);
    injected_error = HAL_OK;
    d1 = 0U;
    assert(MS5611_Read(&dev, &data) == MS5611_ERROR_ADC);
    assert(memcmp(&data, &before, sizeof(data)) == 0);
    d1 = 9085466U;
    unsigned int previous_transfers = transfers;
    dev.config.pressure_osr = (MS5611_Osr_t)3;
    assert(MS5611_Read(&dev, &data) == MS5611_ERROR_ARGUMENT);
    assert(transfers == previous_transfers);
    dev.config.pressure_osr = MS5611_OSR_4096;
    assert(MS5611_Read(NULL, &data) == MS5611_ERROR_ARGUMENT);
    assert(MS5611_Read(&dev, NULL) == MS5611_ERROR_ARGUMENT);
    prom[1] ^= 1U;
    assert(MS5611_Init(&dev) == MS5611_ERROR_PROM && !dev.initialized);
    memset(prom, 0, sizeof(prom));
    assert(MS5611_Init(&dev) == MS5611_ERROR_PROM);
    memset(prom, 0xFF, sizeof(prom));
    assert(MS5611_Init(&dev) == MS5611_ERROR_PROM);
    puts("MS5611 tests passed: compensation, CRC, OSR delays, SPI errors, invalid data/config.");
    return 0;
}
