#include "ms5611.h"
#include <stddef.h>

#define MS5611_CMD_RESET 0x1EU
#define MS5611_CMD_PROM  0xA0U
#define MS5611_CMD_D1    0x40U
#define MS5611_CMD_D2    0x50U
#define MS5611_CMD_ADC   0x00U

static bool valid_osr(MS5611_Osr_t osr)
{
    return osr == MS5611_OSR_256 || osr == MS5611_OSR_512 ||
           osr == MS5611_OSR_1024 || osr == MS5611_OSR_2048 ||
           osr == MS5611_OSR_4096;
}

static bool valid_device(const MS5611_t *dev)
{
    return dev != NULL && dev->hspi != NULL && dev->cs_port != NULL &&
           dev->cs_pin != 0U && dev->config.spi_timeout_ms != 0U &&
           dev->config.spi_timeout_ms != UINT32_MAX &&
           valid_osr(dev->config.pressure_osr) && valid_osr(dev->config.temperature_osr);
}

/* One transaction holds CS low across command and response. Dummy zero bytes
   clock out the response; command byte's received value is discarded. */
static MS5611_Status_t transfer(MS5611_t *dev, uint8_t command,
                                uint8_t *response, uint16_t response_len)
{
    uint8_t tx[4] = { command, 0U, 0U, 0U };
    uint8_t rx[4] = { 0U };
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    dev->last_hal_status = HAL_SPI_TransmitReceive(dev->hspi, tx, rx,
                                                  response_len + 1U,
                                                  dev->config.spi_timeout_ms);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    if (dev->last_hal_status != HAL_OK) {
        return MS5611_ERROR_SPI;
    }
    for (uint16_t i = 0U; i < response_len; ++i) {
        response[i] = rx[i + 1U];
    }
    return MS5611_OK;
}

void MS5611_AssignSPI(MS5611_t *dev, SPI_HandleTypeDef *hspi,
                      GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    if (dev == NULL) {
        return;
    }
    *dev = (MS5611_t) {
        .hspi = hspi, .cs_port = cs_port, .cs_pin = cs_pin,
        .config = MS5611_CONFIG_DEFAULT, .last_hal_status = HAL_OK
    };
    if (cs_port != NULL && cs_pin != 0U) {
        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    }
}

uint8_t MS5611_CalculateCRC4(const uint16_t prom[8])
{
    /* CRC-4 polynomial 0x3000. Mask the last PROM byte during calculation
       (manufacturer's CRC algorithm); do not mutate the caller's PROM. */
    uint16_t remainder = 0U;
    if (prom == NULL) {
        return 0U;
    }
    for (uint8_t i = 0U; i < 16U; ++i) {
        uint16_t word = prom[i / 2U];
        if (i / 2U == 7U) {
            word &= 0xFF00U;
        }
        remainder ^= (i & 1U) ? (word & 0xFFU) : (word >> 8);
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            remainder = (remainder & 0x8000U) ?
                        (uint16_t)((remainder << 1) ^ 0x3000U) :
                        (uint16_t)(remainder << 1);
        }
    }
    return (uint8_t)((remainder >> 12) & 0x0FU);
}

static bool valid_coefficients(const uint16_t prom[8])
{
    for (uint8_t i = 1U; i <= 6U; ++i) {
        if (prom[i] == 0U || prom[i] == 0xFFFFU) {
            return false; // Also rejects typical disconnected-bus patterns.
        }
    }
    return true;
}

MS5611_Status_t MS5611_Init(MS5611_t *dev)
{
    if (dev != NULL) {
        dev->initialized = false;
    }
    if (!valid_device(dev)) {
        return MS5611_ERROR_ARGUMENT;
    }
    MS5611_Status_t status = transfer(dev, MS5611_CMD_RESET, NULL, 0U);
    if (status != MS5611_OK) {
        return status;
    }
    HAL_Delay(4U); // PROM reload requires at least 2.8 ms.
    for (uint8_t i = 0U; i < 8U; ++i) {
        uint8_t bytes[2];
        status = transfer(dev, MS5611_CMD_PROM + 2U * i, bytes, 2U);
        if (status != MS5611_OK) {
            return status;
        }
        dev->prom[i] = ((uint16_t)bytes[0] << 8) | bytes[1];
    }
    if (!valid_coefficients(dev->prom) ||
        MS5611_CalculateCRC4(dev->prom) != (dev->prom[7] & 0x0FU)) {
        return MS5611_ERROR_PROM;
    }
    dev->initialized = true;
    return MS5611_OK;
}

static MS5611_Status_t convert(MS5611_t *dev, uint8_t command,
                               MS5611_Osr_t osr, uint32_t *value)
{
    /* Rounded up from MAX conversion times: 0.60, 1.17, 2.28, 4.54,
       9.04 ms. Never read ADC early or repeat an ADC read. */
    static const uint8_t delay_ms[] = { 1U, 2U, 3U, 5U, 10U };
    MS5611_Status_t status = transfer(dev, command | (uint8_t)osr, NULL, 0U);
    if (status != MS5611_OK) {
        return status;
    }
    HAL_Delay(delay_ms[(uint8_t)osr / 2U]);
    uint8_t bytes[3];
    status = transfer(dev, MS5611_CMD_ADC, bytes, 3U);
    if (status != MS5611_OK) {
        return status;
    }
    *value = ((uint32_t)bytes[0] << 16) | ((uint32_t)bytes[1] << 8) | bytes[2];
    return (*value == 0U || *value == 0xFFFFFFU) ? MS5611_ERROR_ADC : MS5611_OK;
}

MS5611_Status_t MS5611_Compensate(const uint16_t prom[8], uint32_t d1,
                                 uint32_t d2, bool second_order,
                                 MS5611_Data_t *data)
{
    if (prom == NULL || data == NULL || d1 == 0U || d1 >= 0xFFFFFFU ||
        d2 == 0U || d2 >= 0xFFFFFFU || !valid_coefficients(prom)) {
        return MS5611_ERROR_ARGUMENT;
    }
    /* Datasheet pages 8-9. All intermediate arithmetic is signed 64-bit.
       Division avoids implementation-defined shifts of negative values. */
    int64_t dt = (int64_t)d2 - (int64_t)prom[5] * 256;
    int64_t temp = 2000 + dt * prom[6] / 8388608;
    int64_t off = (int64_t)prom[2] * 65536 + dt * prom[4] / 128;
    int64_t sens = (int64_t)prom[1] * 32768 + dt * prom[3] / 256;

    if (second_order && temp < 2000) {
        int64_t t2 = dt * dt / 2147483648LL;
        int64_t delta = temp - 2000;
        int64_t off2 = 5 * delta * delta / 2;
        int64_t sens2 = 5 * delta * delta / 4;
        if (temp < -1500) {
            delta = temp + 1500;
            off2 += 7 * delta * delta;
            sens2 += 11 * delta * delta / 2;
        }
        temp -= t2;
        off -= off2;
        sens -= sens2;
    }
    int64_t pressure = ((int64_t)d1 * sens / 2097152 - off) / 32768;
    *data = (MS5611_Data_t) {
        .pressure_mbar = (float)pressure / 100.0f,
        .temperature_c = (float)temp / 100.0f,
        .pressure_centi_mbar = (int32_t)pressure,
        .temperature_centi_c = (int32_t)temp,
        .raw_pressure = d1, .raw_temperature = d2
    };
    return MS5611_OK;
}

MS5611_Status_t MS5611_Read(MS5611_t *dev, MS5611_Data_t *data)
{
    if (!valid_device(dev) || data == NULL) {
        return MS5611_ERROR_ARGUMENT;
    }
    if (!dev->initialized) {
        return MS5611_ERROR_NOT_INITIALIZED;
    }
    uint32_t d1, d2;
    MS5611_Status_t status = convert(dev, MS5611_CMD_D1, dev->config.pressure_osr, &d1);
    if (status != MS5611_OK) {
        return status;
    }
    status = convert(dev, MS5611_CMD_D2, dev->config.temperature_osr, &d2);
    if (status != MS5611_OK) {
        return status;
    }
    return MS5611_Compensate(dev->prom, d1, d2,
                            dev->config.second_order_compensation, data);
}
