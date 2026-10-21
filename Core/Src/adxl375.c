#include "adxl375.h"
#include <stddef.h>

static bool valid(const ADXL375_t *dev)
{
    return dev && dev->spi && dev->cs_port && dev->cs_pin &&
           dev->spi_timeout_ms && dev->spi_timeout_ms != UINT32_MAX &&
           dev->sample_timeout_ms && dev->sample_timeout_ms < 0x80000000U &&
           dev->spi->Init.DataSize == SPI_DATASIZE_8BIT &&
           dev->spi->Init.Direction == SPI_DIRECTION_2LINES &&
           dev->spi->Init.FirstBit == SPI_FIRSTBIT_MSB &&
           dev->spi->Init.CLKPolarity == SPI_POLARITY_HIGH &&
           dev->spi->Init.CLKPhase == SPI_PHASE_2EDGE &&
           dev->spi->Init.NSS == SPI_NSS_SOFT;
}

static ADXL375_Status_t transfer(ADXL375_t *dev, uint8_t *tx, uint8_t *rx, uint16_t n)
{
    /* Conservative setup/hold and inter-transaction delays for bring-up.
       These can be replaced with validated microsecond delays for throughput. */
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_Delay(1U);
    dev->last_hal_status = HAL_SPI_TransmitReceive(dev->spi, tx, rx, n, dev->spi_timeout_ms);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    HAL_Delay(1U);
    return dev->last_hal_status == HAL_OK ? ADXL375_OK : ADXL375_ERROR_SPI;
}

static ADXL375_Status_t read_regs(ADXL375_t *dev, uint8_t address, uint8_t *out, uint16_t n)
{
    if (!out || n == 0U || n > 6U || address > 0x3FU) {
        return ADXL375_ERROR_ARGUMENT;
    }
    uint8_t tx[7] = { (uint8_t)(address | 0x80U | (n > 1U ? 0x40U : 0U)) };
    uint8_t rx[7];
    ADXL375_Status_t status = transfer(dev, tx, rx, n + 1U);
    if (status == ADXL375_OK) {
        for (uint16_t i = 0; i < n; ++i) out[i] = rx[i + 1U];
    }
    return status;
}

static ADXL375_Status_t write_reg(ADXL375_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { reg, value }, rx[2];
    return transfer(dev, tx, rx, 2U);
}

void ADXL375_AssignSPI(ADXL375_t *dev, SPI_HandleTypeDef *spi,
                       GPIO_TypeDef *port, uint16_t pin)
{
    if (!dev) return;
    *dev = (ADXL375_t) { .spi = spi, .cs_port = port, .cs_pin = pin,
        .spi_timeout_ms = 100U, .sample_timeout_ms = 100U, .last_hal_status = HAL_OK };
    if (port && pin) HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

ADXL375_Status_t ADXL375_ReadStatus(ADXL375_t *dev, ADXL375_Registers_t *status)
{
    if (!valid(dev) || !status) return ADXL375_ERROR_ARGUMENT;
    ADXL375_Registers_t result = {0};
    const uint8_t addresses[] = { 0x00U, 0x2CU, 0x2DU, 0x31U, 0x30U, 0x39U };
    uint8_t *fields[] = { &result.device_id, &result.bandwidth_rate, &result.power_control,
                         &result.data_format, &result.interrupt_source, &result.fifo_status };
    for (unsigned int i = 0; i < sizeof(addresses); ++i) {
        ADXL375_Status_t rc = read_regs(dev, addresses[i], fields[i], 1U);
        if (rc != ADXL375_OK) return rc;
    }
    *status = result;
    return ADXL375_OK;
}

ADXL375_Status_t ADXL375_Init(ADXL375_t *dev)
{
    if (dev) dev->initialized = false;
    if (!valid(dev)) return ADXL375_ERROR_ARGUMENT;
    uint8_t id;
    ADXL375_Status_t rc = read_regs(dev, 0x00U, &id, 1U);
    if (rc != ADXL375_OK) return rc;
    if (id != ADXL375_DEVICE_ID) return ADXL375_ERROR_ID;
    /* Standby first; required fixed bits in DATA_FORMAT are 0x0B.
       Zero offsets avoid stale calibration after a debugger/MCU-only reset. */
    const uint8_t config[][2] = {
        {0x2D, 0x00}, {0x2E, 0x00}, {0x2F, 0x00}, {0x38, 0x00},
        {0x1E, 0x00}, {0x1F, 0x00}, {0x20, 0x00},
        {0x31, 0x0B}, {0x2C, 0x0A}, {0x2D, 0x08}
    };
    for (unsigned int i = 0; i < sizeof(config) / sizeof(config[0]); ++i) {
        rc = write_reg(dev, config[i][0], config[i][1]);
        if (rc != ADXL375_OK) return rc;
    }
    HAL_Delay(50U); // >4 samples at 100 Hz, also covers wakeup time.
    ADXL375_Registers_t registers;
    rc = ADXL375_ReadStatus(dev, &registers);
    if (rc != ADXL375_OK) return rc;
    if (registers.device_id != ADXL375_DEVICE_ID || registers.bandwidth_rate != 0x0AU ||
        registers.power_control != 0x08U || registers.data_format != 0x0BU) {
        return ADXL375_ERROR_CONFIG;
    }
    uint8_t fifo;
    rc = read_regs(dev, 0x38U, &fifo, 1U);
    if (rc != ADXL375_OK) return rc;
    if (fifo != 0U) return ADXL375_ERROR_CONFIG;
    dev->initialized = true;
    return ADXL375_OK;
}

static int16_t signed_le(const uint8_t *p)
{
    uint32_t value = (uint32_t)p[0] | ((uint32_t)p[1] << 8);
    return (int16_t)(value >= 32768U ? (int32_t)value - 65536 : (int32_t)value);
}

ADXL375_Status_t ADXL375_Read(ADXL375_t *dev, ADXL375_Data_t *data)
{
    if (!valid(dev) || !data) return ADXL375_ERROR_ARGUMENT;
    if (!dev->initialized) return ADXL375_ERROR_NOT_INITIALIZED;
    uint32_t start = HAL_GetTick();
    for (;;) {
        uint8_t source;
        ADXL375_Status_t rc = read_regs(dev, 0x30U, &source, 1U);
        if (rc != ADXL375_OK) return rc;
        if (source & 0x80U) break;
        if ((uint32_t)(HAL_GetTick() - start) >= dev->sample_timeout_ms) return ADXL375_ERROR_TIMEOUT;
        HAL_Delay(1U);
    }
    uint8_t bytes[6];
    ADXL375_Status_t rc = read_regs(dev, 0x32U, bytes, 6U);
    if (rc != ADXL375_OK) return rc;
    ADXL375_Data_t result;
    result.x = signed_le(bytes); result.y = signed_le(bytes + 2); result.z = signed_le(bytes + 4);
    result.x_mg = (int32_t)result.x * 49;
    result.y_mg = (int32_t)result.y * 49;
    result.z_mg = (int32_t)result.z * 49;
    *data = result;
    return ADXL375_OK;
}
