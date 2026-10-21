#include "mmc5983.h"
#include <stddef.h>

#define REG_XOUT0   0x00U
#define REG_TOUT    0x07U
#define REG_STATUS  0x08U
#define REG_CTRL0   0x09U
#define REG_CTRL1   0x0AU
#define REG_CTRL2   0x0BU
#define REG_CTRL3   0x0CU
#define REG_ID      0x2FU
#define STATUS_MAG_DONE  0x01U
#define STATUS_TEMP_DONE 0x02U
#define STATUS_OTP_DONE  0x10U
#define CTRL0_TM_M    0x01U
#define CTRL0_TM_T    0x02U
#define CTRL0_SET     0x08U
#define CTRL0_RESET   0x10U
#define CTRL0_AUTO_SR 0x20U
#define CTRL1_SW_RST  0x80U

static bool valid_device(const MMC5983_t *dev)
{
    return dev != NULL && dev->hspi != NULL && dev->cs_port != NULL &&
           dev->cs_pin != 0U && (unsigned int)dev->config.bandwidth <= 3U &&
           dev->config.spi_timeout_ms != 0U && dev->config.spi_timeout_ms != UINT32_MAX &&
           dev->config.measurement_timeout_ms != 0U &&
           dev->config.measurement_timeout_ms < 0x80000000U;
}

static MMC5983_Status_t transfer(MMC5983_t *dev, uint8_t *tx, uint8_t *rx, uint16_t length)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    dev->last_hal_status = HAL_SPI_TransmitReceive(dev->hspi, tx, rx, length,
                                                  dev->config.spi_timeout_ms);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    return dev->last_hal_status == HAL_OK ? MMC5983_OK : MMC5983_ERROR_SPI;
}

/* Fixed-size private helpers: maximum read is the seven magnetic output bytes.
   Bit 7 = read; bit 6 is don't-care, not a multi-byte flag. */
static MMC5983_Status_t read_registers(MMC5983_t *dev, uint8_t reg,
                                      uint8_t *data, uint16_t length)
{
    uint8_t tx[8] = { (uint8_t)(reg | 0x80U), 0U };
    uint8_t rx[8] = { 0U };
    MMC5983_Status_t status = transfer(dev, tx, rx, length + 1U);
    if (status == MMC5983_OK) {
        for (uint16_t i = 0U; i < length; ++i) {
            data[i] = rx[i + 1U];
        }
    }
    return status;
}

static MMC5983_Status_t write_register(MMC5983_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { reg, value };
    uint8_t rx[2];
    return transfer(dev, tx, rx, 2U);
}

static MMC5983_Status_t wait_ready(MMC5983_t *dev, uint8_t mask)
{
    uint32_t start = HAL_GetTick();
    for (;;) {
        uint8_t status_byte;
        MMC5983_Status_t status = read_registers(dev, REG_STATUS, &status_byte, 1U);
        if (status != MMC5983_OK) {
            return status;
        }
        dev->last_sensor_status = status_byte;
        if ((status_byte & mask) == mask) {
            return MMC5983_OK;
        }
        if ((uint32_t)(HAL_GetTick() - start) >= dev->config.measurement_timeout_ms) {
            return MMC5983_ERROR_TIMEOUT;
        }
        HAL_Delay(1U);
    }
}

void MMC5983_AssignSPI(MMC5983_t *dev, SPI_HandleTypeDef *hspi,
                       GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    if (dev == NULL) {
        return;
    }
    *dev = (MMC5983_t) {
        .hspi = hspi, .cs_port = cs_port, .cs_pin = cs_pin,
        .config = MMC5983_CONFIG_DEFAULT, .last_hal_status = HAL_OK
    };
    if (cs_port != NULL && cs_pin != 0U) {
        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    }
}

MMC5983_Status_t MMC5983_ReadDeviceID(MMC5983_t *dev, uint8_t *id)
{
    if (!valid_device(dev) || id == NULL) {
        return MMC5983_ERROR_ARGUMENT;
    }
    return read_registers(dev, REG_ID, id, 1U);
}

MMC5983_Status_t MMC5983_Init(MMC5983_t *dev)
{
    if (dev != NULL) {
        dev->initialized = false;
    }
    if (!valid_device(dev)) {
        return MMC5983_ERROR_ARGUMENT;
    }
    MMC5983_Status_t status = write_register(dev, REG_CTRL1, CTRL1_SW_RST);
    if (status != MMC5983_OK) {
        return status;
    }
    HAL_Delay(15U); // Datasheet gives a 10 ms power-on/reset time.
    status = MMC5983_ReadDeviceID(dev, &dev->product_id);
    if (status != MMC5983_OK) {
        return status;
    }
    if (dev->product_id != MMC5983_PRODUCT_ID) {
        return MMC5983_ERROR_ID;
    }
    status = wait_ready(dev, STATUS_OTP_DONE);
    if (status != MMC5983_OK) {
        return status;
    }
    status = write_register(dev, REG_CTRL1, (uint8_t)dev->config.bandwidth);
    if (status != MMC5983_OK) {
        return status;
    }
    status = write_register(dev, REG_CTRL2, 0U); // Single-shot mode.
    if (status != MMC5983_OK) {
        return status;
    }
    status = write_register(dev, REG_CTRL3, 0U); // 4-wire SPI, no self-test current.
    if (status != MMC5983_OK) {
        return status;
    }
    /* Condition the sensor in SET polarity before measurements. */
    status = write_register(dev, REG_CTRL0, CTRL0_SET);
    if (status != MMC5983_OK) {
        return status;
    }
    HAL_Delay(2U);
    dev->control0 = dev->config.automatic_set_reset ? CTRL0_AUTO_SR : 0U;
    status = write_register(dev, REG_CTRL0, dev->control0);
    if (status == MMC5983_OK) {
        dev->initialized = true;
    }
    return status;
}

static MMC5983_Status_t trigger(MMC5983_t *dev, uint8_t command, uint8_t done_mask)
{
    /* Clear stale done/interrupt flags (write-one-to-clear), then request a new
       conversion. Magnetic and temperature commands must not be set together. */
    MMC5983_Status_t status = write_register(dev, REG_STATUS, done_mask);
    if (status != MMC5983_OK) {
        return status;
    }
    status = write_register(dev, REG_CTRL0, dev->control0 | command);
    return status == MMC5983_OK ? wait_ready(dev, done_mask) : status;
}

static void convert_axis(uint32_t raw, float *ut, int32_t *centi_ut)
{
    /* Datasheet: unsigned offset-binary, 18-bit zero 131072,
       sensitivity 16384 counts/gauss, range +/-8 gauss (+/-800 uT). */
    int32_t centered = (int32_t)raw - 131072;
    *ut = (float)centered * (100.0f / 16384.0f);
    *centi_ut = (int32_t)((int64_t)centered * 10000 / 16384);
}

MMC5983_Status_t MMC5983_ReadMagnetic(MMC5983_t *dev, MMC5983_MagneticData_t *data)
{
    if (!valid_device(dev) || data == NULL) {
        return MMC5983_ERROR_ARGUMENT;
    }
    if (!dev->initialized) {
        return MMC5983_ERROR_NOT_INITIALIZED;
    }
    MMC5983_Status_t status = trigger(dev, CTRL0_TM_M, STATUS_MAG_DONE);
    if (status != MMC5983_OK) {
        return status;
    }
    uint8_t bytes[7];
    status = read_registers(dev, REG_XOUT0, bytes, 7U);
    if (status != MMC5983_OK) {
        return status;
    }
    MMC5983_MagneticData_t result = { 0 };
    result.raw_x = ((uint32_t)bytes[0] << 10) | ((uint32_t)bytes[1] << 2) | (bytes[6] >> 6);
    result.raw_y = ((uint32_t)bytes[2] << 10) | ((uint32_t)bytes[3] << 2) | ((bytes[6] >> 4) & 3U);
    result.raw_z = ((uint32_t)bytes[4] << 10) | ((uint32_t)bytes[5] << 2) | ((bytes[6] >> 2) & 3U);
    convert_axis(result.raw_x, &result.x_ut, &result.x_centi_ut);
    convert_axis(result.raw_y, &result.y_ut, &result.y_centi_ut);
    convert_axis(result.raw_z, &result.z_ut, &result.z_centi_ut);
    result.at_rail = result.raw_x == 0U || result.raw_x == 262143U ||
                     result.raw_y == 0U || result.raw_y == 262143U ||
                     result.raw_z == 0U || result.raw_z == 262143U;
    *data = result;
    return MMC5983_OK;
}

MMC5983_Status_t MMC5983_ReadTemperature(MMC5983_t *dev, MMC5983_Temperature_t *data)
{
    if (!valid_device(dev) || data == NULL) {
        return MMC5983_ERROR_ARGUMENT;
    }
    if (!dev->initialized) {
        return MMC5983_ERROR_NOT_INITIALIZED;
    }
    MMC5983_Status_t status = trigger(dev, CTRL0_TM_T, STATUS_TEMP_DONE);
    if (status != MMC5983_OK) {
        return status;
    }
    uint8_t raw;
    status = read_registers(dev, REG_TOUT, &raw, 1U);
    if (status != MMC5983_OK) {
        return status;
    }
    /* Coarse on-chip temperature: approx. 0.8 C/LSB; raw zero is -75 C. */
    *data = (MMC5983_Temperature_t) {
        .raw = raw, .temperature_c = (float)raw * 0.8f - 75.0f,
        .temperature_centi_c = (int32_t)raw * 80 - 7500
    };
    return MMC5983_OK;
}

static MMC5983_Status_t condition(MMC5983_t *dev, uint8_t command)
{
    if (!valid_device(dev)) {
        return MMC5983_ERROR_ARGUMENT;
    }
    if (!dev->initialized) {
        return MMC5983_ERROR_NOT_INITIALIZED;
    }
    MMC5983_Status_t status = write_register(dev, REG_CTRL0, dev->control0 | command);
    if (status == MMC5983_OK) {
        HAL_Delay(2U);
    }
    return status;
}

MMC5983_Status_t MMC5983_Set(MMC5983_t *dev)
{
    return condition(dev, CTRL0_SET);
}

MMC5983_Status_t MMC5983_Reset(MMC5983_t *dev)
{
    return condition(dev, CTRL0_RESET);
}
