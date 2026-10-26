#include "asm330lhhx.h"

#include <stddef.h>

#define ASM330LHHX_SPI_READ_BIT 0x80U

#define ASM330LHHX_REG_WHO_AM_I 0x0FU
#define ASM330LHHX_REG_CTRL1_XL 0x10U
#define ASM330LHHX_REG_CTRL2_G  0x11U
#define ASM330LHHX_REG_CTRL3_C  0x12U
#define ASM330LHHX_REG_OUTX_L_G 0x22U
#define ASM330LHHX_REG_OUTX_L_A 0x28U

#define ASM330LHHX_CTRL3_C_BDU    0x40U
#define ASM330LHHX_CTRL3_C_IF_INC 0x04U
#define ASM330LHHX_CTRL3_C_SW_RESET 0x01U

static bool valid_config(const ASM330LHHX_Config_t *config)
{
    if (!config || !config->spi_timeout_ms || config->spi_timeout_ms == UINT32_MAX) return false;
    unsigned int accel_odr = (unsigned int)config->accel_odr;
    unsigned int gyro_odr = (unsigned int)config->gyro_odr;
    if (accel_odr > 0xA0U || (accel_odr & 0x0FU) ||
        gyro_odr > 0xA0U || (gyro_odr & 0x0FU)) return false;
    switch (config->accel_fs) {
    case ASM330LHHX_XL_FS_2G: case ASM330LHHX_XL_FS_4G:
    case ASM330LHHX_XL_FS_8G: case ASM330LHHX_XL_FS_16G: break;
    default: return false;
    }
    switch (config->gyro_fs) {
    case ASM330LHHX_G_FS_125DPS: case ASM330LHHX_G_FS_250DPS:
    case ASM330LHHX_G_FS_500DPS: case ASM330LHHX_G_FS_1000DPS:
    case ASM330LHHX_G_FS_2000DPS: case ASM330LHHX_G_FS_4000DPS: break;
    default: return false;
    }
    return true;
}

static void ASM330LHHX_Select(ASM330LHHX_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void ASM330LHHX_Deselect(ASM330LHHX_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static int16_t ASM330LHHX_Int16FromLe(const uint8_t *bytes)
{
    return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

static float ASM330LHHX_AccelSensitivityMg(ASM330LHHX_AccelFullScale_t fs)
{
    switch (fs) {
    case ASM330LHHX_XL_FS_2G:  return 0.061f;
    case ASM330LHHX_XL_FS_4G:  return 0.122f;
    case ASM330LHHX_XL_FS_8G:  return 0.244f;
    case ASM330LHHX_XL_FS_16G: return 0.488f;
    default:                   return 0.061f;
    }
}

static float ASM330LHHX_GyroSensitivityMdps(ASM330LHHX_GyroFullScale_t fs)
{
    switch (fs) {
    case ASM330LHHX_G_FS_125DPS:  return 4.375f;
    case ASM330LHHX_G_FS_250DPS:  return 8.75f;
    case ASM330LHHX_G_FS_500DPS:  return 17.50f;
    case ASM330LHHX_G_FS_1000DPS: return 35.0f;
    case ASM330LHHX_G_FS_2000DPS: return 70.0f;
    case ASM330LHHX_G_FS_4000DPS: return 140.0f;
    default:                      return 8.75f;
    }
}

void ASM330LHHX_AssignSPI(ASM330LHHX_t *dev,
                          SPI_HandleTypeDef *hspi,
                          GPIO_TypeDef *cs_port,
                          uint16_t cs_pin)
{
    if (dev == NULL) {
        return;
    }

    *dev = (ASM330LHHX_t) {
        .hspi = hspi, .cs_port = cs_port, .cs_pin = cs_pin,
        .config = ASM330LHHX_CONFIG_DEFAULT
    };

    if (cs_port != NULL) {
        ASM330LHHX_Deselect(dev);
    }
}

HAL_StatusTypeDef ASM330LHHX_ReadRegister(ASM330LHHX_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    if ((dev == NULL) || (dev->hspi == NULL) || (dev->cs_port == NULL) || (data == NULL) || (len == 0U)) {
        return HAL_ERROR;
    }

    uint8_t addr = reg | ASM330LHHX_SPI_READ_BIT;
    HAL_StatusTypeDef status;

    ASM330LHHX_Select(dev);
    status = HAL_SPI_Transmit(dev->hspi, &addr, 1U, dev->config.spi_timeout_ms);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(dev->hspi, data, len, dev->config.spi_timeout_ms);
    }
    ASM330LHHX_Deselect(dev);

    return status;
}

HAL_StatusTypeDef ASM330LHHX_WriteRegister(ASM330LHHX_t *dev, uint8_t reg, uint8_t value)
{
    if ((dev == NULL) || (dev->hspi == NULL) || (dev->cs_port == NULL)) {
        return HAL_ERROR;
    }

    uint8_t tx[2] = { reg, value };

    ASM330LHHX_Select(dev);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->hspi, tx, 2U, dev->config.spi_timeout_ms);
    ASM330LHHX_Deselect(dev);

    return status;
}

HAL_StatusTypeDef ASM330LHHX_Init(ASM330LHHX_t *dev, const ASM330LHHX_Config_t *config)
{
    if (dev != NULL) dev->initialized = false;
    if (!dev || !dev->hspi || !dev->cs_port || !dev->cs_pin || !valid_config(config)) {
        return HAL_ERROR;
    }
    dev->config = *config;
    HAL_StatusTypeDef status;

    status = ASM330LHHX_WriteRegister(dev, ASM330LHHX_REG_CTRL3_C, ASM330LHHX_CTRL3_C_SW_RESET);
    if (status != HAL_OK) {
        return status;
    }

    HAL_Delay(20U);

    /* Enable block-data-update and register auto-increment for multi-byte reads. */
    status = ASM330LHHX_WriteRegister(dev, ASM330LHHX_REG_CTRL3_C,
                                      ASM330LHHX_CTRL3_C_BDU | ASM330LHHX_CTRL3_C_IF_INC);
    if (status != HAL_OK) {
        return status;
    }

    status = ASM330LHHX_WriteRegister(dev, ASM330LHHX_REG_CTRL1_XL, ((uint8_t)dev->config.accel_odr | (uint8_t)dev->config.accel_fs));
    if (status != HAL_OK) {
        return status;
    }

    status = ASM330LHHX_WriteRegister(dev, ASM330LHHX_REG_CTRL2_G, ((uint8_t)dev->config.gyro_odr | (uint8_t)dev->config.gyro_fs));
    if (status != HAL_OK) {
        return status;
    }

    dev->accel_mg_per_lsb = ASM330LHHX_AccelSensitivityMg(dev->config.accel_fs);
    dev->gyro_mdps_per_lsb = ASM330LHHX_GyroSensitivityMdps(dev->config.gyro_fs);
    dev->initialized = true;

    return HAL_OK;
}

HAL_StatusTypeDef ASM330LHHX_ReadDeviceID(ASM330LHHX_t *dev, uint8_t *device_id)
{
    return ASM330LHHX_ReadRegister(dev, ASM330LHHX_REG_WHO_AM_I, device_id, 1U);
}

HAL_StatusTypeDef ASM330LHHX_ReadAccelRaw(ASM330LHHX_t *dev, ASM330LHHX_RawData_t *accel)
{
    if (dev == NULL || !dev->initialized || accel == NULL) {
        return HAL_ERROR;
    }

    uint8_t bytes[6];
    HAL_StatusTypeDef status = ASM330LHHX_ReadRegister(dev, ASM330LHHX_REG_OUTX_L_A, bytes, sizeof(bytes));
    if (status != HAL_OK) {
        return status;
    }

    accel->x = ASM330LHHX_Int16FromLe(&bytes[0]);
    accel->y = ASM330LHHX_Int16FromLe(&bytes[2]);
    accel->z = ASM330LHHX_Int16FromLe(&bytes[4]);

    return HAL_OK;
}

HAL_StatusTypeDef ASM330LHHX_ReadGyroRaw(ASM330LHHX_t *dev, ASM330LHHX_RawData_t *gyro)
{
    if (dev == NULL || !dev->initialized || gyro == NULL) {
        return HAL_ERROR;
    }

    uint8_t bytes[6];
    HAL_StatusTypeDef status = ASM330LHHX_ReadRegister(dev, ASM330LHHX_REG_OUTX_L_G, bytes, sizeof(bytes));
    if (status != HAL_OK) {
        return status;
    }

    gyro->x = ASM330LHHX_Int16FromLe(&bytes[0]);
    gyro->y = ASM330LHHX_Int16FromLe(&bytes[2]);
    gyro->z = ASM330LHHX_Int16FromLe(&bytes[4]);

    return HAL_OK;
}

HAL_StatusTypeDef ASM330LHHX_ReadAccelMg(ASM330LHHX_t *dev, ASM330LHHX_FloatData_t *accel_mg)
{
    if (accel_mg == NULL) {
        return HAL_ERROR;
    }

    ASM330LHHX_RawData_t raw;
    HAL_StatusTypeDef status = ASM330LHHX_ReadAccelRaw(dev, &raw);
    if (status != HAL_OK) {
        return status;
    }

    accel_mg->x = (float)raw.x * dev->accel_mg_per_lsb;
    accel_mg->y = (float)raw.y * dev->accel_mg_per_lsb;
    accel_mg->z = (float)raw.z * dev->accel_mg_per_lsb;

    return HAL_OK;
}

HAL_StatusTypeDef ASM330LHHX_ReadGyroMdps(ASM330LHHX_t *dev, ASM330LHHX_FloatData_t *gyro_mdps)
{
    if (gyro_mdps == NULL) {
        return HAL_ERROR;
    }

    ASM330LHHX_RawData_t raw;
    HAL_StatusTypeDef status = ASM330LHHX_ReadGyroRaw(dev, &raw);
    if (status != HAL_OK) {
        return status;
    }

    gyro_mdps->x = (float)raw.x * dev->gyro_mdps_per_lsb;
    gyro_mdps->y = (float)raw.y * dev->gyro_mdps_per_lsb;
    gyro_mdps->z = (float)raw.z * dev->gyro_mdps_per_lsb;

    return HAL_OK;
}
