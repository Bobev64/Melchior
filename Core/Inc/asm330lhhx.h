#ifndef ASM330LHHX_H
#define ASM330LHHX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define ASM330LHHX_WHO_AM_I_VALUE 0x6BU

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} ASM330LHHX_RawData_t;

typedef struct {
    float x;
    float y;
    float z;
} ASM330LHHX_FloatData_t;

typedef enum {
    ASM330LHHX_XL_FS_2G  = 0x00U,
    ASM330LHHX_XL_FS_16G = 0x04U,
    ASM330LHHX_XL_FS_4G  = 0x08U,
    ASM330LHHX_XL_FS_8G  = 0x0CU
} ASM330LHHX_AccelFullScale_t;

typedef enum {
    ASM330LHHX_G_FS_125DPS  = 0x02U,
    ASM330LHHX_G_FS_250DPS  = 0x00U,
    ASM330LHHX_G_FS_500DPS  = 0x04U,
    ASM330LHHX_G_FS_1000DPS = 0x08U,
    ASM330LHHX_G_FS_2000DPS = 0x0CU,
    ASM330LHHX_G_FS_4000DPS = 0x01U
} ASM330LHHX_GyroFullScale_t;

typedef enum {
    ASM330LHHX_ODR_POWER_DOWN = 0x00U,
    ASM330LHHX_ODR_12_5     = 0x10U,
    ASM330LHHX_ODR_26       = 0x20U,
    ASM330LHHX_ODR_52       = 0x30U,
    ASM330LHHX_ODR_104      = 0x40U,
    ASM330LHHX_ODR_208      = 0x50U,
    ASM330LHHX_ODR_416      = 0x60U,
    ASM330LHHX_ODR_833      = 0x70U,
    ASM330LHHX_ODR_1667     = 0x80U,
    ASM330LHHX_ODR_3333     = 0x90U,
    ASM330LHHX_ODR_6667     = 0xA0U
} ASM330LHHX_Odr_t;

typedef struct {
    ASM330LHHX_Odr_t accel_odr;
    ASM330LHHX_AccelFullScale_t accel_fs;
    ASM330LHHX_Odr_t gyro_odr;
    ASM330LHHX_GyroFullScale_t gyro_fs;
    uint32_t spi_timeout_ms;
} ASM330LHHX_Config_t;

#define ASM330LHHX_CONFIG_DEFAULT { ASM330LHHX_ODR_104, ASM330LHHX_XL_FS_2G, \
                                   ASM330LHHX_ODR_104, ASM330LHHX_G_FS_250DPS, 100U }

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    ASM330LHHX_Config_t config; /* Driver-owned copy; reinitialize to change. */
    bool initialized;
    float accel_mg_per_lsb;
    float gyro_mdps_per_lsb;
} ASM330LHHX_t;

void ASM330LHHX_AssignSPI(ASM330LHHX_t *dev,
                          SPI_HandleTypeDef *hspi,
                          GPIO_TypeDef *cs_port,
                          uint16_t cs_pin);

/* Non-null config is validated and copied; no caller-owned pointer is kept.
   Init must succeed before reading measurements. ID/register access is allowed
   after AssignSPI. Failed initialization clears initialized. */
HAL_StatusTypeDef ASM330LHHX_Init(ASM330LHHX_t *dev, const ASM330LHHX_Config_t *config);

HAL_StatusTypeDef ASM330LHHX_ReadDeviceID(ASM330LHHX_t *dev, uint8_t *device_id);
HAL_StatusTypeDef ASM330LHHX_ReadAccelRaw(ASM330LHHX_t *dev, ASM330LHHX_RawData_t *accel);
HAL_StatusTypeDef ASM330LHHX_ReadGyroRaw(ASM330LHHX_t *dev, ASM330LHHX_RawData_t *gyro);
HAL_StatusTypeDef ASM330LHHX_ReadAccelMg(ASM330LHHX_t *dev, ASM330LHHX_FloatData_t *accel_mg);
HAL_StatusTypeDef ASM330LHHX_ReadGyroMdps(ASM330LHHX_t *dev, ASM330LHHX_FloatData_t *gyro_mdps);
HAL_StatusTypeDef ASM330LHHX_ReadRegister(ASM330LHHX_t *dev, uint8_t reg, uint8_t *data, uint16_t len);
HAL_StatusTypeDef ASM330LHHX_WriteRegister(ASM330LHHX_t *dev, uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* ASM330LHHX_H */
