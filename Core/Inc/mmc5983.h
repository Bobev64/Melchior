#ifndef MMC5983_H
#define MMC5983_H

#include "stm32h5xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MMC5983_PRODUCT_ID 0x30U

typedef enum {
    MMC5983_BW_100HZ = 0U, /* 8 ms measurement */
    MMC5983_BW_200HZ = 1U, /* 4 ms */
    MMC5983_BW_400HZ = 2U, /* 2 ms */
    MMC5983_BW_800HZ = 3U  /* 0.5 ms */
} MMC5983_Bandwidth_t;

typedef struct {
    MMC5983_Bandwidth_t bandwidth;
    bool automatic_set_reset;
    uint32_t spi_timeout_ms;
    uint32_t measurement_timeout_ms;
} MMC5983_Config_t;

#define MMC5983_CONFIG_DEFAULT { MMC5983_BW_100HZ, true, 100U, 50U }

typedef enum {
    MMC5983_OK = 0,
    MMC5983_ERROR_ARGUMENT,
    MMC5983_ERROR_NOT_INITIALIZED,
    MMC5983_ERROR_SPI,
    MMC5983_ERROR_ID,
    MMC5983_ERROR_TIMEOUT
} MMC5983_Status_t;

typedef struct {
    /* Unsigned 18-bit offset-binary output; nominal zero field is 131072. */
    uint32_t raw_x, raw_y, raw_z;
    float x_ut, y_ut, z_ut; /* microtesla: 1 gauss = 100 uT */
    int32_t x_centi_ut, y_centi_ut, z_centi_ut; /* uT times 100, for printf */
    bool at_rail; /* An axis is exactly 0 or 262143; possible saturation. */
} MMC5983_MagneticData_t;

typedef struct {
    uint8_t raw;
    float temperature_c;
    int32_t temperature_centi_c;
} MMC5983_Temperature_t;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    /* Set after AssignSPI. Re-run Init when bandwidth/auto-SR is changed. */
    MMC5983_Config_t config;
    uint8_t product_id;
    uint8_t last_sensor_status;
    HAL_StatusTypeDef last_hal_status;
    bool initialized;
    uint8_t control0; /* Internal shadow of persistent control bits. */
} MMC5983_t;

/* Requires configured GPIO and SPI: 4-wire, 8-bit, MSB first,
   software CS, SCK <=10 MHz. Does not change the SPI configuration.
   This project uses mode 0; the datasheet describes mode-3 timing.
   Caller must serialize shared bus access.
   All reads are blocking; do not call from an ISR. */
void MMC5983_AssignSPI(MMC5983_t *dev, SPI_HandleTypeDef *hspi,
                       GPIO_TypeDef *cs_port, uint16_t cs_pin);
MMC5983_Status_t MMC5983_ReadDeviceID(MMC5983_t *dev, uint8_t *id);
/* Soft reset, OTP-ready check, ID verification, bandwidth and initial SET.
   Uses single-shot mode (continuous mode/interrupts disabled). */
MMC5983_Status_t MMC5983_Init(MMC5983_t *dev);
/* Trigger a new conversion, poll data-ready, read all 7 output bytes together.
   Output is left unchanged if an operation fails. */
MMC5983_Status_t MMC5983_ReadMagnetic(MMC5983_t *dev, MMC5983_MagneticData_t *data);
MMC5983_Status_t MMC5983_ReadTemperature(MMC5983_t *dev, MMC5983_Temperature_t *data);
/* Manual magnetic conditioning, not a chip reset. RESET reverses magnetic
   polarity; call SET again before normal positive-polarity measurements. */
MMC5983_Status_t MMC5983_Set(MMC5983_t *dev);
MMC5983_Status_t MMC5983_Reset(MMC5983_t *dev);

#ifdef __cplusplus
}
#endif
#endif /* MMC5983_H */
