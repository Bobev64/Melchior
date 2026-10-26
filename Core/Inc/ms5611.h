#ifndef MS5611_H
#define MS5611_H

#include "stm32h5xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Conversion command OSR bits (datasheet pages 3 and 10). */
typedef enum {
    MS5611_OSR_256  = 0x00U,
    MS5611_OSR_512  = 0x02U,
    MS5611_OSR_1024 = 0x04U,
    MS5611_OSR_2048 = 0x06U,
    MS5611_OSR_4096 = 0x08U
} MS5611_Osr_t;

typedef struct {
    MS5611_Osr_t pressure_osr;
    MS5611_Osr_t temperature_osr;
    uint32_t spi_timeout_ms;
    bool second_order_compensation;
} MS5611_Config_t;

#define MS5611_CONFIG_DEFAULT { MS5611_OSR_4096, MS5611_OSR_4096, 100U, true }

typedef enum {
    MS5611_OK = 0,
    MS5611_ERROR_ARGUMENT,
    MS5611_ERROR_NOT_INITIALIZED,
    MS5611_ERROR_SPI,
    MS5611_ERROR_PROM,
    MS5611_ERROR_ADC
} MS5611_Status_t;

typedef struct {
    float pressure_mbar;
    float temperature_c;
    /* Fixed-point results for printing without newlib float printf support. */
    int32_t pressure_centi_mbar;
    int32_t temperature_centi_c;
    uint32_t raw_pressure;
    uint32_t raw_temperature;
} MS5611_Data_t;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    /* Driver-owned copy; reinitialize to change configuration. */
    MS5611_Config_t config;
    /* Read-only diagnostics after initialization: words 1..6 are C1..C6. */
    uint16_t prom[8];
    HAL_StatusTypeDef last_hal_status;
    bool initialized;
} MS5611_t;

/* Call after HAL/GPIO/SPI init. SPI: 8-bit, MSB first, mode 0 or 3,
   software CS, <=20 MHz. PS pin must be low to select SPI.
   Initializes defaults and drives CS high; does not access the sensor. */
void MS5611_AssignSPI(MS5611_t *dev, SPI_HandleTypeDef *hspi,
                      GPIO_TypeDef *cs_port, uint16_t cs_pin);

/* Reset, wait for PROM reload, read all eight words, validate CRC-4.
   There is no WHO_AM_I register on this sensor. Non-null config is validated
   and copied; no caller-owned pointer is retained. Failed init clears initialized. */
MS5611_Status_t MS5611_Init(MS5611_t *dev, const MS5611_Config_t *config);

/* Blocking D1 and D2 conversions, then calibrated pressure/temperature.
   Takes about 20 ms at OSR 4096. Output is unchanged on failure.
   Not ISR-safe or thread-safe: caller must serialize shared SPI access. */
MS5611_Status_t MS5611_Read(MS5611_t *dev, MS5611_Data_t *data);

/* Pure datasheet calculation, also useful for offline validation.
   PROM must contain valid calibration coefficients; call Init first for hardware. */
MS5611_Status_t MS5611_Compensate(const uint16_t prom[8], uint32_t d1,
                                 uint32_t d2, bool second_order,
                                 MS5611_Data_t *data);
uint8_t MS5611_CalculateCRC4(const uint16_t prom[8]);

#ifdef __cplusplus
}
#endif
#endif /* MS5611_H */
