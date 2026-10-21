#ifndef ADXL375_H
#define ADXL375_H
#include "stm32h5xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADXL375_DEVICE_ID 0xE5U

typedef enum {
    ADXL375_OK = 0, ADXL375_ERROR_ARGUMENT, ADXL375_ERROR_SPI,
    ADXL375_ERROR_ID, ADXL375_ERROR_CONFIG, ADXL375_ERROR_NOT_INITIALIZED,
    ADXL375_ERROR_TIMEOUT
} ADXL375_Status_t;

typedef struct {
    uint8_t device_id, bandwidth_rate, power_control, data_format;
    uint8_t interrupt_source, fifo_status;
} ADXL375_Registers_t;

typedef struct {
    int16_t x, y, z;
    int32_t x_mg, y_mg, z_mg;
} ADXL375_Data_t;

typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    uint32_t spi_timeout_ms;
    uint32_t sample_timeout_ms;
    HAL_StatusTypeDef last_hal_status;
    bool initialized;
} ADXL375_t;

/* Caller owns SPI configuration: mode 3, 4-wire, 8-bit, MSB first,
   software CS. For this fixed 100 Hz driver use 50 kHz..5 MHz SCK.
   Blocking calls, not ISR-safe; caller must serialize bus access.
   WARNING: CS high enables I2C. Standby does NOT disable the I2C interface.
   Without the datasheet's SDI OR gate, use an exclusive bus/test or separately
   validate every other device's traffic. Mode switching alone is not isolation. */
void ADXL375_AssignSPI(ADXL375_t *dev, SPI_HandleTypeDef *spi,
                       GPIO_TypeDef *port, uint16_t pin);
/* Configures 100 Hz, normal power, +/-200g, right-justified, FIFO bypass,
   interrupt pins disabled. Checks ID and configuration readback. */
ADXL375_Status_t ADXL375_Init(ADXL375_t *dev);
/* Reading INT_SOURCE clears event flags other than data-related flags.
   DATA_READY is cleared by reading the acceleration data. */
ADXL375_Status_t ADXL375_ReadStatus(ADXL375_t *dev, ADXL375_Registers_t *status);
/* Wait for data-ready, then one 7-byte SPI transfer: F2 + six data bytes.
   Output unchanged on failure. Nominal scale is 49 mg/LSB, NOT 1 mg/LSB. */
ADXL375_Status_t ADXL375_Read(ADXL375_t *dev, ADXL375_Data_t *data);
#ifdef __cplusplus
}
#endif
#endif
