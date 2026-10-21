/* Minimal HAL interface for host-side sensor tests only. */
#ifndef TEST_STM32H5XX_HAL_H
#define TEST_STM32H5XX_HAL_H
#include <stdint.h>
#define SPI_DATASIZE_8BIT 7U
#define SPI_DIRECTION_2LINES 0U
#define SPI_FIRSTBIT_MSB 0U
#define SPI_POLARITY_HIGH 1U
#define SPI_PHASE_2EDGE 1U
#define SPI_NSS_SOFT 1U
typedef struct {
    uint32_t DataSize, Direction, FirstBit, CLKPolarity, CLKPhase, NSS;
} SPI_InitTypeDef;
typedef struct { SPI_InitTypeDef Init; } SPI_HandleTypeDef;
typedef struct { int unused; } GPIO_TypeDef;
typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
void HAL_Delay(uint32_t milliseconds);
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *tx,
                                         uint8_t *rx, uint16_t size, uint32_t timeout);
#endif
