#include "adxl375.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static SPI_HandleTypeDef spi = { .Init = {
    SPI_DATASIZE_8BIT, SPI_DIRECTION_2LINES, SPI_FIRSTBIT_MSB,
    SPI_POLARITY_HIGH, SPI_PHASE_2EDGE, SPI_NSS_SOFT } };
static GPIO_TypeDef gpio;
static uint8_t regs[64];
static uint32_t tick;
static GPIO_PinState cs = GPIO_PIN_SET;
static HAL_StatusTypeDef error = HAL_OK;
static bool ready = true, corrupt_config;
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{ assert(port == &gpio && pin == 1U); cs = state; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *bus, uint8_t *tx,
                                         uint8_t *rx, uint16_t n, uint32_t timeout)
{
    assert(bus == &spi && cs == GPIO_PIN_RESET && timeout == 100U);
    if (error != HAL_OK) return error;
    memset(rx, 0, n);
    uint8_t addr = tx[0] & 0x3FU;
    if (tx[0] & 0x80U) {
        if (addr == 0x32U) {
            assert(n == 7U && tx[0] == 0xF2U);
        } else assert(n == 2U && !(tx[0] & 0x40U));
        for (uint16_t i = 1U; i < n; ++i) rx[i] = regs[addr + i - 1U];
        if (addr == 0x30U) rx[1] = ready ? 0x80U : 0U;
        if (addr == 0x31U && corrupt_config) rx[1] = 0U;
    } else {
        assert(n == 2U);
        regs[addr] = tx[1];
    }
    return HAL_OK;
}
int main(void)
{
    ADXL375_t dev;
    ADXL375_AssignSPI(&dev, &spi, &gpio, 1U);
    ADXL375_Data_t data;
    assert(ADXL375_Read(&dev, &data) == ADXL375_ERROR_NOT_INITIALIZED);
    assert(ADXL375_Init(&dev) == ADXL375_ERROR_ID);
    regs[0] = 0xE5U;
    assert(ADXL375_Init(&dev) == ADXL375_OK);
    assert(regs[0x31] == 0x0BU && regs[0x2C] == 0x0AU && regs[0x2D] == 8U);
    regs[0x32] = 0xFFU; regs[0x33] = 0xFFU;
    regs[0x34] = 20U; regs[0x35] = 0U;
    regs[0x36] = 0U; regs[0x37] = 0x80U;
    assert(ADXL375_Read(&dev, &data) == ADXL375_OK);
    assert(data.x == -1 && data.x_mg == -49 && data.y_mg == 980);
    assert(data.z == -32768 && data.z_mg == -1605632);
    ADXL375_Data_t saved = data;
    ready = false; tick = UINT32_MAX - 5U;
    assert(ADXL375_Read(&dev, &data) == ADXL375_ERROR_TIMEOUT);
    assert(memcmp(&saved, &data, sizeof(data)) == 0 && cs == GPIO_PIN_SET);
    error = HAL_TIMEOUT;
    assert(ADXL375_Read(&dev, &data) == ADXL375_ERROR_SPI);
    assert(cs == GPIO_PIN_SET && dev.last_hal_status == HAL_TIMEOUT);
    error = HAL_OK; corrupt_config = true;
    assert(ADXL375_Init(&dev) == ADXL375_ERROR_CONFIG && !dev.initialized);
    spi.Init.DataSize = 15U;
    assert(ADXL375_Init(&dev) == ADXL375_ERROR_ARGUMENT);
    assert(ADXL375_Read(NULL, &data) == ADXL375_ERROR_ARGUMENT);
    puts("ADXL375 tests passed: ID/config, burst length, signed conversion, scale, timeout, SPI errors.");
    return 0;
}
