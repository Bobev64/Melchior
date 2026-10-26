#include "asm330lhhx.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static SPI_HandleTypeDef spi;
static GPIO_TypeDef gpio;
static uint8_t regs[128], address;
static GPIO_PinState cs = GPIO_PIN_SET;
static HAL_StatusTypeDef error = HAL_OK;
static unsigned int transfers;
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{ assert(port == &gpio && pin == 1U); cs = state; }
void HAL_Delay(uint32_t ms) { (void)ms; assert(cs == GPIO_PIN_SET); }
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *bus, uint8_t *data, uint16_t n, uint32_t timeout)
{
    assert(bus == &spi && cs == GPIO_PIN_RESET && timeout == 25U);
    ++transfers;
    if (error != HAL_OK) return error;
    if (n == 1U) { assert(data[0] & 0x80U); address = data[0] & 0x7FU; }
    else { assert(n == 2U); regs[data[0]] = data[1]; }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *bus, uint8_t *data, uint16_t n, uint32_t timeout)
{
    assert(bus == &spi && cs == GPIO_PIN_RESET && timeout == 25U);
    if (error != HAL_OK) return error;
    memcpy(data, regs + address, n);
    return HAL_OK;
}
int main(void)
{
    ASM330LHHX_t dev;
    ASM330LHHX_Config_t config = ASM330LHHX_CONFIG_DEFAULT;
    ASM330LHHX_FloatData_t data;
    ASM330LHHX_AssignSPI(&dev, &spi, &gpio, 1U);
    assert(!dev.initialized);
    assert(ASM330LHHX_ReadAccelMg(&dev, &data) == HAL_ERROR);
    assert(ASM330LHHX_Init(&dev, NULL) == HAL_ERROR && transfers == 0U);
    config.accel_odr = (ASM330LHHX_Odr_t)0x41U;
    assert(ASM330LHHX_Init(&dev, &config) == HAL_ERROR && transfers == 0U);
    config = (ASM330LHHX_Config_t)ASM330LHHX_CONFIG_DEFAULT;
    config.accel_fs = ASM330LHHX_XL_FS_8G;
    config.gyro_fs = ASM330LHHX_G_FS_500DPS;
    config.spi_timeout_ms = 25U;
    assert(ASM330LHHX_Init(&dev, &config) == HAL_OK && dev.initialized);
    assert(regs[0x10] == 0x4CU && regs[0x11] == 0x44U && regs[0x12] == 0x44U);
    config.spi_timeout_ms = 1U;
    assert(dev.config.spi_timeout_ms == 25U);
    regs[0x28] = 100U; regs[0x29] = 0U;
    assert(ASM330LHHX_ReadAccelMg(&dev, &data) == HAL_OK);
    assert(data.x > 24.39f && data.x < 24.41f);
    regs[0x22] = 2U;
    assert(ASM330LHHX_ReadGyroMdps(&dev, &data) == HAL_OK && data.x == 35.0f);
    error = HAL_TIMEOUT;
    assert(ASM330LHHX_Init(&dev, &dev.config) == HAL_TIMEOUT && !dev.initialized);
    assert(cs == GPIO_PIN_SET);
    assert(ASM330LHHX_ReadAccelMg(&dev, &data) == HAL_ERROR);
    puts("ASM330 config API tests passed.");
    return 0;
}
