#include "mmc5983.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static SPI_HandleTypeDef spi;
static GPIO_TypeDef gpio;
static GPIO_PinState cs = GPIO_PIN_SET;
static HAL_StatusTypeDef injected_error = HAL_OK;
static uint8_t registers[64];
static uint8_t id = MMC5983_PRODUCT_ID;
static uint8_t pending_mask;
static uint8_t temperature_raw = 120U; // 21 C
static uint32_t tick, conversion_start, conversion_duration;
static bool never_ready;
static uint32_t raw_x = 131073U, raw_y = 147458U, raw_z = 114691U;

uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { assert(cs == GPIO_PIN_SET); tick += ms; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    assert(port == &gpio && pin == 1U);
    cs = state;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *bus, uint8_t *tx,
                                         uint8_t *rx, uint16_t size, uint32_t timeout)
{
    assert(bus == &spi && cs == GPIO_PIN_RESET && timeout == 100U);
    if (injected_error != HAL_OK) {
        return injected_error;
    }
    memset(rx, 0, size);
    assert((tx[0] & 0x40U) == 0U); // Bit 6 not required for burst read.
    uint8_t reg = tx[0] & 0x3FU;
    if (tx[0] & 0x80U) {
        if (reg == 0x2FU) {
            assert(size == 2U);
            rx[1] = id;
        } else if (reg == 0x08U) {
            assert(size == 2U);
            if (!never_ready && pending_mask &&
                (uint32_t)(tick - conversion_start) >= conversion_duration) {
                registers[0x08] |= pending_mask;
                pending_mask = 0U;
            }
            rx[1] = registers[0x08];
        } else if (reg == 0U) {
            assert(size == 8U && (registers[0x08] & 1U));
            rx[1] = (uint8_t)(raw_x >> 10); rx[2] = (uint8_t)(raw_x >> 2);
            rx[3] = (uint8_t)(raw_y >> 10); rx[4] = (uint8_t)(raw_y >> 2);
            rx[5] = (uint8_t)(raw_z >> 10); rx[6] = (uint8_t)(raw_z >> 2);
            rx[7] = (uint8_t)(((raw_x & 3U) << 6) | ((raw_y & 3U) << 4) | ((raw_z & 3U) << 2));
        } else {
            assert(reg == 7U && size == 2U && (registers[0x08] & 2U));
            rx[1] = temperature_raw;
        }
    } else {
        assert(size == 2U);
        uint8_t value = tx[1];
        if (reg == 0x0AU && (value & 0x80U)) {
            memset(registers, 0, sizeof(registers));
            pending_mask = 0x10U;
            conversion_start = tick;
            conversion_duration = 10U;
        } else if (reg == 0x08U) {
            registers[reg] &= (uint8_t)~value;
        } else {
            registers[reg] = value;
            if (reg == 0x09U) {
                assert((value & 3U) != 3U); // Never trigger both measurements.
                static const uint8_t magnetic_ms[4] = { 8U, 4U, 2U, 1U };
                if (value & 3U) {
                    pending_mask = value & 3U;
                    registers[0x08] &= (uint8_t)~pending_mask;
                    conversion_start = tick;
                    conversion_duration = value & 1U ? magnetic_ms[registers[0x0A] & 3U] : 8U;
                }
            }
        }
    }
    return HAL_OK;
}

int main(void)
{
    MMC5983_t dev;
    MMC5983_MagneticData_t field;
    MMC5983_Temperature_t temperature;
    MMC5983_AssignSPI(&dev, &spi, &gpio, 1U);
    assert(MMC5983_ReadMagnetic(&dev, &field) == MMC5983_ERROR_NOT_INITIALIZED);
    assert(MMC5983_Init(&dev) == MMC5983_OK);
    assert(dev.product_id == 0x30U && dev.initialized);
    assert(registers[0x0B] == 0U && registers[0x0C] == 0U && registers[0x09] == 0x20U);
    assert(MMC5983_ReadMagnetic(&dev, &field) == MMC5983_OK);
    assert(field.raw_x == raw_x && field.raw_y == raw_y && field.raw_z == raw_z);
    assert(field.x_centi_ut == 0 && field.y_centi_ut == 10001 && field.z_centi_ut == -9998);
    assert(field.x_ut > 0.006f && field.x_ut < 0.007f && !field.at_rail);
    assert(MMC5983_ReadTemperature(&dev, &temperature) == MMC5983_OK);
    assert(temperature.raw == 120U && temperature.temperature_centi_c == 2100);
    assert(temperature.temperature_c > 20.99f && temperature.temperature_c < 21.01f);
    temperature_raw = 90U;
    assert(MMC5983_ReadTemperature(&dev, &temperature) == MMC5983_OK);
    assert(temperature.temperature_centi_c == -300);

    raw_x = 0U; raw_y = 131072U; raw_z = 262143U;
    assert(MMC5983_ReadMagnetic(&dev, &field) == MMC5983_OK);
    assert(field.x_centi_ut == -80000 && field.y_centi_ut == 0 && field.z_centi_ut == 79999);
    assert(field.at_rail);
    for (unsigned int bw = 0U; bw < 4U; ++bw) {
        dev.config.bandwidth = (MMC5983_Bandwidth_t)bw;
        dev.config.automatic_set_reset = false;
        assert(MMC5983_Init(&dev) == MMC5983_OK);
        assert(registers[0x0A] == bw && dev.control0 == 0U);
        assert(MMC5983_ReadMagnetic(&dev, &field) == MMC5983_OK);
    }
    assert(MMC5983_Reset(&dev) == MMC5983_OK && registers[0x09] == 0x10U);
    assert(MMC5983_Set(&dev) == MMC5983_OK && registers[0x09] == 0x08U);
    MMC5983_MagneticData_t before = field;
    injected_error = HAL_TIMEOUT;
    assert(MMC5983_ReadMagnetic(&dev, &field) == MMC5983_ERROR_SPI);
    assert(cs == GPIO_PIN_SET && dev.last_hal_status == HAL_TIMEOUT);
    assert(memcmp(&before, &field, sizeof(field)) == 0);
    injected_error = HAL_OK;
    never_ready = true;
    tick = UINT32_MAX - 2U; // Deadline arithmetic must survive tick wrap.
    assert(MMC5983_ReadMagnetic(&dev, &field) == MMC5983_ERROR_TIMEOUT);
    assert(memcmp(&before, &field, sizeof(field)) == 0 && cs == GPIO_PIN_SET);
    never_ready = false;
    assert(MMC5983_ReadMagnetic(&dev, &field) == MMC5983_OK);
    assert(MMC5983_ReadTemperature(NULL, &temperature) == MMC5983_ERROR_ARGUMENT);
    assert(MMC5983_ReadMagnetic(&dev, NULL) == MMC5983_ERROR_ARGUMENT);
    dev.config.bandwidth = (MMC5983_Bandwidth_t)4;
    assert(MMC5983_Init(&dev) == MMC5983_ERROR_ARGUMENT && !dev.initialized);
    dev.config.bandwidth = MMC5983_BW_100HZ;
    id = 0xFFU;
    assert(MMC5983_Init(&dev) == MMC5983_ERROR_ID && !dev.initialized);
    id = MMC5983_PRODUCT_ID;
    never_ready = true;
    assert(MMC5983_Init(&dev) == MMC5983_ERROR_TIMEOUT && !dev.initialized);
    puts("MMC5983 tests passed: ID/OTP, 18-bit unpacking, units, BW, SET/RESET, timeout and SPI errors.");
    return 0;
}
