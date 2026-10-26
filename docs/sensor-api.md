# Sensor API

Both drivers follow the same lifecycle:

```c
ASM330LHHX_t imu;
MS5611_t barometer;
ASM330LHHX_Config_t imu_config = ASM330LHHX_CONFIG_DEFAULT;
MS5611_Config_t pressure_config = MS5611_CONFIG_DEFAULT;

/* Call after CubeMX HAL/GPIO/SPI initialization. */
ASM330LHHX_AssignSPI(&imu, &hspi2, ASM330_CS_GPIO_Port, ASM330_CS_Pin);
MS5611_AssignSPI(&barometer, &hspi2, MS5611_CS_GPIO_Port, MS5611_CS_Pin);

/* Check return values before reading measurements. */
HAL_StatusTypeDef imu_status = ASM330LHHX_Init(&imu, &imu_config);
MS5611_Status_t pressure_status = MS5611_Init(&barometer, &pressure_config);
```

- `AssignSPI`: initializes the handle and drives CS high; no sensor transaction.
- `Init(device, const config*)`: validates and copies a non-null configuration,
  initializes hardware, and sets `device.initialized` on success. Failure clears
  that flag. Treat `device.config` as driver-owned; change your config and call
  Init again to apply changes. Config objects need not remain alive after Init.
- ASM330 config: accel/gyro ODR and full-scale, SPI timeout in ms.
- MS5611 config: pressure/temperature OSR, SPI timeout in ms, second-order compensation.
- Measurement calls require successful initialization. ASM330 offers
  `ReadAccelRaw`, `ReadGyroRaw`, `ReadAccelMg`, `ReadGyroMdps`; MS5611 offers
  `Read` (mbar/Celsius, plus raw and fixed-point fields).
- ASM330 ID/register calls are available after AssignSPI; `ReadDeviceID` should
  return `ASM330LHHX_WHO_AM_I_VALUE`. Low-level register writes bypass configuration
  bookkeeping: reinitialize before scaled measurements if changing sensor settings.
- Return types remain device-specific: ASM330 uses `HAL_StatusTypeDef`, MS5611
  uses `MS5611_Status_t` with `last_hal_status` for SPI diagnostics.
- Calls are blocking. Caller owns GPIO/SPI setup and shared-bus serialization;
  neither driver switches SPI modes. Configuration is in main's USER CODE blocks.

Breaking API change: replace the ASM330 positional Init arguments and the MS5611
single-argument Init call with the two-argument config-pointer calls above.
