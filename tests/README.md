# MS5611 tests and hardware check

## Host tests

The tests compile the actual library against a mock HAL. They cover the published
1000.09 mbar / 20.07 C example, low/very-low temperature compensation, CRC-4,
all OSR conversion delays, command/response framing, SPI failures, invalid ADC
results, invalid configuration, and disconnected/corrupt PROM data.

With GCC from the repository root (put the executable outside the source tree):

```sh
gcc -std=c11 -Wall -Wextra -Werror -Itests/stubs -ICore/Inc \
    Core/Src/ms5611.c tests/test_ms5611.c -o /tmp/test_ms5611
/tmp/test_ms5611
```

Alternatively, in a Visual Studio x64 Native Tools Command Prompt:

```bat
cl /nologo /std:c11 /W4 /Itests/stubs /ICore/Inc Core/Src/ms5611.c tests/test_ms5611.c /Fo%TEMP%\ /Fe%TEMP%\test_ms5611.exe
%TEMP%\test_ms5611.exe
```

Do not add `tests/stubs` to firmware include paths; it replaces the real HAL only
for host tests.

## Firmware usage and configurable parameters

The library is in `Core/Inc/ms5611.h` and `Core/Src/ms5611.c`. `main.c` already
initializes the MS5611 on SPI2 and logs readings via ITM stimulus port 0.

Edit `ms5611_config` in the `USER CODE BEGIN PV` block in `main.c`:

- `pressure_osr`, `temperature_osr`: `MS5611_OSR_256`, `_512`, `_1024`, `_2048`,
  or `_4096`. Higher OSR improves resolution but increases conversion time.
- `spi_timeout_ms`: finite HAL timeout per transaction (default 100 ms).
- `second_order_compensation`: default `true`; applies the datasheet's extra
  compensation below 20 C, including the extra correction below -15 C.

After `MS5611_AssignSPI`, settings can also be changed directly through
`dev.config` between calls to `MS5611_Read`. `AssignSPI` installs defaults;
set custom configuration **after** that call. The sample defaults use OSR 4096
for both conversions and take roughly 20 ms per reading. Reads are blocking and
must not be called from an interrupt or concurrently on the same SPI bus.

`MS5611_Data_t.pressure_mbar` and `.temperature_c` contain floating-point values
in mbar and Celsius. The `pressure_centi_mbar` and `temperature_centi_c` fields
hold the same results multiplied by 100 for exact two-decimal SWV printing
without enabling floating-point `printf` support. Raw ADC D1/D2 values are
included for diagnostics.

Status values printed by `main.c`:

| Value | Meaning |
| --- | --- |
| 0 | Success |
| 1 | Invalid argument/configuration |
| 2 | Sensor not initialized |
| 3 | HAL SPI error (see the printed HAL status) |
| 4 | Invalid calibration coefficients or PROM CRC mismatch |
| 5 | Invalid ADC response (zero or all ones) |

The MS5611 has no device-ID register. Initialization instead reads all eight
PROM words and verifies calibration/CRC. A math self-test is independent of
sensor connectivity and is explicitly labeled as such.

## On-board verification

1. Build and flash the firmware. CMake lists the new source explicitly; CubeIDE
   should discover it in `Core/Src` when rebuilding its managed project.
2. Ensure MS5611 **PS is low** (SPI selected), CS is wired to `MS5611_CS`, and
   power/ground/SPI wiring are correct. SPI2 remains mode 0 with a /32 prescaler.
3. Set the correct core clock in the debugger's SWV configuration and enable
   ITM stimulus port 0 in the SWV ITM Data Console.
4. Expect `MS5611 datasheet calculation self-test: PASS`, then
   `MS5611 initialized, PROM CRC OK`, followed by the six calibration words.
5. Every approximately 500 ms plus conversion time, expect a line like:
   `MS5611 pressure: 1000.09 mbar, temperature: 20.07 C (D1=..., D2=...)`.
   These numbers are illustrative, not a required ambient reading.
6. Compare pressure against a nearby **absolute/station pressure** reference
   (not sea-level-adjusted weather pressure), and observe temperature changing
   with gentle warming. The measured temperature is the sensor's temperature,
   which can differ from ambient air. Verify readings change rather than being
   frozen, and that no PROM/ADC/SPI errors are reported.

A passing host self-test proves software behavior, not physical wiring or sensor
accuracy. Actual on-board verification is still required.
