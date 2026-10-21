# ADXL375 driver and exclusive bring-up test

Based on `datasheets/adxl375.pdf`, Rev B, particularly pages 15, 17, 20-24,
and 29. Files: `Core/Inc/adxl375.h`, `Core/Src/adxl375.c`.

## Critical shared-bus limitation (page 15)

CS high enables I2C; another device's SPI traffic can resemble an I2C START
and address, causing ADXL375 to interfere. The recommended OR gate is on
**SDA/SDI (pin 13)**, not SCLK. It holds SDA high while CS is high.
An earlier discussion of clock gating was not the datasheet's recommended
circuit. There is no register setting in this driver that disables I2C.
Standby, disabled interrupts, slower clocks and mode switching do not eliminate
this hazard. Never hold ADXL CS low during another device's transaction: this
could select two SPI slaves and corrupt registers or cause contention.

This board has no OR gate. Therefore main defaults to `ADXL375_EXCLUSIVE_TEST=1`:
only the ADXL375 receives SPI transactions, with every other sensor CS held
high. The test runs before any MMC diagnostic, ASM or MS5611 initialization,
and does not return to the old multi-sensor loop. This is a bring-up mitigation,
not a claim of electrically isolating other chips or proving shared-bus safety.
Power-cycle before testing to clear any state left by earlier firmware.

For simultaneous multi-sensor operation, prefer a separate SPI bus, the
recommended SDI gate, or suitable physical isolation. A software-only scheme
would need to control and verify SDA/SCL transitions at every transaction and
mode boundary so no I2C START can occur while ADXL CS is high. The current
hardware-SPI drivers do not establish that guarantee. Arbitrary dummy bytes or
changing all devices to mode 3 is not a sufficient fix.

## Configuration and payloads

- SPI: mode 3, 8-bit, MSB first, 4-wire, software CS.
- Test enables master IO-state retention and establishes idle-high SCK.
- SPI2 PLL1Q is currently 5 MHz; /32 gives 156.25 kHz.
- Driver fixes ODR at 100 Hz, normal power. This exceeds the recommended 50 kHz
  minimum SPI speed for 100 Hz ODR and is below the 5 MHz maximum.
- DATA_FORMAT=0x0B: mandatory fixed bits, right-justified signed output,
  four-wire mode, self-test disabled. Do not use ADXL345 range settings here.
- POWER_CTL=0x08: measurement mode. FIFO bypass, offset registers cleared,
  interrupt outputs disabled. DATA_READY can still be polled.
- ID read: `80 00`, two bytes, expected second byte E5.
- Acceleration burst: `F2` + six dummy bytes, seven bytes / 56 clocks total.
  Bit 6 is required for ADXL multi-byte reads (unlike MMC5983).
- Data is signed little-endian. Nominal conversion is **49 mg/LSB**.

Conservative 1 ms CS setup/hold/gap delays prioritize bring-up correctness,
not maximum throughput. Replace with validated short delays for high-rate
acquisition. FIFO/interrupt-driven logging is not implemented; 500 ms printing
intentionally discards most 100 Hz samples.

## What the verification means

Main prints ID, BW_RATE, POWER_CTL, DATA_FORMAT, INT_SOURCE and FIFO_STATUS,
then waits for DATA_READY and prints a new XYZ sample in mg and raw counts.
Expected configuration: ID=E5 BW=0A POWER=08 FORMAT=0B.
Overrun may be reported when polling much slower than the ODR; this is expected
in this demonstration. Reading INT_SOURCE clears some event flags; reading
acceleration data clears data-related flags.

Rotate the board slowly while stationary between orientations. A gravity-axis
change should be roughly 20 counts per g, but this high-g sensor's offset and
noise mean a strict 1 g magnitude check is not a reliable health test. Passing
ID/configuration/data-ready checks demonstrates communication/configuration
and acquisition, **not full mechanical health or calibration**.

No automatic electrostatic self-test is claimed. Datasheet page 29 requires
stationary before/after averages, at least four sample periods of settling,
and supply-aware interpretation of the Z response. X/Y self-test response is
bimodal and unsuitable as a reliable health criterion. Do not apply a generic
ADXL345 threshold or a fabricated pass limit.

Status codes: 0 OK, 1 bad argument/SPI configuration, 2 HAL SPI error, 3 wrong
ID, 4 readback mismatch, 5 uninitialized, 6 sample timeout.

## Disable/remove

Set `ADXL375_EXCLUSIVE_TEST=0` in main or via compiler definition to restore
previous startup/loop behavior. **This also restores the previous shared-bus
risks**; do not interpret the switch as a safety guarantee. The exclusive helper
is wrapped in `#if ADXL375_EXCLUSIVE_TEST` and can be removed with its call.
The library has no dependency on the test switch.

## Host tests

```sh
gcc -std=c11 -Wall -Wextra -Werror -Itests/stubs -ICore/Inc \
    Core/Src/adxl375.c tests/test_adxl375.c -o /tmp/test_adxl375
/tmp/test_adxl375
```

Tests check framing/counts, mandatory config bits, signed little-endian decode,
49 mg/LSB scaling, readback failure, timeouts, output preservation, and CS release
on SPI failure. They cannot verify electrical contention or physical operation.
