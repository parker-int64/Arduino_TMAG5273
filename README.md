# TMAG5273

Arduino library for the Texas Instruments **TMAG5273** low-power linear 3D
Hall-effect sensor with I2C interface.

The register map, bit fields, timing and conversion formulas in this library
come from the [TMAG5273 datasheet, SLYS045C (revised April 2026)](https://www.ti.com/product/TMAG5273).

## Supported devices

| Orderable part  | Magnetic range (`TMAG5273_RANGE_LOW` / `_HIGH`) | I2C address |
| --------------- | ----------------------------------------------- | ----------- |
| TMAG5273A1 / A2 | ±40 mT / ±80 mT, ±133 mT / ±266 mT              | 0x35        |
| TMAG5273B1 / B2 | same                                            | 0x22        |
| TMAG5273C1 / C2 | same                                            | 0x78        |
| TMAG5273D1 / D2 | same                                            | 0x44        |

Parts ending in `1` (x1) and `2` (x2) are detected automatically from the
`DEVICE_ID` register, so the magnetic field is always scaled with the correct
full scale.

## Wiring

The TMAG5273 works from 1.7 V to 3.6 V, so use a 3.3 V board (or a level
shifter on a 5 V board).

| TMAG5273 | Board   |
| -------- | ------- |
| VCC      | 3V3     |
| GND      | GND     |
| SCL      | SCL     |
| SDA      | SDA     |
| INT      | not connected, unless interrupts are used |

Connect the TEST pin to ground, as required by the datasheet.

## Installation

Copy this folder into your Arduino `libraries` folder, or install it with the
Library Manager. The library depends only on `Wire`.

## Example

```cpp
#include <Wire.h>
#include <TMAG5273.h>

TMAG5273 sensor;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!sensor.begin()) {
    Serial.println("TMAG5273 not found");
    while (true) {
    }
  }

  sensor.enableTemperature();
  sensor.setOperatingMode(TMAG5273_MODE_CONTINUOUS);
}

void loop() {
  Serial.print(sensor.readXMillitesla(), 2);
  Serial.print(", ");
  Serial.print(sensor.readYMillitesla(), 2);
  Serial.print(", ");
  Serial.print(sensor.readZMillitesla(), 2);
  Serial.print(" mT, ");
  Serial.print(sensor.readTemperature(), 2);
  Serial.println(" C");
  delay(100);
}
```

More sketches are in [examples/](examples):

| Sketch | Shows |
| ------ | ----- |
| `ReadMagneticField` | Continuous measurement of X, Y, Z and the temperature |
| `TriggeredRead` | One conversion at a time in standby mode |
| `ReadAngle` | The hardware angle engine and the field magnitude |
| `EncoderPosition` | Multi-turn position with the `TMAG5273Encoder` layer |
| `ProximitySwitch` | Threshold comparison with the `TMAG5273Proximity` layer |
| `ProximityLowPower` | Wake-up and sleep mode with the INT pin |

## API

### Initialization

| Function | Description |
| -------- | ----------- |
| `begin(address = TMAG5273_DEFAULT_ADDRESS, wire = Wire)` | Checks the manufacturer ID and the variant and restores the default configuration: X, Y and Z channels, low magnetic range, 1x averaging, no temperature compensation, standard read mode, CRC disabled, no angle calculation, interrupts off, standby mode. Call `Wire.begin()` first. |
| `end()` | Puts the device into sleep mode (5 nA typical). A sleeping device does not acknowledge its address; call `begin()` to talk to it again. |
| `isConnected()` | Probes the I2C address. Returns `false` while the device sleeps. |
| `getAddress()`, `setAddress(address)` | Read or change the I2C address through the `I2C_ADDRESS` register. The new address is lost at the next power cycle. |
| `getVariant()` | `TMAG5273_VARIANT_1` (x1) or `TMAG5273_VARIANT_2` (x2). |
| `readManufacturerId()`, `readDeviceId()` | Raw identification registers. |

### Channels, ranges and averaging

| Function | Description |
| -------- | ----------- |
| `setMagneticChannels(channels)`, `getMagneticChannels()` | `TMAG5273_CHANNEL_X` ... `TMAG5273_CHANNEL_XYZ`, plus the pseudo-simultaneous `XYX`, `YXY`, `YZY` and `XZX` modes. |
| `setXYRange(range)`, `setZRange(range)` | `TMAG5273_RANGE_LOW` or `TMAG5273_RANGE_HIGH`. |
| `getXYRangeMillitesla()`, `getZRangeMillitesla()` | Full scale in mT for the current variant and range. |
| `setAveraging(averaging)`, `getAveraging()` | `TMAG5273_AVERAGE_1X` ... `TMAG5273_AVERAGE_32X`. |
| `setTemperatureCompensation(compensation)`, `getTemperatureCompensation()` | `TMAG5273_TEMPCO_NONE`, `TMAG5273_TEMPCO_NDFEB`, `TMAG5273_TEMPCO_CERAMIC`. |
| `enableTemperature()`, `disableTemperature()`, `isTemperatureEnabled()` | Enables the temperature channel (`T_CH_EN`). |

### Operating modes and power

| Function | Description |
| -------- | ----------- |
| `setOperatingMode(mode)`, `getOperatingMode()` | `TMAG5273_MODE_STANDBY`, `TMAG5273_MODE_SLEEP`, `TMAG5273_MODE_CONTINUOUS`, `TMAG5273_MODE_WAKE_UP_SLEEP`. |
| `setSleepTime(sleepTime)`, `getSleepTime()` | Interval of the wake-up and sleep mode, `TMAG5273_SLEEP_TIME_1MS` ... `TMAG5273_SLEEP_TIME_20000MS`. |
| `enableLowNoiseMode()`, `disableLowNoiseMode()`, `isLowNoiseModeEnabled()` | `LP_LN`: low noise instead of low active current. |
| `setTriggerMode(mode)`, `getTriggerMode()` | `TMAG5273_TRIGGER_I2C_COMMAND` (default) or `TMAG5273_TRIGGER_INT_PIN`. |
| `enableGlitchFilter()`, `disableGlitchFilter()`, `isGlitchFilterEnabled()` | `I2C_GLITCH_FILTER`, enabled after `begin()`. |
| `triggerConversion()` | Starts a conversion in standby mode. In continuous mode the device converts on its own. |
| `isDataReady()` | True when `CONV_STATUS.RESULT_STATUS` is set. |
| `waitForDataReady(timeout = 100)` | Waits for `triggerConversion()` and the conversion to finish, using the conversion time from the datasheet. |
| `hasDiagnosticFailure()` | `CONV_STATUS.DIAG_STATUS`. |

### Measurement

| Function | Description |
| -------- | ----------- |
| `readMeasurement()` | One transaction for every enabled channel. Returns a `TMAG5273_Measurement` with raw codes, mT, °C, the conversion status and a `valid` flag. |
| `readX()`, `readY()`, `readZ()` | Raw 16-bit two's complement ADC codes. |
| `readXMillitesla()`, `readYMillitesla()`, `readZMillitesla()` | Magnetic field in mT. |
| `readMillitesla(axis)` | Magnetic field of `TMAG5273_AXIS_X`, `_Y` or `_Z` in mT. |
| `readXMicrotesla()`, `readYMicrotesla()`, `readZMicrotesla()` | Magnetic field in µT. |
| `readXGauss()`, `readYGauss()`, `readZGauss()` | Magnetic field in gauss. |
| `readTemperature()` | Die temperature in °C, `NAN` if the read fails. |
| `readAngle()`, `readMagnitude()`, `readMagnitudeMillitesla()` | Angle in degrees and magnitude, see below. |
| `readConversionStatus()`, `readDeviceStatus()` | Raw status registers. |

In the 1-byte read modes `readX()` and the other single value functions read
the whole packet, so `readMeasurement()` is the efficient choice there. Angle
and magnitude results and the device status register are only available in
the standard read mode. The magnetic field functions return `NAN` and the raw
code functions return `0` when the transfer fails, for example while the
device sleeps between two wake-up intervals.

### I2C read modes and CRC

| Function | Description |
| -------- | ----------- |
| `setReadMode(readMode)`, `getReadMode()` | `TMAG5273_READ_STANDARD`, `TMAG5273_READ_16BIT` (one byte command, 16-bit data) or `TMAG5273_READ_8BIT` (one byte command, 8-bit data). |
| `enableCrc()`, `disableCrc()`, `isCrcEnabled()` | Adds and verifies the CRC byte of every read. |

With CRC enabled, a standard read frame always carries four register bytes
and the CRC byte calculated over them (datasheet figure 6-10), so
`readRegisters()` reads in blocks of four and verifies each block. In the
1-byte read modes the CRC covers the command byte and the data of the packet
(datasheet section 6.5.1.3.4). Failed checks make the read functions return
`0`, `NAN` or an invalid measurement.

### Angle, gain and offset

| Function | Description |
| -------- | ----------- |
| `setAngleCalculation(angle)`, `getAngleCalculation()` | `TMAG5273_ANGLE_OFF`, `TMAG5273_ANGLE_XY`, `TMAG5273_ANGLE_YZ`, `TMAG5273_ANGLE_XZ`. |
| `readAngle()` | Angle in degrees (`integer + fraction / 16`), `NAN` when the angle calculation is off or one of the two axes is not enabled. |
| `readMagnitude()`, `readMagnitudeMillitesla()` | Resultant vector magnitude of the two angle axes. |
| `setGainValue(gain)`, `getGainValue()` | Gain correction of one axis, `value / 256`; the value `0` means a gain of 1. |
| `setGainChannel(channel)`, `getGainChannel()` | `TMAG5273_GAIN_CHANNEL_FIRST` or `TMAG5273_GAIN_CHANNEL_SECOND`: which axis of the angle pair the gain value applies to. |
| `setFirstAxisOffsetMillitesla(offset)`, `getFirstAxisOffsetMillitesla()` | Offset correction of the first angle axis. |
| `setSecondAxisOffsetMillitesla(offset)`, `getSecondAxisOffsetMillitesla()` | Offset correction of the second angle axis. |

Gain and offset correction are applied by the device only while the angle
calculation is enabled. The axis that the first and second offset refer to is
defined by `setAngleCalculation()`: XY, YZ or XZ.

### Encoder application layer

`TMAG5273Encoder` builds a multi-turn magnetic encoder on top of the angle
engine. It owns no hardware of its own, it configures a `TMAG5273` instance
and reads from it. See [examples/EncoderPosition](examples/EncoderPosition).

```cpp
#include <TMAG5273Encoder.h>

TMAG5273 sensor;
TMAG5273Encoder encoder(sensor);

void setup() {
  Wire.begin();
  sensor.begin();
  encoder.begin(TMAG5273_ANGLE_XY);  // 32x averaging and continuous mode
}

void loop() {
  float position = encoder.readPosition();  // degrees, counts across turns
  Serial.println(encoder.getTurns());
  delay(50);
}
```

| Function | Description |
| -------- | ----------- |
| `TMAG5273Encoder(sensor)` | Wraps an initialised `TMAG5273`. |
| `begin(anglePair = TMAG5273_ANGLE_XY, averaging = TMAG5273_AVERAGE_32X)` | Enables the two axes of the pair, the angle calculation, the averaging and continuous mode. |
| `readAngle()` | Mechanical angle in degrees, `NAN` when no angle result is available. |
| `readMagnitude()` | Magnitude of the field in mT. |
| `readPosition()` | Position in degrees that keeps counting across full turns. The first call establishes the reference and returns the current angle. |
| `getPosition()`, `getTurns()` | Last position without I2C traffic, and the number of complete turns. |
| `resetPosition(degrees = 0)` | Defines the position at the current angle, for example after homing. |
| `setMagnetThresholdMillitesla()`, `getMagnetThresholdMillitesla()`, `isMagnetDetected()` | Magnet presence supervision, 1 mT by default. |

`readPosition()` unwraps the angle, so it must be called at least twice per
revolution: the magnet may move less than 180 degrees between two calls. The
shorter the loop, the faster the magnet may rotate without losing a turn.

### Proximity application layer

`TMAG5273Proximity` turns the sensor into a magnetic proximity switch: one
axis is compared against a threshold with a hysteresis, which is the pattern
behind smart locks, door and window sensors and proximity switches. See
[examples/ProximitySwitch](examples/ProximitySwitch) for polling and
[examples/ProximityLowPower](examples/ProximityLowPower) for the interrupt
driven low power variant.

```cpp
#include <TMAG5273Proximity.h>

TMAG5273 sensor;
TMAG5273Proximity proximity(sensor);

void setup() {
  Wire.begin();
  sensor.begin();
  proximity.begin(TMAG5273_AXIS_Z, 2.0f);  // near when the field is above 2 mT
  proximity.enableInterrupt();             // INT pin goes low on the event
  proximity.startLowPower(TMAG5273_SLEEP_TIME_100MS);
}

void loop() {
  if (digitalRead(2) == LOW) {             // or an attachInterrupt flag
    bool near = proximity.isNear();        // the device is in standby now
    proximity.startLowPower(TMAG5273_SLEEP_TIME_100MS);  // re-arm
  }
}
```

| Function | Description |
| -------- | ----------- |
| `TMAG5273Proximity(sensor)` | Wraps an initialised `TMAG5273`. |
| `begin(axis = TMAG5273_AXIS_Z, threshold = 2.0f)` | Enables the axis, programs its threshold, selects the absolute threshold mode and continuous measurement. Thresholds of the other axes are cleared. |
| `setAxis(axis)`, `getAxis()` | Selects the sensed axis and moves the threshold with it. |
| `setThresholdMillitesla()`, `getThresholdMillitesla()` | Limit of the selected axis. A threshold of `0` disables the hardware comparison. |
| `setDirection(direction)`, `getDirection()` | `TMAG5273_THRESHOLD_ABOVE` (near when the field is above the limit) or `TMAG5273_THRESHOLD_BELOW`. |
| `setHysteresisMillitesla()`, `getHysteresisMillitesla()` | Width of the band that keeps `isNear()` from chattering, 0.5 mT by default. |
| `readFieldMillitesla()` | Field of the selected axis in mT. |
| `isNear()` | Polls the field and returns the debounced state. Keeps the last state when the transfer fails. |
| `enableInterrupt()`, `disableInterrupt()` | Threshold interrupt on the INT pin, latched until the device is addressed. |
| `startLowPower(interval)`, `stopLowPower()` | Wake-up and sleep mode with the given interval, or back to standby. |

The low power pattern: arm the interrupt, start the wake-up and sleep mode
with the interval the product needs, let the microcontroller sleep on the INT
pin. After a threshold event the device leaves the low power mode and stays
in standby, so `isNear()` returns a fresh field and the read releases the
latch. Call `startLowPower()` again to re-arm.

### Thresholds and interrupts

| Function | Description |
| -------- | ----------- |
| `setXThresholdMillitesla()`, `setYThresholdMillitesla()`, `setZThresholdMillitesla()` | Magnetic limit of an axis. A code of 0 (the default) disables the comparison. |
| `getXThresholdMillitesla()`, `getYThresholdMillitesla()`, `getZThresholdMillitesla()` | Current magnetic limit. |
| `setThresholdMillitesla(axis, threshold)`, `getThresholdMillitesla(axis)` | Same for a `TMAG5273_AXIS_X`, `_Y` or `_Z` value, handy in loops. |
| `setTemperatureThreshold(celsius)`, `getTemperatureThreshold()` | Temperature limit, resolution 8 °C per LSB. |
| `setThresholdMode(mode)`, `getThresholdMode()` | `TMAG5273_THRESHOLD_MODE_ABSOLUTE` or `TMAG5273_THRESHOLD_MODE_SYMMETRIC` (north and south limit of equal size). |
| `setThresholdDirection(direction)`, `getThresholdDirection()` | `TMAG5273_THRESHOLD_ABOVE` or `TMAG5273_THRESHOLD_BELOW`; ignored in symmetric mode. |
| `setThresholdCount(count)`, `getThresholdCount()` | `TMAG5273_THRESHOLD_ONE_CROSSING` or `TMAG5273_THRESHOLD_FOUR_CROSSINGS`. |
| `enableResultInterrupt()`, `disableResultInterrupt()`, `isResultInterruptEnabled()` | Interrupt when the conversions are complete. |
| `enableThresholdInterrupt()`, `disableThresholdInterrupt()`, `isThresholdInterruptEnabled()` | Interrupt when a threshold is crossed. |
| `setInterruptMode(mode)`, `getInterruptMode()` | `TMAG5273_INTERRUPT_DISABLED`, `TMAG5273_INTERRUPT_ON_INT_PIN(_EXCEPT_I2C_BUSY)`, `TMAG5273_INTERRUPT_ON_SCL_PIN(_EXCEPT_I2C_BUSY)`. |
| `setInterruptLatched()`, `setInterruptPulsed()`, `isInterruptLatched()` | Latched until the device is addressed, or a 10 µs pulse. |
| `enableInterruptPin()`, `disableInterruptPin()`, `isInterruptPinEnabled()` | `MASK_INTB`; disable the pin when INT is tied to ground. |

### Diagnostics

| Function | Description |
| -------- | ----------- |
| `hasDeviceError()` | True when one of the `DEVICE_STATUS` error bits is set: `TMAG5273_VCC_UNDERVOLTAGE`, `TMAG5273_OTP_CRC_ERROR`, `TMAG5273_INT_PIN_ERROR`, `TMAG5273_OSCILLATOR_ERROR`. |
| `clearDeviceErrors()` | Clears the error flags (write 1 to clear). |
| `readDeviceStatus()` | Raw `DEVICE_STATUS` register. |

### Low level access

`readRegister()`, `readRegisters()` and `writeRegister()` give direct access
to the register map.

The configuration registers are shadowed inside the object: the getters and
the mT conversions use the values last written through this library and do
not cause I2C traffic, which also keeps them usable while the device sleeps.
A register written directly with `writeRegister()` is therefore not reflected
in the shadow copy; use the API functions for configuration.

## Layer model

The library has two layers:

1. `TMAG5273` is a register faithful driver: every datasheet capability is
   reachable and the names and units follow the datasheet.
2. `TMAG5273Encoder` is a small application layer for one use case.

Splitting the two is intentional. The same chip serves very different jobs:

| Job | Typical products | What the application needs |
| --- | ---------------- | -------------------------- |
| Field vector | Electricity meters, joysticks, drone payload control | Calibrated X, Y, Z field and temperature |
| Rotation | E-bike, mobile robot motor control, thermostat dials | Angle, multi-turn position, direction |
| Presence and tamper | Smart locks, door and window sensors, proximity switches | Field magnitude against a limit, latched events, low power polling |
| Health | All of the above | Power-on reset, undervoltage, OTP CRC and oscillator flags |

## Units

The datasheet specifies the magnetic range in millitesla, so the library
reports millitesla first (1 mT = 10 gauss = 1000 µT). The raw code is a
16-bit two's complement value where one LSB is `2 * range / 65536`
(datasheet equation 10).

Further conversions:

| Value | Formula | Datasheet |
| ----- | ------- | --------- |
| Temperature | `25 °C + (code - 17508) / 58` | 5.6, equation 12 |
| Threshold | `code * range / 128` | tables 8-7 to 8-9 |
| Temperature threshold | `code * 8 - 249` (so `-41 °C` is `0x1A`) | table 8-10 |
| Offset | `code * range / 2048` | equation 16 |
| Angle | `bits[12:4] + bits[3:0] / 16` | equation 14 |
| Magnitude | `code * range / 128` (larger range of the two axes) | equation 15 |

## Implemented features

- [x] Identification, variant detection and default configuration
- [x] Magnetic channel, range, averaging, temperature compensation, sleep time and operating mode configuration
- [x] Triggered and continuous conversions, data-ready polling
- [x] X, Y, Z magnetic field in mT, µT and gauss, die temperature
- [x] CRC on read, in the standard and in the 1-byte read modes
- [x] 1-byte read commands for 16-bit and 8-bit data
- [x] Angle and magnitude results
- [x] Gain and offset correction
- [x] Magnetic and temperature thresholds with interrupt configuration
- [x] Wake-up and sleep mode timing
- [x] User configurable I2C address
- [x] Diagnostic flags of `DEVICE_STATUS`
- [x] Low noise mode, I2C glitch filter, trigger through the INT pin, gain channel selection
- [x] `TMAG5273Encoder` application layer for magnetic rotary encoders
- [x] `TMAG5273Proximity` application layer for magnetic proximity switches
