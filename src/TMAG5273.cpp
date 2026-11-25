/*
  TMAG5273 - Arduino library for the Texas Instruments TMAG5273 low-power
  linear 3D Hall-effect sensor with I2C interface.

  Register addresses, bit fields, timing and conversion formulas used by this
  library are taken from the TMAG5273 datasheet, SLYS045C
  (revised April 2026).
*/

#include "TMAG5273.h"

// Temperature sensor calibration, datasheet section 5.6.
const int16_t TEMPERATURE_ADC_CODE_AT_25C = 17508;
const float TEMPERATURE_CELSIUS_AT_REFERENCE = 25.0;
const float TEMPERATURE_ADC_CODES_PER_CELSIUS = 58.0;

// The magnetic result is a 16-bit two's complement code (datasheet
// equation 10), so one LSB is 2 * range / 2^16.
const float ADC_CODE_FULL_SCALE = 32768.0;

// Threshold and offset resolution is range / 128 and range / 2048
// (datasheet tables 8-7 to 8-9 and equation 16).
const float THRESHOLD_CODES_FULL_SCALE = 128.0;
const float OFFSET_CODES_FULL_SCALE = 2048.0;

// Temperature threshold codes: -41 degC is 1Ah and the resolution is
// 8 degC per LSB (datasheet table 8-10).
const int8_t TEMPERATURE_THRESHOLD_CODE_MINUS_41C = 0x1A;
const float TEMPERATURE_THRESHOLD_CODES_PER_CELSIUS = 8.0;

// DEVICE_CONFIG_1 (datasheet table 8-3).
const uint8_t DEVICE_CONFIG_1_CRC_EN = 0x80;
const uint8_t DEVICE_CONFIG_1_MAG_TEMPCO = 0x60;
const uint8_t DEVICE_CONFIG_1_CONV_AVG = 0x1C;
const uint8_t DEVICE_CONFIG_1_I2C_RD = 0x03;

// DEVICE_CONFIG_2 (datasheet table 8-4).
const uint8_t DEVICE_CONFIG_2_THR_HYST = 0xE0;
const uint8_t DEVICE_CONFIG_2_LP_LN = 0x10;
const uint8_t DEVICE_CONFIG_2_I2C_GLITCH_FILTER = 0x08;
const uint8_t DEVICE_CONFIG_2_TRIGGER_MODE = 0x04;
const uint8_t DEVICE_CONFIG_2_OPERATING_MODE = 0x03;

// SENSOR_CONFIG_1 (datasheet table 8-5).
const uint8_t SENSOR_CONFIG_1_MAG_CH_EN = 0xF0;
const uint8_t SENSOR_CONFIG_1_SLEEPTIME = 0x0F;

// SENSOR_CONFIG_2 (datasheet table 8-6).
const uint8_t SENSOR_CONFIG_2_THRX_COUNT = 0x40;
const uint8_t SENSOR_CONFIG_2_MAG_THR_DIR = 0x20;
const uint8_t SENSOR_CONFIG_2_MAG_GAIN_CH = 0x10;
const uint8_t SENSOR_CONFIG_2_ANGLE_EN = 0x0C;
const uint8_t SENSOR_CONFIG_2_X_Y_RANGE = 0x02;
const uint8_t SENSOR_CONFIG_2_Z_RANGE = 0x01;

// T_CONFIG (datasheet table 8-10).
const uint8_t T_CONFIG_T_THR_CONFIG = 0xFE;
const uint8_t T_CONFIG_T_CH_EN = 0x01;

// INT_CONFIG_1 (datasheet table 8-11).
const uint8_t INT_CONFIG_1_RSLT_INT = 0x80;
const uint8_t INT_CONFIG_1_THRSLD_INT = 0x40;
const uint8_t INT_CONFIG_1_INT_STATE = 0x20;
const uint8_t INT_CONFIG_1_INT_MODE = 0x1C;
const uint8_t INT_CONFIG_1_MASK_INTB = 0x01;

// Bit masks used to decode the magnetic channel configuration.
const uint8_t AXIS_X = 0x01;
const uint8_t AXIS_Y = 0x02;
const uint8_t AXIS_Z = 0x04;

// CRC-8, polynomial x^8 + x^2 + x + 1, initial value FFh (datasheet
// section 6.5.1.3.6).
static uint8_t crc8(const uint8_t *data, size_t length) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static int32_t roundToInteger(float value) {
  return (int32_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

// The 8-bit read mode returns the eight most significant bits of the
// conversion result, so the code is sign extended from that byte
// (datasheet equation 11).
static int16_t decodeCode(const uint8_t *data, bool eightBit) {
  if (eightBit) {
    return (int16_t)((int8_t)data[0] << 8);
  }
  return (int16_t)((uint16_t)data[0] << 8 | data[1]);
}

TMAG5273::TMAG5273() {
  _wire = &Wire;
  _address = TMAG5273_DEFAULT_ADDRESS;
  _variant = TMAG5273_VARIANT_UNKNOWN;
  _deviceConfig1 = 0x00;
  _deviceConfig2 = 0x00;
  _sensorConfig1 = 0x00;
  _sensorConfig2 = 0x00;
  _xThreshold = 0x00;
  _yThreshold = 0x00;
  _zThreshold = 0x00;
  _tConfig = 0x00;
  _intConfig1 = 0x00;
  _magGain = 0x00;
  _offset1 = 0x00;
  _offset2 = 0x00;
  _conversionStartMicros = 0;
  _conversionPending = false;
}

bool TMAG5273::begin(uint8_t address, TwoWire &wire) {
  _wire = &wire;
  _address = address;

  // tstart_power_up is 270us typical (datasheet section 5.11).
  delay(1);

  // Writes are accepted in every read mode but not in sleep mode, so restore
  // plain reads first and, if the device does not answer, wake it up and try
  // once more. This also recovers a device that a previous sketch left in
  // one of the 1-byte read modes or in sleep mode.
  bool identified = false;
  for (uint8_t attempt = 0; attempt < 2 && !identified; attempt++) {
    if (attempt > 0) {
      wakeUp();
    }
    _deviceConfig1 = 0x00;
    writeRegister(TMAG5273_REG_DEVICE_CONFIG_1, _deviceConfig1);
    identified = readManufacturerId() == TMAG5273_MANUFACTURER_ID;
  }
  if (!identified) {
    return false;
  }

  uint8_t version = readDeviceId() & 0x03;
  if (version != TMAG5273_VARIANT_1 && version != TMAG5273_VARIANT_2) {
    return false;
  }
  _variant = (TMAG5273_Variant)version;

  if (!resetConfiguration()) {
    return false;
  }

  // Acknowledge the power-on-reset flag so later status reads are meaningful.
  return writeRegister(TMAG5273_REG_CONV_STATUS, TMAG5273_POWER_ON_RESET);
}

bool TMAG5273::end() {
  return setOperatingMode(TMAG5273_MODE_SLEEP);
}

bool TMAG5273::isConnected() {
  _wire->beginTransmission(_address);
  return _wire->endTransmission() == 0;
}

uint8_t TMAG5273::getAddress() const {
  return _address;
}

bool TMAG5273::setAddress(uint8_t address) {
  if (address == 0x00) {
    return false;
  }
  // I2C_ADDRESS[7:1] holds the new address, I2C_ADDRESS[0] enables the
  // update. The device answers on the new address right after the write and
  // forgets it at the next power cycle (datasheet table 8-15).
  if (!writeRegister(TMAG5273_REG_I2C_ADDRESS,
                     (uint8_t)(address << 1 | 0x01))) {
    return false;
  }
  _address = address;
  return true;
}

TMAG5273_Variant TMAG5273::getVariant() const {
  return _variant;
}

uint16_t TMAG5273::readManufacturerId() {
  uint8_t data[2];
  if (!readRegisters(TMAG5273_REG_MANUFACTURER_ID_LSB, data, 2)) {
    return 0;
  }
  return (uint16_t)(data[1] << 8 | data[0]);
}

uint8_t TMAG5273::readDeviceId() {
  return readRegister(TMAG5273_REG_DEVICE_ID);
}

bool TMAG5273::setOperatingMode(TMAG5273_OperatingMode mode) {
  if (mode > TMAG5273_MODE_WAKE_UP_SLEEP) {
    return false;
  }
  if (getOperatingMode() == TMAG5273_MODE_SLEEP) {
    wakeUp();
  }
  return writeField(TMAG5273_REG_DEVICE_CONFIG_2, _deviceConfig2,
                    DEVICE_CONFIG_2_OPERATING_MODE, mode);
}

TMAG5273_OperatingMode TMAG5273::getOperatingMode() const {
  return (TMAG5273_OperatingMode)(_deviceConfig2 & DEVICE_CONFIG_2_OPERATING_MODE);
}

bool TMAG5273::enableLowNoiseMode() {
  return writeField(TMAG5273_REG_DEVICE_CONFIG_2, _deviceConfig2,
                    DEVICE_CONFIG_2_LP_LN, DEVICE_CONFIG_2_LP_LN);
}

bool TMAG5273::disableLowNoiseMode() {
  return writeField(TMAG5273_REG_DEVICE_CONFIG_2, _deviceConfig2,
                    DEVICE_CONFIG_2_LP_LN, 0x00);
}

bool TMAG5273::isLowNoiseModeEnabled() const {
  return (_deviceConfig2 & DEVICE_CONFIG_2_LP_LN) != 0;
}

bool TMAG5273::enableGlitchFilter() {
  return writeField(TMAG5273_REG_DEVICE_CONFIG_2, _deviceConfig2,
                    DEVICE_CONFIG_2_I2C_GLITCH_FILTER, 0x00);
}

bool TMAG5273::disableGlitchFilter() {
  return writeField(TMAG5273_REG_DEVICE_CONFIG_2, _deviceConfig2,
                    DEVICE_CONFIG_2_I2C_GLITCH_FILTER,
                    DEVICE_CONFIG_2_I2C_GLITCH_FILTER);
}

bool TMAG5273::isGlitchFilterEnabled() const {
  return (_deviceConfig2 & DEVICE_CONFIG_2_I2C_GLITCH_FILTER) == 0;
}

bool TMAG5273::setTriggerMode(TMAG5273_TriggerMode mode) {
  if (mode > TMAG5273_TRIGGER_INT_PIN) {
    return false;
  }
  return writeField(TMAG5273_REG_DEVICE_CONFIG_2, _deviceConfig2,
                    DEVICE_CONFIG_2_TRIGGER_MODE, (uint8_t)(mode << 2));
}

TMAG5273_TriggerMode TMAG5273::getTriggerMode() const {
  return (TMAG5273_TriggerMode)((_deviceConfig2 & DEVICE_CONFIG_2_TRIGGER_MODE) >> 2);
}

bool TMAG5273::setMagneticChannels(TMAG5273_MagneticChannel channels) {
  if (channels > TMAG5273_CHANNEL_XZX) {
    return false;
  }
  return writeField(TMAG5273_REG_SENSOR_CONFIG_1, _sensorConfig1,
                    SENSOR_CONFIG_1_MAG_CH_EN, (uint8_t)(channels << 4));
}

TMAG5273_MagneticChannel TMAG5273::getMagneticChannels() const {
  return (TMAG5273_MagneticChannel)((_sensorConfig1 & SENSOR_CONFIG_1_MAG_CH_EN) >> 4);
}

bool TMAG5273::setSleepTime(TMAG5273_SleepTime sleepTime) {
  if (sleepTime > TMAG5273_SLEEP_TIME_20000MS) {
    return false;
  }
  return writeField(TMAG5273_REG_SENSOR_CONFIG_1, _sensorConfig1,
                    SENSOR_CONFIG_1_SLEEPTIME, sleepTime);
}

TMAG5273_SleepTime TMAG5273::getSleepTime() const {
  return (TMAG5273_SleepTime)(_sensorConfig1 & SENSOR_CONFIG_1_SLEEPTIME);
}

bool TMAG5273::setXYRange(TMAG5273_Range range) {
  if (range > TMAG5273_RANGE_HIGH) {
    return false;
  }
  return writeField(TMAG5273_REG_SENSOR_CONFIG_2, _sensorConfig2,
                    SENSOR_CONFIG_2_X_Y_RANGE, (uint8_t)(range << 1));
}

TMAG5273_Range TMAG5273::getXYRange() const {
  return (TMAG5273_Range)((_sensorConfig2 & SENSOR_CONFIG_2_X_Y_RANGE) >> 1);
}

float TMAG5273::getXYRangeMillitesla() const {
  return fullScaleMillitesla(getXYRange());
}

bool TMAG5273::setZRange(TMAG5273_Range range) {
  if (range > TMAG5273_RANGE_HIGH) {
    return false;
  }
  return writeField(TMAG5273_REG_SENSOR_CONFIG_2, _sensorConfig2,
                    SENSOR_CONFIG_2_Z_RANGE, range);
}

TMAG5273_Range TMAG5273::getZRange() const {
  return (TMAG5273_Range)(_sensorConfig2 & SENSOR_CONFIG_2_Z_RANGE);
}

float TMAG5273::getZRangeMillitesla() const {
  return fullScaleMillitesla(getZRange());
}

bool TMAG5273::setAveraging(TMAG5273_Averaging averaging) {
  if (averaging > TMAG5273_AVERAGE_32X) {
    return false;
  }
  return writeField(TMAG5273_REG_DEVICE_CONFIG_1, _deviceConfig1,
                    DEVICE_CONFIG_1_CONV_AVG, (uint8_t)(averaging << 2));
}

TMAG5273_Averaging TMAG5273::getAveraging() const {
  return (TMAG5273_Averaging)((_deviceConfig1 & DEVICE_CONFIG_1_CONV_AVG) >> 2);
}

bool TMAG5273::setTemperatureCompensation(
    TMAG5273_TemperatureCompensation compensation) {
  if (compensation != TMAG5273_TEMPCO_NONE &&
      compensation != TMAG5273_TEMPCO_NDFEB &&
      compensation != TMAG5273_TEMPCO_CERAMIC) {
    return false;
  }
  return writeField(TMAG5273_REG_DEVICE_CONFIG_1, _deviceConfig1,
                    DEVICE_CONFIG_1_MAG_TEMPCO, (uint8_t)(compensation << 5));
}

TMAG5273_TemperatureCompensation TMAG5273::getTemperatureCompensation() const {
  return (TMAG5273_TemperatureCompensation)(
      (_deviceConfig1 & DEVICE_CONFIG_1_MAG_TEMPCO) >> 5);
}

bool TMAG5273::setReadMode(TMAG5273_ReadMode readMode) {
  if (readMode > TMAG5273_READ_8BIT) {
    return false;
  }
  return writeField(TMAG5273_REG_DEVICE_CONFIG_1, _deviceConfig1,
                    DEVICE_CONFIG_1_I2C_RD, readMode);
}

TMAG5273_ReadMode TMAG5273::getReadMode() const {
  return (TMAG5273_ReadMode)(_deviceConfig1 & DEVICE_CONFIG_1_I2C_RD);
}

bool TMAG5273::enableCrc() {
  return writeField(TMAG5273_REG_DEVICE_CONFIG_1, _deviceConfig1,
                    DEVICE_CONFIG_1_CRC_EN, DEVICE_CONFIG_1_CRC_EN);
}

bool TMAG5273::disableCrc() {
  return writeField(TMAG5273_REG_DEVICE_CONFIG_1, _deviceConfig1,
                    DEVICE_CONFIG_1_CRC_EN, 0x00);
}

bool TMAG5273::isCrcEnabled() const {
  return (_deviceConfig1 & DEVICE_CONFIG_1_CRC_EN) != 0;
}

bool TMAG5273::setAngleCalculation(TMAG5273_Angle angle) {
  if (angle > TMAG5273_ANGLE_XZ) {
    return false;
  }
  return writeField(TMAG5273_REG_SENSOR_CONFIG_2, _sensorConfig2,
                    SENSOR_CONFIG_2_ANGLE_EN, (uint8_t)(angle << 2));
}

TMAG5273_Angle TMAG5273::getAngleCalculation() const {
  return (TMAG5273_Angle)((_sensorConfig2 & SENSOR_CONFIG_2_ANGLE_EN) >> 2);
}

bool TMAG5273::setGainValue(uint8_t gain) {
  if (!writeRegister(TMAG5273_REG_MAG_GAIN_CONFIG, gain)) {
    return false;
  }
  _magGain = gain;
  return true;
}

uint8_t TMAG5273::getGainValue() const {
  return _magGain;
}

bool TMAG5273::setGainChannel(TMAG5273_GainChannel channel) {
  if (channel > TMAG5273_GAIN_CHANNEL_SECOND) {
    return false;
  }
  return writeField(TMAG5273_REG_SENSOR_CONFIG_2, _sensorConfig2,
                    SENSOR_CONFIG_2_MAG_GAIN_CH, (uint8_t)(channel << 4));
}

TMAG5273_GainChannel TMAG5273::getGainChannel() const {
  return (TMAG5273_GainChannel)((_sensorConfig2 & SENSOR_CONFIG_2_MAG_GAIN_CH) >> 4);
}

bool TMAG5273::setFirstAxisOffsetMillitesla(float offset) {
  float firstAxis;
  float secondAxis;
  if (!angleAxisFullScale(firstAxis, secondAxis)) {
    return false;
  }
  int32_t code = roundToInteger(offset * OFFSET_CODES_FULL_SCALE / firstAxis);
  if (code < -128 || code > 127) {
    return false;
  }
  if (!writeRegister(TMAG5273_REG_MAG_OFFSET_CONFIG_1, (uint8_t)(int8_t)code)) {
    return false;
  }
  _offset1 = (uint8_t)(int8_t)code;
  return true;
}

float TMAG5273::getFirstAxisOffsetMillitesla() const {
  float firstAxis;
  float secondAxis;
  if (!angleAxisFullScale(firstAxis, secondAxis)) {
    return NAN;
  }
  return (int8_t)_offset1 * firstAxis / OFFSET_CODES_FULL_SCALE;
}

bool TMAG5273::setSecondAxisOffsetMillitesla(float offset) {
  float firstAxis;
  float secondAxis;
  if (!angleAxisFullScale(firstAxis, secondAxis)) {
    return false;
  }
  int32_t code = roundToInteger(offset * OFFSET_CODES_FULL_SCALE / secondAxis);
  if (code < -128 || code > 127) {
    return false;
  }
  if (!writeRegister(TMAG5273_REG_MAG_OFFSET_CONFIG_2, (uint8_t)(int8_t)code)) {
    return false;
  }
  _offset2 = (uint8_t)(int8_t)code;
  return true;
}

float TMAG5273::getSecondAxisOffsetMillitesla() const {
  float firstAxis;
  float secondAxis;
  if (!angleAxisFullScale(firstAxis, secondAxis)) {
    return NAN;
  }
  return (int8_t)_offset2 * secondAxis / OFFSET_CODES_FULL_SCALE;
}

bool TMAG5273::setXThresholdMillitesla(float threshold) {
  int32_t code =
      roundToInteger(threshold * THRESHOLD_CODES_FULL_SCALE /
                     fullScaleMillitesla(getXYRange()));
  if (code < -128 || code > 127) {
    return false;
  }
  return writeField(TMAG5273_REG_X_THR_CONFIG, _xThreshold, 0xFF,
                    (uint8_t)(int8_t)code);
}

float TMAG5273::getXThresholdMillitesla() const {
  return (int8_t)_xThreshold * fullScaleMillitesla(getXYRange()) /
         THRESHOLD_CODES_FULL_SCALE;
}

bool TMAG5273::setYThresholdMillitesla(float threshold) {
  int32_t code =
      roundToInteger(threshold * THRESHOLD_CODES_FULL_SCALE /
                     fullScaleMillitesla(getXYRange()));
  if (code < -128 || code > 127) {
    return false;
  }
  return writeField(TMAG5273_REG_Y_THR_CONFIG, _yThreshold, 0xFF,
                    (uint8_t)(int8_t)code);
}

float TMAG5273::getYThresholdMillitesla() const {
  return (int8_t)_yThreshold * fullScaleMillitesla(getXYRange()) /
         THRESHOLD_CODES_FULL_SCALE;
}

bool TMAG5273::setZThresholdMillitesla(float threshold) {
  int32_t code = roundToInteger(threshold * THRESHOLD_CODES_FULL_SCALE /
                                fullScaleMillitesla(getZRange()));
  if (code < -128 || code > 127) {
    return false;
  }
  return writeField(TMAG5273_REG_Z_THR_CONFIG, _zThreshold, 0xFF,
                    (uint8_t)(int8_t)code);
}

float TMAG5273::getZThresholdMillitesla() const {
  return (int8_t)_zThreshold * fullScaleMillitesla(getZRange()) /
         THRESHOLD_CODES_FULL_SCALE;
}

bool TMAG5273::setThresholdMillitesla(TMAG5273_Axis axis, float threshold) {
  switch (axis) {
    case TMAG5273_AXIS_X:
      return setXThresholdMillitesla(threshold);
    case TMAG5273_AXIS_Y:
      return setYThresholdMillitesla(threshold);
    case TMAG5273_AXIS_Z:
      return setZThresholdMillitesla(threshold);
  }
  return false;
}

float TMAG5273::getThresholdMillitesla(TMAG5273_Axis axis) const {
  switch (axis) {
    case TMAG5273_AXIS_X:
      return getXThresholdMillitesla();
    case TMAG5273_AXIS_Y:
      return getYThresholdMillitesla();
    case TMAG5273_AXIS_Z:
      return getZThresholdMillitesla();
  }
  return NAN;
}

bool TMAG5273::setTemperatureThreshold(float celsius) {
  int32_t code = TEMPERATURE_THRESHOLD_CODE_MINUS_41C +
                 roundToInteger((celsius + 41.0f) /
                                TEMPERATURE_THRESHOLD_CODES_PER_CELSIUS);
  if (code < TEMPERATURE_THRESHOLD_CODE_MINUS_41C || code > 0x34) {
    return false;
  }
  return writeField(TMAG5273_REG_T_CONFIG, _tConfig, T_CONFIG_T_THR_CONFIG,
                    (uint8_t)(code << 1));
}

float TMAG5273::getTemperatureThreshold() const {
  uint8_t code = (uint8_t)(_tConfig >> 1);
  return code * TEMPERATURE_THRESHOLD_CODES_PER_CELSIUS - 249.0f;
}

bool TMAG5273::setThresholdMode(TMAG5273_ThresholdMode mode) {
  if (mode > TMAG5273_THRESHOLD_MODE_SYMMETRIC) {
    return false;
  }
  return writeField(TMAG5273_REG_DEVICE_CONFIG_2, _deviceConfig2,
                    DEVICE_CONFIG_2_THR_HYST, (uint8_t)(mode << 5));
}

TMAG5273_ThresholdMode TMAG5273::getThresholdMode() const {
  return (TMAG5273_ThresholdMode)((_deviceConfig2 & DEVICE_CONFIG_2_THR_HYST) >> 5);
}

bool TMAG5273::setThresholdDirection(TMAG5273_ThresholdDirection direction) {
  if (direction > TMAG5273_THRESHOLD_BELOW) {
    return false;
  }
  return writeField(TMAG5273_REG_SENSOR_CONFIG_2, _sensorConfig2,
                    SENSOR_CONFIG_2_MAG_THR_DIR, (uint8_t)(direction << 5));
}

TMAG5273_ThresholdDirection TMAG5273::getThresholdDirection() const {
  return (TMAG5273_ThresholdDirection)(
      (_sensorConfig2 & SENSOR_CONFIG_2_MAG_THR_DIR) >> 5);
}

bool TMAG5273::setThresholdCount(TMAG5273_ThresholdCount count) {
  if (count > TMAG5273_THRESHOLD_FOUR_CROSSINGS) {
    return false;
  }
  return writeField(TMAG5273_REG_SENSOR_CONFIG_2, _sensorConfig2,
                    SENSOR_CONFIG_2_THRX_COUNT, (uint8_t)(count << 6));
}

TMAG5273_ThresholdCount TMAG5273::getThresholdCount() const {
  return (TMAG5273_ThresholdCount)(
      (_sensorConfig2 & SENSOR_CONFIG_2_THRX_COUNT) >> 6);
}

bool TMAG5273::enableResultInterrupt() {
  return writeField(TMAG5273_REG_INT_CONFIG_1, _intConfig1,
                    INT_CONFIG_1_RSLT_INT, INT_CONFIG_1_RSLT_INT);
}

bool TMAG5273::disableResultInterrupt() {
  return writeField(TMAG5273_REG_INT_CONFIG_1, _intConfig1,
                    INT_CONFIG_1_RSLT_INT, 0x00);
}

bool TMAG5273::isResultInterruptEnabled() const {
  return (_intConfig1 & INT_CONFIG_1_RSLT_INT) != 0;
}

bool TMAG5273::enableThresholdInterrupt() {
  return writeField(TMAG5273_REG_INT_CONFIG_1, _intConfig1,
                    INT_CONFIG_1_THRSLD_INT, INT_CONFIG_1_THRSLD_INT);
}

bool TMAG5273::disableThresholdInterrupt() {
  return writeField(TMAG5273_REG_INT_CONFIG_1, _intConfig1,
                    INT_CONFIG_1_THRSLD_INT, 0x00);
}

bool TMAG5273::isThresholdInterruptEnabled() const {
  return (_intConfig1 & INT_CONFIG_1_THRSLD_INT) != 0;
}

bool TMAG5273::setInterruptMode(TMAG5273_InterruptMode mode) {
  if (mode > TMAG5273_INTERRUPT_ON_SCL_PIN_EXCEPT_I2C_BUSY) {
    return false;
  }
  return writeField(TMAG5273_REG_INT_CONFIG_1, _intConfig1,
                    INT_CONFIG_1_INT_MODE, (uint8_t)(mode << 2));
}

TMAG5273_InterruptMode TMAG5273::getInterruptMode() const {
  return (TMAG5273_InterruptMode)((_intConfig1 & INT_CONFIG_1_INT_MODE) >> 2);
}

bool TMAG5273::setInterruptLatched() {
  return writeField(TMAG5273_REG_INT_CONFIG_1, _intConfig1,
                    INT_CONFIG_1_INT_STATE, 0x00);
}

bool TMAG5273::setInterruptPulsed() {
  return writeField(TMAG5273_REG_INT_CONFIG_1, _intConfig1,
                    INT_CONFIG_1_INT_STATE, INT_CONFIG_1_INT_STATE);
}

bool TMAG5273::isInterruptLatched() const {
  return (_intConfig1 & INT_CONFIG_1_INT_STATE) == 0;
}

bool TMAG5273::enableInterruptPin() {
  return writeField(TMAG5273_REG_INT_CONFIG_1, _intConfig1,
                    INT_CONFIG_1_MASK_INTB, 0x00);
}

bool TMAG5273::disableInterruptPin() {
  return writeField(TMAG5273_REG_INT_CONFIG_1, _intConfig1,
                    INT_CONFIG_1_MASK_INTB, INT_CONFIG_1_MASK_INTB);
}

bool TMAG5273::isInterruptPinEnabled() const {
  return (_intConfig1 & INT_CONFIG_1_MASK_INTB) == 0;
}

bool TMAG5273::enableTemperature() {
  return writeField(TMAG5273_REG_T_CONFIG, _tConfig, T_CONFIG_T_CH_EN,
                    T_CONFIG_T_CH_EN);
}

bool TMAG5273::disableTemperature() {
  return writeField(TMAG5273_REG_T_CONFIG, _tConfig, T_CONFIG_T_CH_EN, 0x00);
}

bool TMAG5273::isTemperatureEnabled() const {
  return (_tConfig & T_CONFIG_T_CH_EN) != 0;
}

bool TMAG5273::triggerConversion() {
  // A write frame whose register address byte has the conversion trigger bit
  // set starts a conversion and writes no register (datasheet figure 6-7).
  _wire->beginTransmission(_address);
  _wire->write((uint8_t)(TMAG5273_CONVERSION_TRIGGER |
                         TMAG5273_REG_DEVICE_CONFIG_1));
  if (_wire->endTransmission() != 0) {
    return false;
  }
  _conversionStartMicros = micros();
  _conversionPending = true;
  return true;
}

bool TMAG5273::isDataReady() {
  return (readConversionStatus() & TMAG5273_RESULT_READY) != 0;
}

bool TMAG5273::waitForDataReady(uint32_t timeoutMilliseconds) {
  uint32_t startTime = millis();

  if (_conversionPending) {
    _conversionPending = false;
    uint32_t conversionMicroseconds = conversionTimeMicroseconds();
    while ((uint32_t)(micros() - _conversionStartMicros) <
           conversionMicroseconds) {
      if (millis() - startTime > timeoutMilliseconds) {
        return false;
      }
    }
  }

  while (millis() - startTime <= timeoutMilliseconds) {
    if (isDataReady()) {
      return true;
    }
    delay(1);
  }
  return false;
}

TMAG5273_Measurement TMAG5273::readMeasurement() {
  TMAG5273_Measurement measurement;
  bool success = getReadMode() == TMAG5273_READ_STANDARD
                     ? readStandardMeasurement(measurement)
                     : readPacketMeasurement(measurement);
  if (!success) {
    return measurement;
  }

  uint8_t axes = activeAxes();
  if (axes & AXIS_X) {
    measurement.xMillitesla = convertToMillitesla(measurement.xCode, getXYRange());
  }
  if (axes & AXIS_Y) {
    measurement.yMillitesla = convertToMillitesla(measurement.yCode, getXYRange());
  }
  if (axes & AXIS_Z) {
    measurement.zMillitesla = convertToMillitesla(measurement.zCode, getZRange());
  }
  if (isTemperatureEnabled()) {
    measurement.temperatureCelsius = convertToCelsius(measurement.temperatureCode);
  }
  measurement.valid = true;
  return measurement;
}

int16_t TMAG5273::readX() {
  if (getReadMode() != TMAG5273_READ_STANDARD) {
    return readMeasurement().xCode;
  }
  int16_t code = 0;
  readAxisCode(TMAG5273_REG_X_MSB_RESULT, code);
  return code;
}

int16_t TMAG5273::readY() {
  if (getReadMode() != TMAG5273_READ_STANDARD) {
    return readMeasurement().yCode;
  }
  int16_t code = 0;
  readAxisCode(TMAG5273_REG_Y_MSB_RESULT, code);
  return code;
}

int16_t TMAG5273::readZ() {
  if (getReadMode() != TMAG5273_READ_STANDARD) {
    return readMeasurement().zCode;
  }
  int16_t code = 0;
  readAxisCode(TMAG5273_REG_Z_MSB_RESULT, code);
  return code;
}

float TMAG5273::readXMillitesla() {
  if (getReadMode() != TMAG5273_READ_STANDARD) {
    return readMeasurement().xMillitesla;
  }
  int16_t code;
  if (!readAxisCode(TMAG5273_REG_X_MSB_RESULT, code)) {
    return NAN;
  }
  return convertToMillitesla(code, getXYRange());
}

float TMAG5273::readYMillitesla() {
  if (getReadMode() != TMAG5273_READ_STANDARD) {
    return readMeasurement().yMillitesla;
  }
  int16_t code;
  if (!readAxisCode(TMAG5273_REG_Y_MSB_RESULT, code)) {
    return NAN;
  }
  return convertToMillitesla(code, getXYRange());
}

float TMAG5273::readZMillitesla() {
  if (getReadMode() != TMAG5273_READ_STANDARD) {
    return readMeasurement().zMillitesla;
  }
  int16_t code;
  if (!readAxisCode(TMAG5273_REG_Z_MSB_RESULT, code)) {
    return NAN;
  }
  return convertToMillitesla(code, getZRange());
}

float TMAG5273::readMillitesla(TMAG5273_Axis axis) {
  switch (axis) {
    case TMAG5273_AXIS_X:
      return readXMillitesla();
    case TMAG5273_AXIS_Y:
      return readYMillitesla();
    case TMAG5273_AXIS_Z:
      return readZMillitesla();
  }
  return NAN;
}

float TMAG5273::readXMicrotesla() {
  return readXMillitesla() * 1000.0;
}

float TMAG5273::readYMicrotesla() {
  return readYMillitesla() * 1000.0;
}

float TMAG5273::readZMicrotesla() {
  return readZMillitesla() * 1000.0;
}

float TMAG5273::readXGauss() {
  return readXMillitesla() * 10.0;
}

float TMAG5273::readYGauss() {
  return readYMillitesla() * 10.0;
}

float TMAG5273::readZGauss() {
  return readZMillitesla() * 10.0;
}

float TMAG5273::readTemperature() {
  if (getReadMode() != TMAG5273_READ_STANDARD) {
    return readMeasurement().temperatureCelsius;
  }
  uint8_t data[2];
  if (!readRegisters(TMAG5273_REG_T_MSB_RESULT, data, 2)) {
    return NAN;
  }
  return convertToCelsius((int16_t)((uint16_t)data[0] << 8 | data[1]));
}

float TMAG5273::readAngle() {
  TMAG5273_Angle angle = getAngleCalculation();
  if (angle == TMAG5273_ANGLE_OFF ||
      getReadMode() != TMAG5273_READ_STANDARD) {
    return NAN;
  }
  uint8_t requiredAxes = angle == TMAG5273_ANGLE_XY
                             ? (AXIS_X | AXIS_Y)
                             : (angle == TMAG5273_ANGLE_YZ ? (AXIS_Y | AXIS_Z)
                                                           : (AXIS_X | AXIS_Z));
  if ((activeAxes() & requiredAxes) != requiredAxes) {
    return NAN;
  }
  uint8_t data[2];
  if (!readRegisters(TMAG5273_REG_ANGLE_RESULT_MSB, data, 2)) {
    return NAN;
  }
  // ANGLE_RESULT[12:4] is the integer part in degrees, ANGLE_RESULT[3:0] the
  // fraction in sixteenths (datasheet equation 14).
  uint16_t value = (uint16_t)(((uint16_t)data[0] << 8 | data[1]) & 0x1FFF);
  return (float)(value >> 4) + (float)(value & 0x0F) / 16.0;
}

uint8_t TMAG5273::readMagnitude() {
  return readRegister(TMAG5273_REG_MAGNITUDE_RESULT);
}

float TMAG5273::readMagnitudeMillitesla() {
  float firstAxis;
  float secondAxis;
  if (!angleAxisFullScale(firstAxis, secondAxis) ||
      getReadMode() != TMAG5273_READ_STANDARD) {
    return NAN;
  }
  uint8_t data;
  if (!readRegisters(TMAG5273_REG_MAGNITUDE_RESULT, &data, 1)) {
    return NAN;
  }
  // MAGNITUDE_RESULT holds the eight most significant bits of
  // sqrt(ch1^2 + ch2^2) (datasheet equation 15). The larger of the two axis
  // ranges is used because the register does not resolve them separately.
  float scale = firstAxis > secondAxis ? firstAxis : secondAxis;
  return data * scale / THRESHOLD_CODES_FULL_SCALE;
}

uint8_t TMAG5273::readConversionStatus() {
  if (getReadMode() != TMAG5273_READ_STANDARD) {
    return readMeasurement().status;
  }
  return readRegister(TMAG5273_REG_CONV_STATUS);
}

uint8_t TMAG5273::readDeviceStatus() {
  return readRegister(TMAG5273_REG_DEVICE_STATUS);
}

bool TMAG5273::hasDiagnosticFailure() {
  return (readConversionStatus() & TMAG5273_DIAGNOSTIC_FAIL) != 0;
}

bool TMAG5273::hasDeviceError() {
  return (readDeviceStatus() & 0x0F) != 0;
}

bool TMAG5273::clearDeviceErrors() {
  // OSC_ER, INT_ER, OTP_CRC_ER and VCC_UV_ER are cleared by writing 1.
  return writeRegister(TMAG5273_REG_DEVICE_STATUS, 0x0F);
}

uint8_t TMAG5273::readRegister(uint8_t registerAddress) {
  uint8_t value = 0;
  readRegisters(registerAddress, &value, 1);
  return value;
}

bool TMAG5273::readRegisters(uint8_t registerAddress, uint8_t *values,
                             size_t length) {
  if (!isCrcEnabled()) {
    return readBytes(registerAddress, values, length);
  }
  // With CRC enabled a standard read frame carries four register bytes and
  // the CRC byte calculated over them (datasheet figure 6-10).
  while (length > 0) {
    size_t chunk = length > 4 ? 4 : length;
    if (!readCrcBlock(registerAddress, values, chunk)) {
      return false;
    }
    registerAddress = (uint8_t)(registerAddress + chunk);
    values += chunk;
    length -= chunk;
  }
  return true;
}

bool TMAG5273::writeRegister(uint8_t registerAddress, uint8_t value) {
  _wire->beginTransmission(_address);
  _wire->write(registerAddress);
  _wire->write(value);
  return _wire->endTransmission() == 0;
}

bool TMAG5273::resetConfiguration() {
  // Reset values of datasheet table 8-1, with all three magnetic channels
  // enabled. Device configuration 1 comes first so that the following
  // transfers use plain read frames without CRC.
  const uint8_t defaults[][2] = {
      {TMAG5273_REG_DEVICE_CONFIG_1, 0x00},
      {TMAG5273_REG_DEVICE_CONFIG_2, 0x00},
      {TMAG5273_REG_SENSOR_CONFIG_1, 0x70},
      {TMAG5273_REG_SENSOR_CONFIG_2, 0x00},
      {TMAG5273_REG_X_THR_CONFIG, 0x00},
      {TMAG5273_REG_Y_THR_CONFIG, 0x00},
      {TMAG5273_REG_Z_THR_CONFIG, 0x00},
      {TMAG5273_REG_T_CONFIG, 0x00},
      {TMAG5273_REG_INT_CONFIG_1, 0x00},
      {TMAG5273_REG_MAG_GAIN_CONFIG, 0x00},
      {TMAG5273_REG_MAG_OFFSET_CONFIG_1, 0x00},
      {TMAG5273_REG_MAG_OFFSET_CONFIG_2, 0x00},
  };
  for (size_t i = 0; i < sizeof(defaults) / sizeof(defaults[0]); i++) {
    if (!writeRegister(defaults[i][0], defaults[i][1])) {
      return false;
    }
  }

  _deviceConfig1 = 0x00;
  _deviceConfig2 = 0x00;
  _sensorConfig1 = 0x70;
  _sensorConfig2 = 0x00;
  _xThreshold = 0x00;
  _yThreshold = 0x00;
  _zThreshold = 0x00;
  _tConfig = 0x00;
  _intConfig1 = 0x00;
  _magGain = 0x00;
  _offset1 = 0x00;
  _offset2 = 0x00;
  return true;
}

bool TMAG5273::writeField(uint8_t registerAddress, uint8_t &shadow,
                          uint8_t mask, uint8_t value) {
  uint8_t newValue = (uint8_t)((shadow & ~mask) | (value & mask));
  if (!writeRegister(registerAddress, newValue)) {
    return false;
  }
  shadow = newValue;
  return true;
}

bool TMAG5273::readBytes(uint8_t registerAddress, uint8_t *values,
                         size_t length) {
  _wire->beginTransmission(_address);
  _wire->write(registerAddress);
  if (_wire->endTransmission(false) != 0) {
    return false;
  }
  if (_wire->requestFrom(_address, (uint8_t)length) != length) {
    return false;
  }
  for (size_t i = 0; i < length; i++) {
    values[i] = (uint8_t)_wire->read();
  }
  return true;
}

bool TMAG5273::readCrcBlock(uint8_t registerAddress, uint8_t *values,
                            size_t length) {
  uint8_t block[5];
  if (!readBytes(registerAddress, block, sizeof(block))) {
    return false;
  }
  if (block[4] != crc8(block, 4)) {
    return false;
  }
  memcpy(values, block, length);
  return true;
}

bool TMAG5273::readPacket(uint8_t *values, size_t length) {
  // The 1-byte read commands consist of the address byte with the read bit
  // set, the data follows without a register address (datasheet figures
  // 6-11 to 6-14).
  if (_wire->requestFrom(_address, (uint8_t)length) != length) {
    return false;
  }
  for (size_t i = 0; i < length; i++) {
    values[i] = (uint8_t)_wire->read();
  }
  return true;
}

bool TMAG5273::readStandardMeasurement(TMAG5273_Measurement &measurement) {
  uint8_t data[9];  // T_MSB_RESULT to CONV_STATUS
  if (!readRegisters(TMAG5273_REG_T_MSB_RESULT, data, sizeof(data))) {
    return false;
  }
  measurement.temperatureCode = (int16_t)((uint16_t)data[0] << 8 | data[1]);
  measurement.xCode = (int16_t)((uint16_t)data[2] << 8 | data[3]);
  measurement.yCode = (int16_t)((uint16_t)data[4] << 8 | data[5]);
  measurement.zCode = (int16_t)((uint16_t)data[6] << 8 | data[7]);
  measurement.status = data[8];
  return true;
}

bool TMAG5273::readPacketMeasurement(TMAG5273_Measurement &measurement) {
  bool eightBit = getReadMode() == TMAG5273_READ_8BIT;
  size_t bytesPerChannel = eightBit ? 1 : 2;
  uint8_t axes = activeAxes();
  size_t dataBytes = channelCount() * bytesPerChannel;

  uint8_t buffer[1 + 2 * 4 + 1];
  size_t length = dataBytes + 1 + (isCrcEnabled() ? 1 : 0);
  if (length > sizeof(buffer) || !readPacket(buffer, length)) {
    return false;
  }

  if (isCrcEnabled()) {
    // In the 1-byte read modes the CRC covers the command byte and all data
    // bytes of the packet, including CONV_STATUS (datasheet section
    // 6.5.1.3.4).
    uint8_t crcData[1 + 2 * 4 + 1];
    crcData[0] = (uint8_t)(_address << 1 | 0x01);
    memcpy(&crcData[1], buffer, length - 1);
    if (buffer[length - 1] != crc8(crcData, length)) {
      return false;
    }
  }

  // Data order is T, X, Y, Z for the enabled channels.
  size_t index = 0;
  if (isTemperatureEnabled()) {
    measurement.temperatureCode = decodeCode(&buffer[index], eightBit);
    index += bytesPerChannel;
  }
  if (axes & AXIS_X) {
    measurement.xCode = decodeCode(&buffer[index], eightBit);
    index += bytesPerChannel;
  }
  if (axes & AXIS_Y) {
    measurement.yCode = decodeCode(&buffer[index], eightBit);
    index += bytesPerChannel;
  }
  if (axes & AXIS_Z) {
    measurement.zCode = decodeCode(&buffer[index], eightBit);
    index += bytesPerChannel;
  }
  measurement.status = buffer[index];
  return true;
}

uint8_t TMAG5273::activeAxes() const {
  switch (getMagneticChannels()) {
    case TMAG5273_CHANNEL_X:
      return AXIS_X;
    case TMAG5273_CHANNEL_Y:
      return AXIS_Y;
    case TMAG5273_CHANNEL_XY:
    case TMAG5273_CHANNEL_XYX:
    case TMAG5273_CHANNEL_YXY:
      return AXIS_X | AXIS_Y;
    case TMAG5273_CHANNEL_Z:
      return AXIS_Z;
    case TMAG5273_CHANNEL_ZX:
    case TMAG5273_CHANNEL_XZX:
      return AXIS_X | AXIS_Z;
    case TMAG5273_CHANNEL_YZ:
    case TMAG5273_CHANNEL_YZY:
      return AXIS_Y | AXIS_Z;
    case TMAG5273_CHANNEL_XYZ:
      return AXIS_X | AXIS_Y | AXIS_Z;
    default:
      return 0;
  }
}

uint8_t TMAG5273::channelCount() const {
  uint8_t axes = activeAxes();
  uint8_t count = (axes & AXIS_X ? 1 : 0) + (axes & AXIS_Y ? 1 : 0) +
                  (axes & AXIS_Z ? 1 : 0);
  if (isTemperatureEnabled()) {
    count++;
  }
  return count;
}

bool TMAG5273::readAxisCode(uint8_t msbRegister, int16_t &code) {
  uint8_t data[2];
  if (!readRegisters(msbRegister, data, 2)) {
    return false;
  }
  code = (int16_t)((uint16_t)data[0] << 8 | data[1]);
  return true;
}

float TMAG5273::fullScaleMillitesla(TMAG5273_Range range) const {
  if (_variant == TMAG5273_VARIANT_2) {
    return range == TMAG5273_RANGE_HIGH ? 266.0 : 133.0;
  }
  return range == TMAG5273_RANGE_HIGH ? 80.0 : 40.0;
}

float TMAG5273::convertToMillitesla(int16_t rawCode,
                                    TMAG5273_Range range) const {
  return rawCode / ADC_CODE_FULL_SCALE * fullScaleMillitesla(range);
}

float TMAG5273::convertToCelsius(int16_t rawCode) const {
  return TEMPERATURE_CELSIUS_AT_REFERENCE +
         ((float)rawCode - TEMPERATURE_ADC_CODE_AT_25C) /
             TEMPERATURE_ADC_CODES_PER_CELSIUS;
}

bool TMAG5273::angleAxisFullScale(float &firstAxis, float &secondAxis) const {
  switch (getAngleCalculation()) {
    case TMAG5273_ANGLE_XY:
      firstAxis = fullScaleMillitesla(getXYRange());
      secondAxis = fullScaleMillitesla(getXYRange());
      return true;
    case TMAG5273_ANGLE_YZ:
      firstAxis = fullScaleMillitesla(getXYRange());
      secondAxis = fullScaleMillitesla(getZRange());
      return true;
    case TMAG5273_ANGLE_XZ:
      firstAxis = fullScaleMillitesla(getXYRange());
      secondAxis = fullScaleMillitesla(getZRange());
      return true;
    default:
      return false;
  }
}

uint32_t TMAG5273::conversionTimeMicroseconds() const {
  // tmeasure is 50us for a single magnetic channel at 1x averaging and grows
  // by 25us per averaged sample (datasheet section 5.11), which gives
  // 25us * (1 + samples) with samples = channels * averaging factor.
  uint8_t channels = channelCount();
  uint32_t averagingFactor = 1UL << getAveraging();
  return 25UL * (1UL + (uint32_t)channels * averagingFactor);
}

void TMAG5273::wakeUp() {
  // The device does not acknowledge I2C traffic in sleep mode; a valid
  // address wakes it up. Wait for tstart_sleep before the next transaction
  // (datasheet section 6.4.2).
  _wire->beginTransmission(_address);
  _wire->endTransmission();
  delay(1);
}
