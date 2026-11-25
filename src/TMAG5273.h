/*
  TMAG5273 - Arduino library for the Texas Instruments TMAG5273 low-power
  linear 3D Hall-effect sensor with I2C interface.

  Register addresses, bit fields, timing and conversion formulas used by this
  library are taken from the TMAG5273 datasheet, SLYS045C
  (revised April 2026).
*/

#ifndef TMAG5273_H
#define TMAG5273_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

/** Default 7-bit I2C address (TMAG5273A1, TMAG5273A2). */
#define TMAG5273_DEFAULT_ADDRESS 0x35

/** Manufacturer ID reported by the device ("TI"). */
#define TMAG5273_MANUFACTURER_ID 0x5449

/**
  Conversion trigger bit of the register address byte. Writing a register
  address with this bit set starts a new conversion (datasheet section
  6.5.1.3.1).
*/
#define TMAG5273_CONVERSION_TRIGGER 0x80

/** Register map (datasheet table 8-1). */
enum TMAG5273_Register : uint8_t {
  TMAG5273_REG_DEVICE_CONFIG_1 = 0x00,
  TMAG5273_REG_DEVICE_CONFIG_2 = 0x01,
  TMAG5273_REG_SENSOR_CONFIG_1 = 0x02,
  TMAG5273_REG_SENSOR_CONFIG_2 = 0x03,
  TMAG5273_REG_X_THR_CONFIG = 0x04,
  TMAG5273_REG_Y_THR_CONFIG = 0x05,
  TMAG5273_REG_Z_THR_CONFIG = 0x06,
  TMAG5273_REG_T_CONFIG = 0x07,
  TMAG5273_REG_INT_CONFIG_1 = 0x08,
  TMAG5273_REG_MAG_GAIN_CONFIG = 0x09,
  TMAG5273_REG_MAG_OFFSET_CONFIG_1 = 0x0A,
  TMAG5273_REG_MAG_OFFSET_CONFIG_2 = 0x0B,
  TMAG5273_REG_I2C_ADDRESS = 0x0C,
  TMAG5273_REG_DEVICE_ID = 0x0D,
  TMAG5273_REG_MANUFACTURER_ID_LSB = 0x0E,
  TMAG5273_REG_MANUFACTURER_ID_MSB = 0x0F,
  TMAG5273_REG_T_MSB_RESULT = 0x10,
  TMAG5273_REG_T_LSB_RESULT = 0x11,
  TMAG5273_REG_X_MSB_RESULT = 0x12,
  TMAG5273_REG_X_LSB_RESULT = 0x13,
  TMAG5273_REG_Y_MSB_RESULT = 0x14,
  TMAG5273_REG_Y_LSB_RESULT = 0x15,
  TMAG5273_REG_Z_MSB_RESULT = 0x16,
  TMAG5273_REG_Z_LSB_RESULT = 0x17,
  TMAG5273_REG_CONV_STATUS = 0x18,
  TMAG5273_REG_ANGLE_RESULT_MSB = 0x19,
  TMAG5273_REG_ANGLE_RESULT_LSB = 0x1A,
  TMAG5273_REG_MAGNITUDE_RESULT = 0x1B,
  TMAG5273_REG_DEVICE_STATUS = 0x1C
};

/** Bits of the CONV_STATUS register (datasheet table 8-27). */
enum TMAG5273_ConversionStatus : uint8_t {
  TMAG5273_RESULT_READY = 0x01,
  TMAG5273_DIAGNOSTIC_FAIL = 0x02,
  TMAG5273_POWER_ON_RESET = 0x10
};

/** Bits of the DEVICE_STATUS register (datasheet table 8-31). */
enum TMAG5273_DeviceStatus : uint8_t {
  TMAG5273_VCC_UNDERVOLTAGE = 0x01,
  TMAG5273_OTP_CRC_ERROR = 0x02,
  TMAG5273_INT_PIN_ERROR = 0x04,
  TMAG5273_OSCILLATOR_ERROR = 0x08,
  TMAG5273_INT_PIN_LEVEL = 0x10
};

/** Sensitivity option of the orderable part, read from DEVICE_ID[1:0]. */
enum TMAG5273_Variant : uint8_t {
  TMAG5273_VARIANT_UNKNOWN = 0,
  TMAG5273_VARIANT_1 = 1, /**< TMAG5273x1: +/-40 mT, +/-80 mT */
  TMAG5273_VARIANT_2 = 2  /**< TMAG5273x2: +/-133 mT, +/-266 mT */
};

/** Operating mode, DEVICE_CONFIG_2[1:0] (datasheet table 8-4). */
enum TMAG5273_OperatingMode : uint8_t {
  TMAG5273_MODE_STANDBY = 0x00,       /**< Starts a conversion on trigger */
  TMAG5273_MODE_SLEEP = 0x01,         /**< Lowest power, results not retained */
  TMAG5273_MODE_CONTINUOUS = 0x02,    /**< Measures continuously */
  TMAG5273_MODE_WAKE_UP_SLEEP = 0x03  /**< Measures at every SLEEPTIME interval */
};

/** Magnet temperature compensation, DEVICE_CONFIG_1[6:5]. */
enum TMAG5273_TemperatureCompensation : uint8_t {
  TMAG5273_TEMPCO_NONE = 0x00,   /**< 0% */
  TMAG5273_TEMPCO_NDFEB = 0x01,  /**< 0.12%/degC (NdFeB magnets) */
  TMAG5273_TEMPCO_CERAMIC = 0x03 /**< 0.2%/degC (ceramic magnets) */
};

/** Averaging of the conversion result, DEVICE_CONFIG_1[4:2]. */
enum TMAG5273_Averaging : uint8_t {
  TMAG5273_AVERAGE_1X = 0x00,
  TMAG5273_AVERAGE_2X = 0x01,
  TMAG5273_AVERAGE_4X = 0x02,
  TMAG5273_AVERAGE_8X = 0x03,
  TMAG5273_AVERAGE_16X = 0x04,
  TMAG5273_AVERAGE_32X = 0x05
};

/** I2C read frame, DEVICE_CONFIG_1[1:0] (datasheet section 6.5.1.3). */
enum TMAG5273_ReadMode : uint8_t {
  TMAG5273_READ_STANDARD = 0x00, /**< Standard read of register data */
  TMAG5273_READ_16BIT = 0x01,    /**< One byte command, 16-bit results */
  TMAG5273_READ_8BIT = 0x02      /**< One byte command, 8-bit results */
};

/** Magnetic field range, SENSOR_CONFIG_2 X_Y_RANGE and Z_RANGE bits. */
enum TMAG5273_Range : uint8_t {
  TMAG5273_RANGE_LOW = 0x00, /**< +/-40 mT (x1), +/-133 mT (x2) */
  TMAG5273_RANGE_HIGH = 0x01 /**< +/-80 mT (x1), +/-266 mT (x2) */
};

/** Magnetic channels that are converted, SENSOR_CONFIG_1[7:4]. */
enum TMAG5273_MagneticChannel : uint8_t {
  TMAG5273_CHANNEL_NONE = 0x00,
  TMAG5273_CHANNEL_X = 0x01,
  TMAG5273_CHANNEL_Y = 0x02,
  TMAG5273_CHANNEL_XY = 0x03,
  TMAG5273_CHANNEL_Z = 0x04,
  TMAG5273_CHANNEL_ZX = 0x05,
  TMAG5273_CHANNEL_YZ = 0x06,
  TMAG5273_CHANNEL_XYZ = 0x07,
  TMAG5273_CHANNEL_XYX = 0x08, /**< Pseudo-simultaneous X, Y sampling */
  TMAG5273_CHANNEL_YXY = 0x09, /**< Pseudo-simultaneous Y, X sampling */
  TMAG5273_CHANNEL_YZY = 0x0A, /**< Pseudo-simultaneous Y, Z sampling */
  TMAG5273_CHANNEL_XZX = 0x0B  /**< Pseudo-simultaneous X, Z sampling */
};

/** Magnetic axis of the device. */
enum TMAG5273_Axis : uint8_t {
  TMAG5273_AXIS_X = 0x00,
  TMAG5273_AXIS_Y = 0x01,
  TMAG5273_AXIS_Z = 0x02
};

/** Low power interval of the wake-up and sleep mode, SENSOR_CONFIG_1[3:0]. */
enum TMAG5273_SleepTime : uint8_t {
  TMAG5273_SLEEP_TIME_1MS = 0x00,
  TMAG5273_SLEEP_TIME_5MS = 0x01,
  TMAG5273_SLEEP_TIME_10MS = 0x02,
  TMAG5273_SLEEP_TIME_15MS = 0x03,
  TMAG5273_SLEEP_TIME_20MS = 0x04,
  TMAG5273_SLEEP_TIME_30MS = 0x05,
  TMAG5273_SLEEP_TIME_50MS = 0x06,
  TMAG5273_SLEEP_TIME_100MS = 0x07,
  TMAG5273_SLEEP_TIME_500MS = 0x08,
  TMAG5273_SLEEP_TIME_1000MS = 0x09,
  TMAG5273_SLEEP_TIME_2000MS = 0x0A,
  TMAG5273_SLEEP_TIME_5000MS = 0x0B,
  TMAG5273_SLEEP_TIME_20000MS = 0x0C
};

/** Axis pair of the angle calculation, SENSOR_CONFIG_2[3:2]. */
enum TMAG5273_Angle : uint8_t {
  TMAG5273_ANGLE_OFF = 0x00, /**< No angle, gain and offset correction */
  TMAG5273_ANGLE_XY = 0x01,  /**< X first, Y second */
  TMAG5273_ANGLE_YZ = 0x02,  /**< Y first, Z second */
  TMAG5273_ANGLE_XZ = 0x03   /**< X first, Z second */
};

/** Conversion trigger source, DEVICE_CONFIG_2 TRIGGER_MODE. */
enum TMAG5273_TriggerMode : uint8_t {
  TMAG5273_TRIGGER_I2C_COMMAND = 0x00, /**< Trigger bit of a write frame */
  TMAG5273_TRIGGER_INT_PIN = 0x01      /**< Falling edge on the INT pin */
};

/** Axis whose gain is corrected, SENSOR_CONFIG_2 MAG_GAIN_CH. */
enum TMAG5273_GainChannel : uint8_t {
  TMAG5273_GAIN_CHANNEL_FIRST = 0x00,
  TMAG5273_GAIN_CHANNEL_SECOND = 0x01
};

/** Threshold shape, DEVICE_CONFIG_2[7:5]. */
enum TMAG5273_ThresholdMode : uint8_t {
  TMAG5273_THRESHOLD_MODE_ABSOLUTE = 0x00,  /**< Signed threshold per axis */
  TMAG5273_THRESHOLD_MODE_SYMMETRIC = 0x01  /**< North and south threshold */
};

/** Threshold crossing direction, SENSOR_CONFIG_2 MAG_THR_DIR. */
enum TMAG5273_ThresholdDirection : uint8_t {
  TMAG5273_THRESHOLD_ABOVE = 0x00,
  TMAG5273_THRESHOLD_BELOW = 0x01
};

/** Crossings needed for an interrupt, SENSOR_CONFIG_2 THRX_COUNT. */
enum TMAG5273_ThresholdCount : uint8_t {
  TMAG5273_THRESHOLD_ONE_CROSSING = 0x00,
  TMAG5273_THRESHOLD_FOUR_CROSSINGS = 0x01
};

/** Interrupt output, INT_CONFIG_1[4:2]. */
enum TMAG5273_InterruptMode : uint8_t {
  TMAG5273_INTERRUPT_DISABLED = 0x00,
  TMAG5273_INTERRUPT_ON_INT_PIN = 0x01,
  TMAG5273_INTERRUPT_ON_INT_PIN_EXCEPT_I2C_BUSY = 0x02,
  TMAG5273_INTERRUPT_ON_SCL_PIN = 0x03,
  TMAG5273_INTERRUPT_ON_SCL_PIN_EXCEPT_I2C_BUSY = 0x04
};

/** Result of one conversion of all enabled channels. */
struct TMAG5273_Measurement {
  int16_t xCode = 0;
  int16_t yCode = 0;
  int16_t zCode = 0;
  int16_t temperatureCode = 0;
  float xMillitesla = NAN;
  float yMillitesla = NAN;
  float zMillitesla = NAN;
  float temperatureCelsius = NAN;
  uint8_t status = 0;
  bool valid = false;
};

class TMAG5273 {
 public:
  TMAG5273();

  bool begin(uint8_t address = TMAG5273_DEFAULT_ADDRESS,
             TwoWire &wire = Wire);
  bool end();

  bool isConnected();
  uint8_t getAddress() const;
  bool setAddress(uint8_t address);
  TMAG5273_Variant getVariant() const;

  uint16_t readManufacturerId();
  uint8_t readDeviceId();

  bool setOperatingMode(TMAG5273_OperatingMode mode);
  TMAG5273_OperatingMode getOperatingMode() const;

  bool enableLowNoiseMode();
  bool disableLowNoiseMode();
  bool isLowNoiseModeEnabled() const;

  bool enableGlitchFilter();
  bool disableGlitchFilter();
  bool isGlitchFilterEnabled() const;

  bool setTriggerMode(TMAG5273_TriggerMode mode);
  TMAG5273_TriggerMode getTriggerMode() const;

  bool setMagneticChannels(TMAG5273_MagneticChannel channels);
  TMAG5273_MagneticChannel getMagneticChannels() const;

  bool setXYRange(TMAG5273_Range range);
  TMAG5273_Range getXYRange() const;
  float getXYRangeMillitesla() const;

  bool setZRange(TMAG5273_Range range);
  TMAG5273_Range getZRange() const;
  float getZRangeMillitesla() const;

  bool setAveraging(TMAG5273_Averaging averaging);
  TMAG5273_Averaging getAveraging() const;

  bool setTemperatureCompensation(
      TMAG5273_TemperatureCompensation compensation);
  TMAG5273_TemperatureCompensation getTemperatureCompensation() const;

  bool setSleepTime(TMAG5273_SleepTime sleepTime);
  TMAG5273_SleepTime getSleepTime() const;

  bool setReadMode(TMAG5273_ReadMode readMode);
  TMAG5273_ReadMode getReadMode() const;

  bool enableCrc();
  bool disableCrc();
  bool isCrcEnabled() const;

  bool setAngleCalculation(TMAG5273_Angle angle);
  TMAG5273_Angle getAngleCalculation() const;

  bool setGainValue(uint8_t gain);
  uint8_t getGainValue() const;
  bool setGainChannel(TMAG5273_GainChannel channel);
  TMAG5273_GainChannel getGainChannel() const;

  bool setFirstAxisOffsetMillitesla(float offset);
  float getFirstAxisOffsetMillitesla() const;
  bool setSecondAxisOffsetMillitesla(float offset);
  float getSecondAxisOffsetMillitesla() const;

  bool setXThresholdMillitesla(float threshold);
  float getXThresholdMillitesla() const;
  bool setYThresholdMillitesla(float threshold);
  float getYThresholdMillitesla() const;
  bool setZThresholdMillitesla(float threshold);
  float getZThresholdMillitesla() const;
  bool setThresholdMillitesla(TMAG5273_Axis axis, float threshold);
  float getThresholdMillitesla(TMAG5273_Axis axis) const;
  bool setTemperatureThreshold(float celsius);
  float getTemperatureThreshold() const;

  bool setThresholdMode(TMAG5273_ThresholdMode mode);
  TMAG5273_ThresholdMode getThresholdMode() const;
  bool setThresholdDirection(TMAG5273_ThresholdDirection direction);
  TMAG5273_ThresholdDirection getThresholdDirection() const;
  bool setThresholdCount(TMAG5273_ThresholdCount count);
  TMAG5273_ThresholdCount getThresholdCount() const;

  bool enableResultInterrupt();
  bool disableResultInterrupt();
  bool isResultInterruptEnabled() const;
  bool enableThresholdInterrupt();
  bool disableThresholdInterrupt();
  bool isThresholdInterruptEnabled() const;
  bool setInterruptMode(TMAG5273_InterruptMode mode);
  TMAG5273_InterruptMode getInterruptMode() const;
  bool setInterruptLatched();
  bool setInterruptPulsed();
  bool isInterruptLatched() const;
  bool enableInterruptPin();
  bool disableInterruptPin();
  bool isInterruptPinEnabled() const;

  bool enableTemperature();
  bool disableTemperature();
  bool isTemperatureEnabled() const;

  bool triggerConversion();
  bool isDataReady();
  bool waitForDataReady(uint32_t timeoutMilliseconds = 100);

  TMAG5273_Measurement readMeasurement();

  int16_t readX();
  int16_t readY();
  int16_t readZ();
  float readXMillitesla();
  float readYMillitesla();
  float readZMillitesla();
  float readMillitesla(TMAG5273_Axis axis);
  float readXMicrotesla();
  float readYMicrotesla();
  float readZMicrotesla();
  float readXGauss();
  float readYGauss();
  float readZGauss();
  float readTemperature();

  float readAngle();
  uint8_t readMagnitude();
  float readMagnitudeMillitesla();

  uint8_t readConversionStatus();
  uint8_t readDeviceStatus();
  bool hasDiagnosticFailure();
  bool hasDeviceError();
  bool clearDeviceErrors();

  uint8_t readRegister(uint8_t registerAddress);
  bool readRegisters(uint8_t registerAddress, uint8_t *values, size_t length);
  bool writeRegister(uint8_t registerAddress, uint8_t value);

 private:
  TwoWire *_wire;
  uint8_t _address;
  TMAG5273_Variant _variant;

  // Shadow copies of the configuration registers. Reads are not used to
  // update them because a read returns conversion data in the 1-byte read
  // modes.
  uint8_t _deviceConfig1;
  uint8_t _deviceConfig2;
  uint8_t _sensorConfig1;
  uint8_t _sensorConfig2;
  uint8_t _xThreshold;
  uint8_t _yThreshold;
  uint8_t _zThreshold;
  uint8_t _tConfig;
  uint8_t _intConfig1;
  uint8_t _magGain;
  uint8_t _offset1;
  uint8_t _offset2;

  uint32_t _conversionStartMicros;
  bool _conversionPending;

  bool resetConfiguration();
  bool writeField(uint8_t registerAddress, uint8_t &shadow, uint8_t mask,
                  uint8_t value);
  bool readBytes(uint8_t registerAddress, uint8_t *values, size_t length);
  bool readCrcBlock(uint8_t registerAddress, uint8_t *values, size_t length);
  bool readPacket(uint8_t *values, size_t length);
  bool readStandardMeasurement(TMAG5273_Measurement &measurement);
  bool readPacketMeasurement(TMAG5273_Measurement &measurement);
  uint8_t activeAxes() const;
  uint8_t channelCount() const;
  bool readAxisCode(uint8_t msbRegister, int16_t &code);
  float fullScaleMillitesla(TMAG5273_Range range) const;
  float convertToMillitesla(int16_t rawCode, TMAG5273_Range range) const;
  float convertToCelsius(int16_t rawCode) const;
  bool angleAxisFullScale(float &firstAxis, float &secondAxis) const;
  uint32_t conversionTimeMicroseconds() const;
  void wakeUp();
};

#endif
