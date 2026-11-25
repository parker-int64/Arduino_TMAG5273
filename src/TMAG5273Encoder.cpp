#include "TMAG5273Encoder.h"

#include <math.h>

static TMAG5273_MagneticChannel channelsForAngle(TMAG5273_Angle angle) {
  switch (angle) {
    case TMAG5273_ANGLE_XY:
      return TMAG5273_CHANNEL_XY;
    case TMAG5273_ANGLE_YZ:
      return TMAG5273_CHANNEL_YZ;
    case TMAG5273_ANGLE_XZ:
      return TMAG5273_CHANNEL_ZX;
    default:
      return TMAG5273_CHANNEL_NONE;
  }
}

TMAG5273Encoder::TMAG5273Encoder(TMAG5273 &sensor)
    : _sensor(&sensor),
      _position(0.0f),
      _lastAngle(0.0f),
      _magnetThreshold(1.0f),
      _hasReference(false),
      _positionSet(false) {}

bool TMAG5273Encoder::begin(TMAG5273_Angle anglePair,
                            TMAG5273_Averaging averaging) {
  TMAG5273_MagneticChannel channels = channelsForAngle(anglePair);
  if (channels == TMAG5273_CHANNEL_NONE) {
    return false;
  }
  if (!_sensor->setMagneticChannels(channels) ||
      !_sensor->setAngleCalculation(anglePair) ||
      !_sensor->setAveraging(averaging) ||
      !_sensor->setOperatingMode(TMAG5273_MODE_CONTINUOUS)) {
    return false;
  }
  _position = 0.0f;
  _lastAngle = 0.0f;
  _hasReference = false;
  _positionSet = false;
  return true;
}

float TMAG5273Encoder::readAngle() {
  float angle = _sensor->readAngle();
  if (isnan(angle)) {
    return NAN;
  }
  // The angle result is defined from 0 to 360 degrees; a value of exactly
  // 360 wraps to 0 so the value can be used in modular arithmetic.
  if (angle >= 360.0f) {
    angle -= 360.0f;
  }
  return angle;
}

float TMAG5273Encoder::readMagnitude() {
  return _sensor->readMagnitudeMillitesla();
}

float TMAG5273Encoder::readPosition() {
  float angle = readAngle();
  if (isnan(angle)) {
    return NAN;
  }

  if (!_hasReference) {
    _lastAngle = angle;
    if (!_positionSet) {
      _position = angle;
      _positionSet = true;
    }
    _hasReference = true;
    return _position;
  }

  // Unwrap the angle into a position that keeps counting beyond one turn.
  // The magnet must move less than 180 degrees between two calls.
  float delta = angle - _lastAngle;
  if (delta > 180.0f) {
    delta -= 360.0f;
  } else if (delta < -180.0f) {
    delta += 360.0f;
  }
  _position += delta;
  _lastAngle = angle;
  return _position;
}

float TMAG5273Encoder::getPosition() const {
  return _position;
}

int32_t TMAG5273Encoder::getTurns() const {
  return (int32_t)floor(_position / 360.0f);
}

void TMAG5273Encoder::resetPosition(float degrees) {
  _position = degrees;
  _positionSet = true;
  float angle = readAngle();
  if (isnan(angle)) {
    _hasReference = false;
    return;
  }
  _lastAngle = angle;
  _hasReference = true;
}

void TMAG5273Encoder::setMagnetThresholdMillitesla(float threshold) {
  _magnetThreshold = threshold > 0.0f ? threshold : 0.0f;
}

float TMAG5273Encoder::getMagnetThresholdMillitesla() const {
  return _magnetThreshold;
}

bool TMAG5273Encoder::isMagnetDetected() {
  float magnitude = readMagnitude();
  if (isnan(magnitude)) {
    return false;
  }
  return magnitude >= _magnetThreshold;
}
