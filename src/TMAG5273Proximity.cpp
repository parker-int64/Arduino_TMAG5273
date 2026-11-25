#include "TMAG5273Proximity.h"

#include <math.h>

static TMAG5273_MagneticChannel channelForAxis(TMAG5273_Axis axis) {
  switch (axis) {
    case TMAG5273_AXIS_X:
      return TMAG5273_CHANNEL_X;
    case TMAG5273_AXIS_Y:
      return TMAG5273_CHANNEL_Y;
    case TMAG5273_AXIS_Z:
      return TMAG5273_CHANNEL_Z;
  }
  return TMAG5273_CHANNEL_NONE;
}

TMAG5273Proximity::TMAG5273Proximity(TMAG5273 &sensor)
    : _sensor(&sensor),
      _axis(TMAG5273_AXIS_Z),
      _direction(TMAG5273_THRESHOLD_ABOVE),
      _threshold(2.0f),
      _hysteresis(0.5f),
      _near(false) {}

bool TMAG5273Proximity::begin(TMAG5273_Axis axis, float thresholdMillitesla) {
  // Reject a threshold that does not fit the selected range before any
  // register is changed.
  float previousThreshold = _threshold;
  if (!_sensor->setThresholdMillitesla(axis, thresholdMillitesla)) {
    return false;
  }
  _threshold = thresholdMillitesla;
  _near = false;
  if (!setAxis(axis)) {
    _threshold = previousThreshold;
    return false;
  }
  if (!_sensor->setThresholdMode(TMAG5273_THRESHOLD_MODE_ABSOLUTE)) {
    return false;
  }
  if (!setDirection(TMAG5273_THRESHOLD_ABOVE)) {
    return false;
  }
  return _sensor->setOperatingMode(TMAG5273_MODE_CONTINUOUS);
}

bool TMAG5273Proximity::setAxis(TMAG5273_Axis axis) {
  TMAG5273_MagneticChannel channel = channelForAxis(axis);
  if (channel == TMAG5273_CHANNEL_NONE) {
    return false;
  }
  if (!_sensor->setMagneticChannels(channel)) {
    return false;
  }
  _axis = axis;

  // Only the selected axis may take part in the threshold comparison,
  // otherwise an interrupt could still fire from the threshold of an axis
  // that was selected earlier.
  bool success = true;
  for (uint8_t index = 0; index < 3; index++) {
    TMAG5273_Axis current = (TMAG5273_Axis)index;
    float threshold = current == axis ? _threshold : 0.0f;
    if (!_sensor->setThresholdMillitesla(current, threshold)) {
      success = false;
    }
  }
  return success;
}

TMAG5273_Axis TMAG5273Proximity::getAxis() const {
  return _axis;
}

bool TMAG5273Proximity::setThresholdMillitesla(float threshold) {
  if (!_sensor->setThresholdMillitesla(_axis, threshold)) {
    return false;
  }
  _threshold = threshold;
  return true;
}

float TMAG5273Proximity::getThresholdMillitesla() const {
  return _threshold;
}

bool TMAG5273Proximity::setDirection(TMAG5273_ThresholdDirection direction) {
  if (!_sensor->setThresholdDirection(direction)) {
    return false;
  }
  _direction = direction;
  return true;
}

TMAG5273_ThresholdDirection TMAG5273Proximity::getDirection() const {
  return _direction;
}

void TMAG5273Proximity::setHysteresisMillitesla(float hysteresis) {
  _hysteresis = hysteresis >= 0.0f ? hysteresis : 0.0f;
}

float TMAG5273Proximity::getHysteresisMillitesla() const {
  return _hysteresis;
}

float TMAG5273Proximity::readFieldMillitesla() {
  return _sensor->readMillitesla(_axis);
}

bool TMAG5273Proximity::isNear() {
  float field = readFieldMillitesla();
  if (isnan(field)) {
    // No fresh measurement, for example while the device sleeps between two
    // wake-up intervals: keep the last state.
    return _near;
  }

  bool above = _direction == TMAG5273_THRESHOLD_ABOVE;
  float enter = above ? _threshold + _hysteresis : _threshold - _hysteresis;
  float leave = above ? _threshold - _hysteresis : _threshold + _hysteresis;

  if (_near) {
    if (above ? (field < leave) : (field > leave)) {
      _near = false;
    }
  } else {
    if (above ? (field > enter) : (field < enter)) {
      _near = true;
    }
  }
  return _near;
}

bool TMAG5273Proximity::enableInterrupt() {
  // The device drives the INT pin low when the threshold is crossed and
  // keeps it low until the microcontroller addresses it.
  if (!_sensor->setThresholdDirection(_direction)) {
    return false;
  }
  if (!_sensor->enableThresholdInterrupt()) {
    return false;
  }
  if (!_sensor->setInterruptMode(TMAG5273_INTERRUPT_ON_INT_PIN)) {
    return false;
  }
  if (!_sensor->setInterruptLatched()) {
    return false;
  }
  return _sensor->enableInterruptPin();
}

bool TMAG5273Proximity::disableInterrupt() {
  if (!_sensor->disableThresholdInterrupt()) {
    return false;
  }
  return _sensor->setInterruptMode(TMAG5273_INTERRUPT_DISABLED);
}

bool TMAG5273Proximity::startLowPower(TMAG5273_SleepTime interval) {
  // In the wake-up and sleep mode the device measures once per interval and
  // stays in standby after a threshold event, so the interrupt stays valid
  // until the microcontroller handles it.
  if (!_sensor->setSleepTime(interval)) {
    return false;
  }
  return _sensor->setOperatingMode(TMAG5273_MODE_WAKE_UP_SLEEP);
}

bool TMAG5273Proximity::stopLowPower() {
  return _sensor->setOperatingMode(TMAG5273_MODE_STANDBY);
}
