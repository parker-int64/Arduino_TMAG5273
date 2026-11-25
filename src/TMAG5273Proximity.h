/*
  TMAG5273Proximity - Magnetic proximity switch built on the TMAG5273.

  The sensor compares one magnetic axis against a threshold. This class
  configures that comparison, keeps a hysteresis so the state does not
  chatter, and sets up the interrupt and wake-up and sleep mode that a
  battery powered door, window or lock sensor needs.
*/

#ifndef TMAG5273_PROXIMITY_H
#define TMAG5273_PROXIMITY_H

#include <Arduino.h>

#include "TMAG5273.h"

class TMAG5273Proximity {
 public:
  explicit TMAG5273Proximity(TMAG5273 &sensor);

  bool begin(TMAG5273_Axis axis = TMAG5273_AXIS_Z,
             float thresholdMillitesla = 2.0f);

  bool setAxis(TMAG5273_Axis axis);
  TMAG5273_Axis getAxis() const;

  bool setThresholdMillitesla(float threshold);
  float getThresholdMillitesla() const;

  bool setDirection(TMAG5273_ThresholdDirection direction);
  TMAG5273_ThresholdDirection getDirection() const;

  void setHysteresisMillitesla(float hysteresis);
  float getHysteresisMillitesla() const;

  float readFieldMillitesla();
  bool isNear();

  bool enableInterrupt();
  bool disableInterrupt();

  bool startLowPower(TMAG5273_SleepTime interval);
  bool stopLowPower();

 private:
  TMAG5273 *_sensor;
  TMAG5273_Axis _axis;
  TMAG5273_ThresholdDirection _direction;
  float _threshold;
  float _hysteresis;
  bool _near;
};

#endif
