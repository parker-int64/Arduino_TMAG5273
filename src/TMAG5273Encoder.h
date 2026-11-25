/*
  TMAG5273Encoder - Magnetic rotary encoder built on the TMAG5273 angle
  engine (CORDIC).

  The sensor calculates the angle of a magnet that rotates in the plane of
  two magnetic axes. This class configures that measurement and adds the
  bookkeeping an application needs: a position that keeps counting across
  full turns, a magnet presence check and the sensor diagnostics.
*/

#ifndef TMAG5273_ENCODER_H
#define TMAG5273_ENCODER_H

#include <Arduino.h>

#include "TMAG5273.h"

class TMAG5273Encoder {
 public:
  explicit TMAG5273Encoder(TMAG5273 &sensor);

  bool begin(TMAG5273_Angle anglePair = TMAG5273_ANGLE_XY,
             TMAG5273_Averaging averaging = TMAG5273_AVERAGE_32X);

  float readAngle();
  float readMagnitude();

  float readPosition();
  float getPosition() const;
  int32_t getTurns() const;
  void resetPosition(float degrees = 0.0f);

  void setMagnetThresholdMillitesla(float threshold);
  float getMagnetThresholdMillitesla() const;
  bool isMagnetDetected();

 private:
  TMAG5273 *_sensor;
  float _position;
  float _lastAngle;
  float _magnetThreshold;
  bool _hasReference;
  bool _positionSet;
};

#endif
