/*
  TMAG5273 - Read angle

  Prints the angle of a magnet rotating in the X-Y plane and the magnitude of
  the magnetic field. The TMAG5273 calculates the angle from two magnetic
  axes in hardware (CORDIC), so no trigonometry is needed in the sketch.

  Wiring (the TMAG5273 works from 1.7 V to 3.6 V, use a 3.3 V board):
    TMAG5273 VCC -> 3V3
    TMAG5273 GND -> GND
    TMAG5273 SCL -> SCL
    TMAG5273 SDA -> SDA

  This example code is in the public domain.
*/

#include <Wire.h>
#include <TMAG5273.h>

TMAG5273 sensor;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!sensor.begin()) {
    Serial.println("TMAG5273 not found, check the wiring and the I2C address.");
    while (true) {
      delay(1000);
    }
  }

  sensor.setMagneticChannels(TMAG5273_CHANNEL_XYZ);
  sensor.setAngleCalculation(TMAG5273_ANGLE_XY);
  sensor.setAveraging(TMAG5273_AVERAGE_32X);
  sensor.setOperatingMode(TMAG5273_MODE_CONTINUOUS);
}

void loop() {
  float angle = sensor.readAngle();

  if (isnan(angle)) {
    Serial.println("No angle result available.");
  } else {
    Serial.print("Angle: ");
    Serial.print(angle, 2);
    Serial.print(" deg  Magnitude: ");
    Serial.print(sensor.readMagnitudeMillitesla(), 2);
    Serial.println(" mT");
  }

  delay(100);
}
