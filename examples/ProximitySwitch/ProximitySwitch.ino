/*
  TMAG5273 - Proximity switch

  Uses the TMAG5273Proximity layer: one magnetic axis is compared against a
  threshold with a hysteresis, so the sketch sees a stable "magnet near"
  state, the way a door or window sensor reports open and closed.

  Wiring (the TMAG5273 works from 1.7 V to 3.6 V, use a 3.3 V board):
    TMAG5273 VCC -> 3V3
    TMAG5273 GND -> GND
    TMAG5273 SCL -> SCL
    TMAG5273 SDA -> SDA

  This example code is in the public domain.
*/

#include <Wire.h>
#include <TMAG5273.h>
#include <TMAG5273Proximity.h>

TMAG5273 sensor;
TMAG5273Proximity proximity(sensor);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!sensor.begin()) {
    Serial.println("TMAG5273 not found, check the wiring and the I2C address.");
    while (true) {
      delay(1000);
    }
  }

  // The magnet is sensed along the Z axis and the field is above 2 mT while
  // the door is closed. If the magnet produces the opposite sign, select
  // TMAG5273_THRESHOLD_BELOW instead.
  if (!proximity.begin(TMAG5273_AXIS_Z, 2.0f)) {
    Serial.println("Threshold does not fit the selected range.");
    while (true) {
      delay(1000);
    }
  }
  proximity.setHysteresisMillitesla(0.5f);

  Serial.print("Threshold: ");
  Serial.print(proximity.getThresholdMillitesla(), 2);
  Serial.println(" mT");
}

void loop() {
  static bool lastState = false;

  bool near = proximity.isNear();
  if (near != lastState) {
    lastState = near;
    Serial.print(near ? "near" : "far ");
    Serial.print("  field: ");
    Serial.print(proximity.readFieldMillitesla(), 2);
    Serial.println(" mT");
  }

  delay(50);
}
