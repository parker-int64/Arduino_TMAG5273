/*
  TMAG5273 - Encoder position

  Uses the TMAG5273Encoder layer to turn the sensor into a multi-turn
  magnetic encoder: the sketch reports the accumulated position, the number
  of full turns, the mechanical angle and whether a magnet is in range.
  See the ReadAngle example for the raw angle registers.

  Wiring (the TMAG5273 works from 1.7 V to 3.6 V, use a 3.3 V board):
    TMAG5273 VCC -> 3V3
    TMAG5273 GND -> GND
    TMAG5273 SCL -> SCL
    TMAG5273 SDA -> SDA

  This example code is in the public domain.
*/

#include <Wire.h>
#include <TMAG5273.h>
#include <TMAG5273Encoder.h>

TMAG5273 sensor;
TMAG5273Encoder encoder(sensor);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!sensor.begin()) {
    Serial.println("TMAG5273 not found, check the wiring and the I2C address.");
    while (true) {
      delay(1000);
    }
  }

  // The low noise mode trades active current for a lower noise floor, which
  // is worth it for an encoder.
  sensor.enableLowNoiseMode();

  if (!encoder.begin(TMAG5273_ANGLE_XY)) {
    Serial.println("Encoder configuration failed.");
    while (true) {
      delay(1000);
    }
  }
}

void loop() {
  float position = encoder.readPosition();

  if (isnan(position)) {
    Serial.println("No angle result, is the magnet in place?");
  } else {
    Serial.print("Position: ");
    Serial.print(position, 2);
    Serial.print(" deg, turns: ");
    Serial.print(encoder.getTurns());
    Serial.print(", angle: ");
    Serial.print(encoder.readAngle(), 2);
    Serial.print(" deg, magnet: ");
    Serial.println(encoder.isMagnetDetected() ? "yes" : "no");
  }

  delay(50);
}
