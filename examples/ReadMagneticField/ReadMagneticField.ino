/*
  TMAG5273 - Read magnetic field

  Prints the X, Y and Z magnetic field in millitesla and the temperature of
  the die in degrees Celsius. The sensor is left in continuous measurement
  mode, so the result registers are updated all the time.

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

  sensor.enableTemperature();
  sensor.setAveraging(TMAG5273_AVERAGE_32X);
  sensor.setOperatingMode(TMAG5273_MODE_CONTINUOUS);

  Serial.print("TMAG5273 with ");
  if (sensor.getVariant() == TMAG5273_VARIANT_2) {
    Serial.print("+/-133 mT and +/-266 mT");
  } else {
    Serial.print("+/-40 mT and +/-80 mT");
  }
  Serial.print(" range, measuring +/-");
  Serial.print(sensor.getXYRangeMillitesla(), 0);
  Serial.println(" mT");
}

void loop() {
  Serial.print("X: ");
  Serial.print(sensor.readXMillitesla(), 2);
  Serial.print(" mT  Y: ");
  Serial.print(sensor.readYMillitesla(), 2);
  Serial.print(" mT  Z: ");
  Serial.print(sensor.readZMillitesla(), 2);
  Serial.print(" mT  T: ");
  Serial.print(sensor.readTemperature(), 2);
  Serial.println(" C");

  delay(100);
}
