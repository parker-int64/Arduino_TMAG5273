/*
  TMAG5273 - Triggered read

  Takes one measurement at a time. The sensor stays in standby mode between
  measurements, which draws far less current than continuous mode.

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

  // X, Y, Z and temperature are converted on every trigger. Averaging 32
  // samples per channel gives the lowest noise (datasheet table 6-4).
  sensor.setMagneticChannels(TMAG5273_CHANNEL_XYZ);
  sensor.enableTemperature();
  sensor.setAveraging(TMAG5273_AVERAGE_32X);
  // begin() leaves the sensor in standby mode, which converts on trigger only.
}

void loop() {
  sensor.triggerConversion();

  if (!sensor.waitForDataReady(100)) {
    Serial.println("Conversion did not finish in time.");
    return;
  }

  Serial.print("X: ");
  Serial.print(sensor.readXMillitesla(), 2);
  Serial.print(" mT  Y: ");
  Serial.print(sensor.readYMillitesla(), 2);
  Serial.print(" mT  Z: ");
  Serial.print(sensor.readZMillitesla(), 2);
  Serial.print(" mT  T: ");
  Serial.print(sensor.readTemperature(), 2);
  Serial.print(" C  raw X: ");
  Serial.println(sensor.readX());

  delay(500);
}
