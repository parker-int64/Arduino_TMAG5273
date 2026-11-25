/*
  TMAG5273 - Low power proximity switch

  The sensor wakes up once per interval, measures the selected axis and only
  leaves the wake-up and sleep mode when the threshold is crossed. That makes
  the average supply current a small fraction of the continuous mode, which
  is what a battery powered lock, door or window sensor needs.

  The INT pin goes low on the event and stays low until the microcontroller
  reads the sensor, so it can wake the microcontroller as well.

  Wiring (the TMAG5273 works from 1.7 V to 3.6 V, use a 3.3 V board):
    TMAG5273 VCC -> 3V3
    TMAG5273 GND -> GND
    TMAG5273 SCL -> SCL
    TMAG5273 SDA -> SDA
    TMAG5273 INT -> D2

  This example code is in the public domain.
*/

#include <Wire.h>
#include <TMAG5273.h>
#include <TMAG5273Proximity.h>

const uint8_t interruptPin = 2;
const TMAG5273_SleepTime measureInterval = TMAG5273_SLEEP_TIME_100MS;

TMAG5273 sensor;
TMAG5273Proximity proximity(sensor);
volatile bool eventFlag = false;

void onInterrupt() {
  eventFlag = true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!sensor.begin()) {
    Serial.println("TMAG5273 not found, check the wiring and the I2C address.");
    while (true) {
      delay(1000);
    }
  }

  if (!proximity.begin(TMAG5273_AXIS_Z, 2.0f)) {
    Serial.println("Threshold does not fit the selected range.");
    while (true) {
      delay(1000);
    }
  }

  proximity.enableInterrupt();
  proximity.startLowPower(measureInterval);

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), onInterrupt, FALLING);

  Serial.println("Armed, measuring every 100 ms.");
}

void loop() {
  if (!eventFlag) {
    return;
  }
  eventFlag = false;

  // After a threshold event the device stays in standby, so the field can be
  // read directly. The read also releases the latched interrupt.
  bool near = proximity.isNear();
  Serial.print(near ? "Magnet near" : "Magnet far ");
  Serial.print(proximity.readFieldMillitesla(), 2);
  Serial.println(" mT");

  // Arm the low power measurement again.
  proximity.startLowPower(measureInterval);
}
