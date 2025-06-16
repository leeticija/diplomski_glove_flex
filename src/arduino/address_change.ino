#include <Wire.h>
#include "SparkFun_Displacement_Sensor_Arduino_Library.h"

ADS myFlexSensor;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Starting I2C Address Change Example");

  Wire.begin();

  if (myFlexSensor.begin(0x13) == false) {
    Serial.println("Sensor not detected at 0x13. Check wiring.");
    while (1);
  }
  Serial.println("Sensor found on 0x13! Changing address...");
  uint8_t newAddress = 0x12;

  if (myFlexSensor.setAddress(newAddress)) {
    Serial.print("Successfully changed I2C address to 0x");
    Serial.println(newAddress, HEX);
  } else {
    Serial.println("Failed to change address.");
  }

  Serial.println("Now power cycle the sensor to apply the new address.");
}

void loop() {}
