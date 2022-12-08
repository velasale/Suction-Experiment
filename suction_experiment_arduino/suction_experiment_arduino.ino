/*!
 * @file mprls_simpletest.ino
 *
 * A basic test of the sensor with default settings
 * 
 * Designed specifically to work with the MPRLS sensor from Adafruit
 * ----> https://www.adafruit.com/products/3965
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.  
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#include <Wire.h>
#include "Adafruit_MPRLS.h"

//You don't *need* a reset and EOC pin for most usees, so we set to -1 and don't connect
#define RESET_PIN -1
#define EOC_PIN   -1
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN)


void setup() {
  Serial.begin(115200)
  Serial.println("MPRLS Simple Test")
  if (! mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?")
  }
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

  // Read Sensor

  // Actuate Valve


}
