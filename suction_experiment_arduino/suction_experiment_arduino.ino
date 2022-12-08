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
#include <ros.h>
#include <std_msgs/Bool.h>
#include <Wire.h>
#include "Adafruit_MPRLS.h"

//You don't *need* a reset and EOC pin for most usees, so we set to -1 and don't connect
#define RESET_PIN -1
#define EOC_PIN   -1
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN)

std_msgs::Float32 press_msg;
ros::NodeHandle nh;
ros::Publisher pub_press("pressure", &press_msg);
ros::Subscriber<std_msgs::Empty> sub_valve("toggle_valve", &messageCb);

void messageCb(const std_msgs::Empty& toggle_msg){
  digitalWrite(2, HIGH-digitalRead(2));
}

// Constants
const int valvePin = 2;   // output pin for valve
const int sensorAddress = 0x18

// Variables
long publisher_timer;


void setup() {
  // Initialize VALVE pin as output
  pinMode(valvePin, OUTPUT);

  // ROS stuff
  nh.initNode();
  nh.subscribe(sub);

  // Serial Port
  Serial.begin(115200)
  Serial.println("MPRLS Simple Test")
  if (! mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?")
    while (1) {
      delay(10);
    }
  }
  Serial.println("Found MPRLS sensor")
}

void loop() {
  float pressure_hPa = mpr.rearPressure();

  while (!nh.connected()){
     nh.spinOnce();
  }


  // Read Sensor
  Serial.print("Pressure (hPa): "); Serial.println(pressure_hPa);
  Serial.print("Pressure (PSI): "); Serial.println(pressure_hPa / 68.947572932);
  delay(1000);
 

  // Actuate Valve
  digitalWrite(valvePin, HIGH);
  delay(1000);
  digitalWrite(valvePin, LOW);
  delay;

}
