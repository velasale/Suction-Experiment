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
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);


/*************************** Base ROS Setup ******************************/
// General ROS packages/nodes
#include <ros.h>
ros::NodeHandle nh;

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

// Variables to be published
std_msgs::Float32 press_msg;
std_msgs::String str_msg;

// Topics
ros::Publisher pub_press("pressure", &press_msg);
ros::Publisher chatter("chatter", &str_msg);


// Constants
const int valvePin = 2;   // output pin for valve
const int sensorAddress = 0x18;
char hello[13] = "hello world!";

// Variables
long publisher_timer;


void setup() {
  // Initialize VALVE pin as output
  pinMode(valvePin, OUTPUT);

  Wire.begin();

  mpr.begin();

  // ROS stuff
  nh.initNode();
  nh.advertise(pub_press);
  nh.advertise(chatter);    
}

// Choice a: Read the sensor using the built-in function mpr.read
// Chice b: Read the sensor using i2c differently, so it doesnt mess the publishing
//          using the Wire.h library

void loop() {

  if (millis() > publisher_timer){
    float pressure_hPa = mpr.readPressure();
    str_msg.data = hello;
    delay(100);
    press_msg.data = pressure_hPa;
    pub_press.publish(&press_msg);      
    chatter.publish(&str_msg);
   
    publisher_timer = millis() + 100;    
  }
  
  nh.spinOnce();
  
}
