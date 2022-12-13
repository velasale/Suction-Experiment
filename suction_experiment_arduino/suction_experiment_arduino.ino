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
std_msgs::String messages_message;
ros::Publisher publisher_messages("/gripper/messages", &messages_message);

/*************************** ROS Services Setuo **************************/
#include <std_srvs/Trigger.h>
//void openValveService(const std_srvs::Trigger::Request &req,
//                            std_srvs::Trigger::Response &res) {
//      res.success = openValve();
//                            }
//ros::ServiceServer<std_srvs::Trigger::Request,
//                   std_srvs::Trigger::Response>
//                   service_open("/gripper/open_valve", &openValveService);


void openValve(const std_msgs::Empty& toggle_msg){
  digitalWrite(LED_BUILTIN, HIGH);    
}

void closeValve(const std_msgs::Empty& toggle_msg){
  digitalWrite(LED_BUILTIN, LOW);    
}


// Topics
ros::Publisher pub_press("pressure", &press_msg);
ros::Subscriber<std_msgs::Empty> sub1("toggle_valve", &openValve);
ros::Subscriber<std_msgs::Empty> sub2("toggle_valve", &closeValve);


// Constants
const int valvePin = 2;   // output pin for valve
const int sensorAddress = 0x18;
char hello[13] = "hello world!";

// Variables
long publisher_timer;


void setup() {
  // Initialize VALVE pin as output
  pinMode(LED_BUILTIN, OUTPUT);

  Wire.begin();

  // Initialize pressure sensor
  mpr.begin();

  // ROS stuff
  nh.initNode();
  nh.advertise(pub_press);
  nh.subscribe(sub1);
  nh.subscribe(sub2);
}


void loop() {

  if (millis() > publisher_timer){
    float pressure_hPa = mpr.readPressure();
    delay(100);
    press_msg.data = pressure_hPa;
    pub_press.publish(&press_msg);      
   
    publisher_timer = millis() + 10;    
  }
  
  nh.spinOnce();
  
}


//bool openValve(){
//  bool success = true;
//  digitalWrite(LED_BUILTIN, HIGH);  
//
//  return success;
//}
//
//
//bool closeValve(){
//  bool success = true;
//  digitalWrite(LED_BUILTIN, LOW);  
//
//  return success;
//}
