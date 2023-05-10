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

/***** Constants *****/
const bool USE_ROSSERIAL = false;
const uint8_t NUM_SCUPS = 3;          /*Number of suction cups*/

#include <Wire.h>
#include "Adafruit_MPRLS.h"

//You don't *need* a reset and EOC pin for most usees, so we set to -1 and don't connect
#define RESET_PIN -1
#define EOC_PIN   -1
#define VALVE 13
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);


/*************************** Base ROS Setup ******************************/
// General ROS packages/nodes
#include <ros.h>
ros::NodeHandle nh;




/*************************** ROS Services Setup **************************/
#include <std_srvs/Trigger.h>

void closeValveService(const std_srvs::Trigger::Request &req,
                       std_srvs::TriggerResponse &res){
                       res.success = closeValve();
                       }
ros::ServiceServer<std_srvs::Trigger::Request,
    std_srvs::Trigger::Response>
    service_close("closeValve", &closeValveService);


void openValveService(const std_srvs::Trigger::Request &req,
                      std_srvs::TriggerResponse &res){
                      res.success = openValve();
                      }
ros::ServiceServer<std_srvs::Trigger::Request,
    std_srvs::Trigger::Response>
    service_open("openValve", &openValveService);
                      


/************************** ROS Publishers Setup ************************/
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

std_msgs::Float32 press_msg[NUM_SCUPS];

ros::Publisher publisher_pressure[NUM_SCUPS]{
  ros::Publisher("/gripper/pressure/sc1", &press_msg[0]),
  ros::Publisher("/gripper/pressure/sc1", &press_msg[1]),
  ros::Publisher("/gripper/pressure/sc1", &press_msg[2])
};

std_msgs::String messages_msg
ros::Publisher publisher_messages("/gripper/messages", &messages_msg);


// Constants
const int PCAADR = 0x70;
//const int sensorAddress = 0x18;
const int VALVE_DELAY = 10;

// Variables
long publisher_timer;




/********************************* Setup ********************************/
void setup() {
  // Initialize serial:
  Serial.begin(115200);
  
  // Initialize VALVE pin as output
  pinMode(VALVE, OUTPUT);

  // Initialize pressure sensor
  //Wire.begin();
  mpr.begin();

  // Initialize ROS stuff
  if (USE_ROSSERIAL){
    // Initialize ROS node
    nh.initNode();
    
    // Advertise Publishers  
    for (int s=0; s < NUM_SCUPS; s++){
      nh.advertise(publisher_pressure[s]);
    }    

    // Advertise Services
    nh.advertiseService(service_open);
    nh.advertiseService(service_close);
    
  } else {
    // Communicate through Serial instead
    Serial.println("Starting Suction Gripper on Serial...")
  }    

  // Initialize hardware outputs in their default state
  digitalWrite(VALVE, LOW);   
 
  
}


/**************************** Loop **************************************/
void loop() {
  
  // Check for service calls
  if(USE_ROSSERIAL){
    nh.spinOnce();
  } else {
    //do nothing
  }
  

  // Get data
  for (int i = = 0; i<3; i++){
    pcaselect(i);
    float pressure_hPa = mpr.readPressure();
    
    pres_msg[i].data = pressure_hPa;
    
  }

  TODO Parallel
  TODO Switch to integers instead of Floats


  // Send data
  if (USE_ROSSERIAL){
    nh.spinOnce();
  
    for (int i = = 0; i<3; i++){      
      publisher_pressure[i] = pres_msg[i].data      
    }
    
  } else {
    for (int i = = 0; i<3; i++){      
      Serial.print(pres_msg[i].data);    
      Serial.print(" "
    }
    Serial.println("")    
  }

  

  if (millis() > publisher_timer){

    for (int i = 0; i <3; i++){      

      // 0 = Suction A,  1 = Suction B, 2 = Suction C
      pcaselect(i);
      
      float pressure_hPa = mpr.readPressure();
      delay(10);
      
      press_msg.data = pressure_hPa;

      
      publisher_pressure.publish(&press_msg);      
     
      publisher_timer = millis() + 10;    
  
      
    }
  }
  
}


/****************************** Data Reading Helper Function ****************/



/****************************** Control Functions ***************************/
bool closeValve(){
  bool success = true;
  delay(VALVE_DELAY);
  digitalWrite(VALVE, LOW);
  delay(VALVE_DELAY); 
  return success;  
}


bool openValve(){
  bool success = true;
  delay(VALVE_DELAY);
  digitalWrite(VALVE, HIGH);
  delay(VALVE_DELAY); 
  return success;  
}


/**************************** I2C Multiplexer *********************************/
// Reference: https://learn.adafruit.com/adafruit-pca9546-4-channel-i2c-multiplexer/arduino
void pcaselect(uint8_t i){
  if (i > 3) return;
  
  Wire.beginTransmission(PCAADR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
