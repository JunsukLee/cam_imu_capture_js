// https://forum.arduino.cc/index.php?topic=52534.0
// https://www.arduino.cc/en/Hacking/PinMapping2560
#include <ros.h>
#include "sensor_msgs/TimeReference.h"
#define TRIGGER_PIN_CAM 8//0x20//8
#define TRIGGER_PIN_IMU 9//0x40//9

#define TriggerSyncThreshold 5

#define ONOFF_Delay 1
#define Trigger_Delay 100
const int FrequencyIMU = 10000-(int)Trigger_Delay-(int)ONOFF_Delay; // 1000 is 1 ms

int counter_IMU_data = 1;
int mFrequency_imu = FrequencyIMU - Trigger_Delay;

ros::NodeHandle nh;
sensor_msgs::TimeReference msg_triggerTime;
ros::Publisher pub_triggerTime("/imu/trigger_time", &msg_triggerTime);

void trigger_IMU();
void trigger_IMU_CAM();

void setup() {
  nh.getHardware()->setBaud(115200); //or what ever baud you want
  nh.initNode();
  nh.advertise(pub_triggerTime);
  msg_triggerTime.header.frame_id = "trigger";
  
  // put your setup code here, to run once:
  //digitalWrite(TRIGGER_PIN_CAM, LOW);  //  drive it low without temporarily driving it high
  //digitalWrite(TRIGGER_PIN_IMU, LOW);  //  drive it low without temporarily driving it high
  //pinMode(TRIGGER_PIN_CAM, OUTPUT);
  //pinMode(TRIGGER_PIN_IMU, OUTPUT);
  DDRH = B11100000; // set PORTD (digital 7~0) to outputs  
}

void loop() {
  // put your main code here, to run repeatedly:
  delayMicroseconds(FrequencyIMU);
  if(counter_IMU_data == TriggerSyncThreshold)
  {
    trigger_IMU_CAM();
    counter_IMU_data = 1;
  }else
  {
    trigger_IMU();
    ++counter_IMU_data;
  }  
}

void trigger_IMU() {
  PORTH = B01000000; // digital 9 HIGH
  delayMicroseconds(Trigger_Delay);
  msg_triggerTime.header.frame_id = "imu";
  msg_triggerTime.header.stamp = nh.now();  
  pub_triggerTime.publish(&msg_triggerTime);
  nh.spinOnce();
  PORTH = B00000000; // digital 9 LOW
}

void trigger_IMU_CAM(){
  PORTH = B01100000; // digital 8~9 HIGH
  delayMicroseconds(Trigger_Delay);
  msg_triggerTime.header.frame_id = "camimu";
  msg_triggerTime.header.stamp = nh.now();
  pub_triggerTime.publish(&msg_triggerTime);
  nh.spinOnce();
  PORTH = B00000000; // digital 8~9 LOW  
}
