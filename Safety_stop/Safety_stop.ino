/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/Byte.h>

const int START_BUTTON = 4
const int SAFETY_BUTTON = 5;
const int DRIVE_CTRL = 6;
const int LED_PIN = 13

ros::NodeHandle  nh;

std_msgs::Byte alert_msg;
ros::Publisher safety_stop("safety_stop", &alert_msg);

void setup()
{
  
  nh.initNode();
  nh.advertise(safety_stop);
}

void loop()
{
  alert_msg.data = 1;
  safety_stop.publish( &alert_msg );
  nh.spinOnce();
  delay(250);
}
