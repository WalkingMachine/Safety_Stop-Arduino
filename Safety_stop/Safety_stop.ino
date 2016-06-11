/*
 * Arduino based safety stop
 * Service: safety_stop_srv
 * Topic: start_button
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

typedef enum{
  RUNNING =1,
  SAFETY_STOP,
}State;

const int START_BUTTON = 4;
const int SAFETY_BUTTON = 5;
const int DRIVE_CTRL = 6;
const int POWER_CTRL = 7;
const int GREEN_LED_PIN = 6;
const int RED_LED_PIN = 11;
const int INTERVAL = 500;

int led_state = LOW; 
int power_state = HIGH;
State system_state;

ros::NodeHandle  nh;

ros::ServiceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response> safety_stop_srv("safety_stop_srv");

std_msgs::Bool start_msg;
ros::Publisher start_button("start_button", &start_msg);

void start_button_handle();
void safety_stop_handler();
void led_handler();

void setup()
{
  pinMode(START_BUTTON, INPUT);
  pinMode(SAFETY_BUTTON, INPUT);
  pinMode(DRIVE_CTRL, OUTPUT);
  pinMode(POWER_CTRL, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  
  digitalWrite(POWER_CTRL, power_state);
  digitalWrite(GREEN_LED_PIN, HIGH);
  
  system_state = RUNNING;
  
  nh.initNode();
  nh.advertise(start_button);
  nh.serviceClient(safety_stop_srv);
  
  while(!nh.connected()) nh.spinOnce();
  nh.loginfo("Safety stop startup complete");
}

void loop()
{
  if(system_state == RUNNING)
  { 
    start_button_handle();
    led_handler();
    safety_stop_handler();
  }
  
  if(system_state == SAFETY_STOP)
  {
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, HIGH);
    safety_stop_handler();
  }
  
  digitalWrite(POWER_CTRL, power_state);
  nh.spinOnce();
}

void start_button_handle()
{
  static bool last_reading = false;
  bool reading = digitalRead(START_BUTTON);
  
  if (last_reading!= reading){
    last_reading = reading;
    if(reading){
      start_msg.data = reading;
      start_button.publish(&start_msg);
    }
  }
}

void safety_stop_handler()
{
  static bool last_reading = true;
  bool reading = digitalRead(SAFETY_BUTTON);
  std_srvs::SetBool::Request req;
  std_srvs::SetBool::Response res;
  
  if (last_reading!= reading){
    last_reading = reading;
    req.data = reading;
    if(safety_stop_srv.call(req, res))
    {
      if(reading)
      {
        system_state = RUNNING;
      }
      else
      {
        system_state = SAFETY_STOP;
      }
    }
    else
    {
      system_state = SAFETY_STOP;
      power_state = LOW;
    }
  }
}

void led_handler()
{
  static unsigned long last_time = 0;
  
  if (millis() - last_time >= INTERVAL)
  {
    last_time = millis();
    
    if (led_state == LOW) {
      led_state = HIGH;
    } else {
      led_state = LOW;
    }
    digitalWrite(GREEN_LED_PIN, led_state);
  }
}

