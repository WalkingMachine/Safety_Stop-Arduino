/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

typedef enum{
  RUNNING =1,
  SAFETY_STOP,
}State;

const int START_BUTTON = 4;
const int SAFETY_BUTTON = 5;
const int DRIVE_CTRL = 6;
const int POWER_CTRL = 7;
const int LED_PIN = 13;

unsigned long last_time = 0;
const long INTERVAL = 500;
int led_state = LOW; 
int power_state = HIGH;
State system_state;

bool last_reading = false;
unsigned long last_debounce_time=0;
const unsigned long debounce_delay=50;
bool published = true;

ros::NodeHandle  nh;

std_msgs::Bool alert_msg;
ros::Publisher safety_stop("safety_stop", &alert_msg);

std_msgs::Bool start_msg;
ros::Publisher start_button("start_button", &start_msg);

void setup()
{
  pinMode(START_BUTTON, INPUT);
  pinMode(SAFETY_BUTTON, INPUT);
  pinMode(DRIVE_CTRL, OUTPUT);
  pinMode(POWER_CTRL, OUTPUT);
  digitalWrite(POWER_CTRL, power_state);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, led_state);
  
  system_state = RUNNING;
  start_msg.data = true;
  nh.initNode();
  nh.advertise(safety_stop);
  nh.advertise(start_button);
}

void loop()
{
  
  // Start button publishing ------------------------------
  if(system_state == RUNNING){
    bool reading = digitalRead(START_BUTTON);
  
    if (last_reading!= reading){
      last_reading = reading;
      if(reading){
        start_button.publish(&start_msg);
      }
    }
  
  
  // Led flashing (state dependant) -------------------------------------
  if (millis() - last_time >= INTERVAL && system_state == RUNNING)
  {
    // save the last time you blinked the LED
    last_time = millis();
    
    if (led_state == LOW) {
      led_state = HIGH;
    } else {
      led_state = LOW;
    }
    digitalWrite(LED_PIN, led_state);
  }
  else{
    digitalWrite(LED_PIN, HIGH);
  }
  
  // Safety stop --------------------------------
  if(digitalRead(SAFETY_BUTTON) == LOW){
    system_state = SAFETY_STOP;
    
    //send service
  }
  
  nh.spinOnce();
}

