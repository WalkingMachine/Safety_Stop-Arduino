/*
 * ATmega328P based safety stop
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
  REQUEST_PENDING,
}State;

const int START_BUTTON = 4;
const int SAFETY_BUTTON = 2;
//const int DRIVE_CTRL = 6;
const int POWER_CTRL = 3;
//const int GREEN_LED_PIN = 6;
const int RED_LED_PIN = 13;

const int INTERVAL = 500;
const int NUMBER_OF_TRY = 3;

bool start_last_reading = false;

int power_state = HIGH;
State system_state;
bool last_reading;

ros::NodeHandle  nh;

ros::ServiceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response> safety_stop_srv("safety_stop_srv");

std_msgs::Bool start_msg;
ros::Publisher start_button("start_button_msg", &start_msg);

void start_button_handle();
void safety_stop_handler();
void led_handler();

void setup()
{
  pinMode(START_BUTTON, INPUT);
  pinMode(SAFETY_BUTTON, INPUT);
  //pinMode(DRIVE_CTRL, OUTPUT);
  pinMode(POWER_CTRL, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  //pinMode(GREEN_LED_PIN, OUTPUT);
  
  digitalWrite(POWER_CTRL, power_state);
  //digitalWrite(GREEN_LED_PIN, HIGH);
  
  
  last_reading = digitalRead(SAFETY_BUTTON);
  if(last_reading == HIGH)
  {
    system_state = RUNNING;
  }
  else
  {
    system_state = SAFETY_STOP;
  }
  
  nh.initNode();
  nh.advertise(start_button);
  //nh.serviceClient(safety_stop_srv);
  
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
    //digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, HIGH);
    safety_stop_handler();
  }
  
  digitalWrite(POWER_CTRL, power_state);
  nh.spinOnce();
}

void start_button_handle()
{
  bool reading = digitalRead(START_BUTTON);
  
  if (start_last_reading!= reading){
    start_last_reading = reading;
    if(reading){
      start_msg.data = reading;
      start_button.publish(&start_msg);
    }
  }
}

void safety_stop_handler()
{
  bool reading = digitalRead(SAFETY_BUTTON);
  
  if (last_reading != reading){
    std_srvs::SetBool::Request req;
    std_srvs::SetBool::Response res;
    last_reading = reading;
    req.data = reading;
    system_state = REQUEST_PENDING;
    
    for(int i=0; i<NUMBER_OF_TRY; i++)
    {
      safety_stop_srv.call(req, res);
      if(res.success)
      {
        if(reading == HIGH)
        {
          system_state = RUNNING;
        }
        else
        {
          system_state = SAFETY_STOP;
        }
        break;
      }
      delay(10);
    }
    
    if(system_state == REQUEST_PENDING)
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
    digitalWrite(RED_LED_PIN, HIGH - digitalRead(RED_LED_PIN));
  }
}

