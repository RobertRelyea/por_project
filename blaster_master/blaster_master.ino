
#define FLYWHEEL_PIN (11)
#define FEEDER_PIN (10)

// ROS 
#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

void triggerCB(const std_msgs::Int16& trigger_msg)
{
  digitalWrite(FLYWHEEL_PIN, HIGH);
  delay(75);
  digitalWrite(FEEDER_PIN, HIGH);
  delay(30);
  digitalWrite(FEEDER_PIN, LOW);
  delay(20);
  digitalWrite(FLYWHEEL_PIN, LOW);
  
}

ros::Subscriber<std_msgs::Int16> trigger_sub("/nerf_trigger", &triggerCB);

void setup() {
  pinMode(FLYWHEEL_PIN, OUTPUT);
  pinMode(FEEDER_PIN, OUTPUT);

  nh.initNode();
  nh.subscribe(trigger_sub);
}

void loop() {
  delay(50);

  nh.spinOnce();
}
