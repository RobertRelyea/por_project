// Software Serial Sample
// Copyright (c) 2012 Dimension Engineering LLC
// See license.txt for license details.

#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>

SoftwareSerial SWSerial(12, 11); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.

ros::NodeHandle nh;

void cmdVelCB(const geometry_msgs::Twist& cmd_vel_msg)
{

  float linear_vel = cmd_vel_msg.linear.x * 50;
  float angular_vel = cmd_vel_msg.angular.z * 50;

  float right_motor_value = linear_vel - angular_vel;
  float left_motor_value = linear_vel + angular_vel;
  
  ST.motor(1, right_motor_value);
  ST.motor(2, left_motor_value);
  
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &cmdVelCB);

void setup()
{
  SWSerial.begin(9600);
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}

