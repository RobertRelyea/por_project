// Arduino PID Library
#include <PID_v1.h>

// Sabertooth motor controller communications
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

// ROS 
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <dart_delivery/RosieOdom.h>

// Quaternion conversion
//#include "quaternion.h"

// Control loop frequency
#define LOOP_FREQ (20)

// Sabertooth communications declarations
SoftwareSerial SWSerial(12, 11); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.

// PID control input and output variables
double setpoint, right_setpoint, left_input, left_output;
double left_setpoint, right_input, right_output;

// PID control structure instantiation
PID right_PID(&right_input, &right_output, &right_setpoint, 0.01, 0.5, 0.001, DIRECT);
PID left_PID(&left_input, &left_output, &left_setpoint, 0.01, 0.5, 0.001, DIRECT);

// Encoder ticks per meter values for both tracks
int right_ticks_per_meter = 361;
int left_ticks_per_meter = 372;

// Approximate "radii" for both tracks
double right_radius = 0.2177 * 4.4;
double left_radius = 0.2224 * 4.4;
// Axle Track
double L = 0.508 * 1.25; 

// Velocity calculation globals
// Latest accumulated encoder tick position
long newpositionL =0;
long newpositionR =0;
// Previously used accumulated encoder tick position
long oldpositionL = 0;
long oldpositionR = 0;
// Timestamps for encoder position reads
long newtime = 0;
long oldtime = 0;
// Calculated encoder velocities (ticks/s)
long velL = 0;
long velR = 0;

#include <Encoder.h>
#define ENCODER_USE_INTERRUPTS // Move this before encoder include?

// Encoder structure and interrupt setup
Encoder knobLeft(7, 8);
Encoder knobRight(9, 10);

ros::NodeHandle nh;

// cmd_vel callback function, sets PID target velocities
void cmdVelCB(const geometry_msgs::Twist& cmd_vel_msg)
{
  // Extract relevant velocities from Twist
  double linear_vel = cmd_vel_msg.linear.x;
  double angular_vel = cmd_vel_msg.angular.z;

  // Differential drive kinematic equations
  double v_left = linear_vel - ((L / 2) * angular_vel);
  v_left /= left_radius;

  double v_right = linear_vel + ((L / 2) * angular_vel);
  v_right /= right_radius;

  // Update PID setpoints
  left_setpoint = v_left * left_ticks_per_meter;
  right_setpoint = v_right * right_ticks_per_meter;
}

// Set up subscribers and publishers
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &cmdVelCB);
dart_delivery::RosieOdom odom_msg;
ros::Publisher odom_pub("/rosie_odom", &odom_msg);

void setup() {
  // Sabertooth communications setup
  SWSerial.begin(9600);
  // For some reason, the first message to the sabertooth is bad,
  // flush it with two to begin with.
  ST.motor(1,0);
  ST.motor(2,0);
  delay(1); // Lets hope we don't break something
  ST.motor(1,0);
  ST.motor(2,0);

  // Encoder PID setup
  left_setpoint = 0;
  right_setpoint = 0;

  left_input = velL;
  left_PID.SetMode(AUTOMATIC);
  left_PID.SetOutputLimits(-127.0, 127.0); // Limits
  
  right_input = velR;
  right_PID.SetMode(AUTOMATIC);
  right_PID.SetOutputLimits(-127.0, 127.0); // Limits
  
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(odom_pub);

}

void loop() {

  /******           Encoder tick updates            ******/
  // Read encoder tick counts
  newpositionL = knobLeft.read();
  newpositionR = knobRight.read();
  // Calculate encoder tick velocities
  newtime = millis();
  velL = (newpositionL-oldpositionL) * 1000 /(newtime-oldtime);
  velR = (newpositionR-oldpositionR) * 1000 /(newtime-oldtime);


  /******           Velocity PID update            ******/
  // Calculate left motor command
  int left_motor_val = 0;
  left_input = velL;
  left_PID.Compute();
  left_motor_val = left_output;

  // Calculate right motor command
  int right_motor_val = 0;
  right_input = velR;
  right_PID.Compute();
  right_motor_val = right_output;
  
//   Send new motor commands
  ST.motor(2, (int)left_motor_val * -1);
  ST.motor(1, (int)right_motor_val * -1);

  /******           Odometry update            ******/
  // Convert tick velocities to meter velocities
  float velL_meters = (float)velL / ((float)left_ticks_per_meter);
  float velR_meters = (float)velR / ((float)right_ticks_per_meter);

  // Update velocities based on differential drive model
  float vx = (((left_radius / 2.) * velL_meters) + (((right_radius / 2.) * velR_meters))) * cos(odom_msg.theta);
  float vy = (((left_radius / 2.) * velL_meters) + (((right_radius / 2.) * velR_meters))) * sin(odom_msg.theta);
  float vth = (1 / L) * ((velR_meters*right_radius) - (velL_meters*left_radius));
  
  // Compute odometry change
  float dt = (newtime - oldtime) / 1000.0;
  float delta_x = vx * dt;
  float delta_y = vy * dt;
  float delta_th = vth * dt;

  // Populate RosieOdom message
  odom_msg.x += delta_x;
  odom_msg.y += delta_y;
  odom_msg.theta += delta_th;

  // Debug
//  odom_msg.x = newpositionL;
//  odom_msg.y = newpositionR;
//  odom_msg.theta = vth;

  // Update encoder information
  oldpositionL = newpositionL;
  oldpositionR = newpositionR;
  oldtime = newtime;

  // Publish new RosieOdom message
  odom_pub.publish(&odom_msg);

  // Control loop delay
  delay(1000 / LOOP_FREQ); // LOOP_FREQ Hz (ish) control loop
  nh.spinOnce();
}

