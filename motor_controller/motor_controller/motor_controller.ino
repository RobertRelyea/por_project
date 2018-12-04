#include <PID_v1.h>
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>

// Control loop frequency
#define LOOP_FREQ (20)


SoftwareSerial SWSerial(12, 11); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.




// PID control for left track
double setpoint, right_setpoint, left_input, left_output;
double left_setpoint, right_input, right_output;

PID right_PID(&right_input, &right_output, &right_setpoint, 0.01, 0.5, 0.0, DIRECT);
PID left_PID(&left_input, &left_output, &left_setpoint, 0.01, 0.5, 0.0, DIRECT);

int right_ticks_per_meter = 361;
int left_ticks_per_meter = 372;

double right_radius = 0.2177 * 4;
double left_radius = 0.2224 * 4;

long newpositionL =0;
long newpositionR =0;
long oldpositionL = 0;
long oldpositionR = 0;
long oldtime = 0;
long newtime = 0;
long velL = 0;
long velR = 0;
double L = 0.508; // Axle Track

#include <Encoder.h>
#define ENCODER_USE_INTERRUPTS

Encoder knobLeft(7, 8);
Encoder knobRight(9, 10);

ros::NodeHandle nh;

// cmd_vel callback function, sets PID target velocities
void cmdVelCB(const geometry_msgs::Twist& cmd_vel_msg)
{

  double linear_vel = cmd_vel_msg.linear.x;
  double angular_vel = cmd_vel_msg.angular.z;

  // Differential drive kinematic equations
  double v_left = linear_vel - ((L / 2) * angular_vel);
  v_left /= left_radius;

  double v_right = linear_vel + ((L / 2) * angular_vel);
  v_right /= right_radius;
  
  left_setpoint = v_left * left_ticks_per_meter;
  right_setpoint = v_right * right_ticks_per_meter;
}

// Set up subscribers
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &cmdVelCB);

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

  // Encoder setup
 
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

}

void loop() {
  long newLeft, newRight;
  newpositionL = knobLeft.read();
  newpositionR = knobRight.read();
  newtime = millis();
  velL = (newpositionL-oldpositionL) * 1000 /(newtime-oldtime);
  velR = (newpositionR-oldpositionR) * 1000 /(newtime-oldtime);
  
  int left_motor_val = 0;
  left_input = velL;
  left_PID.Compute();
  left_motor_val = left_output;
  
  int right_motor_val = 0;
  right_input = velR;
  right_PID.Compute();
  right_motor_val = right_output;

  ST.motor(2, (int)left_motor_val * -1);
  ST.motor(1, (int)right_motor_val * -1);

  delay(1000 / LOOP_FREQ); // 20 Hz (ish) control loop
  oldpositionL = newpositionL;
  oldpositionR = newpositionR;
  oldtime = newtime;

  nh.spinOnce();
}
