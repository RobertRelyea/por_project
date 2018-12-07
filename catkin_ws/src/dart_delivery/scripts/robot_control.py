#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Bool, Int16
from geometry_msgs.msg import Twist



MAX_Z = 1.0
MIN_Z = 0
DEADZONE = 0.2
FIRING_ZONE = 0.2
# Cooldown for nerf blaster in seconds
NERF_COOLDOWN = 5

angle = 99
angle_time = 0
apf_enable = False
velocity_pub = {}
drive_enabled = False
nerf_enabled = False
turret_mode = False


def joy_callback(data):
    global drive_enabled, nerf_enabled, turret_mode
    # Toggle movement with right trigger
    if data.buttons[7] == 1:
        drive_enabled = not drive_enabled
    # Enable targeting if left trigger is held down
    if data.buttons[1] == 1:
        turret_mode = True
    else:
        turret_mode = False
    if data.buttons[6] == 1:
        nerf_enabled = True
    else:
        nerf_enabled = False


def apfCmdCB(data):
    global apf_enable, turret_mode
    if apf_enable and (not turret_mode):
        velocity_pub.publish(data)


def angleCB(data):
    global angle, angle_time

    angle = data.data
    angle_time = rospy.get_time()


def robot_control():
    global angle, angle_time, apf_enable, velocity_pub, drive_enabled, nerf_enabled, turret_mode
    # Starts a new node
    rospy.init_node('robot_control', anonymous=True)
    rospy.Subscriber('cmd_vel_apf', Twist, apfCmdCB)
    rospy.Subscriber('omnivision_principles/blob_angle', Float32, angleCB)
    rospy.Subscriber('joy', Joy, joy_callback)
    velocity_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    nerf_pub = rospy.Publisher('/nerf_trigger', Int16, queue_size=10)

    last_nerf_time = rospy.get_time()
    r = rospy.Rate(15)
    stopped = True
    z = 0
    while(not rospy.is_shutdown()):
        if drive_enabled:
            stopped = False
            # Determine if we have received a recent angle update
            if ((rospy.get_time() - angle_time < 1.0 / 15.0) and nerf_enabled and (rospy.get_time() > last_nerf_time)) or turret_mode:
                apf_enable = False

                if abs(angle) <= FIRING_ZONE and z  <= 0.1 and (rospy.get_time() > last_nerf_time):
                    nerf_msg = Int16()
                    nerf_pub.publish(nerf_msg)
                    last_nerf_time = rospy.get_time()
                    last_nerf_time += NERF_COOLDOWN
                z = angle

                if z > MAX_Z:
                    z = MAX_Z
                elif z < -MAX_Z:
                    z = -MAX_Z
                vel_msg = Twist()
                vel_msg.angular.z = z
                velocity_pub.publish(vel_msg)
            else:
                apf_enable = True
        else: # ESTOP
            apf_enable = False
            if not stopped:
                vel_msg = Twist()
                velocity_pub.publish(vel_msg)
                stopped = True
        r.sleep()

    # Stop the bot
    vel_msg = Twist()
    velocity_pub.publish(vel_msg)


if __name__ == '__main__':
    try:
        #Testing our function
        robot_control()
    except rospy.ROSInterruptException: pass
