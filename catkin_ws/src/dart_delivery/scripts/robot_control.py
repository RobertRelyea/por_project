#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist



MAX_Z = 0.7
MIN_Z = 0.4

angle = 0
angle_time = 0
apf_enable = False
velocity_pub = {}
drive_enabled = False

def joy_callback(data):
    global drive_enabled
    if data.buttons[7] == 1:
        drive_enabled = not drive_enabled

def apfCmdCB(data):
    global apf_enable
    if apf_enable:
        velocity_pub.publish(data)


def angleCB(data):
    global angle, angle_time

    angle = -data.data
    angle_time = rospy.get_time()

def robot_control():
    global angle, angle_time, apf_enable, velocity_pub, drive_enabled
    # Starts a new node
    rospy.init_node('robot_control', anonymous=True)
    rospy.Subscriber('cmd_vel_apf', Twist, apfCmdCB)
    rospy.Subscriber('omnivision_principles/blob_angle', Float32, angleCB)
    rospy.Subscriber('joy', Joy, joy_callback)
    velocity_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    r = rospy.Rate(15)
    stopped = True
    while(not rospy.is_shutdown()):
        if drive_enabled:
            stopped = False
            # Determine if we have received a recent angle update
            if rospy.get_time() - angle_time < 1.0 / 15.0:
                apf_enable = False
                z = (MIN_Z * np.sign(angle)) + angle * 2
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
