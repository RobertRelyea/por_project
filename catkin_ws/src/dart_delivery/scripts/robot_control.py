#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist

angle = 0
angle_time = 0
apf_enable = False
velocity_pub = {}

def apfCmdCB(data):
    global apf_enable
    if apf_enable:
        velocity_pub.publish(data)


def angleCB(data):
    global angle, angle_time

    angle = data.data
    angle_time = rospy.get_time()

def robot_control():
    global angle, angle_time, apf_enable, velocity_pub
    # Starts a new node
    rospy.init_node('robot_control', anonymous=True)
    rospy.Subscriber('cmd_vel_apf', Twist, apfCmdCB)
    rospy.Subscriber('omnivision/blob_angle', Float32, angleCB)
    velocity_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    r = rospy.Rate(15)
    while(not rospy.is_shutdown()):
        # Determine if we have received a recent angle update
        if rospy.get_time() - angle_time < 1.0 / 15.0:
            apf_enable = False
            z = angle * 2
            if z > 0.5:
                z = 0.5
            elif z < -0.5:
                z = -0.5
            vel_msg = Twist()
            vel_msg.angular.z = z
            velocity_pub.publish(vel_msg)
        else:
            apf_enable = True
        r.sleep()

    # Stop the bot
    vel_msg = Twist()
    velocity_pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        #Testing our function
        robot_control()
    except rospy.ROSInterruptException: pass
