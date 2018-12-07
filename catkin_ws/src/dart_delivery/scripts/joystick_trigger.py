#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16

nerf_pub = {}


def joy_callback(data):
    global nerf_pub
    # Fire with right trigger
    if data.buttons[6] == 1:
        trigger_msg = Int16()
        nerf_pub.publish(trigger_msg)

def joystick_trigger():
    global nerf_pub
    # Starts a new node
    rospy.init_node('joystick_trigger', anonymous=True)
    rospy.Subscriber('joy', Joy, joy_callback)
    nerf_pub = rospy.Publisher('/nerf_trigger', Int16, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        joystick_trigger()
    except rospy.ROSInterruptException: pass
