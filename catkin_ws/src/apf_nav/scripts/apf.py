#!/usr/bin/env python
import math
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import pdb

# Variable to hold current net force on bot
# Calculated from input laserscan
net_force = [0, 0]

# Maximum obstacle distance to be used for apf calculation
max_obs_dist = 5
# Negative scaling factor for obstacle repulsive forces
neg_scale = -0.02

def laserscanCB(data):
    global net_force

    # Extract data from laserscan message
    min_angle = data.angle_min
    max_angle = data.angle_max
    angle_increment = data.angle_increment

    net_x = 0
    net_y = 0
    angle_idx = 0

    a = np.arange(min_angle, max_angle - angle_increment, angle_increment)
    # Calculate net force
    for angle in np.arange(min_angle, max_angle - angle_increment, angle_increment):
        # Retrieve distance reading for current angle
        range_val = data.ranges[angle_idx]
        force = 0
        # Define a repulsive force for the given distance
        if range_val != 0:
            force = 0.5 * neg_scale * math.pow((1 / (range_val) - 1/max_obs_dist), 2)

        # Compute x and y components of the repulsive force
        force_x = math.cos(angle) * force
        force_y = math.sin(angle) * force
        # Sum repulsive forces
        net_x += force_x
        net_y += force_y
        # Increment angle
        angle_idx += 1

    net_force = [net_x, net_y]

def apf():
    # Starts a new node
    rospy.init_node('apf_nav', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, laserscanCB)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    r = rospy.Rate(100)
    while(not rospy.is_shutdown()):
        vel_msg = Twist()

        # Determine linear motion required
        x = net_force[0] / 10
        if x > 0.2:
            x = 0.2
        elif x < -0.3:
            x = -0.3
        vel_msg.linear.x = x + 0.1

        # Determine angular motion required

        z = net_force[1] / 5
        if z > 0.4:
            z = 0.4
        elif z < -0.4:
            z = -0.4
        vel_msg.angular.z = z


        velocity_publisher.publish(vel_msg)
        print(net_force)    
        r.sleep()

    # Stop the bot
    vel_msg = Twist()
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        #Testing our function
        apf()
    except rospy.ROSInterruptException: pass