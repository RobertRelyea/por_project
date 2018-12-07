#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Quaternion
from dart_delivery.msg import RosieOdom
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

odom_pub = {}
odom_broadcaster = {}
marker_pub = {}


# Publishes incoming RosieOdom messages as Odometry messages.
# Updates odom->base_footprint transform.
def odomCB(data):
    global odom_pub, odom_broadcaster
    odom = Odometry()
    current_time = rospy.Time.now()
    odom.header.stamp = current_time

    odom.header.frame_id = "/odom"
    odom.child_frame_id = "/base_footprint"

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, data.theta)
    odom.pose.pose = Pose(Point(data.x, data.y, 0), Quaternion(*odom_quat))

    odom_pub.publish(odom)

    odom_broadcaster.sendTransform(
        (data.x, data.y, 0),
        odom_quat,
        current_time,
        "/base_footprint",
        "/odom"
    )
    markerMaker()

def markerMaker():
    global marker_pub
    marker = Marker()
    marker.header.frame_id='/base_footprint'
    marker.type = marker.ARROW
    marker.action = marker.MODIFY
    marker.scale.x = 0.5
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.b = 1.0
    marker.color.g = 0.2
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker_pub.publish(marker)

def rosie_odom_pub():
    global odom_pub, odom_broadcaster, marker_pub
    # Starts a new node
    rospy.init_node('rosie_odom_pub', anonymous=True)
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    marker_pub = rospy.Publisher("base_marker", Marker, queue_size=10)
    odom_broadcaster = tf.TransformBroadcaster()
    rospy.Subscriber('/rosie_odom', RosieOdom, odomCB)
    markerMaker()

    
    rospy.spin()


if __name__ == '__main__':
    try:
        rosie_odom_pub()
    except rospy.ROSInterruptException: pass
