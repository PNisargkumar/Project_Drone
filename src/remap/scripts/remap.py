#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def remapping(odo):

    drone_pose = PoseStamped()
    drone_pose.header = Header()
    drone_pose.header.stamp = rospy.get_rostime()
    drone_pose.header.frame_id = "base_link"
    drone_pose.pose.position.x = odo.pose.pose.position.y
    drone_pose.pose.position.y = -odo.pose.pose.position.x
    drone_pose.pose.position.z = odo.pose.pose.position.z
    drone_pose.pose.orientation.x = odo.pose.pose.orientation.x
    drone_pose.pose.orientation.y = odo.pose.pose.orientation.y
    drone_pose.pose.orientation.z = odo.pose.pose.orientation.z
    drone_pose.pose.orientation.w = odo.pose.pose.orientation.w
    pos_pub.publish(drone_pose)

    
if __name__ == "__main__":
    rospy.init_node("remap_node")

    pose_sub = rospy.Subscriber("odometry/filtered", Odometry, callback = remapping)
    pos_pub = rospy.Publisher("mavros/vision_pose/pose", PoseStamped, queue_size=10)
    while not rospy.is_shutdown():
        rospy.spin()

