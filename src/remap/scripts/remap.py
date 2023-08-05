#!/usr/bin/env python

# Import the necessary ROS message types and services
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

# Callback function for remapping the odometry data
def remapping(odo):
    # Create a PoseStamped message for the drone's new pose in the remapped coordinate system
    drone_pose = PoseStamped()
    drone_pose.header = Header()
    drone_pose.header.stamp = rospy.get_rostime()
    drone_pose.header.frame_id = "base_link"

    # Perform the remapping
    drone_pose.pose.position.x = odo.pose.pose.position.y
    drone_pose.pose.position.y = -odo.pose.pose.position.x
    drone_pose.pose.position.z = odo.pose.pose.position.z
    drone_pose.pose.orientation.x = odo.pose.pose.orientation.x
    drone_pose.pose.orientation.y = odo.pose.pose.orientation.y
    drone_pose.pose.orientation.z = odo.pose.pose.orientation.z
    drone_pose.pose.orientation.w = odo.pose.pose.orientation.w

    # Publish the remapped pose
    pos_pub.publish(drone_pose)
    
if __name__ == "__main__":
    # Initialize the ROS node with the name "remap_node"
    rospy.init_node("remap_node")

    # Subscribe to the "odometry/filtered" topic to get odometry data
    pose_sub = rospy.Subscriber("odometry/filtered", Odometry, callback=remapping)

    # Create a publisher to publish the remapped pose to the "mavros/vision_pose/pose" topic
    pos_pub = rospy.Publisher("mavros/vision_pose/pose", PoseStamped, queue_size=10)

    # Enter the ROS event loop and keep the node running until it is shut down
    while not rospy.is_shutdown():
        # Spin to receive messages and execute the callback
        rospy.spin()


