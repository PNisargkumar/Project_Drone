#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import Imu

prev_time = None

def calculate_inertial_odometry(msg):
    global prev_time, prev_vel, prev_pos
    current_time = rospy.Time.now()

    if prev_time is None:
        prev_time = current_time
        return None

    dt = (current_time - prev_time).to_sec()

    # Get orientation and angular velocity from IMU
    orientation = msg.orientation
    current_quat = [orientation.x, orientation.y, orientation.z, orientation.w]
    angular_vel = msg.angular_velocity

    # Extract the linear acceleration from the IMU msg
    linear_accel = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
    # Calculate current position and velocity using inertial navigation equations
    current_pos = [prev_pos[i] + prev_vel[i] * dt + 0.5 * linear_accel[i] * dt ** 2 for i in range(3)]
    current_vel = [prev_vel[i] + linear_accel[i] * dt for i in range(3)]

    # Update the previous values
    prev_time = current_time
    prev_vel = current_vel
    prev_pos = current_pos
    # Create a PoseStamped message to represent the odometry
    odometry = PoseStamped()
    odometry.header = Header()
    odometry.header.stamp = current_time
    odometry.header.frame_id = "base_link"
    odometry.pose.position.x = current_pos[0]
    odometry.pose.position.y = current_pos[1]
    odometry.pose.position.z = current_pos[2]
    # Set the orientation if available
    odometry.pose.orientation = orientation
    return odometry

def imu_callback(msg):
    inertial_odometry = calculate_inertial_odometry(msg)
    if inertial_odometry is not None:
        # Publish the inertial odometry to a ROS topic
        inertial_odom_pub.publish(inertial_odometry)

if __name__ == '__main__':
    rospy.init_node('inertial_odometry_node')
    prev_time = rospy.Time.now()
    prev_pos = [0.0, 0.0, 0.0]
    prev_vel = [0.0, 0.0, 0.0]
    # Assuming you have subscribed to the IMU topic
    imu_sub = rospy.Subscriber('/mavros/imu/data_raw', Imu, imu_callback)

    # Assuming you have created a publisher for the inertial odometry
    inertial_odom_pub = rospy.Publisher('/inertial_odom', PoseStamped, queue_size=10)

    rate = rospy.Rate(20)  # 10 Hz update rate (adjust as needed)

    while not rospy.is_shutdown():
        rate.sleep()