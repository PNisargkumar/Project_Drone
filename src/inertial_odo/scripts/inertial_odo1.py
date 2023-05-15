#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

odom_msg = Odometry()
def imu_callback(msg):
    global odom_msg
    global prev_pos
    global prev_vel
    global prev_quat
    # Calculate time difference since last measurement
    current_time = rospy.Time.now()
    dt = (current_time - prev_time).to_sec()

    # Get orientation and angular velocity from IMU
    orientation = msg.orientation
    quat = [orientation.x, orientation.y, orientation.z, orientation.w]
    angular_vel = msg.angular_velocity

    # Convert quaternion to Euler angles
    roll, pitch, yaw = euler_from_quaternion(quat)

    # Calculate linear acceleration
    linear_accel = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]

    # Calculate current position and velocity using inertial navigation equations
    current_pos = [prev_pos[i] + prev_vel[i] * dt + 0.5 * linear_accel[i] * dt ** 2 for i in range(3)]
    current_vel = [prev_vel[i] + linear_accel[i] * dt for i in range(3)]

    # Create Odometry message and publish it
    odom_msg.header.stamp = current_time
    odom_msg.header.frame_id = 'odom'
    odom_msg.child_frame_id = 'base_link'
    odom_msg.pose.pose.position.x = current_pos[0]
    odom_msg.pose.pose.position.y = current_pos[1]
    odom_msg.pose.pose.position.z = current_pos[2]
    odom_msg.pose.pose.orientation.x = quat[0]
    odom_msg.pose.pose.orientation.y = quat[1]
    odom_msg.pose.pose.orientation.z = quat[2]
    odom_msg.pose.pose.orientation.w = quat[3]
    odom_msg.twist.twist.linear.x = current_vel[0]
    odom_msg.twist.twist.linear.y = current_vel[1]
    odom_msg.twist.twist.linear.z = current_vel[2]
    odom_msg.twist.twist.angular.x = angular_vel.x
    odom_msg.twist.twist.angular.y = angular_vel.y
    odom_msg.twist.twist.angular.z = angular_vel.z
    odom_pub.publish(odom_msg)

    # Update variables for next iteration
    prev_time = current_time
    prev_pos = [0.0, 0.0, 0.0]
    prev_vel = [0.0, 0.0, 0.0]
    prev_quat = [0.0, 0.0, 0.0, 0.0]

    
        
if __name__ == '__main__':
    try:
            # Initialize variables for odometry calculation
        prev_time = rospy.Time.now()
        prev_pos = [0.0, 0.0, 0.0]
        prev_vel = [0.0, 0.0, 0.0]
        prev_quat = [0.0, 0.0, 0.0, 0.0]
        # Initialize ROS node and publishers/subscribers
        rospy.init_node('inertial_odometry_node')
        odom_pub = rospy.Publisher('inertial_odometry', Odometry, queue_size=10)
        imu_sub = rospy.Subscriber('mavros/imu/data/raw', Imu, imu_callback)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
