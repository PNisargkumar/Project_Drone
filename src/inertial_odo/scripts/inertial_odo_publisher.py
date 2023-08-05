#!/usr/bin/env python

# Import necessary ROS libraries and messages
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import Imu

# Initialize global variables to keep track of time, position, and velocity
prev_time = None
prev_pos = [0.0, 0.0, 0.0]
prev_vel = [0.0, 0.0, 0.0]

# Function to perform quaternion multiplication
def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return [w, x, y, z]

# Function to calculate inertial odometry
def calculate_inertial_odometry(msg):
    global prev_time, prev_vel, prev_pos
    current_time = rospy.Time.now()

    if prev_time is None:
        prev_time = current_time
        return None

    dt = (current_time - prev_time).to_sec()

    # Get orientation and angular velocity from IMU
    current_quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    current_quat_conj = [-msg.orientation.x, -msg.orientation.y, -msg.orientation.z, msg.orientation.w]
    angular_vel = msg.angular_velocity

    # Extract the linear acceleration from the IMU msg
    linear_accel = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]

    # Compensate for gravity based on the orientation
    g = [0, 0, 9.81, 0]
    gravity_compensation = quaternion_multiply(quaternion_multiply(current_quat, g), current_quat_conj)

    linear_accel = [linear_accel[i] - gravity_compensation[i] for i in range(3)]

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
    odometry.pose.orientation.x = current_quat[0]
    odometry.pose.orientation.y = current_quat[1]
    odometry.pose.orientation.z = current_quat[2]
    odometry.pose.orientation.w = current_quat[3]
    return odometry

# Callback function for the IMU data
def imu_callback(msg):
    inertial_odometry = calculate_inertial_odometry(msg)
    if inertial_odometry is not None:
        # Publish the inertial odometry to a ROS topic
        inertial_odom_pub.publish(inertial_odometry)

# Main entry point of the script
if __name__ == '__main__':
    # Initialize the ROS node with the name "inertial_odometry_node"
    rospy.init_node('inertial_odometry_node')

    # Get the current time and initialize previous time, position, and velocity
    prev_time = rospy.Time.now()
    prev_pos = [0.0, 0.0, 0.0]
    prev_vel = [0.0, 0.0, 0.0]

    # Subscribe to the IMU topic ("/mavros/imu/data") and set the callback function to imu_callback
    imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)

    # Create a publisher to publish the inertial odometry on the topic "/inertial_odom"
    inertial_odom_pub = rospy.Publisher('/inertial_odom', PoseStamped, queue_size=10)

    rate = rospy.Rate(20)  # 20 Hz update rate (adjust as needed)

    # Enter the ROS spin loop to keep the node alive and continuously process incoming data
    rospy.spin()
