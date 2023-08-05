#!/usr/bin/env python

# Import the necessary ROS message types and services
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import smbus
import time

# Function to read the distance from the SRF08 ultrasonic range sensor
def read_distance():
    bus = smbus.SMBus(1)
    sensor_address = 0x70  # Use the correct I2C address for the SRF08 sensor
    bus.write_byte_data(sensor_address, 0, 81)  # Command to start a ranging in cm
    time.sleep(0.8)  # Wait for the measurement to be taken (can be adjusted based on sensor specs)
    distance = bus.read_word_data(sensor_address, 2)  # Read the distance value from the sensor
    distance = (distance >> 8) + ((distance & 0xFF) << 8)  # Swap bytes for little-endian format
    return distance

# Main function to read the SRF08 sensor and publish the distance as a pose message
def srf08_reader():
    # Initialize the ROS node with the name 'srf10_reader' and make it anonymous
    rospy.init_node('srf10_reader', anonymous=True)

    # Create a publisher to publish the sensor readings as pose messages to the 'drone/sonic_sensor' topic
    pub_pose = rospy.Publisher('drone/sonic_sensor', Odometry, queue_size=10)

    # Enter the ROS event loop and keep the node running until it is shut down
    while not rospy.is_shutdown():
        # Read the distance from the SRF08 sensor
        distance = read_distance()

        # Create an Odometry message for the sensor reading
        posemsg = Odometry()
        posemsg.header = Header()
        posemsg.header.stamp = rospy.get_rostime()
        posemsg.header.frame_id = "odom"
        posemsg.child_frame_id = "sonic_sensor"
        posemsg.pose.pose.position.z = distance / 100.0  # Convert distance from cm to meters

        # Publish the pose message
        pub_pose.publish(posemsg)

if __name__ == '__main__':
    try:
        # Call the main function 'srf08_reader' and enter the ROS event loop with 'rospy.spin()'
        srf08_reader()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
