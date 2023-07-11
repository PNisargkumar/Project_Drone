#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped
import smbus
import time

def read_distance():
    bus = smbus.SMBus(1)
    sensor_address = 0x70  # Use the correct I2C address
    bus.write_byte_data(sensor_address, 0, 81)  # Command to start a ranging in cm
    time.sleep(0.08)  # Wait for measurement to be taken
    distance = bus.read_word_data(sensor_address, 2)  # Read the distance value
    distance = (distance >> 8) + ((distance & 0xFF) << 8)  # Swap bytes for little endian
    return distance

def srf10_reader():
    rospy.init_node('srf10_reader', anonymous=True)
    pub_range = rospy.Publisher('mavros/px4flow/ground_distance', Range, queue_size=10)
    pub_pose = rospy.Publisher('sonic_sensor', PoseWithCovarianceStamped, queue_size=10)
    while not rospy.is_shutdown():
        distance = read_distance()
        #rospy.loginfo("Distance: {} cm".format(distance))
        range_msg = Range()
        range_msg.header = Header()
        range_msg.header.stamp = rospy.Time.now()
        range_msg.header.frame_id = "sonic_sensor"
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.523599
        range_msg.min_range = 0.003
        range_msg.max_range = 6.0
        range_msg.range = distance / 100.0  # Convert distance from cm to meters
        pub_range.publish(range_msg)
        posemsg = PoseWithCovarianceStamped()
        posemsg.header = Header()
        posemsg.header.stamp = rospy.get_rostime()
        posemsg.header.frame_id = "sonic_sensor"
        posemsg.pose.pose.position.z = distance / 100
        pub_pose.publish(posemsg)

if __name__ == '__main__':
    try:
        srf10_reader()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass