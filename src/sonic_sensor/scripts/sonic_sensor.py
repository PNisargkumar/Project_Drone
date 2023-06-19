#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range

import smbus
import time

def read_distance():
    bus = smbus.SMBus(1)
    sensor_address = 0x72  # Use the correct I2C address for your setup
    bus.write_byte_data(sensor_address, 0, 81)  # Command to start a ranging in cm
    time.sleep(0.08)  # Wait for measurement to be taken
    distance = bus.read_word_data(sensor_address, 2)  # Read the distance value
    distance = (distance >> 8) + ((distance & 0xFF) << 8)  # Swap bytes for little endian
    return distance

def srf10_reader():
    rospy.init_node('srf10_reader', anonymous=True)
    pub = rospy.Publisher('mavros/px4flow/ground_distance', Range, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        distance = read_distance()
        rospy.loginfo("Distance: {} cm".format(distance))

        range_msg = Range()
        range_msg.header.stamp = rospy.Time.now()
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.1
        range_msg.min_range = 0.006
        range_msg.max_range = 2.0
        range_msg.range = distance / 100.0  # Convert distance from cm to meters

        pub.publish(range_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        srf10_reader()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass