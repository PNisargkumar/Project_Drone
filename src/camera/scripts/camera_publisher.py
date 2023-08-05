#!/usr/bin/env python

# Import necessary ROS and OpenCV libraries
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Create a VideoCapture object to access the camera
cap = cv2.VideoCapture(0)
# Optionally, configure camera properties (e.g., autofocus, frame width, frame height) - currently commented out
#cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Check if the camera is successfully opened
if not cap.isOpened():
    rospy.loginfo("Could not open Camera")
else:
    rospy.loginfo("Camera Connected")

# Create a CvBridge object to convert between OpenCV images and ROS Image messages
bridge = CvBridge()

# Function to publish camera frames as ROS Image messages
def img_publisher():
    # Create a publisher to publish images on the "/drone/camera/image" topic
    pub = rospy.Publisher('/drone/camera/image', Image, queue_size=1)
    # Initialize the ROS node with the name "my_camera"
    rospy.init_node('my_camera')
    # Log the camera resolution (width x height)
    rospy.loginfo("camera resolution: "+ str(cap.get(3)) + "x"+ str(cap.get(4)))
    # Continuously publish frames until the node is shut down
    while not rospy.is_shutdown():
        # Read a frame from the camera
        ret, frame = cap.read()
        # Check if the frame was successfully read
        if not ret:
            break
        # Convert the OpenCV image to a ROS Image message
        msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        # Publish the ROS Image message
        pub.publish(msg)
    
    # Release the camera when the node is shutting down
    cap.release()

# Main entry point of the script
if __name__ == "__main__":
    # Call the img_publisher function to start publishing camera frame
    try:
        img_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("Unexpected error")
        pass
