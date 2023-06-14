#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import time
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image 
from mavros_msgs.msg import OpticalFlowRad
from std_msgs.msg import Header 

lk_params = dict(winSize  = (15, 15),
                maxLevel = 2,
                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

feature_params = dict(maxCorners = 20,
                    qualityLevel = 0.3,
                    minDistance = 10,
                    blockSize = 7 )


trajectory_len = 5
detect_interval = 1
trajectories = []
frame_idx = 0
bridge = CvBridge()
new_frame = None
old_frame = None
opticalflow = OpticalFlowRad()
intrinsic = np.array([[629.39855957, 0, 329.88459537], [0, 627.90997314, 229.05112011], [0, 0, 1]])

def image_callback(new_image):
    start = time.time()
    global bridge
    global new_frame
    global old_frame
    global trajectories
    global frame_idx
    global prev_time
    global opticalflow
    global intrinsic
    new_frame = bridge.imgmsg_to_cv2(new_image, "mono8")
    mean_flow_x = 0
    mean_flow_y = 0
    count = 0

    if len(trajectories) > 0:
        p0 = np.float32([trajectory[-1] for trajectory in trajectories]).reshape(-1, 1, 2)
        p1, _st, _err = cv2.calcOpticalFlowPyrLK(old_frame, new_frame, p0, None, **lk_params)
        p0r, _st, _err = cv2.calcOpticalFlowPyrLK(new_frame, old_frame, p1, None, **lk_params)
        d = abs(p0-p0r).reshape(-1, 2).max(-1)
        good = d < 1

        new_trajectories = []

        # Get all the trajectories
        for trajectory, (x, y), good_flag in zip(trajectories, p1.reshape(-1, 2), good):
            if not good_flag:
                continue
            trajectory.append((x, y))
            if len(trajectory) > trajectory_len:
                del trajectory[0]
            new_trajectories.append(trajectory)
        # Calculate velocity in x and y directions
            if len(trajectory) > 2:
                prev_x, prev_y = trajectory[1]
                curr_x, curr_y = trajectory[0]
                mean_flow_x += (curr_x - prev_x)
                mean_flow_y += (curr_y - prev_y)
                count += 1
        if len(trajectories[0])>2 and len(trajectories) > 25:
            try:
                dt = time.time() - prev_time
                prev_time = time.time()
                mean_flow_x = mean_flow_x / (count)
                mean_flow_y = mean_flow_y / (count)
                opticalflow.integration_time_us = int(dt*1e+6)
                opticalflow.header = Header()
                opticalflow.integrated_x = math.atan2(mean_flow_x, intrinsic[0,0])
                opticalflow.integrated_y = math.atan2(mean_flow_y, intrinsic[1,1])
                optflow_pub.publish(opticalflow)

            except:
                pass
        trajectories = new_trajectories
    mask = np.zeros_like(new_frame)
    mask[:] = 255
    

        # Detect the good features to track
    p = cv2.goodFeaturesToTrack(new_frame, mask = mask, **feature_params)
    if p is not None:
            # If good features can be tracked - add that to the trajectories
        for x, y in np.float32(p).reshape(-1, 2):
            trajectories.append([(x, y)])
    if frame_idx == 0:
        prev_time = time.time()

    frame_idx += 1
    old_frame = new_frame
    

if __name__ == "__main__":
    #intrinsic = np.load('/home/zeelpatel/Desktop/intrinsicNew.npy')
    rospy.init_node("optical_flow_node")
    optflow_pub = rospy.Publisher("/mavros/px4flow/raw/optical_flow_rad", OpticalFlowRad, queue_size=10)
    image_sub = rospy.Subscriber("/camera/image", Image, image_callback)
    while not rospy.is_shutdown():
        rospy.spin()