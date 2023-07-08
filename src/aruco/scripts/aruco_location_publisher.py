#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

tag42 = np.array(((1,0,0,0),
                 (0,1,0,0),
                 (0,0,1,0),
                 (0,0,0,1)))

tag43 = np.array(((1,0,0,0.2),
                 (0,1,0,0),
                 (0,0,1,0),
                 (0,0,0,1)))

tag44 = np.array(((1,0,0,0),
                 (0,1,0,1),
                 (0,0,1,0),
                 (0,0,0,1)))

tag45 = np.array(((1,0,0,1),
                 (0,1,0,1),
                 (0,0,1,0),
                 (0,0,0,1)))

tag46 = np.array(((1,0,0,0),
                 (0,1,0,-0.2),
                 (0,0,1,0),
                 (0,0,0,1)))

aruco_IDs = [42,43,44,45,46]
aruco_loc = [tag42,tag43,tag44,tag45,tag46]
aruco_size = 0.1
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_100)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
bridge = CvBridge()

intrinsic = np.load('/home/ubuntu/Project_drone/src/aruco/scripts/camera_matrix_r.npy')
distortion = np.load('/home/ubuntu/Project_drone/src/aruco/scripts/camera_matrix_r.npy')
#intrinsic = np.load('/home/zeelpatel/Desktop/camera_matrix_l.npy')
#distortion = np.load('/home/zeelpatel/Desktop/dist_coeffs_l.npy')

aruco_Points = np.zeros((4,1,3), dtype=np.float32)
aruco_Points[0][0] = [-aruco_size/2, aruco_size/2,0]
aruco_Points[1][0] = [aruco_size/2, aruco_size/2,0]
aruco_Points[2][0] = [aruco_size/2, -aruco_size/2,0]
aruco_Points[3][0] = [-aruco_size/2, -aruco_size/2,0]

def matrix_to_quaternion(matrix):
    trace = matrix[0, 0] + matrix[1, 1] + matrix[2, 2]
    
    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * S
        qx = (matrix[2, 1] - matrix[1, 2]) / S
        qy = (matrix[0, 2] - matrix[2, 0]) / S
        qz = (matrix[1, 0] - matrix[0, 1]) / S
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        S = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        qw = (matrix[2, 1] - matrix[1, 2]) / S
        qx = 0.25 * S
        qy = (matrix[0, 1] + matrix[1, 0]) / S
        qz = (matrix[0, 2] + matrix[2, 0]) / S
    elif matrix[1, 1] > matrix[2, 2]:
        S = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        qw = (matrix[0, 2] - matrix[2, 0]) / S
        qx = (matrix[0, 1] + matrix[1, 0]) / S
        qy = 0.25 * S
        qz = (matrix[1, 2] + matrix[2, 1]) / S
    else:
        S = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        qw = (matrix[1, 0] - matrix[0, 1]) / S
        qx = (matrix[0, 2] + matrix[2, 0]) / S
        qy = (matrix[1, 2] + matrix[2, 1]) / S
        qz = 0.25 * S

    return np.array([qx, qy, qz, qw])

def hmatrix(rmat, tvec):
    hmat = np.eye(4)
    hmat[:3, :3] = rmat
    hmat[0, 3] = tvec[0]
    hmat[1, 3] = tvec[1]
    hmat[2, 3] = tvec[2]
    return hmat

def combine_poses(camera_pose, object_pose):
    camera_rotation = camera_pose[:3, :3]
    camera_translation = camera_pose[:3, 3] 
    inverse_camera_rotation = camera_rotation.T
    inverse_camera_translation = -inverse_camera_rotation @ camera_translation
    combined_rotation = camera_rotation @ object_pose[:3, :3]
    combined_translation = camera_rotation @ object_pose[:3, 3] + camera_translation
    combined_pose = np.eye(4)
    combined_pose[:3, :3] = combined_rotation
    combined_pose[:3, 3] = combined_translation
    return combined_pose


def image_callback(new_image):
    new_frame = new_frame = bridge.imgmsg_to_cv2(new_image, "mono8")
    marker_corners, marker_IDs, reject = detector.detectMarkers(new_frame)
    if len(marker_corners) > 0:
        for i in range(0, len(marker_IDs)):
            suc, rvec, tvec = cv2.solvePnP(aruco_Points,marker_corners[i],intrinsic,distortion)
            if suc:
                arucoPose = PoseStamped()
                rmat, _ = cv2.Rodrigues(rvec)
                hmat = hmatrix(rmat,tvec)
                aruco_index = aruco_IDs.index(marker_IDs[i])
                position = combine_poses(hmat, aruco_loc[aruco_index])
                print(f"ID:\t {marker_IDs[i]} \n {np.round(position,3)}")
                rquat = matrix_to_quaternion(rmat)
                arucoPose.header = Header()
                arucoPose.header.frame_id = "camera"
                arucoPose.pose.position.x = tvec[0]
                arucoPose.pose.position.y = tvec[1]
                arucoPose.pose.position.z = tvec[2]
                arucoPose.pose.orientation.x = rquat[0]
                arucoPose.pose.orientation.y = rquat[1]
                arucoPose.pose.orientation.z = rquat[2]
                arucoPose.pose.orientation.w = rquat[3]
                aruco_pub.publish(arucoPose)
    print('end loop')
if __name__ == "__main__":
    
    rospy.init_node("aruco_tracker_node")
    aruco_pub = rospy.Publisher("/aruco/pose", PoseStamped, queue_size=10)
    image_sub = rospy.Subscriber("/camera/image", Image, image_callback)
    while not rospy.is_shutdown():
        rospy.spin()