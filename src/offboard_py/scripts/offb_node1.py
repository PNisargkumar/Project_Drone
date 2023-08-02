#! /usr/bin/env python

import rospy
from rosnode import rosnode_ping
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()
pose = PoseStamped()
def state_cb(msg):
    global current_state
    current_state = msg

def changepose(msg):
    global pose
    if rosnode_ping("user", max_count = 3):
        rospy.loginfo("User input")
        pose.pose.position.x = msg.pose.position.x
        pose.pose.position.y = msg.pose.position.y
        pose.pose.position.z = msg.pose.position.z
    else:
        rospy.loginfo("User lost - Waiting for connection...")


if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    user_pos_sub = rospy.Subscriber("user_inp", PoseStamped, callback = changepose)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()


    while(not rospy.is_shutdown()):
        if(current_state.mode == "OFFBOARD" and current_state.armed): 
            rospy.loginfo("Vehicle armed and OFFBOARD enabled")
        else:
            rospy.loginfo("Arm vehicle and enable OFFBOARD")

        local_pos_pub.publish(pose)

        rate.sleep()