#! /usr/bin/env python

# Import the necessary ROS and MAVROS message types and services
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

# Create a global variable to store the current state of the drone
current_state = State()

# Callback function to update the current state of the drone
def state_cb(msg):
    global current_state
    current_state = msg

# Main function
if __name__ == "__main__":
    # Initialize the ROS node with the name "offb_node_py"
    rospy.init_node("offb_node_py")

    # Subscribe to the "mavros/state" topic to get the current state of the drone
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    # Create a publisher to send local position setpoints to the drone
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    # Wait for the "mavros/cmd/arming" service to become available
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    # Wait for the "mavros/set_mode" service to become available
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing rate, which should be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for the Flight Controller connection before proceeding
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    # Create a PoseStamped message to specify the desired position the drone should reach after switching to OFFBOARD Mode (x, y, z)
    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0.5

    # Send a few setpoints before starting to ensure the drone receives them
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    # Prepare to switch the drone to "OFFBOARD" mode
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    # Prepare to arm the drone
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    # If the current mode is not "OFFBOARD", try to switch it
    if(current_state.mode != "OFFBOARD"):
        if(set_mode_client.call(offb_set_mode).mode_sent == True):
            rospy.loginfo("OFFBOARD enabled")

    # If the drone is not armed, try to arm it
    if(not current_state.armed):
        if(arming_client.call(arm_cmd).success == True):
            rospy.loginfo("Vehicle armed")

    # Main loop to continuously publish the setpoint position
    while(not rospy.is_shutdown()):
        local_pos_pub.publish(pose)
        rate.sleep()