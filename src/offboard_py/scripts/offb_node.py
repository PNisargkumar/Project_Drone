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
    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)

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
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    # Create a PoseStamped message to specify the desired position (x, y, z)
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0.5

    # Send a few setpoints before starting to ensure the drone receives them
    for i in range(100):
        if rospy.is_shutdown():
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    # Prepare to switch the drone to "OFFBOARD" mode
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    # Prepare to arm the drone
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    # Main loop to continuously publish the setpoint position and check state
    while not rospy.is_shutdown():
        # Check if the drone is in "OFFBOARD" mode and armed
        if current_state.mode == "OFFBOARD" and current_state.armed:
            rospy.loginfo("Vehicle armed and OFFBOARD enabled")
        else:
            rospy.loginfo("Arm vehicle and enable OFFBOARD")

        # Publish the setpoint position to the drone
        local_pos_pub.publish(pose)

        # Sleep to ensure the setpoint publishing rate is met
        rate.sleep()
