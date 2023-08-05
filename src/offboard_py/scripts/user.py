#! /usr/bin/env python

# Import the necessary ROS and MAVROS message types and services
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

# Create a PoseStamped message to store the desired position (setpoint) of the drone
pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 0.5

# Variable to keep track of the program exit condition
exit = False

# Function to set the desired position to takeoff
def takeoff():
    height = eval(input("Takeoff height: "))
    pose.pose.position.z = height

# Function to set the desired position to land
def land():
    pose.pose.position.z = 0

# Function to set the desired position to a specific point (x, y, z)
def goto():
    x = eval(input("Enter x position: "))
    y = eval(input("Enter y position: "))
    z = eval(input("Enter z position: "))
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
       
# Callback function to update the current state of the drone
def state_cb(msg):
    global current_state
    current_state = msg
    
if __name__ == "__main__":
    # Initialize the ROS node with the name "user"
    rospy.init_node("user")
    
    # Create a publisher to send user input as setpoints to the drone
    user_pos_pub = rospy.Publisher("user_inp", PoseStamped, queue_size=10)
   
    # Setpoint publishing rate, which should be faster than 2Hz
    rate = rospy.Rate(20)

    # Main loop for user interaction
    while not rospy.is_shutdown() and not exit:
        # Print menu options for user commands
        rospy.loginfo("\nEnter 't' to takeoff \nEnter 'g' to go to setpoint \nEnter 'l' to land \nEnter 'q' to quit\n")
        cmd = input("Command: ")
        if cmd == "t":
            # Command to takeoff and set the desired height
            takeoff()
        elif cmd == "g":
            # Command to go to a specific setpoint
            goto()
        elif cmd == "l":
            # Command to land and set the desired height to 0
            land()
        elif cmd == "q":
            # Command to quit the program
            exit = True
            rospy.loginfo("Vehicle hovering")
        else:
            rospy.loginfo("Enter valid command ")

        # Publish the current desired setpoint
        user_pos_pub.publish(pose)

        # Sleep to ensure the setpoint publishing rate is met
        rate.sleep()