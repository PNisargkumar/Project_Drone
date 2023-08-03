#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 0
exit = False
def takeoff():
    height = eval(input("Take off height: "))
    pose.pose.position.x = pose.pose.position.x
    pose.pose.position.y = pose.pose.position.y
    pose.pose.position.z = height

def land():
    #pose.pose.position.x = pose.pose.position.x
    #pose.pose.position.y = pose.pose.position.y
    pose.pose.position.z = 0

def goto():
    x = eval(input("Enter x position: "))
    y = eval(input("Enter y position: "))
    z = eval(input("Enter z position: "))
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
       
def state_cb(msg):
    global current_state
    current_state = msg
    
if __name__ == "__main__":
    rospy.init_node("user")
    
    user_pos_pub = rospy.Publisher("user_inp", PoseStamped, queue_size = 10)
   
    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    while(not rospy.is_shutdown() and exit==False):
        rospy.loginfo("\nEnter 't' to takeoff \nEnter 'g' to go to setpoint \nEnter 'l' to land \nEnter 'q' to quit\n")
        cmd = input("Command: ")
        if cmd == "t":
            takeoff()
        elif cmd == "g":
            goto()
        elif cmd == "l":
            land()
        elif cmd == "q":
            exit = True
            rospy.loginfo("Vehicle hovering")
        else:
            rospy.loginfo("Enter valid command ")
        user_pos_pub.publish(pose)
        rate.sleep()