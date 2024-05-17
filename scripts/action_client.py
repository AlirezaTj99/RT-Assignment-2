#! /usr/bin/env python

"""
.. module: action_client

	:platform: Unix
	:synopsis: action_client
.. moduleauthor:: Alireza Tajabadi Farahani

This node allows user to set the target or cancel the target.

Subscribes to:
	/odom

Publishes to:
	/robot_pos_vel
"""

import rospy
import time
import select
import sys
import actionlib
import actionlib.msg
from std_srvs.srv import *
from nav_msgs.msg import Odometry
from assignment_2_2023.msg import Info, PlanningAction, PlanningGoal
from geometry_msgs.msg import Point

# Global variables
pub = None
sub = None

def clbk_odom(msg):
    """
    Callback function to process robot's position and velocity data.
    """
    new_info = Info()
    new_info.x = msg.pose.pose.position.x
    new_info.y = msg.pose.pose.position.y
    new_info.vel_x = msg.twist.twist.linear.x
    new_info.vel_z = msg.twist.twist.angular.z
    pub.publish(new_info)

def clbk_feedback(feedback):
    """
    Callback function to process feedback received from the action server.
    """
    if feedback.stat == "Target reached!":
        print(feedback)
        print("Press 'Enter' to set a new goal")
    elif feedback.stat == "Target cancelled!":
        print(feedback)

def action():
    """
    Function to handle user's goal input and send it to the action server.
    """
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    client.wait_for_server()

    while not rospy.is_shutdown():
        time.sleep(0.5)
        print("Set the goal coordinates")

        try:
            x = float(input("Enter x coordinate: "))
            y = float(input("Enter y coordinate: "))
            if -9 <= x <= 9 and -9 <= y <= 9:
                print(f"Coordinates set: (x={x}, y={y})")
            else:
                print("Invalid input. Please enter x and y coordinates in the range -9 to 9.")
                continue
        except ValueError:
            print("Invalid input. Please try again")
            continue

        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        client.send_goal(goal, None, None, clbk_feedback)

        while not client.get_result():
            print("Press 'c' to cancel the goal!")
            cancel = select.select([sys.stdin], [], [], 0.1)
            if cancel:
                user_input = sys.stdin.readline().strip()
                if user_input == 'c':
                    client.cancel_goal()
                    break

def main():
    """
    Main function to initialize ROS node, publisher, subscriber, and start action.
    """
    global pub, sub
    rospy.init_node('action_client')
    pub = rospy.Publisher('/robot_pos_vel', Info, queue_size=10)
    sub = rospy.Subscriber('/odom', Odometry, clbk_odom)
    action()

if __name__ == "__main__":
    main()
