#! /usr/bin/env python

"""
.. module: last_target_service

	:platform: Unix
	:synopsis: action_client
.. moduleauthor:: Alireza Tajabadi Farahani

When triggered, this service node furnishes the coordinates of the latest target transmitted by the user.

Subscribes to:
	/reaching_goal/goal
"""

import rospy
import assignment_2_2023.msg
from assignment_2_2023.srv import Last_target, Last_targetResponse

# Global variables to store the coordinates of the most recent target
last_target_x = 0
last_target_y = 0

def clbk_service(request):
    """
    Callback function invoked to provide the coordinates of the most recent target.
    """
    global last_target_x, last_target_y
    print("Last target x-coordinate:", last_target_x)
    print("Last target y-coordinate:", last_target_y)
    print("---------------------------------")
    return Last_targetResponse(last_target_x, last_target_y)

def clbk_goal(msg):
    """
    Callback function to update the coordinates of the most recent target.
    """
    global last_target_x, last_target_y
    # Retrieve coordinates of the goal
    last_target_x = msg.goal.target_pose.pose.position.x
    last_target_y = msg.goal.target_pose.pose.position.y

def main():
    """
    Main function initializes the ROS node and sets up the service server and subscriber.
    """
    rospy.init_node("last_target")
    
    # Create ROS service server to provide the coordinates of the last target
    rospy.Service("last_target", Last_target, clbk_service)
    
    # Create ROS subscriber to update the last target coordinates
    rospy.Subscriber('/reaching_goal/goal', assignment_2_2023.msg.PlanningActionGoal, clbk_goal)
    
    # Keep the node running
    rospy.spin()

if __name__ == "__main__":
    main()
