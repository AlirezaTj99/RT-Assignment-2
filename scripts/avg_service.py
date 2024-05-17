#! /usr/bin/env python

"""
.. module: avg_service

	:platform: Unix
	:synopsis: action_client
.. moduleauthor:: Alireza Tajabadi Farahani

This service node listens to updates about the robot's position and velocity, which are transmitted using a specialized message format named Info.
It also operates a server that computes and delivers two essential details: firstly, the distance between the robot and its target, and secondly, the average speed of the robot.
Subscribes to:
	/robot_pos_vel
"""

import rospy
import math
from assignment_2_2023.srv import Avg_dist_vel, Avg_dist_velResponse
from assignment_2_2023.msg import Info

# Global variables to store mean velocities and distance
mean_vel_x = 0
mean_vel_z = 0
dist = 0

def clbk_info(msg):
    """
    Callback function to process robot's position and velocity data.
    """
    global mean_vel_x, mean_vel_z, dist
    linear_vel_x = []
    angular_vel_z = []
    window_size = rospy.get_param("window_size")
    x_robot = msg.x
    y_robot = msg.y
    linear_vel_x.append(msg.vel_x)
    angular_vel_z.append(msg.vel_z)
    x_target = rospy.get_param("des_pos_x")
    y_target = rospy.get_param("des_pos_y")
    instant_vel_x = linear_vel_x[-window_size:]
    instant_vel_z = angular_vel_z[-window_size:]
    mean_vel_x = sum(instant_vel_x) / len(instant_vel_x)
    mean_vel_z = sum(instant_vel_z) / len(instant_vel_z)
    dist = math.sqrt((x_target - x_robot)**2 + (y_target - y_robot)**2)

def clbk_avg(request):
    """
    Callback function to return distance and average velocity information.
    """
    global mean_vel_x, mean_vel_z, dist
    dist = round(dist, 3)
    mean_vel_x = round(mean_vel_x, 3)
    mean_vel_z = round(mean_vel_z, 3)
    print(f"Distance between robot and target: {dist} m")
    print(f"Average linear velocity along x-axis: {mean_vel_x} m/s")
    print(f"Average angular velocity around z-axis: {mean_vel_z} rad/s")
    print("----------------------------------------------------------------")
    return Avg_dist_velResponse(dist, mean_vel_x, mean_vel_z)

def main():
    """
    Main function to initialize ROS node, service server, and subscriber.
    """
    rospy.init_node("avg_dist_vel")
    rospy.Service("avg_dist_vel", Avg_dist_vel, clbk_avg)
    rospy.Subscriber('/robot_pos_vel', Info, clbk_info)
    rospy.spin()

if __name__ == "__main__":
    main()