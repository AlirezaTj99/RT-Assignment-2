#!/usr/bin/env python3

import rospy  # ROS Python library
import math  # Python math library
from assignment_2_2023.msg import Vel  # Custom ROS message type
from assignment_2_2023.srv import Ave_pos_vel, Ave_pos_velResponse  # Custom ROS service types

# Define a class for the service
class InfoService:
    def __init__(self):
        # Initialize class variables for the average velocity and distance
        self.averageVx = 0
        self.distance = 0

        # Initialize the node with the name 'info_service'
        rospy.init_node('info_service')  # Initialize ROS node
        rospy.loginfo("Info service node initialized")  # Log node initialization

        # Provide a service named 'info_service', using the custom service type Ave_pos_vel
        rospy.Service("info_service", Ave_pos_vel, self.get_values)  # Provide service for retrieving info
        # Subscribe to the '/pos_vel' topic, using the custom message type Vel
        rospy.Subscriber("/pos_vel", Vel, self.dAndAverageVCalculator)  # Subscribe to position and velocity topic

    # Callback function for the subscriber
    def dAndAverageVCalculator(self, msg):
        try:
            GX = rospy.get_param('/des_pos_x')  # Get desired X position from parameter server
            GY = rospy.get_param('/des_pos_y')  # Get desired Y position from parameter server

            velocity_window_size = rospy.get_param('/window_size')  # Get window size for velocity calculation
        
        except rospy.ROSException as e:
            rospy.logerr(f"{rospy.get_name()}: Parameter retrieval failed: {e}")
            return
        
        # Get the actual x and y positions from the message
        x, y = msg.pos_x, msg.pos_y

        # Calculate the distance between the desired and actual positions
        Gcoordinates = [GX, GY]
        coordinates = [x, y]
        self.distance = math.dist(Gcoordinates, coordinates)

        # Calculate the average velocity
        if isinstance(msg.Vx, list):
            vel_data = msg.Vx[-velocity_window_size:]
        else:
            vel_data = [msg.Vx]

        self.averageVx = sum(vel_data) / min(len(vel_data), velocity_window_size)

    # Callback function for the service
    def get_values(self, _):      
        # Return a response with the distance and average velocity
        return Ave_pos_velResponse(self.distance, self.averageVx)		      

    # Function to keep the node running
    def keepRun(self):
        rospy.spin()  # Keep the node running using rospy.spin()

# Main function
if __name__ == "__main__":
    # Create an instance of the service class
    service = InfoService()  # Create an instance of the InfoService class
    dist_vel_service = rospy.ServiceProxy('info_service', Ave_pos_vel)  # ROS ServiceProxy for calling the service

    while not rospy.is_shutdown():
        # Call the service
        response = dist_vel_service()
        rospy.loginfo(f"Service response:\n {response}")

    # Start the node
    service.keepRun()  # Keep the node running using rospy.spin()
