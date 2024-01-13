#!/usr/bin/env python3

import rospy
import math
from assignment_2_2023.msg import Vel
from assignment_2_2023.srv import AvePosVel, AvePosVelResponse


# Define a class for the service
class InfoService:
    def __init__(self):
        # Initialize class variables for the average velocity and distance
        self.averageVx = 0
        self.distance = 0

        # Initialize the node with the name 'info_service'
        rospy.init_node('info_service')
        rospy.loginfo("Info service node initialized")

        # Provide a service named 'info_service', using the custom service type AvePosVel
        rospy.Service("info_service", AvePosVel, self.get_values)
        # Subscribe to the '/pos_vel' topic, using the custom message type Vel
        rospy.Subscriber("/pos_vel", Vel, self.dandAverageVCalculator)

    # Callback function for the subscriber
    def dandAverageVCalculator(self, msg):
        # Get the desired x and y positions from the parameter server
        GX = rospy.get_param('/des_pos_x')
        GY = rospy.get_param('/des_pos_y')

        # Get the window size for the velocity calculation from the parameter server
        velocity_window_size = rospy.get_param('/window_size')
        
        # Get the actual x and y positions from the message
        x = msg.pos_x
        y = msg.pos_y
        
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
        return AvePosVelResponse(self.distance, self.average_vel_x)		      

    # Function to keep the node running
    def spin(self):
        rospy.spin()

# Main function
if __name__ == "__main__":
    # Create an instance of the service class
    service = InfoService()
    dist_vel_service = rospy.ServiceProxy('info_service', AvePosVel)

    while not rospy.is_shutdown():
            # Call the service
            response = dist_vel_service()
 
            rospy.loginfo(f"Service response:\n {response}")
 

    # Start the node
    service.spin()