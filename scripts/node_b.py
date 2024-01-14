#!/usr/bin/env python3

import rospy  # ROS Python library
from assignment_2_2023.msg import Vel  # Custom ROS message type
from assignment_2_2023.srv import Input, InputResponse  # Custom ROS service types

# Define a class for the service
class LastTargetService:
    def __init__(self):
        # Initialize class variables for the last desired x and y positions
        self.lastGX = 0
        self.lastGY = 0

        # Initialize the node with the name 'last_target_service'
        rospy.init_node('last_target_service')  # Initialize ROS node
        rospy.loginfo("Last target node initialized")  # Log node initialization

        # Provide a service named 'input', using the custom service type Input
        rospy.Service('input', Input, self.inputCall)  # Provide service for receiving input

    # Callback function for the service
    def inputCall(self, _):
        # Create a response message
        response = InputResponse()
        
        try:
            # Set the x and y inputs in the response to the last desired positions
            self.lastGX = rospy.get_param('/des_pos_x')  # Get desired X position from parameter server
            self.lastGY = rospy.get_param('/des_pos_y')  # Get desired Y position from parameter server
        except rospy.ROSException as e:
            rospy.logerr(f"Error getting desired positions: {e}")
            # Handle the error appropriately
            
        response.inpux = self.lastGX
        response.inpuy = self.lastGY

        # Return the response
        return response

    # Function to keep the node running
    def keepRun(self):
        rospy.spin()  # Keep the node running

# Main function
if __name__ == "__main__":
    # Create an instance of the service class
    service = LastTargetService()  # Create an instance of the LastTargetService class
    # Start the node
    service.keepRun()  # Keep the node running using rospy.spin()
