#!/usr/bin/env python3

import rospy
from assignment_2_2023.msg import Vel
from assignment_2_2023.srv import Input, InputResponse

# Define a class for the service
class LastTargetService:
    def __init__(self):
        # Initialize class variables for the last desired x and y positions
        self.lastGX = 0
        self.lastGY = 0

        # Initialize the node with the name 'last_target_service'
        rospy.init_node('last_target_service')
        rospy.loginfo("Last target node initialized")

        # Provide a service named 'input', using the custom service type Input
        rospy.Service('input', Input, self.inputCall)

    # Callback function for the service
    def inputCall(self, _):
        # Create a response message
        response = InputResponse()
        # Set the x and y inputs in the response to the last desired positions
        self.lastGX = rospy.get_param('/des_pos_x')
        self.lastGY = rospy.get_param('/des_pos_y')
        response.inpux = self.lastGX
        response.inpuy = self.lastGY

        # Return the response
        return response

    # Function to keep the node running
    def keepRun(self):
        rospy.spin()

# Main function
if __name__ == "__main__":
    # Create an instance of the service class
    service = LastTargetService()
    # Start the node
    service.keepRun()
