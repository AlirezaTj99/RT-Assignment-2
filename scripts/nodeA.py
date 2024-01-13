#! /usr/bin/env python3

# Import necessary libraries
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from assignment_2_2023.msg import Vel
from assignment_2_2023.msg import PlanningAction, PlanningGoal, PlanningResult
from std_srvs.srv import SetBool
from actionlib_msgs.msg import GoalStatus

#defining the class
class GoalHandler:
    def __init__(self):
        # Initialize publisher and action client
        self.pub = rospy.Publisher("/pos_vel", Vel, queue_size=1)
        self.client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
        self.client.wait_for_server()
        self.goal_cancelled = True  # Flag to track if the current goal has been cancelled

    def handle_goal_commands(self):
        while not rospy.is_shutdown():
            # Subscribe to /odom topic and publish position and velocity
            rospy.Subscriber("/odom", Odometry, self.publish_position_velocity)
            # Get user command
            command = input("Press 'y' to set a new goal or 'c' to cancel the current goal: ")
            # Get current target position
            target_pos_x = rospy.get_param('/des_pos_x')
            target_pos_y = rospy.get_param('/des_pos_y')

            # Create a new goal with the current target position
            goal = assignment_2_2023.msg.PlanningGoal()
            goal.target_pose.pose.position.x = target_pos_x
            goal.target_pose.pose.position.y = target_pos_y
            rospy.loginfo("Current goal: target_x = %f, target_y = %f", target_pos_x, target_pos_y)

            if command == 'y':
                try:
                    # Get new goal coordinates from user
                    input_x = float(input("Enter the x-coordinate for the new goal: "))
                    input_y = float(input("Enter the y-coordinate for the new goal: "))
                except ValueError:
                    rospy.logwarn("Invalid input. Please enter a valid number.")
                    continue

                # Update target position parameters and the goal
                rospy.set_param('/des_pos_x', input_x)
                rospy.set_param('/des_pos_y', input_y)
                goal.target_pose.pose.position.x = input_x
                goal.target_pose.pose.position.y = input_y
                
                # Send the new goal to the action server
                self.client.send_goal(goal)
                self.goal_cancelled = False

            elif command == 'c':
                if not self.goal_cancelled:
                    # Cancel the current goal if there is one
                    self.goal_cancelled = True
                    self.client.cancel_goal()
                    rospy.loginfo("Current goal has been cancelled")
                else:
                    rospy.loginfo("No active goal to cancel")
            else:
                rospy.logwarn("Invalid command. Please enter 'y' or 'c'.")

            rospy.loginfo("Last received goal: target_x = %f, target_y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)

    def publish_position_velocity(self, msg):
        # Extract current position and velocity from the Odometry message
        current_pos = msg.pose.pose.position
        current_vel_linear = msg.twist.twist.linear
        current_vel_angular = msg.twist.twist.angular

        # Create a new Vel message with the current position and velocity
        pos_and_vel = Vel()
        pos_and_vel.pos_x = current_pos.x
        pos_and_vel.pos_y = current_pos.y
        pos_and_vel.vel_x = current_vel_linear.x
        pos_and_vel.vel_z = current_vel_angular.z

        # Publish the Vel message
        self.pub.publish(pos_and_vel)

def main():
    # Initialize the node and start handling goal commands
    rospy.init_node('set_target_client')
    handler = GoalHandler()
    handler.handle_goal_commands()

if __name__ == '__main__':
    main()