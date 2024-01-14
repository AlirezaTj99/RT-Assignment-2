#! /usr/bin/env python3

# Import necessary libraries
import rospy  # ROS Python library
from geometry_msgs.msg import Point, Pose, Twist  # ROS geometry message types
from nav_msgs.msg import Odometry  # ROS navigation message types
import actionlib  # ROS actionlib library
import actionlib.msg  # ROS actionlib message types
import assignment_2_2023.msg  # Custom ROS message types
from assignment_2_2023.msg import Vel  # Custom ROS message type
from assignment_2_2023.msg import PlanningAction, PlanningGoal, PlanningResult  # Custom ROS action message types
from std_srvs.srv import SetBool  # ROS standard services
from actionlib_msgs.msg import GoalStatus  # ROS actionlib message type

# Defining the class
class Goal:
    def __init__(self):
        # Initialize publisher and action client
        self.pub = rospy.Publisher("/pos_vel", Vel, queue_size=1)  # ROS publisher for position and velocity
        self.client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)  # ROS action client
        self.client.wait_for_server()  # Wait for the action server to be available
        self.goal_cancelled = True  # Flag to track if the current goal has been cancelled

    def goal(self):
        while not rospy.is_shutdown():
            # Subscribe to /odom topic and publish position and velocity
            rospy.Subscriber("/odom", Odometry, self.pubPosVelocity)  # Subscribe to odometry topic
            # Get user command
            command = input("If you want to set a goal (press '1') or cancel the current process (press '0')?")
            # Get current target position
            gPosx = rospy.get_param('/des_pos_x')  # Get desired X position from parameter server
            gPosy = rospy.get_param('/des_pos_y')  # Get desired Y position from parameter server

            # Create a new goal with the current target position
            goal = assignment_2_2023.msg.PlanningGoal()
            goal.target_pose.pose.position.x = gPosx
            goal.target_pose.pose.position.y = gPosy
            rospy.loginfo("We are going to: [X , Y] = [%f , %f]", gPosx, gPosy)

            if command == '1':
                try:
                    # Get new goal coordinates from the user
                    inpux = float(input("Where do you wanna go? (in X direction) "))
                    inpuy = float(input("Where do you wanna go? (in Y direction) "))
                except ValueError:
                    rospy.logwarn("The input is not acceptable.")
                    continue

                # Update target position parameters and the goal
                rospy.set_param('/des_pos_x', inpux)
                rospy.set_param('/des_pos_y', inpuy)
                goal.target_pose.pose.position.x = inpux
                goal.target_pose.pose.position.y = inpuy
                
                # Send the new goal to the action server
                self.client.send_goal(goal)
                self.goal_cancelled = False

            elif command == '0':
                if not self.goal_cancelled:
                    # Cancel the current goal if there is one
                    self.goal_cancelled = True
                    self.client.cancel_goal()
                    rospy.loginfo("We have cancelled the goal... ")
                else:
                    rospy.loginfo("What you doing? I see no active goal to cancel")
            else:
                rospy.logwarn("Cannot understand your input. You gotta enter '0' or '1'")

            rospy.loginfo("Your last input is: [X , Y] = [%f , %f]", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)

    def pubPosVelocity(self, msg):
        # Extract current position and velocity from the Odometry message
        current_pos = msg.pose.pose.position
        current_vel_linear = msg.twist.twist.linear
        current_vel_angular = msg.twist.twist.angular

        # Create a new Vel message with the current position and velocity
        pos_and_vel = Vel()
        pos_and_vel.pos_x = current_pos.x
        pos_and_vel.pos_y = current_pos.y
        pos_and_vel.Vx = current_vel_linear.x
        pos_and_vel.Wz = current_vel_angular.z

        # Publish the Vel message
        self.pub.publish(pos_and_vel)

def main():
    # Initialize the node and start handling goal commands
    rospy.init_node('set_target_client')  # Initialize ROS node
    handler = Goal()  # Create an instance of the Goal class
    handler.goal()  # Call the goal method

if __name__ == '__main__':
    main()
