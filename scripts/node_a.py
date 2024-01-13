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
class Goal:
    def __init__(self):
        # Initialize publisher and action client
        self.pub = rospy.Publisher("/pos_vel", Vel, queue_size=1)
        self.client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
        self.client.wait_for_server()
        self.goal_cancelled = True  # Flag to track if the current goal has been cancelled

    def goal(self):
        while not rospy.is_shutdown():
            # Subscribe to /odom topic and publish position and velocity
            rospy.Subscriber("/odom", Odometry, self.pubPosVelocity)
            # Get user command
            command = input("If you want to set a goal (press '1') or cancel the current process (press '0')?")
            # Get current target position
            gPosx = rospy.get_param('/des_pos_x')
            gPosy = rospy.get_param('/des_pos_y')

            # Create a new goal with the current target position
            goal = assignment_2_2023.msg.PlanningGoal()
            goal.target_pose.pose.position.x = gPosx
            goal.target_pose.pose.position.y = gPosy
            rospy.loginfo("We are going to: [X , Y] = [%f , %f]", gPosx, gPosy)

            if command == '1':
                try:
                    # Get new goal coordinates from user
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
                    rospy.loginfo("What you doing? I see no active goal to cancell")
            else:
                rospy.logwarn("Can not understand your input. You gotta enter '0' or '1'")

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
    rospy.init_node('set_target_client')
    handler = Goal()
    handler.goal()

if __name__ == '__main__':
    main()
