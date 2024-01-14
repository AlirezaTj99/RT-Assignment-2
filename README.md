# RT-Assignment-2
In this assignment our task is to create 3 programmes named as `Node A`, `Node B`, `Node C` the task and a brief over view will be written down here.

# Node A: `node_a.py`
 A node that implements an action client, allowing the user to set a target (x, y) or to cancel it.

## Overview
This is a ROS (Robot Operating System) Python script (`node_a.py`) designed to interact with a robot's navigation system. It subscribes to the `/odom` topic to receive odometry information, and it allows the user to set a goal position for the robot using the `reaching_goal` action server. Additionally, it provides an option to cancel the current goal. The node continuously prompts the user for input and sends the goal to the action server when requested.

## Dependencies
- ROS (Melodic or later)
- Python 3
- `geometry_msgs`, `nav_msgs`, `actionlib`, `std_srvs` ROS packages

## Usage
1. Ensure that your ROS workspace is properly sourced.
2. Run the ROS master node (if not already running): `roscore`.
3. Execute the node: `rosrun assignment_2_2023 node_a.py`.
4. Follow the on-screen prompts to set or cancel goals for the robot.

# Node B: `node_b.py`
A service node that, when called, returnsthe coordinates of the last target sent by the user

## Overview
This is a ROS Python script (`node_b.py`) serving as a ROS service that provides information about the last desired position set by the user in `node_a.py`. It responds to service requests by returning the last set X and Y coordinates.

## Dependencies
- ROS (Melodic or later)
- Python 3
- `assignment_2_2023` ROS package (containing the custom message `Vel` and service `Input`)

## Usage
1. Ensure that your ROS workspace is properly sourced.
2. Run the ROS master node (if not already running): `roscore`.
3. Execute the node: `rosrun assignment_2_2023 node_b.py`.
4. Use the provided service `input` to get the last desired position: `rosservice call /input`.

# Node C: `node_c.py`
Anotherservice node thatsubscribes to the robot’s position and velocity (using the custom message) and implements a server to retrieve the distance of the robot from the target and the robot’s average speed.

## Overview
This is a ROS Python script (`node_c.py`) serving as a ROS service similar to `node_b.py`, providing information about the last desired position set by the user in `node_a.py`. It responds to service requests by returning the last set X and Y coordinates.

## Dependencies
- ROS (Melodic or later)
- Python 3
- `assignment_2_2023` ROS package (containing the custom message `Vel` and service `Input`)

## Usage
1. Ensure that your ROS workspace is properly sourced.
2. Run the ROS master node (if not already running): `roscore`.
3. Execute the node: `rosrun assignment_2_2023 node_c.py`.
4. Use the provided service `input` to get the last desired position: `rosservice call /input`.

## Custom Messages

### Vel.msg

This message type defines the structure for publishing the robot's position and velocity.

- `float64 pos_x`: X position of the robot.
- `float64 pos_y`: Y position of the robot.
- `float64 Vx`: Linear velocity of the robot.
- `float64 Wz`: Angular velocity of the robot.

---

## Custom Services

### Ave_pos_vel.srv

This service type defines the structure for calculating the average velocity given a distance.

- `float64 distance`: The distance traveled by the robot.
- `float64 average_velocity`: The calculated average velocity.

### Input.srv

This service type defines the structure for providing the last desired X and Y positions.

- `float64 inpux`: Last desired X position for the robot.
- `float64 inpuy`: Last desired Y position for the robot.

---


# RUN THE CODE
First clone the project in a folder named `src` then you need to move the `run.sh` program to the directory where your src folder is located.

You have to open your terminal inside the scripts folder and run these commands bellow to make your programms executable:

`chmod +x node_a.py
chmod +x node_b.py
chmod +x node_c.py
chmod +x bug_as.py
chmod +x go_to_point_service.py
chmod +x  wall_follow_service.py`

before using the command above make sure to install `xterm` if it's not already installed using command bellow:
`sudo apt-get install xterm`

after that head back to the directory where your src folder is located in.

make shure your run program executable using `chmod +x run.sh`after these your program will be able to run the whole assignment when you call the `run.sh` program by using `./run.sh` command.

# Pseudocode for Node A: set_target_client.py

`Import necessary libraries

Define the GoalHandler class
    Initialize the class
        Create a publisher to the /pos_vel topic
        Create an action client for the /reaching_goal action server
        Wait for the action server to be available
        Initialize a flag to indicate if the current goal has been cancelled

    Define the goal method
        Loop until ROS is shutdown
            Subscribe to the /odom topic and call pubPosVelocity method when a message is received
            Prompt the user to enter a command
            Get the current target position from the parameter server
            Create a new goal with the current target position

            If the user command is '1'
                Prompt the user to enter the x and y coordinates for the new goal
                If the input is valid, update the target position parameters and the goal
                Send the new goal to the action server
                Update the goal cancelled flag

            If the user command is '0'
                If there is an active goal, cancel it and update the goal cancelled flag
                Log the last received goal

            Log the last received goal

    Define the pubPosVelocity method
        Extract the current position and velocity from the Odometry message
        Create a new Vel message with the current position and velocity
        Publish the Vel message

Define the main function
    Initialize the node
    Create an instance of the GoalHandler class
    Call the goal method of the GoalHandler instance

If the script is the main program, call the main function`

# Pseudocode for Node B and Node C: last_target_service.py

`Import necessary libraries

Define the LastTargetService class
    Initialize the class
        Initialize class variables for the last desired x and y positions
        Initialize the node with the name 'last_target_service'
        Provide a service named 'input', using the custom service type Input
            Define the inputCall method
                Create a response message
                Try to set the x and y inputs in the response to the last desired positions
                If an error occurs, log the error
                Set the x and y inputs in the response to the last desired positions
                Return the response

            Define the keepRun method
                Keep the node running using rospy.spin()

    Main function
        Create an instance of the LastTargetService class
        Start the node by calling the keepRun method

If the script is the main program, create an instance of the LastTargetService class and start the node by calling the keepRun method`
