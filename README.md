# RT-Assignment-2
# Node A: `node_a.py`

## Overview
This is a ROS (Robot Operating System) Python script (`node_a.py`) designed to interact with a robot's navigation system. It subscribes to the `/odom` topic to receive odometry information, and it allows the user to set a goal position for the robot using the `reaching_goal` action server. Additionally, it provides an option to cancel the current goal. The node continuously prompts the user for input and sends the goal to the action server when requested.

## Dependencies
- ROS (Melodic or later)
- Python 3
- `geometry_msgs`, `nav_msgs`, `actionlib`, `std_srvs` ROS packages

## Usage
1. Ensure that your ROS workspace is properly sourced.
2. Run the ROS master node (if not already running): `roscore`.
3. Execute the node: `rosrun <your_package_name> node_a.py`.
4. Follow the on-screen prompts to set or cancel goals for the robot.

# Node B: `node_b.py`

## Overview
This is a ROS Python script (`node_b.py`) serving as a ROS service that provides information about the last desired position set by the user in `node_a.py`. It responds to service requests by returning the last set X and Y coordinates.

## Dependencies
- ROS (Melodic or later)
- Python 3
- `assignment_2_2023` ROS package (containing the custom message `Vel` and service `Input`)

## Usage
1. Ensure that your ROS workspace is properly sourced.
2. Run the ROS master node (if not already running): `roscore`.
3. Execute the node: `rosrun <your_package_name> node_b.py`.
4. Use the provided service `input` to get the last desired position: `rosservice call /input`.

# Node C: `node_c.py`

## Overview
This is a ROS Python script (`node_c.py`) serving as a ROS service similar to `node_b.py`, providing information about the last desired position set by the user in `node_a.py`. It responds to service requests by returning the last set X and Y coordinates.

## Dependencies
- ROS (Melodic or later)
- Python 3
- `assignment_2_2023` ROS package (containing the custom message `Vel` and service `Input`)

## Usage
1. Ensure that your ROS workspace is properly sourced.
2. Run the ROS master node (if not already running): `roscore`.
3. Execute the node: `rosrun <your_package_name> node_c.py`.
4. Use the provided service `input` to get the last desired position: `rosservice call /input`.

