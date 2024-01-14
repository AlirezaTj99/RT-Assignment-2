#!/bin/bash

# Build the ROS workspace
catkin_make

# Source the setup file
source devel/setup.bash

# Run roscore in the current terminal
roscore &

# Wait for a few seconds to ensure that roscore is ready
sleep 5

# Open a new terminal in the same directory
gnome-terminal --working-directory=$(pwd) -- bash -c "source devel/setup.bash; roslaunch assignment_2_2023 assignment1.launch; exec bash"
