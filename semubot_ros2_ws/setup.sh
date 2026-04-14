#!/bin/bash

# Setup automatic removal of build files and make project
# easier to build again.
rm -rf build/*

source ~/.bashrc
colcon build --symlink-install
source install/setup.bash

# Press enter to start program

read -p "Press [Enter] to start the program..."
ros2 launch semubot display_robot.launch.py