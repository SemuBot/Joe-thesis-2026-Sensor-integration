#!/bin/bash
source ~/.bashrc
colcon build --packages-select ultrasonic_nav
source install/setup.bash

read -p "Press [Enter] to start the program..."
ros2 launch ultrasonic_nav obstacle_avoidance.launch.py
